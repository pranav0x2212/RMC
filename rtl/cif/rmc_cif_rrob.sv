// rmc_cif_rrob.sv
// READ reorder buffer. Admits read requests, tracks completion, retires in program
// order to drive the AXI R response path.
//
// Slot ring (no shift register, no per-slot valid bit):
//   - alloc_ptr  : next free slot; advances on admit (many requests may be mid-split)
//   - retire_ptr : oldest slot; advances on retire
//   - occ        : entries in flight; gives full / empty
//
// Multi-outstanding: the pipelined rseg splits several requests at once, so admit is
// decoupled from SEG_LAST. alloc_ptr advances at admit (handing ROB_IDX out); SEG_LAST
// arrives later, carrying SEG_ROB_IDX to close that specific entry. rseg's seg_last is
// in program order, but indexing by SEG_ROB_IDX keeps the ROB robust to it either way.
//
// n_pkts ships EARLY: it is known the moment the request is admitted (the segmenter
// computes it at enqueue), so the ROB writes it at admit. MC completions can beat the
// split's SEG_LAST, so the count target must already be valid. SEG_LAST then only marks
// the split done and delivers the resp meta.
//
// One request:
//   admit    : write {id, err, n_pkts} at alloc_ptr, clear outflow, advance alloc_ptr
//   handoff  : rseg captures ROB_IDX (= alloc_ptr) on admit; it rides the request
//   SEG_LAST : inflow_complt[SEG_ROB_IDX]=1, mem_meta <= {len,size,offset,head}
//   cmpl     : outflow_cnt[rob_index]++ ; cmpl_err tags a downstream error
//   retire   : when inflow_complt && outflow_cnt==n_pkts, drive RESP_* (id, resp, and
//              the read meta head_ptr/offset/len/size for the resp-path trim), pop slot
//
// Completion is a pulse {rob_index, err}; the ROB counts pulses, no packet number.

import rmc_cfg_pkg::*;

module rmc_cif_rrob #(
  parameter int ROB_DEPTH = 32,   // physical entries (16-32)        [root knob]
  parameter int MAX_PKTS  = 64,   // max packets per request         [root knob]
  parameter int AXI_IDW   = 8,    // AXI id width                    [root knob]
  parameter int N_SLOTS   = 32,   // RD-SRAM slots (head_mem_ptr width) [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W  = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1, // rob_idx tag width
  localparam int PTR_W      = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1, // ring pointer width
  localparam int CNT_W      = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1, // packet-count width
  localparam int REQ_META_W = AXI_IDW + 1,                    // admit payload {id, validated}
  localparam int SLOT_PTR_W = (N_SLOTS > 1) ? $clog2(N_SLOTS) : 1, // RD-SRAM head-slot width
  localparam int META_W     = 8 + 3 + PKT_OFF_W + SLOT_PTR_W  // resp meta {len8,size3,offset,head_ptr}
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // completion in (MC compl async FIFO). Pulse, never back-pressures.
  input  logic                   cmpl_valid,
  input  logic [ROB_IDX_W-1:0]   cmpl_rob_index,
  input  logic                   cmpl_err,         // downstream error -> SLVERR

  // admit (from the pre-ROB AR_req buffer, post request_validator). Only {id, validated}
  // here - addr/len/size ride the segmenter queue and come back as resp meta at SEG_LAST.
  // n_pkts is known at enqueue, so it ships EARLY with admit (see ADMIT_NPKTS).
  input  logic [REQ_META_W-1:0]  AW_REQUEST,       // {A_ID, A_VALIDATED}
  input  logic [CNT_W-1:0]       ADMIT_NPKTS,      // total packets for this request (at admit)
  input  logic                   A_ID_VALID,
  output logic                   A_ID_READY,       // free slot

  // rrob -> rseg handoff. ROB_IDX (= alloc_ptr) is handed out at admit and rides the
  // request; the entry is "filled" (n_pkts written) right then. Completions for it may
  // arrive before the split even finishes, so n_pkts must already be valid - hence the
  // early ship. SEG_LAST later marks the split done for that index and brings the resp
  // meta; it no longer carries n_pkts.
  output logic [ROB_IDX_W-1:0]   ROB_IDX,          // offered slot index (= alloc_ptr), at admit
  input  logic                   SEG_LAST,         // a request finished splitting
  input  logic [ROB_IDX_W-1:0]   SEG_ROB_IDX,      // which entry it was (closes that slot)
  // read meta from rseg, latched at SEG_LAST
  input  logic [PKT_OFF_W-1:0]   SEG_OFFSET,       // unaligned start offset
  input  logic [7:0]             SEG_LEN,          // ar_len  (beats-1)
  input  logic [2:0]             SEG_SIZE,         // ar_size (log2 beat bytes)
  input  logic [SLOT_PTR_W-1:0]  SEG_HEAD_PTR,     // RD-SRAM head slot

  // retire / R response
  output logic                   RESP_VALID,
  input  logic                   RESP_READY,
  output logic [AXI_IDW-1:0]     RESP_ID,
  output logic [1:0]             RESP_RESP,        // 00 OKAY / 10 SLVERR
  output logic [SLOT_PTR_W-1:0]  RESP_HEAD_PTR,    // RD-SRAM head slot
  output logic [PKT_OFF_W-1:0]   RESP_OFFSET,      // start byte offset (pre/post trim)
  output logic [7:0]             RESP_LEN,         // ar_len
  output logic [2:0]             RESP_SIZE,        // ar_size

  // DFT taps (passive, output only). Per-slot buses are slot-major: slot i at [i*W +: W].
  output logic [PTR_W-1:0]                  dft_alloc_ptr,    // next slot to admit
  output logic [PTR_W-1:0]                  dft_retire_ptr,   // oldest slot
  output logic [$clog2(ROB_DEPTH+1)-1:0]    dft_occ,          // entries in flight
  output logic                              dft_admit_fire,
  output logic                              dft_retire_fire,
  output logic [ROB_DEPTH-1:0]              dft_inflow_complt,
  output logic [ROB_DEPTH-1:0]              dft_ent_err,      // validation-fail flag per slot
  output logic [ROB_DEPTH-1:0]              dft_ent_derr,     // downstream-error flag per slot
  output logic [ROB_DEPTH*CNT_W-1:0]        dft_outflow_cnt,
  output logic [ROB_DEPTH*CNT_W-1:0]        dft_n_pkts,       // written at admit
  output logic [ROB_DEPTH*SLOT_PTR_W-1:0]   dft_slot_head
);

  // slot ring
  localparam int OCC_W = $clog2(ROB_DEPTH + 1);    // occupancy counter width
  logic [PTR_W-1:0] alloc_ptr;                      // next free slot (advances on admit)
  logic [PTR_W-1:0] retire_ptr;                     // oldest slot (advances on retire)
  logic [OCC_W-1:0] occ;                            // entries in flight
  logic             rob_full;                       // no free slot
  logic             rob_empty;                      // nothing in flight
  assign rob_full  = (occ == OCC_W'(ROB_DEPTH));
  assign rob_empty = (occ == '0);

  // per-slot entry (flops)
  logic [AXI_IDW-1:0] ent_id        [ROB_DEPTH];    // AXI id (-> RESP_ID)
  logic               ent_err       [ROB_DEPTH]; // validation fail -> SLVERR
  logic               ent_derr      [ROB_DEPTH]; // downstream error -> SLVERR
  logic               inflow_complt [ROB_DEPTH]; // all packets issued (SEG_LAST)
  logic [CNT_W-1:0]   n_pkts        [ROB_DEPTH];    // packets owed (set at admit)
  logic [CNT_W-1:0]   outflow_cnt   [ROB_DEPTH];    // completions seen so far

  // admit payload field-slice (len/size reach mem_meta from rseg at SEG_LAST)
  logic [AXI_IDW-1:0] admit_id;                     // id sliced from AW_REQUEST
  logic               admit_ok;                     // validated bit (0 -> SLVERR)
  assign admit_id = AW_REQUEST[REQ_META_W-1 -: AXI_IDW];
  assign admit_ok = AW_REQUEST[0];

  // handoff: A_ID_READY is the ROB's ready. Multi-outstanding - admit only needs a free
  // slot, so the segmenter can keep several requests in flight at once. ROB_IDX (the slot
  // handed out) rides with the request and tags every packet for completion routing.
  wire  admit_fire = A_ID_VALID & A_ID_READY;  // a request is admitted this cycle
  assign A_ID_READY = ~rob_full;
  assign ROB_IDX    = alloc_ptr;

  // retire / response (comb)
  assign RESP_VALID = ~rob_empty
                    & inflow_complt[retire_ptr]
                    & (outflow_cnt[retire_ptr] == n_pkts[retire_ptr]);
  assign RESP_ID    = ent_id[retire_ptr];
  assign RESP_RESP  = (ent_err[retire_ptr] | ent_derr[retire_ptr]) ? 2'b10 : 2'b00;
  wire retire_fire  = RESP_VALID & RESP_READY;  // oldest entry pops this cycle

  // metadata RAM: written at SEG_LAST from rseg, read out on the resp extras
  // layout (MSB->LSB): {len[8], size[3], start_offset[PKT_OFF_W], head_mem_ptr[SLOT_PTR_W]}
  logic [META_W-1:0] mem_meta [ROB_DEPTH];      // per-slot resp meta (trim info)
  always_ff @(posedge aclk)
    if (SEG_LAST) mem_meta[SEG_ROB_IDX] <= {SEG_LEN, SEG_SIZE, SEG_OFFSET, SEG_HEAD_PTR};
  assign RESP_LEN      = mem_meta[retire_ptr][META_W-1 -: 8];
  assign RESP_SIZE     = mem_meta[retire_ptr][SLOT_PTR_W+PKT_OFF_W +: 3];
  assign RESP_OFFSET   = mem_meta[retire_ptr][SLOT_PTR_W +: PKT_OFF_W];
  assign RESP_HEAD_PTR = mem_meta[retire_ptr][SLOT_PTR_W-1:0];

  // DFT taps (wiring only)
  assign dft_alloc_ptr   = alloc_ptr;
  assign dft_retire_ptr  = retire_ptr;
  assign dft_occ         = occ;
  assign dft_admit_fire  = admit_fire;
  assign dft_retire_fire = retire_fire;
  for (genvar i = 0; i < ROB_DEPTH; i++) begin : g_dft_slot
    assign dft_inflow_complt[i]                      = inflow_complt[i];
    assign dft_ent_err[i]                            = ent_err[i];
    assign dft_ent_derr[i]                           = ent_derr[i];
    assign dft_outflow_cnt[i*CNT_W +: CNT_W]         = outflow_cnt[i];
    assign dft_n_pkts[i*CNT_W +: CNT_W]              = n_pkts[i];
    assign dft_slot_head[i*SLOT_PTR_W +: SLOT_PTR_W] = mem_meta[i][SLOT_PTR_W-1:0];
  end

  // all flop updates (admit / SEG_LAST / cmpl / retire are parallel)
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      alloc_ptr     <= '0;
      retire_ptr    <= '0;
      occ           <= '0;
    end else begin
      // admit: claim the slot, write n_pkts (known now), advance alloc_ptr.
      if (admit_fire) begin
        ent_id[alloc_ptr]        <= admit_id;
        ent_err[alloc_ptr]       <= ~admit_ok;
        ent_derr[alloc_ptr]      <= 1'b0;
        inflow_complt[alloc_ptr] <= 1'b0;
        outflow_cnt[alloc_ptr]   <= '0;
        n_pkts[alloc_ptr]        <= ADMIT_NPKTS;
        alloc_ptr <= (alloc_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : alloc_ptr + 1'b1;
      end
      // SEG_LAST closes the entry named by SEG_ROB_IDX (not alloc_ptr): its split done.
      // n_pkts was already written at admit; here only inflow + the resp meta land.
      if (SEG_LAST)
        inflow_complt[SEG_ROB_IDX] <= 1'b1;
      if (cmpl_valid) begin
        outflow_cnt[cmpl_rob_index] <= outflow_cnt[cmpl_rob_index] + 1'b1;
        if (cmpl_err) ent_derr[cmpl_rob_index] <= 1'b1;
      end
      if (retire_fire)
        retire_ptr <= (retire_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : retire_ptr + 1'b1;
      occ <= occ + OCC_W'(admit_fire) - OCC_W'(retire_fire);
    end
  end

endmodule : rmc_cif_rrob
