// rmc_cif_rrob.sv
// READ reorder buffer. Admits read requests, tracks completion, retires in program
// order to drive the AXI R response path.
//
// Slot ring (no shift register, no per-slot valid bit):
//   - alloc_ptr  : current fill/split slot; advances on SEG_LAST
//   - retire_ptr : oldest slot; advances on retire
//   - occ        : entries in flight; gives full / empty
//
// One request:
//   admit    : write {id, err} at alloc_ptr, clear counters, set pending_split
//   handoff  : rseg captures ROB_IDX (= alloc_ptr) on req_in_valid & A_ID_READY
//   SEG_LAST : inflow_complt=1, n_pkts=SEG_NPKTS, mem_meta <= {len,size,offset,head}
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
  localparam int ROB_IDX_W  = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int PTR_W      = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int CNT_W      = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int REQ_META_W = AXI_IDW + 8 + 3 + 1,            // {id,len8,size3,validated1}
  localparam int SLOT_PTR_W = (N_SLOTS > 1) ? $clog2(N_SLOTS) : 1,
  localparam int META_W     = 8 + 3 + PKT_OFF_W + SLOT_PTR_W  // {len8,size3,offset,head_ptr}
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // completion in (MC compl async FIFO). Pulse, never back-pressures.
  input  logic                   cmpl_valid,
  input  logic [ROB_IDX_W-1:0]   cmpl_rob_index,
  input  logic                   cmpl_err,         // downstream error -> SLVERR

  // admit (from the pre-ROB AR_req buffer, post request_validator)
  input  logic [REQ_META_W-1:0]  AW_REQUEST,       // {A_ID, A_LEN, A_SIZE, A_VALIDATED}
  input  logic                   A_ID_VALID,
  output logic                   A_ID_READY,       // free slot, none mid-split

  // rrob -> rseg handoff
  output logic [ROB_IDX_W-1:0]   ROB_IDX,          // offered slot index (= alloc_ptr)
  input  logic                   SEG_LAST,         // last packet issued
  input  logic [CNT_W-1:0]       SEG_NPKTS,        // packet count (valid at SEG_LAST)
  // read meta from rseg, latched at SEG_LAST
  input  logic [PKT_OFF_W-1:0]   SEG_OFFSET,
  input  logic [7:0]             SEG_LEN,
  input  logic [2:0]             SEG_SIZE,
  input  logic [SLOT_PTR_W-1:0]  SEG_HEAD_PTR,

  // retire / R response
  output logic                   RESP_VALID,
  input  logic                   RESP_READY,
  output logic [AXI_IDW-1:0]     RESP_ID,
  output logic [1:0]             RESP_RESP,        // 00 OKAY / 10 SLVERR
  output logic [SLOT_PTR_W-1:0]  RESP_HEAD_PTR,    // RD-SRAM head slot
  output logic [PKT_OFF_W-1:0]   RESP_OFFSET,      // start byte offset (pre/post trim)
  output logic [7:0]             RESP_LEN,         // ar_len
  output logic [2:0]             RESP_SIZE,        // ar_size

  // DFT taps (passive, output only; no effect on ROB state). Per-slot buses are
  // slot-major: slot i occupies bits [i*W +: W].
  output logic [PTR_W-1:0]                  dft_alloc_ptr,    // slot being filled/split
  output logic [PTR_W-1:0]                  dft_retire_ptr,   // oldest slot
  output logic [$clog2(ROB_DEPTH+1)-1:0]    dft_occ,          // entries in flight
  output logic                              dft_admit_fire,
  output logic                              dft_retire_fire,
  output logic                              dft_pending_split, // slot handed to segmenter
  output logic [ROB_DEPTH-1:0]              dft_inflow_complt,
  output logic [ROB_DEPTH-1:0]              dft_ent_err,      // validation-fail flag per slot
  output logic [ROB_DEPTH-1:0]              dft_ent_derr,     // downstream-error flag per slot
  output logic [ROB_DEPTH*CNT_W-1:0]        dft_outflow_cnt,
  output logic [ROB_DEPTH*CNT_W-1:0]        dft_n_pkts,
  output logic [ROB_DEPTH*SLOT_PTR_W-1:0]    dft_slot_head
);

  // slot ring: occupancy gives full/empty; pointers wrap at ROB_DEPTH
  localparam int OCC_W = $clog2(ROB_DEPTH + 1);
  logic [PTR_W-1:0] alloc_ptr;
  logic [PTR_W-1:0] retire_ptr;
  logic [OCC_W-1:0] occ;
  logic             rob_full;
  logic             rob_empty;
  assign rob_full  = (occ == OCC_W'(ROB_DEPTH));
  assign rob_empty = (occ == '0);

  // per-slot entry (flops)
  logic [AXI_IDW-1:0] ent_id        [ROB_DEPTH];
  logic               ent_err       [ROB_DEPTH]; // validation fail -> SLVERR
  logic               ent_derr      [ROB_DEPTH]; // downstream error -> SLVERR
  logic               inflow_complt [ROB_DEPTH]; // all packets issued (SEG_LAST)
  logic [CNT_W-1:0]   n_pkts        [ROB_DEPTH];
  logic [CNT_W-1:0]   outflow_cnt   [ROB_DEPTH];

  // admit payload field-slice (len/size reach mem_meta from rseg at SEG_LAST)
  logic [AXI_IDW-1:0] admit_id;
  logic               admit_ok;
  assign admit_id = AW_REQUEST[REQ_META_W-1 -: AXI_IDW];
  assign admit_ok = AW_REQUEST[0];

  // handoff: A_ID_READY is the ROB's ready (also rseg's ROB_READY). pending_split
  // holds it low through the split so the next admit waits.
  logic pending_split;
  wire  admit_fire = A_ID_VALID & A_ID_READY;
  assign A_ID_READY = ~rob_full & ~pending_split;
  assign ROB_IDX    = alloc_ptr;

  // retire / response (comb)
  assign RESP_VALID = ~rob_empty
                    & inflow_complt[retire_ptr]
                    & (outflow_cnt[retire_ptr] == n_pkts[retire_ptr]);
  assign RESP_ID    = ent_id[retire_ptr];
  assign RESP_RESP  = (ent_err[retire_ptr] | ent_derr[retire_ptr]) ? 2'b10 : 2'b00;
  wire retire_fire  = RESP_VALID & RESP_READY;

  // metadata RAM (unreset, reads X until written): written at SEG_LAST from rseg, read out on the resp extras
  // layout (MSB->LSB): {len[8], size[3], start_offset[PKT_OFF_W], head_mem_ptr[SLOT_PTR_W]}
  logic [META_W-1:0] mem_meta [ROB_DEPTH];
  always_ff @(posedge aclk)
    if (SEG_LAST) mem_meta[alloc_ptr] <= {SEG_LEN, SEG_SIZE, SEG_OFFSET, SEG_HEAD_PTR};
  assign RESP_LEN      = mem_meta[retire_ptr][META_W-1 -: 8];
  assign RESP_SIZE     = mem_meta[retire_ptr][SLOT_PTR_W+PKT_OFF_W +: 3];
  assign RESP_OFFSET   = mem_meta[retire_ptr][SLOT_PTR_W +: PKT_OFF_W];
  assign RESP_HEAD_PTR = mem_meta[retire_ptr][SLOT_PTR_W-1:0];


  // DFT taps (wiring only; no logic)
  assign dft_alloc_ptr     = alloc_ptr;
  assign dft_retire_ptr    = retire_ptr;
  assign dft_occ           = occ;
  assign dft_admit_fire    = admit_fire;
  assign dft_retire_fire   = retire_fire;
  assign dft_pending_split = pending_split;
  for (genvar i = 0; i < ROB_DEPTH; i++) begin : g_dft_slot
    assign dft_inflow_complt[i]              = inflow_complt[i];
    assign dft_ent_err[i]                    = ent_err[i];
    assign dft_ent_derr[i]                   = ent_derr[i];
    assign dft_outflow_cnt[i*CNT_W +: CNT_W] = outflow_cnt[i];
    assign dft_n_pkts[i*CNT_W +: CNT_W]      = n_pkts[i];
    assign dft_slot_head[i*SLOT_PTR_W +: SLOT_PTR_W] = mem_meta[i][SLOT_PTR_W-1:0];
  end

  // all flop updates (admit / SEG_LAST / cmpl / retire are parallel)
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      alloc_ptr     <= '0;
      retire_ptr    <= '0;
      occ           <= '0;
      pending_split <= 1'b0;
    end else begin
      if (admit_fire) begin
        ent_id[alloc_ptr]        <= admit_id;
        ent_err[alloc_ptr]       <= ~admit_ok;
        ent_derr[alloc_ptr]      <= 1'b0;
        inflow_complt[alloc_ptr] <= 1'b0;
        outflow_cnt[alloc_ptr]   <= '0;
        pending_split            <= 1'b1;
      end
      if (SEG_LAST) begin
        inflow_complt[alloc_ptr] <= 1'b1;
        n_pkts[alloc_ptr]        <= SEG_NPKTS;
        pending_split            <= 1'b0;
        alloc_ptr <= (alloc_ptr == PTR_W'(ROB_DEPTH-1)) ? '0 : alloc_ptr + 1'b1;
      end
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
