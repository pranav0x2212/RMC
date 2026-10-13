// rmc_cif_rseg.sv
// READ burst segmenter - full, pipelined. A pre-ROB queue (ring, QUEUE_DEPTH) holds
// admitted requests; N_CH per-channel engines (rmc_cif_rseg_ch) each walk the ring at
// their own pace and issue their channel's 64B packets. An entry pops (in order) when
// every channel has marked it done, pulsing seg_last to the ROB.
//
// No lockstep: a channel that owns many packets holds only its own rd_ptr on one entry;
// the other engines run ahead onto later entries. rob_idx is pre-allocated at enqueue
// and rides in the queue entry, tagging every packet so MC completions route back.
//
// Queue entry {aligned_base, offset, n_pkts, dbuf_base, rob_idx} is written at enqueue;
// n_pkts (needs the unaligned offset) is computed here, once. The offset is stored and
// ridden out on seg_offset for the ROB response trim - engines never touch it.
//
// QUEUE_DEPTH must be a power of 2 (ring index wraps on the low bits).

import rmc_cfg_pkg::*;

module rmc_cif_rseg #(
  parameter int ROB_DEPTH   = 32,         // ROB entries (rob_idx width)   [root knob]
  parameter int MAX_PKTS    = 64,         // max packets per request       [root knob]
  parameter int AXI_AW      = SYS_ADDR_W, // AXI address width             [root knob]
  parameter int N_SLOTS     = 32,         // RD-SRAM slots                 [root knob]
  parameter int QUEUE_DEPTH = ROB_DEPTH,  // pre-ROB queue entries; = ROB_DEPTH so a channel
                                          // can hold every outstanding request (no premature
                                          // per-channel admit HOL; stalls only on ROB-full) [knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W  = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1, // rob_idx tag width
  localparam int CNT_W      = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1, // packet-count width (0..MAX_PKTS)
  localparam int SLOT_W     = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1, // RD-SRAM slot index width
  localparam int LINE_W     = AXI_AW - PKT_OFF_W,                     // 64B line number width
  localparam int QIDX_W     = (QUEUE_DEPTH > 1) ? $clog2(QUEUE_DEPTH) : 1, // queue ring index width
  localparam int QPTR_W     = QIDX_W + 1,                             // queue pointer = index + wrap bit
  localparam int DATA_BYTES = AXI_DW / 8,                            // AXI beat width in bytes (= N_CH*8)
  localparam int SIZE_MAX   = (DATA_BYTES   > 1) ? $clog2(DATA_BYTES) : 0, // max awsize (log2 bytes/beat)
  localparam int MAX_BEATS  = MAX_REQ_BYTES / DATA_BYTES,            // max beats in one request
  localparam int LEN_W      = (MAX_BEATS    > 1) ? $clog2(MAX_BEATS)  : 1, // awlen width (beats-1)
  localparam int SIZE_W     = (SIZE_MAX     > 0) ? $clog2(SIZE_MAX+1) : 1, // awsize width
  localparam int SPAN_W     = $clog2(MAX_REQ_BYTES + 1)              // bytes-spanned width (<=4KB)
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // live address map (CSR), broadcast to every engine
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // enqueue (one request into the pre-ROB queue)
  input  logic                   start_req,       // a request is offered this cycle
  input  logic [AXI_AW-1:0]      req_addr,         // byte start address
  input  logic [LEN_W-1:0]       req_len,          // ar_len  (beats-1)
  input  logic [SIZE_W-1:0]      req_size,         // ar_size (log2 beat bytes)
  input  logic [SLOT_W-1:0]      req_dbuf_base,    // RD-SRAM base slot for this request
  input  logic [ROB_IDX_W-1:0]   req_rob_idx,      // pre-allocated ROB index (rides the entry)
  output logic                   ready,            // ~full: queue can accept a request

  // per-channel packet streams out (one req FIFO per channel -> MC)
  output logic [N_CH-1:0]                pkt_valid,    // packet offered on this channel
  input  logic [N_CH-1:0]                pkt_ready,    // channel req FIFO can take it
  input  logic [N_CH-1:0]                pkt_dbuf_adr_ready, // RD-SRAM has a slot for it
  output logic [N_CH-1:0][ROB_IDX_W-1:0] pkt_rob_idx,  // owning request's ROB index
  output logic [N_CH-1:0][DADDR_W-1:0]   pkt_daddr,    // mapped DRAM coords for the line
  output logic [N_CH-1:0][SLOT_W-1:0]    pkt_dbuf_adr, // RD-SRAM slot = dbuf_base + k

  // to the ROB: one pulse when an entry is fully segmented (in-order head pop),
  // plus the read meta the ROB latches for the R-response trim.
  output logic                   seg_last,         // a request finished splitting
  output logic [ROB_IDX_W-1:0]   seg_rob_idx,      // which ROB entry it was
  output logic [CNT_W-1:0]       seg_npkts,        // its total packet count
  output logic [PKT_OFF_W-1:0]   seg_offset,       // unaligned start offset addr[5:0]
  output logic [LEN_W-1:0]       seg_len,          // ar_len  (beats-1)
  output logic [SIZE_W-1:0]      seg_size,         // ar_size (log2 beat bytes)
  output logic [SLOT_W-1:0]      seg_head_ptr,     // RD-SRAM head slot (= dbuf_base)

  // DFT taps (passive, output only; per-slot / per-channel buses are index-major)
  output logic [QPTR_W-1:0]      dft_alloc_ptr,    // queue tail (enqueue)
  output logic [QPTR_W-1:0]      dft_pop_ptr,      // queue head (in-order pop)
  output logic [QUEUE_DEPTH*N_CH-1:0] dft_ch_done, // done bits, slot e at [e*N_CH +: N_CH]
  output logic [N_CH*QPTR_W-1:0] dft_eng_rd_ptr,   // each engine's ring read pointer
  output logic [N_CH*CNT_W-1:0]  dft_eng_m         // packets each engine issued for its entry
);

  // Complete as a splitter. Remaining integration / open items live in rmc_cif.sv
  // (RD-SRAM allocator for req_dbuf_base, per-channel req FIFOs + N_CH->1 arbiter for
  // pkt_*, dbuf-addressing choice, scattered-map fallback).

  // ---- pre-ROB queue (ring) ----
  // Holds admitted-but-not-yet-split requests. One slot per request; the N_CH engines
  // read these fields (via the wrapper mux) while they issue that request's packets, and
  // ch_done[slot][c] records that channel c has finished its share of the slot.
  logic [LINE_W-1:0]    q_aligned_base [QUEUE_DEPTH];   // 64B base line (addr >> 6)
  logic [PKT_OFF_W-1:0] q_offset       [QUEUE_DEPTH];   // unaligned start offset (addr[5:0])
  logic [CNT_W-1:0]     q_npkts        [QUEUE_DEPTH];   // packets this request splits into
  logic [LEN_W-1:0]     q_len          [QUEUE_DEPTH];   // ar_len  -> resp trim
  logic [SIZE_W-1:0]    q_size         [QUEUE_DEPTH];   // ar_size -> resp trim
  logic [SLOT_W-1:0]    q_dbuf_base    [QUEUE_DEPTH];   // RD-SRAM head slot (= seg_head_ptr)
  logic [ROB_IDX_W-1:0] q_rob_idx      [QUEUE_DEPTH];   // pre-allocated ROB index
  logic [N_CH-1:0]      ch_done        [QUEUE_DEPTH];   // per-channel "done this slot" bits

  // ring pointers: alloc at the tail (enqueue), pop at the head (in-order free). Both
  // carry an extra top bit over the index so full and empty are distinguishable even
  // when the low index bits are equal.
  logic [QPTR_W-1:0] alloc_ptr;    // enqueue (extended, wrap bit)
  logic [QPTR_W-1:0] pop_ptr;      // oldest entry (in-order free)

  wire [QIDX_W-1:0] alloc_idx = alloc_ptr[QIDX_W-1:0];  // tail slot index (array addr)
  wire [QIDX_W-1:0] pop_idx   = pop_ptr  [QIDX_W-1:0];  // head slot index (array addr)
  wire queue_full  = (alloc_ptr - pop_ptr) == QPTR_W'(QUEUE_DEPTH); // all slots occupied
  wire queue_empty = (alloc_ptr == pop_ptr);            // no live entries

  assign ready = ~queue_full;      // accept a request whenever the ring has a free slot
  wire enqueue = start_req & ready; // a request is actually written this cycle

  // n_pkts at enqueue: span from 64B line start = align_down(offset, size) +
  // (len+1)<<size; ceil(/64). AXI aligns every beat after the first to the beat size,
  // so the burst ends at align_down(addr, size) + (len+1)*2^size - the raw offset
  // would count one line too many for an unaligned start.
  logic [PKT_OFF_W-1:0] off_aligned;   // start offset aligned down to the beat size
  logic [SPAN_W-1:0]    span;          // bytes touched, from the 64B line start
  logic [CNT_W-1:0]     npkts_calc;    // ceil(span/64) = packets for this request
  assign off_aligned = req_addr[PKT_OFF_W-1:0] & ~PKT_OFF_W'((SPAN_W'(1) << req_size) - 1'b1);
  assign span        = SPAN_W'(off_aligned) + ((SPAN_W'(req_len) + 1'b1) << req_size);
  assign npkts_calc = CNT_W'((span + SPAN_W'((1 << PKT_OFF_W) - 1)) >> PKT_OFF_W);

  // engines (one per channel)
  // Each engine owns a channel and its own rd_ptr into the ring; it walks entries at its
  // own pace. The wrapper feeds it the entry at its rd_ptr (q_* muxed by rdi below) and
  // collects its set_done pulse.
  logic [QPTR_W-1:0] eng_rd_ptr   [N_CH];   // each engine's read pointer into the ring
  logic              eng_set_done [N_CH];   // engine c finished its share of its entry
  logic [CNT_W-1:0]  eng_m        [N_CH];   // engine c packets issued (DFT tap)

  // set mask: fan each engine's set_done onto the ch_done bit of the entry it is on, so
  // the flop block can OR them in (engines may be on different entries this cycle).
  logic [N_CH-1:0]   set_bits [QUEUE_DEPTH];
  always_comb begin
    for (int e = 0; e < QUEUE_DEPTH; e++) set_bits[e] = '0;
    for (int c = 0; c < N_CH; c++)
      if (eng_set_done[c]) set_bits[eng_rd_ptr[c][QIDX_W-1:0]][c] = 1'b1;
  end

  // instance the N_CH engines; entry_valid = this engine has not caught up to the tail.
  genvar c;
  generate
    for (c = 0; c < N_CH; c++) begin : g_ch
      wire [QIDX_W-1:0] rdi = eng_rd_ptr[c][QIDX_W-1:0];  // slot engine c is reading
      rmc_cif_rseg_ch #(
        .ROB_DEPTH   (ROB_DEPTH),
        .MAX_PKTS    (MAX_PKTS),
        .AXI_AW      (AXI_AW),
        .N_SLOTS     (N_SLOTS),
        .QUEUE_DEPTH (QUEUE_DEPTH)
      ) u_ch (
        .aclk           (aclk),
        .aresetn        (aresetn),
        .ch_id          (CH_W'(c)),
        .addr_map       (addr_map),
        .rd_ptr         (eng_rd_ptr[c]),
        .entry_valid    (eng_rd_ptr[c] != alloc_ptr),
        .e_aligned_base (q_aligned_base[rdi]),
        .e_n_pkts       (q_npkts[rdi]),
        .e_dbuf_base    (q_dbuf_base[rdi]),
        .e_rob_idx      (q_rob_idx[rdi]),
        .e_ch_done      (ch_done[rdi][c]),
        .set_done       (eng_set_done[c]),
        .pkt_valid      (pkt_valid[c]),
        .pkt_ready      (pkt_ready[c]),
        .pkt_dbuf_adr_ready (pkt_dbuf_adr_ready[c]),
        .pkt_rob_idx    (pkt_rob_idx[c]),
        .pkt_daddr      (pkt_daddr[c]),
        .pkt_dbuf_adr   (pkt_dbuf_adr[c]),
        .dft_m          (eng_m[c])
      );
    end
  endgenerate

  // head pop + seg_last (in order)
  // When every channel has finished the oldest entry, retire it: pulse seg_last to the
  // ROB with that request's rob_idx + the resp meta, and free the slot. In-order pop
  // keeps seg_last in program order even though engines finish entries out of order.
  wire head_all_done = ~queue_empty & (&ch_done[pop_idx]); // oldest entry fully segmented
  assign seg_last     = head_all_done;
  assign seg_rob_idx  = q_rob_idx[pop_idx];
  assign seg_npkts    = q_npkts[pop_idx];
  assign seg_offset   = q_offset[pop_idx];
  assign seg_len      = q_len[pop_idx];
  assign seg_size     = q_size[pop_idx];
  assign seg_head_ptr = q_dbuf_base[pop_idx];

  // queue flops
  // Three parallel updates: enqueue writes a new tail slot and bumps alloc_ptr; ch_done
  // clears for a freshly enqueued slot and otherwise accumulates the engines' set_bits;
  // head_all_done bumps pop_ptr to free the oldest slot.
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      alloc_ptr <= '0;
      pop_ptr   <= '0;
      for (int e = 0; e < QUEUE_DEPTH; e++) ch_done[e] <= '0;
    end else begin
      // enqueue: latch the request into the tail slot (addr split into line + offset,
      // n_pkts precomputed, len/size/dbuf/rob_idx carried for the split + resp).
      if (enqueue) begin
        q_aligned_base[alloc_idx] <= req_addr[AXI_AW-1:PKT_OFF_W];
        q_offset      [alloc_idx] <= req_addr[PKT_OFF_W-1:0];
        q_npkts       [alloc_idx] <= npkts_calc;
        q_len         [alloc_idx] <= req_len;
        q_size        [alloc_idx] <= req_size;
        q_dbuf_base   [alloc_idx] <= req_dbuf_base;
        q_rob_idx     [alloc_idx] <= req_rob_idx;
        alloc_ptr                 <= alloc_ptr + 1'b1;
      end
      // ch_done: fresh slot clears; otherwise OR in the engines' sets
      for (int e = 0; e < QUEUE_DEPTH; e++) begin
        if (enqueue & (alloc_idx == QIDX_W'(e)))
          ch_done[e] <= '0;
        else
          ch_done[e] <= ch_done[e] | set_bits[e];
      end
      if (head_all_done)
        pop_ptr <= pop_ptr + 1'b1;
    end
  end

  // DFT taps (wiring only)
  assign dft_alloc_ptr = alloc_ptr;
  assign dft_pop_ptr   = pop_ptr;
  for (genvar e = 0; e < QUEUE_DEPTH; e++) begin : g_dft_slot
    assign dft_ch_done[e*N_CH +: N_CH] = ch_done[e];
  end
  for (genvar k = 0; k < N_CH; k++) begin : g_dft_eng
    assign dft_eng_rd_ptr[k*QPTR_W +: QPTR_W] = eng_rd_ptr[k];
    assign dft_eng_m[k*CNT_W +: CNT_W]        = eng_m[k];
  end

endmodule : rmc_cif_rseg
