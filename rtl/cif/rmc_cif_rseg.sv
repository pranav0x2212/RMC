// rmc_cif_rseg.sv
// READ burst segmenter. Splits one admitted read request into 64B-line packets,
// one CAS each, and streams them to MC via the per-channel req async FIFOs; pulses
// SEG_LAST + SEG_NPKTS back to r_rob to close inflow, and forwards the read meta
// {offset, len, size, head slot} for r_rob to latch at SEG_LAST (resp-path trim).
//
// Packet k = 64B line (addr>>6) + k. The address map is a runtime bit permutation,
// so the seg steps the LINE number in system-address space and maps each packet
// itself (one rmc_cif_addrmap per window position, muxes only); the map also gives
// the packet's channel. Each cycle the next N_CH packets form a window and issue in
// order: a packet goes when every earlier one went, its channel is free this cycle
// and that channel's FIFO is ready. Packets never overtake.
//
// Read slots are CONTIGUOUS from the request base: slot = slot_base + pkt_num (the
// resp path recomputes offsets from head_ptr+k). Writes are different (free-list) -
// see rmc_cif_wseg.
//
// Relies on: INCR bursts only; a request never spans more than MAX_PKTS lines
// (validator rejects it); addr_map static while a request is split; N_CH pow2.

import rmc_cfg_pkg::*;

module rmc_cif_rseg #(
  parameter int ROB_DEPTH = 32,         // ROB entries             [root knob]
  parameter int MAX_PKTS  = 64,         // max packets per request [root knob]
  parameter int AXI_AW    = SYS_ADDR_W, // AXI address width       [root knob]
  parameter int N_SLOTS   = 32,         // RD-SRAM slots           [root knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int PKT_NUM_W = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS)   : 1,
  localparam int CNT_W     = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int SLOT_W    = (N_SLOTS   > 1) ? $clog2(N_SLOTS)    : 1,
  localparam int LINE_W    = AXI_AW - PKT_OFF_W                    // 64B line number
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // request in (from the pre-ROB AR_req buffer): address + burst params. Captured on
  // req_in_valid & ROB_READY (the admit beat); ROB owns the pop, seg has no ready.
  input  logic                   req_in_valid,     // buffer seg-half data valid
  input  logic [AXI_AW-1:0]      req_in_addr,      // system byte address of the request
  input  logic [7:0]             req_in_len,       // AXI len (beats - 1)
  input  logic [2:0]             req_in_size,      // AXI size
  input  logic [SLOT_W-1:0]      req_in_slot_base, // head RD-SRAM slot (contiguous base)

  // live address map (CSR), same bus the validator-side rmc_cif_addrmap uses
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // admitted slot index from r_rob + its ready; capture on req_in_valid & ROB_READY
  input  logic [ROB_IDX_W-1:0]   ROB_IDX,
  input  logic                   ROB_READY,        // r_rob admit-ready (= its A_ID_READY)

  // back to r_rob when the split finishes
  output logic                   SEG_LAST,         // last packet of the request issued
  output logic [CNT_W-1:0]       SEG_NPKTS,        // packet count of the request
  // read meta -> r_rob, latched there at SEG_LAST (resp-path pre/post trim)
  output logic [PKT_OFF_W-1:0]   SEG_OFFSET,       // start byte offset
  output logic [7:0]             SEG_LEN,          // AXI len
  output logic [2:0]             SEG_SIZE,         // AXI size
  output logic [SLOT_W-1:0]      SEG_HEAD_PTR,     // RD-SRAM head slot (= slot_base)

  // packet stream out, ONE PER CHANNEL: up to N_CH packets per cycle, one into each
  // channel's req async FIFO. packet_valid is the FIFO wr_en, gated with packet_ready.
  output logic [N_CH-1:0]                packet_valid,
  input  logic [N_CH-1:0]                packet_ready,
  output logic [N_CH-1:0][ROB_IDX_W-1:0] packet_rob_index, // = ROB_IDX (same all ch)
  output logic [N_CH-1:0][DADDR_W-1:0]   packet_daddr,     // per-packet DRAM coords
  output logic [N_CH-1:0][PKT_NUM_W-1:0] packet_pkt_num,   // 0 .. N-1
  output logic [N_CH-1:0]                packet_last_in_txn,
  output logic [N_CH-1:0][SLOT_W-1:0]    packet_slot,      // slot = slot_base + pkt_num
  output logic [N_CH-1:0][PKT_OFF_W-1:0] packet_offset,    // start byte in packet 0
  output logic [N_CH-1:0]                packet_narrow,    // not a full-width 64B line

  // DFT taps (passive, output only)
  output logic                   dft_busy,         // a request is being split
  output logic [CNT_W-1:0]       dft_pkt_cnt       // packets issued so far
);

  // captured request (held through the split; req_in_* is the NEXT request after)
  logic                 busy;
  logic [CNT_W-1:0]     pkt_cnt;
  logic [LINE_W-1:0]    req_line;      // 64B line of packet 0 (addr >> 6)
  logic [PKT_OFF_W-1:0] req_offset;    // start byte in that line (addr[5:0])
  logic [7:0]           req_len;
  logic [2:0]           req_size;
  logic [SLOT_W-1:0]    req_slot_base;
  logic [ROB_IDX_W-1:0] req_rob_idx;

  wire capture = req_in_valid & ROB_READY;

  // packet count (INCR): burst ends at align_down(addr,size)+(len+1)*2^size; one
  // packet per 64B line touched.
  localparam int SPAN_W = 16;
  logic [PKT_OFF_W-1:0] off_aligned;
  logic [SPAN_W-1:0]    span;
  logic [SPAN_W-1:0]    n_pkts;
  assign off_aligned = req_offset & ~PKT_OFF_W'((SPAN_W'(1) << req_size) - 1'b1);
  assign span        = SPAN_W'(off_aligned) + ((SPAN_W'(req_len) + 1'b1) << req_size);
  assign n_pkts      = (span + SPAN_W'(PKT_BYTES - 1)) >> PKT_OFF_W;
  // NPKTS is derived from the captured request regs, so only meaningful while busy;
  // it truncates to CNT_W bits if a request needs more than MAX_PKTS packets.
  assign SEG_NPKTS   = CNT_W'(n_pkts);

  // window: the next N_CH packets, each mapped to {channel, DRAM coords}
  localparam int NUM_W = CNT_W + $clog2(N_CH) + 1;
  logic [N_CH-1:0][NUM_W-1:0]   win_num;
  logic [N_CH-1:0][CH_W-1:0]    win_ch;
  logic [N_CH-1:0][DADDR_W-1:0] win_daddr;

  generate
    for (genvar j = 0; j < N_CH; j++) begin : g_win
      logic [LINE_W-1:0] line;
      assign win_num[j] = NUM_W'(pkt_cnt) + NUM_W'(j);
      assign line       = req_line + LINE_W'(win_num[j]);
      rmc_cif_addrmap #(.AXI_AW (AXI_AW)) u_map (
        .sys_addr ({line, {PKT_OFF_W{1'b0}}}),
        .addr_map (addr_map),
        .daddr    (win_daddr[j]),
        .ch       (win_ch[j]),
        .offset   ()
      );
    end
  endgenerate

  // narrow: partial first/last line or a sub-bus beat (write accumulator masks it)
  wire size_narrow = (32'(1) << req_size) < (AXI_DW / 8);
  wire tail_part   = (span[PKT_OFF_W-1:0] != '0);

  // issue: walk the window in order, stop at the first packet that cannot go
  logic [N_CH-1:0]  ch_used;
  logic             take;
  logic [NUM_W-1:0] n_take;
  logic             w_last;
  always_comb begin
    packet_valid       = '0;
    packet_daddr       = '0;
    packet_pkt_num     = '0;
    packet_last_in_txn = '0;
    packet_slot        = '0;
    packet_offset      = '0;
    packet_narrow      = '0;
    ch_used            = '0;
    n_take             = '0;
    take               = busy;
    w_last             = 1'b0;
    for (int j = 0; j < N_CH; j++) begin
      take = take & (SPAN_W'(win_num[j]) < n_pkts)
                  & ~ch_used[win_ch[j]] & packet_ready[win_ch[j]];
      if (take) begin
        w_last                        = (SPAN_W'(win_num[j]) + 1'b1 == n_pkts);
        ch_used[win_ch[j]]            = 1'b1;
        packet_valid[win_ch[j]]       = 1'b1;
        packet_daddr[win_ch[j]]       = win_daddr[j];
        packet_pkt_num[win_ch[j]]     = PKT_NUM_W'(win_num[j]);
        packet_last_in_txn[win_ch[j]] = w_last;
        packet_slot[win_ch[j]]        = req_slot_base + SLOT_W'(win_num[j]);
        packet_offset[win_ch[j]]      = (win_num[j] == '0) ? req_offset : '0;
        packet_narrow[win_ch[j]]      = size_narrow
                                      | ((win_num[j] == '0) & (req_offset != '0))
                                      | (w_last & tail_part);
        n_take = NUM_W'(j + 1);
      end
    end
  end

  assign packet_rob_index = {N_CH{req_rob_idx}};

  // last packet leaves this cycle: close the request at r_rob
  assign SEG_LAST = busy & (n_take != '0)
                  & (SPAN_W'(pkt_cnt) + SPAN_W'(n_take) == n_pkts);

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      busy          <= 1'b0;
      pkt_cnt       <= '0;
      req_line      <= '0;
      req_offset    <= '0;
      req_len       <= '0;
      req_size      <= '0;
      req_slot_base <= '0;
      req_rob_idx   <= '0;
    end else begin
      if (busy) begin
        pkt_cnt <= pkt_cnt + CNT_W'(n_take);
        if (SEG_LAST) busy <= 1'b0;
      end
      if (capture) begin            // ROB_READY low while busy, so never mid-split
        busy          <= 1'b1;
        pkt_cnt       <= '0;
        req_line      <= req_in_addr[AXI_AW-1:PKT_OFF_W];
        req_offset    <= req_in_addr[PKT_OFF_W-1:0];
        req_len       <= req_in_len;
        req_size      <= req_in_size;
        req_slot_base <= req_in_slot_base;
        req_rob_idx   <= ROB_IDX;
      end
    end
  end

  // DFT taps (wiring only)
  assign dft_busy    = busy;
  assign dft_pkt_cnt = pkt_cnt;

  // read meta from the captured copy (req_in_* has moved on by SEG_LAST)
  assign SEG_OFFSET   = req_offset;
  assign SEG_LEN      = req_len;
  assign SEG_SIZE     = req_size;
  assign SEG_HEAD_PTR = req_slot_base;

endmodule : rmc_cif_rseg
