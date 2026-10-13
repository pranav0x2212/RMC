// rmc_cif_r_burst_spltr_ch.sv
// READ burst segmenter - one channel, ring walker. Instanced once per DRAM channel by
// rmc_cif_r_burst_spltr. Walks the shared pre-ROB queue at its own pace (own rd_ptr), issuing
// only the 64B packets that land in its channel (ch_id), and marks the entry done for
// this channel when its share is drained.
//
// The queue entry {aligned_base, n_pkts, dbuf_base, rob_idx} is presented by the
// wrapper, muxed at this engine's rd_ptr. n_pkts and the start offset are computed at
// enqueue; this engine never touches the offset bits.
//
// STRIDE (contract map: ch = low CH_W bits of the line number):
//   base_ch = aligned_base[CH_W-1:0]
//   k0      = (ch_id - base_ch) mod N_CH           first packet index of this channel
//   k       = k0 + m*N_CH                           m = packets of this channel issued so far
//   line addr = (aligned_base + k) << 6 ; addrmap(line) -> daddr. dbuf = dbuf_base + k.
// Assumes the ch field is the low contiguous line field (contract/default map).

import rmc_cfg_pkg::*;

module rmc_cif_r_burst_spltr_ch #(
  parameter int ROB_DEPTH   = 32,         // ROB entries (rob_idx width)   [root knob]
  parameter int MAX_PKTS    = 64,         // max packets per request       [root knob]
  parameter int AXI_AW      = SYS_ADDR_W, // AXI address width             [root knob]
  parameter int N_SLOTS     = 32,         // RD-SRAM slots                 [root knob]
  parameter int QUEUE_DEPTH = ROB_DEPTH,  // pre-ROB queue entries; = ROB_DEPTH (pow2)  [knob]

  // derived (localparam: never passed in)
  localparam int ROB_IDX_W = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1, // rob_idx tag width
  localparam int CNT_W     = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1, // packet-count width
  localparam int SLOT_W    = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1, // RD-SRAM slot index width
  localparam int LINE_W    = AXI_AW - PKT_OFF_W,                     // 64B line number width
  localparam int QIDX_W    = (QUEUE_DEPTH > 1) ? $clog2(QUEUE_DEPTH) : 1, // queue ring index width
  localparam int QPTR_W    = QIDX_W + 1                              // queue pointer = index + wrap bit
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  input  logic [CH_W-1:0]        ch_id,           // the DRAM channel this engine serves
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,

  // queue read: wrapper presents the entry at this engine's rd_ptr
  output logic [QPTR_W-1:0]      rd_ptr,          // this engine's ring read pointer
  input  logic                   entry_valid,     // rd_ptr != alloc_ptr (entry occupied)
  input  logic [LINE_W-1:0]      e_aligned_base,  // entry's 64B base line
  input  logic [CNT_W-1:0]       e_n_pkts,        // entry's total packet count
  input  logic [SLOT_W-1:0]      e_dbuf_base,     // entry's RD-SRAM base slot
  input  logic [ROB_IDX_W-1:0]   e_rob_idx,       // entry's ROB index
  input  logic                   e_ch_done,       // this entry's done bit for ch_id
  output logic                   set_done,        // set that bit this cycle

  // packet stream out, this channel only
  output logic                   pkt_valid,       // packet offered this cycle
  input  logic                   pkt_ready,       // channel req FIFO can take it
  input  logic                   pkt_dbuf_adr_ready, // RD-SRAM has a slot for this packet
  output logic [ROB_IDX_W-1:0]   pkt_rob_idx,     // owning request's ROB index
  output logic [DADDR_W-1:0]     pkt_daddr,       // mapped DRAM coords for the line
  output logic [SLOT_W-1:0]      pkt_dbuf_adr,    // RD-SRAM slot = dbuf_base + k

  output logic [CNT_W-1:0]       dft_m            // DFT tap: packets issued for this entry
);

  logic [CNT_W-1:0] m;                 // packets of this channel issued for current entry

  // first packet index for this channel (contract map; N_CH is a power of 2)
  logic [CH_W-1:0]  base_ch;           // channel of line 0 (= aligned_base[CH_W-1:0])
  logic [CH_W-1:0]  k0;                // first line index that lands in ch_id
  logic [CNT_W-1:0] k;                 // current line index in the request (k0 + m*N_CH)
  assign base_ch = e_aligned_base[CH_W-1:0];
  assign k0      = (ch_id - base_ch) & CH_W'(N_CH - 1);
  assign k       = CNT_W'(k0) + (CNT_W'(m) << CH_W);     // k0 + m*N_CH

  // current line -> DRAM coords. map_ch/map_off are the map's other outputs; this engine
  // trusts the stride for routing and leaves the write offset to the queue, so they idle.
  logic [AXI_AW-1:0]    line_byte_addr; // byte address of line k ((aligned_base+k)<<6)
  logic [CH_W-1:0]      map_ch;         // map's channel output (unused; stride routes)
  logic [PKT_OFF_W-1:0] map_off;        // map's offset output (unused for reads)
  assign line_byte_addr = AXI_AW'(e_aligned_base + k) << PKT_OFF_W;

  rmc_cif_addrmap #(
    .AXI_AW  (AXI_AW),
    .DADDR_W (DADDR_W)
  ) u_map (
    .sys_addr (line_byte_addr),
    .addr_map (addr_map),
    .daddr    (pkt_daddr),
    .ch       (map_ch),
    .offset   (map_off)
  );

  wire owes_entry = entry_valid & ~e_ch_done;   // this engine still owes the current entry

  assign pkt_rob_idx  = e_rob_idx;
  assign pkt_dbuf_adr = e_dbuf_base + SLOT_W'(k);
  // dbuf-full is a per-channel issue stall (not a head/admit stall): if the RD-SRAM
  // has no slot for this packet, hold here - other channels and earlier packets drain.
  assign pkt_valid    = owes_entry & (k <  e_n_pkts) & pkt_dbuf_adr_ready;
  assign set_done     = owes_entry & (k >= e_n_pkts);     // my share drained (incl zero-pkt)

  wire skip = entry_valid & e_ch_done;                   // already done here -> just advance

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      rd_ptr <= '0;
      m      <= '0;
    end else if (set_done | skip) begin
      rd_ptr <= rd_ptr + 1'b1;                           // move to next entry
      m      <= '0;
    end else if (pkt_valid & pkt_ready) begin
      m      <= m + 1'b1;                                // next channel-ch_id packet
    end
  end

  assign dft_m = m;

endmodule : rmc_cif_r_burst_spltr_ch
