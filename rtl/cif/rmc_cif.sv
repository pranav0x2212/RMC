// rmc_cif.sv
// Client interface (CIF) top. Sits between the client AXI4 bus and the MC core.
// Owns: address map (system -> DRAM coords), burst segmentation (<=16-beat
// packets), reorder buffer (program order + completion), and the client-side
// ports of the WD/RD data SRAMs.
//
// Boundary to MC = two async FIFOs (external, one per direction). CIF presents a
// synchronous valid/ready stream on its side; CDC lives in the FIFO, not here.
//   out : request packets  {rob_index, op, daddr, pkt_num, last_in_txn, sram_slot}
//   in  : completions      {rob_index}  (a pulse; ROB counts them, retires at N)
// Read data is written by MC straight into RD_SRAM; CIF drains it to the R channel.
//
// STUB: top ports + sub-block instances + internal nets. Logic TODO per block.

import rmc_cfg_pkg::*;

module rmc_cif #(
  // client AXI4
  parameter int AXI_IDW   = 8,
  parameter int AXI_AW    = 48,        // map needs >=41 (row ends at bit 40)
  // AXI_DW is NOT a free param: it is rmc_cfg_pkg::AXI_DW = N_CH*DDR_CHANNEL_W
  // (matched BW, inflow==outflow). Imported below; do not override per-instance.
  // reorder buffer / packetization
  parameter int ROB_DEPTH = 32,        // physical ROB entries (16-32)   [root knob]
  parameter int MAX_PKTS  = 16,        // packets per request (<=16-beat) [root knob]
  // data buffers (client-side ports; SRAM may be instanced at MC top)
  parameter int N_WDB     = 32,        // WD_SRAM slots
  parameter int N_RDB     = 32,        // RD_SRAM slots

  // derived widths (localparam: not overridable, visible to port list)
  localparam int SRAM_W    = PKT_BYTES * 8,   // one 64B packet per line (= BL*DQ_W)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH) : 1,  // rob_index tag
  localparam int PKT_NUM_W = (MAX_PKTS > 1) ? $clog2(MAX_PKTS) : 1,
  localparam int CNT_W     = (MAX_PKTS > 1) ? $clog2(MAX_PKTS+1) : 1,  // packet-count width
  localparam int SPAN_W    = $clog2(MAX_REQ_BYTES + 1),                // bytes-spanned width
  localparam int SLOT_W    = (($clog2(N_WDB) > $clog2(N_RDB)) ?
                              $clog2(N_WDB) : $clog2(N_RDB)),
  localparam int DADDR_W   = RANK_W + BG_W + BANK_PER_BG_W + ROW_W + COL_W,
  // dft_obs width, from the same function the DFT module checks against
  localparam int DFT_OBS_W = rmc_cif_dft_pkg::dft_obs_w(AXI_IDW, AXI_AW, ROB_DEPTH,
                                                         MAX_PKTS, N_WDB, N_RDB)
)(
  input  logic                   aclk,
  input  logic                   aresetn,

  // client AXI4 target (subset; lock/cache/prot/qos/region TODO)
  // AW
  input  logic [AXI_IDW-1:0]     awid,
  input  logic [AXI_AW-1:0]      awaddr,
  input  logic [7:0]             awlen,
  input  logic [2:0]             awsize,
  input  logic [1:0]             awburst,
  input  logic                   awvalid,
  output logic                   awready,
  // W
  input  logic [AXI_DW-1:0]      wdata,
  input  logic [AXI_DW/8-1:0]    wstrb,
  input  logic                   wlast,
  input  logic                   wvalid,
  output logic                   wready,
  // B
  output logic [AXI_IDW-1:0]     bid,
  output logic [1:0]             bresp,
  output logic                   bvalid,
  input  logic                   bready,
  // AR
  input  logic [AXI_IDW-1:0]     arid,
  input  logic [AXI_AW-1:0]      araddr,
  input  logic [7:0]             arlen,
  input  logic [2:0]             arsize,
  input  logic [1:0]             arburst,
  input  logic                   arvalid,
  output logic                   arready,
  // R
  output logic [AXI_IDW-1:0]     rid,
  output logic [AXI_DW-1:0]      rdata,
  output logic [1:0]             rresp,
  output logic                   rlast,
  output logic                   rvalid,
  input  logic                   rready,

  // request stream to MC. async_ prefix = bus crosses into the req async FIFO
  // (MC clock domain); CIF drives it synchronously on this side.
  output logic                   async_mc_req_valid,
  input  logic                   async_mc_req_ready,
  output logic [ROB_IDX_W-1:0]   async_mc_req_rob_index,
  output logic                   async_mc_req_op,          // 0=read, 1=write
  output logic [DADDR_W-1:0]     async_mc_req_daddr,       // mapped DRAM coords
  output logic [PKT_NUM_W-1:0]   async_mc_req_pkt_num,
  output logic                   async_mc_req_last_in_txn, // auto-precharge hint
  output logic [SLOT_W-1:0]      async_mc_req_sram_slot,   // dbuf_addr(RD) | wd_slot(WR)

  // completion stream from MC (via compl async FIFO)
  input  logic                   async_mc_cmpl_valid,
  output logic                   async_mc_cmpl_ready,
  // tied high permanently: a completion only bumps an already-allocated entry's
  // counter, always absorbable in one cycle, so it must never back-pressure
  // (blocking a done-signal deadlocks the round trip). Only drop on soft-reset drain.
  input  logic [ROB_IDX_W-1:0]   async_mc_cmpl_rob_index,  // completion = pulse + this

  // WD_SRAM client-side write port (CIF writes W beats; MC reads)
  output logic                     wdb_we,
  output logic [$clog2(N_WDB)-1:0] wdb_addr,
  output logic [SRAM_W-1:0]        wdb_din,

  // RD_SRAM client-side read port (MC writes; CIF drains to R)
  output logic                     rdb_re,
  output logic [$clog2(N_RDB)-1:0] rdb_addr,
  input  logic [SRAM_W-1:0]        rdb_dout,

  // DFT observation bus (passive, output only). Field order: rmc_cif_dft.sv.
  output logic [DFT_OBS_W-1:0]     dft_obs
);

  // nets between sub-blocks (TODO: size/name as logic fills in)
  // daddr = DRAM coords for the req packet
  logic [DADDR_W-1:0]  aw_daddr, ar_daddr;  // TODO: into req-packet build

  // address map: system addr -> {rank,bg,bank,row,col}, runtime CSR-programmable.
  // Live map = per-dest-bit source select; reset value = STAGE-24 default layout.
  // TODO: drive addr_map from a CSR reg (reset to default_addr_map()); constant for now.
  logic [CH_W-1:0]      aw_ch, ar_ch;        // channel select (route to core)
  logic [PKT_OFF_W-1:0] aw_off, ar_off;      // byte-in-packet (write mask)
  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map;
  assign addr_map = default_addr_map();

  rmc_cif_addrmap #(
    .AXI_AW  (AXI_AW),
    .DADDR_W (DADDR_W)
  ) u_addrmap_aw (
    .sys_addr (awaddr),
    .addr_map (addr_map),
    .daddr    (aw_daddr),
    .ch       (aw_ch),
    .offset   (aw_off)
  );

  rmc_cif_addrmap #(
    .AXI_AW  (AXI_AW),
    .DADDR_W (DADDR_W)
  ) u_addrmap_ar (
    .sys_addr (araddr),
    .addr_map (addr_map),
    .daddr    (ar_daddr),
    .ch       (ar_ch),
    .offset   (ar_off)
  );

  // request validator: tag each AXI request address ok(1)/err(0) before the ROB.
  // Addr-only; an err request still rides through (err bit) to retire with SLVERR.
  // TODO: carry aw_ok/ar_ok into ROB alloc as the per-request err bit.
  logic aw_ok, ar_ok;
  rmc_cif_reqvalidator #(.AXI_AW(AXI_AW)) u_val_aw (
    .req_addr (awaddr), .req_len (awlen), .req_size (awsize), .req_ok (aw_ok));
  rmc_cif_reqvalidator #(.AXI_AW(AXI_AW)) u_val_ar (
    .req_addr (araddr), .req_len (arlen), .req_size (arsize), .req_ok (ar_ok));

  // ====================================================================
  // READ PATH LEFT TO DO (rseg/rrob control is wired; these remain):
  //   - req_dbuf_base: the RD-SRAM slot allocator is not built; u_rseg's
  //     req_dbuf_base is stubbed. Allocate a contiguous RD-SRAM base per
  //     admitted read and feed it here.
  //   - pkt_* -> MC: u_rseg emits N_CH per-channel packet streams, but this
  //     block has a single async_mc_req_* port. Add per-channel req FIFOs +
  //     an N_CH->1 arbiter (or widen the MC req interface to per-channel).
  //   - dbuf addressing: global dbuf_base+k (per-channel SRAM sparse) vs a
  //     packed per-channel index - open; affects RD-SRAM size + resp readback.
  //   - scattered-map fallback: rseg assumes the contract map (ch = low CH_W
  //     line bits); an arbitrary CSR map needs the scan path (not built).
  //   - RESP_* from u_rrob -> the R response channel (drain RD-SRAM, trim).
  // ====================================================================

  // read segmenter <-> read ROB handoff nets
  logic                   ar_rseg_ready;    // rseg queue has room
  logic                   ar_rrob_rdy;      // rrob has a free slot (~rob_full)
  logic                   ar_admit;         // a read request is admitted this cycle
  logic [ROB_IDX_W-1:0]   ar_rob_idx;       // slot rrob hands out, rides with the request
  logic                   ar_seg_last;      // a read request finished splitting
  logic [ROB_IDX_W-1:0]   ar_seg_rob_idx;   // which entry it was
  logic [CNT_W-1:0]       ar_seg_npkts;
  logic [PKT_OFF_W-1:0]   ar_seg_offset;
  logic [7:0]             ar_seg_len;
  logic [2:0]             ar_seg_size;
  logic [SLOT_W-1:0]      ar_seg_head_ptr;

  // Downstream of rseg is unfinished upstream. These nets keep the pins visible to DFT
  // and float exactly as the previously unconnected pins did ("undriven"); the TB forces
  // them in T10. ar_pkt_dbuf_rdy keeps upstream's constant tie.
  localparam int RDB_AW_G = (N_RDB > 1) ? $clog2(N_RDB) : 1;   // guarded: never 0 bits
  localparam int WDB_AW_G = (N_WDB > 1) ? $clog2(N_WDB) : 1;
  localparam int OCC_W    = $clog2(ROB_DEPTH + 1);
  localparam int QPTR_W   = ((ROB_DEPTH > 1) ? $clog2(ROB_DEPTH) : 1) + 1;  // rseg queue ptr
  logic [RDB_AW_G-1:0]            ar_dbuf_base;      // undriven (RD-SRAM allocator TODO)
  logic [N_CH-1:0]                ar_pkt_ready;      // undriven (per-channel req FIFO TODO)
  logic [N_CH-1:0]                ar_pkt_dbuf_rdy;   // tied high (allocator TODO)
  assign ar_pkt_dbuf_rdy = {N_CH{1'b1}};
  logic [N_CH-1:0]                ar_pkt_valid;
  logic [N_CH-1:0][ROB_IDX_W-1:0] ar_pkt_rob_idx;
  logic [N_CH-1:0][DADDR_W-1:0]   ar_pkt_daddr;
  logic [N_CH-1:0][RDB_AW_G-1:0]  ar_pkt_dbuf_adr;
  // rseg / rrob DFT taps
  logic [QPTR_W-1:0]              rs_dft_alloc_ptr, rs_dft_pop_ptr;
  logic [ROB_DEPTH*N_CH-1:0]      rs_dft_ch_done;
  logic [N_CH*QPTR_W-1:0]         rs_dft_eng_rd_ptr;
  logic [N_CH*CNT_W-1:0]          rs_dft_eng_m;
  logic [ROB_IDX_W-1:0]           r_dft_alloc_ptr, r_dft_retire_ptr;
  logic [OCC_W-1:0]               r_dft_occ;
  logic                           r_dft_admit_fire, r_dft_retire_fire;
  logic [ROB_DEPTH-1:0]           r_dft_inflow_complt, r_dft_ent_err, r_dft_ent_derr;
  logic [ROB_DEPTH*CNT_W-1:0]     r_dft_outflow_cnt, r_dft_n_pkts;
  logic [ROB_DEPTH*RDB_AW_G-1:0]  r_dft_slot_head;
  logic                           r_resp_valid;
  logic [AXI_IDW-1:0]             r_resp_id;
  logic [1:0]                     r_resp_resp;
  logic [RDB_AW_G-1:0]            r_resp_head;
  logic [PKT_OFF_W-1:0]           r_resp_offset;
  logic [7:0]                     r_resp_len;
  logic [2:0]                     r_resp_size;

  // admit both together: the ROB slot and the segmenter queue entry are allocated on the
  // same cycle, so gate each on the other's readiness.
  assign ar_admit = arvalid & ar_rseg_ready & ar_rrob_rdy;

  // admit-time packet count for the ROB (known now; ships early so completions that beat
  // the split still count). Same geometry rseg uses: size-aligned start + burst, ceil/64.
  wire [PKT_OFF_W-1:0] ar_off_aligned = araddr[PKT_OFF_W-1:0]
                                      & ~PKT_OFF_W'((SPAN_W'(1) << arsize) - 1'b1);
  wire [SPAN_W-1:0]    ar_span   = SPAN_W'(ar_off_aligned) + ((SPAN_W'(arlen) + 1'b1) << arsize);
  wire [CNT_W-1:0]     ar_npkts  = CNT_W'((ar_span + SPAN_W'((1 << PKT_OFF_W) - 1)) >> PKT_OFF_W);

  // read segmenter. TODO: req_dbuf_base from the RD-SRAM slot allocator (not built) and
  // pkt_* -> the per-channel req async FIFOs (single async_mc_req_* port today; needs an
  // N_CH->1 arbiter or a per-channel FIFO interface).
  rmc_cif_r_burst_spltr #(
    .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_AW (AXI_AW), .N_SLOTS (N_RDB)
  ) u_rseg (
    .aclk         (aclk),
    .aresetn      (aresetn),
    .addr_map     (addr_map),
    .start_req    (ar_admit),
    .req_addr     (araddr),
    .req_len      (arlen),
    .req_size     (arsize),
    .req_dbuf_base(ar_dbuf_base),         // TODO: RD-SRAM slot allocator
    .req_rob_idx  (ar_rob_idx),
    .ready        (ar_rseg_ready),
    .pkt_valid    (ar_pkt_valid),         // TODO: per-channel req FIFO
    .pkt_ready    (ar_pkt_ready),
    .pkt_dbuf_adr_ready (ar_pkt_dbuf_rdy), // TODO: from RD-SRAM allocator (slot availability)
    .pkt_rob_idx  (ar_pkt_rob_idx),
    .pkt_daddr    (ar_pkt_daddr),
    .pkt_dbuf_adr (ar_pkt_dbuf_adr),
    .seg_last     (ar_seg_last),
    .seg_rob_idx  (ar_seg_rob_idx),
    .seg_npkts    (ar_seg_npkts),
    .seg_offset   (ar_seg_offset),
    .seg_len      (ar_seg_len),
    .seg_size     (ar_seg_size),
    .seg_head_ptr (ar_seg_head_ptr),
    .dft_alloc_ptr  (rs_dft_alloc_ptr),
    .dft_pop_ptr    (rs_dft_pop_ptr),
    .dft_ch_done    (rs_dft_ch_done),
    .dft_eng_rd_ptr (rs_dft_eng_rd_ptr),
    .dft_eng_m      (rs_dft_eng_m)
  );

  // write segmenter - still a stub. TODO: mirror rseg (ring + per-channel engines) with
  // free-list slots + chain; wire to u_wrob and the W/AW path.
  logic [N_CH-1:0]                w_pkt_ready;      // undriven (wseg is a stub)
  logic                           w_seg_last;
  logic [CNT_W-1:0]               w_seg_npkts;
  logic [WDB_AW_G-1:0]            w_seg_head;
  logic                           w_free_slot_take;
  logic [N_CH-1:0]                w_pkt_valid;
  logic [N_CH-1:0][ROB_IDX_W-1:0] w_pkt_rob_index;
  logic [N_CH-1:0][DADDR_W-1:0]   w_pkt_daddr;
  logic [N_CH-1:0][PKT_NUM_W-1:0] w_pkt_num;
  logic [N_CH-1:0]                w_pkt_last;
  logic [N_CH-1:0][WDB_AW_G-1:0]  w_pkt_slot;
  logic [N_CH-1:0][PKT_OFF_W-1:0] w_pkt_offset;
  logic [N_CH-1:0]                w_pkt_narrow;
  rmc_cif_wseg #(
    .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_AW (AXI_AW), .N_SLOTS (N_WDB)
  ) u_wseg (
    .aclk (aclk), .aresetn (aresetn),
    .SEG_LAST         (w_seg_last),
    .SEG_NPKTS        (w_seg_npkts),
    .SEG_HEAD_PTR     (w_seg_head),
    .free_slot_take   (w_free_slot_take),
    .packet_valid     (w_pkt_valid),
    .packet_ready     (w_pkt_ready),
    .packet_rob_index (w_pkt_rob_index),
    .packet_daddr     (w_pkt_daddr),
    .packet_pkt_num   (w_pkt_num),
    .packet_last_in_txn (w_pkt_last),
    .packet_slot      (w_pkt_slot),
    .packet_offset    (w_pkt_offset),
    .packet_narrow    (w_pkt_narrow)
  );

  // completion never back-pressures: tie ready high. A completion only bumps a
  // counter on an already-allocated entry, always absorbable in one cycle.
  // TODO: route cmpl to r_rob/w_rob by a direction bit in the tag; drop only on
  // a soft-reset drain.
  assign async_mc_cmpl_ready = 1'b1;

  // PLAN (not built here yet): a pre-ROB AW_req buffer sits post request_validator
  // and before the ROB, one per direction. It holds the phys addr (which the ROB
  // does NOT store) and feeds the burst splitter directly, and it decouples up to
  // N_OUTSTANDING requests. The ROB only ADMITS: it takes {id,len,size,validated}
  // on AW_REQUEST/A_ID_VALID and backpressures via A_ID_READY. The buffer is not
  // built yet - for now admit is wired straight from the validator + AXI channel.

  // read ROB. admit direct from AR channel + validator (AR_req buffer not built yet).
  // A_ID_VALID is gated by rseg readiness so the ROB slot and the segmenter queue entry
  // allocate together; arready reflects both. RESP_* -> R response path is still TODO.
  assign arready = ar_rseg_ready & ar_rrob_rdy;
  rmc_cif_rrob #(
    .ROB_DEPTH (ROB_DEPTH),
    .MAX_PKTS  (MAX_PKTS),
    .AXI_IDW   (AXI_IDW),
    .N_SLOTS   (N_RDB)
  ) u_rrob (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .cmpl_valid     (async_mc_cmpl_valid),
    .cmpl_rob_index (async_mc_cmpl_rob_index),
    .AW_REQUEST     ({arid, ar_ok}),
    .ADMIT_NPKTS    (ar_npkts),
    .A_ID_VALID     (arvalid & ar_rseg_ready),
    .A_ID_READY     (ar_rrob_rdy),
    .ROB_IDX        (ar_rob_idx),
    .SEG_LAST       (ar_seg_last),
    .SEG_ROB_IDX    (ar_seg_rob_idx),
    .SEG_OFFSET     (ar_seg_offset),
    .SEG_LEN        (ar_seg_len),
    .SEG_SIZE       (ar_seg_size),
    .SEG_HEAD_PTR   (ar_seg_head_ptr),
    // RESP_READY and cmpl_err stay unconnected (R path TODO); RESP_* only feed DFT.
    .RESP_VALID     (r_resp_valid),
    .RESP_ID        (r_resp_id),
    .RESP_RESP      (r_resp_resp),
    .RESP_HEAD_PTR  (r_resp_head),
    .RESP_OFFSET    (r_resp_offset),
    .RESP_LEN       (r_resp_len),
    .RESP_SIZE      (r_resp_size),
    .dft_alloc_ptr    (r_dft_alloc_ptr),
    .dft_retire_ptr   (r_dft_retire_ptr),
    .dft_occ          (r_dft_occ),
    .dft_admit_fire   (r_dft_admit_fire),
    .dft_retire_fire  (r_dft_retire_fire),
    .dft_inflow_complt(r_dft_inflow_complt),
    .dft_ent_err      (r_dft_ent_err),
    .dft_ent_derr     (r_dft_ent_derr),
    .dft_outflow_cnt  (r_dft_outflow_cnt),
    .dft_n_pkts       (r_dft_n_pkts),
    .dft_slot_head    (r_dft_slot_head)
  );

  // write ROB. admit direct from AW channel + validator (AW_req buffer not built yet).
  logic [ROB_IDX_W-1:0]           w_rob_idx;
  logic                           w_resp_valid;
  logic [AXI_IDW-1:0]             w_resp_id;
  logic [1:0]                     w_resp_resp;
  logic [WDB_AW_G-1:0]            w_resp_head;
  logic [ROB_IDX_W-1:0]           w_dft_alloc_ptr, w_dft_retire_ptr;
  logic [OCC_W-1:0]               w_dft_occ;
  logic                           w_dft_admit_fire, w_dft_retire_fire, w_dft_pending_split;
  logic [ROB_DEPTH-1:0]           w_dft_inflow_complt, w_dft_ent_err, w_dft_ent_derr;
  logic [ROB_DEPTH*CNT_W-1:0]     w_dft_outflow_cnt, w_dft_n_pkts;
  logic [ROB_DEPTH*WDB_AW_G-1:0]  w_dft_slot_head;
  rmc_cif_wrob #(
    .ROB_DEPTH (ROB_DEPTH),
    .MAX_PKTS  (MAX_PKTS),
    .AXI_IDW   (AXI_IDW),
    .N_SLOTS   (N_WDB)
  ) u_wrob (
    .aclk           (aclk),
    .aresetn        (aresetn),
    .cmpl_valid     (async_mc_cmpl_valid),
    .cmpl_rob_index (async_mc_cmpl_rob_index),
    .AW_REQUEST     ({awid, awlen, awsize, aw_ok}),
    .A_ID_VALID     (awvalid),
    .A_ID_READY     (awready),
    // TODO: SEG_* from u_wseg; RESP_READY / cmpl_err / B path
    .ROB_IDX        (w_rob_idx),
    .RESP_VALID     (w_resp_valid),
    .RESP_ID        (w_resp_id),
    .RESP_RESP      (w_resp_resp),
    .RESP_HEAD_PTR  (w_resp_head),
    .dft_alloc_ptr    (w_dft_alloc_ptr),
    .dft_retire_ptr   (w_dft_retire_ptr),
    .dft_occ          (w_dft_occ),
    .dft_admit_fire   (w_dft_admit_fire),
    .dft_retire_fire  (w_dft_retire_fire),
    .dft_pending_split(w_dft_pending_split),
    .dft_inflow_complt(w_dft_inflow_complt),
    .dft_ent_err      (w_dft_ent_err),
    .dft_ent_derr     (w_dft_ent_derr),
    .dft_outflow_cnt  (w_dft_outflow_cnt),
    .dft_n_pkts       (w_dft_n_pkts),
    .dft_slot_head    (w_dft_slot_head)
  );

  // request builder / response stubs (TODO)
  assign async_mc_req_valid       = 1'b0;
  assign async_mc_req_rob_index   = '0;
  assign async_mc_req_op          = 1'b0;
  assign async_mc_req_daddr       = '0;
  assign async_mc_req_pkt_num     = '0;
  assign async_mc_req_last_in_txn = 1'b0;
  assign async_mc_req_sram_slot   = '0;

  // awready / arready are driven by the ROBs' A_ID_READY (admit backpressure).
  assign wready  = 1'b0;
  assign bid     = '0;
  assign bresp   = 2'b00;
  assign bvalid  = 1'b0;
  assign rid     = '0;
  assign rdata   = '0;
  assign rresp   = 2'b00;
  assign rlast   = 1'b0;
  assign rvalid  = 1'b0;

  assign wdb_we   = 1'b0;
  assign wdb_addr = '0;
  assign wdb_din  = '0;
  assign rdb_re   = 1'b0;
  assign rdb_addr = '0;

  // DFT observation (passive; wiring only)
  rmc_cif_dft #(
    .AXI_IDW (AXI_IDW), .AXI_AW (AXI_AW), .ROB_DEPTH (ROB_DEPTH),
    .MAX_PKTS (MAX_PKTS), .N_WDB (N_WDB), .N_RDB (N_RDB)
  ) u_dft (
    .awvalid (awvalid), .awready (awready), .awaddr (awaddr),
    .awlen (awlen), .awsize (awsize), .awid (awid),
    .arvalid (arvalid), .arready (arready), .araddr (araddr),
    .arlen (arlen), .arsize (arsize), .arid (arid),
    .wvalid (wvalid), .wready (wready), .wstrb (wstrb), .wlast (wlast),
    .bvalid (bvalid), .bready (bready), .rvalid (rvalid), .rready (rready),
    .aw_daddr (aw_daddr), .ar_daddr (ar_daddr),
    .aw_ch (aw_ch), .ar_ch (ar_ch), .aw_off (aw_off), .ar_off (ar_off),
    .addr_map (addr_map),
    .aw_ok (aw_ok), .ar_ok (ar_ok),
    // read ROB
    .rd_rob_index (ar_rob_idx), .rd_rob_ready (ar_rrob_rdy),
    .rd_alloc_vld (r_dft_admit_fire),
    .rd_head (r_dft_retire_ptr), .rd_tail (r_dft_alloc_ptr), .rd_occ (r_dft_occ),
    .rd_retire_vld (r_resp_valid), .rd_retire_fire (r_dft_retire_fire),
    .rd_inflow_complt (r_dft_inflow_complt), .rd_outflow_cnt (r_dft_outflow_cnt),
    .rd_n_pkts (r_dft_n_pkts), .rd_ent_err (r_dft_ent_err), .rd_ent_derr (r_dft_ent_derr),
    .rd_slot_head (r_dft_slot_head),
    .rd_resp_id (r_resp_id), .rd_resp_resp (r_resp_resp), .rd_resp_head (r_resp_head),
    .rd_resp_offset (r_resp_offset), .rd_resp_len (r_resp_len), .rd_resp_size (r_resp_size),
    // write ROB
    .wr_rob_index (w_rob_idx), .wr_idx_vld (w_dft_pending_split),
    .wr_alloc_vld (w_dft_admit_fire),
    .wr_head (w_dft_retire_ptr), .wr_tail (w_dft_alloc_ptr), .wr_occ (w_dft_occ),
    .wr_retire_vld (w_resp_valid), .wr_retire_fire (w_dft_retire_fire),
    .wr_inflow_complt (w_dft_inflow_complt), .wr_outflow_cnt (w_dft_outflow_cnt),
    .wr_n_pkts (w_dft_n_pkts), .wr_ent_err (w_dft_ent_err), .wr_ent_derr (w_dft_ent_derr),
    .wr_slot_head (w_dft_slot_head),
    .wr_resp_id (w_resp_id), .wr_resp_resp (w_resp_resp), .wr_resp_head (w_resp_head),
    // read segmenter: queue + per-channel walkers
    .rs_ready (ar_rseg_ready), .rs_q_alloc (rs_dft_alloc_ptr), .rs_q_pop (rs_dft_pop_ptr),
    .rs_q_ch_done (rs_dft_ch_done), .rs_eng_rd_ptr (rs_dft_eng_rd_ptr), .rs_eng_m (rs_dft_eng_m),
    .rs_req_dbuf_base (ar_dbuf_base),
    .rs_seg_last (ar_seg_last), .rs_seg_rob_idx (ar_seg_rob_idx), .rs_seg_npkts (ar_seg_npkts),
    .rs_seg_offset (ar_seg_offset), .rs_seg_len (ar_seg_len), .rs_seg_size (ar_seg_size),
    .rs_seg_head (ar_seg_head_ptr[RDB_AW_G-1:0]),
    .rs_pkt_valid (ar_pkt_valid), .rs_pkt_ready (ar_pkt_ready), .rs_pkt_dbuf_rdy (ar_pkt_dbuf_rdy),
    .rs_pkt_rob_idx (ar_pkt_rob_idx), .rs_pkt_daddr (ar_pkt_daddr), .rs_pkt_dbuf_adr (ar_pkt_dbuf_adr),
    // write segmenter (stub, tied 0)
    .ws_seg_last (w_seg_last), .ws_seg_npkts (w_seg_npkts), .ws_seg_head (w_seg_head),
    .ws_free_slot_take (w_free_slot_take),
    .ws_pkt_valid (w_pkt_valid), .ws_pkt_ready (w_pkt_ready),
    .ws_pkt_rob_index (w_pkt_rob_index), .ws_pkt_daddr (w_pkt_daddr),
    .ws_pkt_num (w_pkt_num), .ws_pkt_last (w_pkt_last), .ws_pkt_slot (w_pkt_slot),
    .ws_pkt_offset (w_pkt_offset), .ws_pkt_narrow (w_pkt_narrow),
    // async FIFO boundary
    .async_mc_req_valid (async_mc_req_valid), .async_mc_req_ready (async_mc_req_ready),
    .async_mc_req_rob_index (async_mc_req_rob_index), .async_mc_req_op (async_mc_req_op),
    .async_mc_req_daddr (async_mc_req_daddr), .async_mc_req_pkt_num (async_mc_req_pkt_num),
    .async_mc_req_last_in_txn (async_mc_req_last_in_txn),
    .async_mc_req_sram_slot (async_mc_req_sram_slot),
    .async_mc_cmpl_valid (async_mc_cmpl_valid), .async_mc_cmpl_ready (async_mc_cmpl_ready),
    .async_mc_cmpl_rob_index (async_mc_cmpl_rob_index),
    // data SRAM
    .wdb_we (wdb_we), .wdb_addr (wdb_addr), .wdb_din (wdb_din),
    .rdb_re (rdb_re), .rdb_addr (rdb_addr), .rdb_dout (rdb_dout),
    .aclk (aclk), .aresetn (aresetn),
    .dft_obs (dft_obs)
  );

endmodule : rmc_cif
