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

  // guarded widths for DFT / slot nets (a parameter of 1 never gives a 0-bit bus)
  localparam int RDB_AW_G  = (N_RDB > 1) ? $clog2(N_RDB) : 1;
  localparam int WDB_AW_G  = (N_WDB > 1) ? $clog2(N_WDB) : 1;
  localparam int CNT_W     = (MAX_PKTS > 1) ? $clog2(MAX_PKTS+1) : 1;
  localparam int OCC_W     = $clog2(ROB_DEPTH + 1);

  // Read segmenter nets. "undriven" = ROB<->seg handoff still TODO upstream (floats as
  // the unconnected pins did); T10 forces them.
  logic                          r_seg_req_valid;          // undriven
  logic [AXI_AW-1:0]             r_seg_req_addr;           // undriven
  logic [7:0]                    r_seg_req_len;            // undriven
  logic [2:0]                    r_seg_req_size;           // undriven
  logic [RDB_AW_G-1:0]           r_seg_slot_base;          // undriven
  logic                          r_seg_rob_ready;          // undriven
  logic [N_CH-1:0]               r_pkt_ready;              // undriven
  // rseg outputs: observation only, not yet consumed by the ROB or request path
  logic                          r_seg_last;
  logic [CNT_W-1:0]              r_seg_npkts;
  logic [PKT_OFF_W-1:0]          r_seg_offset;
  logic [7:0]                    r_seg_len;
  logic [2:0]                    r_seg_size;
  logic [RDB_AW_G-1:0]           r_seg_head;
  logic [N_CH-1:0]               r_pkt_valid;
  logic [N_CH-1:0][ROB_IDX_W-1:0] r_pkt_rob_index;
  logic [N_CH-1:0][DADDR_W-1:0]  r_pkt_daddr;
  logic [N_CH-1:0][PKT_NUM_W-1:0] r_pkt_num;
  logic [N_CH-1:0]               r_pkt_last;
  logic [N_CH-1:0][RDB_AW_G-1:0] r_pkt_slot;
  logic [N_CH-1:0][PKT_OFF_W-1:0] r_pkt_offset;
  logic [N_CH-1:0]               r_pkt_narrow;
  logic                          r_seg_busy;           // dft: request being split
  logic [CNT_W-1:0]              r_seg_pkt_cnt;        // dft: packets issued so far

  // Write segmenter nets (wseg is a stub: its outputs are tied 0)
  logic [N_CH-1:0]               w_pkt_ready;              // undriven
  logic                          w_seg_last;
  logic [CNT_W-1:0]              w_seg_npkts;
  logic [WDB_AW_G-1:0]           w_seg_head;
  logic                          w_free_slot_take;
  logic [N_CH-1:0]               w_pkt_valid;
  logic [N_CH-1:0][ROB_IDX_W-1:0] w_pkt_rob_index;
  logic [N_CH-1:0][DADDR_W-1:0]  w_pkt_daddr;
  logic [N_CH-1:0][PKT_NUM_W-1:0] w_pkt_num;
  logic [N_CH-1:0]               w_pkt_last;
  logic [N_CH-1:0][WDB_AW_G-1:0] w_pkt_slot;
  logic [N_CH-1:0][PKT_OFF_W-1:0] w_pkt_offset;
  logic [N_CH-1:0]               w_pkt_narrow;

  // ROB outputs and DFT taps; r_* = read ROB, w_* = write ROB (kept separate)
  logic [ROB_IDX_W-1:0]          r_rob_idx, w_rob_idx;
  logic                          r_resp_valid, w_resp_valid;
  logic [AXI_IDW-1:0]            r_resp_id, w_resp_id;
  logic [1:0]                    r_resp_resp, w_resp_resp;
  logic [RDB_AW_G-1:0]           r_resp_head;
  logic [PKT_OFF_W-1:0]          r_resp_offset;
  logic [7:0]                    r_resp_len;
  logic [2:0]                    r_resp_size;
  logic [WDB_AW_G-1:0]           w_resp_head;
  // dft_* come from output-only taps inside rrob/wrob; per-slot buses are slot-major
  logic [ROB_IDX_W-1:0]          r_dft_alloc_ptr, w_dft_alloc_ptr;
  logic [ROB_IDX_W-1:0]          r_dft_retire_ptr, w_dft_retire_ptr;
  logic [OCC_W-1:0]              r_dft_occ, w_dft_occ;
  logic                          r_dft_admit_fire, w_dft_admit_fire;
  logic                          r_dft_retire_fire, w_dft_retire_fire;
  logic                          r_dft_pending_split, w_dft_pending_split;
  logic [ROB_DEPTH-1:0]          r_dft_inflow_complt, w_dft_inflow_complt;
  logic [ROB_DEPTH-1:0]          r_dft_ent_err, w_dft_ent_err;
  logic [ROB_DEPTH-1:0]          r_dft_ent_derr, w_dft_ent_derr;
  logic [ROB_DEPTH*CNT_W-1:0]    r_dft_outflow_cnt, w_dft_outflow_cnt;
  logic [ROB_DEPTH*CNT_W-1:0]    r_dft_n_pkts, w_dft_n_pkts;
  logic [ROB_DEPTH*RDB_AW_G-1:0] r_dft_slot_head;
  logic [ROB_DEPTH*WDB_AW_G-1:0] w_dft_slot_head;

  // segmentation: read + write burst splitters, one per direction. Handoff to the
  // ROBs / request FIFOs is TODO; outputs go to observation nets only.
  rmc_cif_rseg #(
    .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_AW (AXI_AW), .N_SLOTS (N_RDB)
  ) u_rseg (
    .aclk (aclk), .aresetn (aresetn),
    .req_in_valid     (r_seg_req_valid),
    .req_in_addr      (r_seg_req_addr),
    .req_in_len       (r_seg_req_len),
    .req_in_size      (r_seg_req_size),
    .req_in_slot_base (r_seg_slot_base),
    .addr_map         (addr_map),
    .ROB_IDX          (r_rob_idx),
    .ROB_READY        (r_seg_rob_ready),
    .SEG_LAST         (r_seg_last),
    .SEG_NPKTS        (r_seg_npkts),
    .SEG_OFFSET       (r_seg_offset),
    .SEG_LEN          (r_seg_len),
    .SEG_SIZE         (r_seg_size),
    .SEG_HEAD_PTR     (r_seg_head),
    .packet_valid     (r_pkt_valid),
    .packet_ready     (r_pkt_ready),
    .packet_rob_index (r_pkt_rob_index),
    .packet_daddr     (r_pkt_daddr),
    .packet_pkt_num   (r_pkt_num),
    .packet_last_in_txn (r_pkt_last),
    .packet_slot      (r_pkt_slot),
    .packet_offset    (r_pkt_offset),
    .packet_narrow    (r_pkt_narrow),
    .dft_busy         (r_seg_busy),
    .dft_pkt_cnt      (r_seg_pkt_cnt)
  );
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
  // SEG_*, RESP_READY and cmpl_err are left unconnected (TODO); RESP_* only feed DFT.
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
    .AW_REQUEST     ({arid, arlen, arsize, ar_ok}),
    .A_ID_VALID     (arvalid),
    .A_ID_READY     (arready),
    // TODO: SEG_* from u_rseg; RESP_READY / R response path
    .ROB_IDX        (r_rob_idx),
    .RESP_VALID     (r_resp_valid),
    .RESP_ID        (r_resp_id),
    .RESP_RESP      (r_resp_resp),
    .RESP_HEAD_PTR  (r_resp_head),
    .RESP_OFFSET    (r_resp_offset),
    .RESP_LEN       (r_resp_len),
    .RESP_SIZE      (r_resp_size),
    .dft_alloc_ptr     (r_dft_alloc_ptr),
    .dft_retire_ptr    (r_dft_retire_ptr),
    .dft_occ           (r_dft_occ),
    .dft_admit_fire    (r_dft_admit_fire),
    .dft_retire_fire   (r_dft_retire_fire),
    .dft_pending_split (r_dft_pending_split),
    .dft_inflow_complt (r_dft_inflow_complt),
    .dft_ent_err       (r_dft_ent_err),
    .dft_ent_derr      (r_dft_ent_derr),
    .dft_outflow_cnt   (r_dft_outflow_cnt),
    .dft_n_pkts        (r_dft_n_pkts),
    .dft_slot_head     (r_dft_slot_head)
  );

  // write ROB. admit direct from AW channel + validator (AW_req buffer not built yet).
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
    // TODO: SEG_* from u_wseg; RESP_READY / B response path
    .ROB_IDX        (w_rob_idx),
    .RESP_VALID     (w_resp_valid),
    .RESP_ID        (w_resp_id),
    .RESP_RESP      (w_resp_resp),
    .RESP_HEAD_PTR  (w_resp_head),
    .dft_alloc_ptr     (w_dft_alloc_ptr),
    .dft_retire_ptr    (w_dft_retire_ptr),
    .dft_occ           (w_dft_occ),
    .dft_admit_fire    (w_dft_admit_fire),
    .dft_retire_fire   (w_dft_retire_fire),
    .dft_pending_split (w_dft_pending_split),
    .dft_inflow_complt (w_dft_inflow_complt),
    .dft_ent_err       (w_dft_ent_err),
    .dft_ent_derr      (w_dft_ent_derr),
    .dft_outflow_cnt   (w_dft_outflow_cnt),
    .dft_n_pkts        (w_dft_n_pkts),
    .dft_slot_head     (w_dft_slot_head)
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
    .rd_rob_index (r_rob_idx), .rd_idx_vld (r_dft_pending_split),
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
    // read segmenter (implemented)
    .rs_busy (r_seg_busy), .rs_pkt_cnt (r_seg_pkt_cnt),
    .rs_seg_last (r_seg_last), .rs_seg_npkts (r_seg_npkts),
    .rs_seg_offset (r_seg_offset), .rs_seg_len (r_seg_len), .rs_seg_size (r_seg_size),
    .rs_seg_head (r_seg_head),
    .rs_pkt_valid (r_pkt_valid), .rs_pkt_ready (r_pkt_ready),
    .rs_pkt_rob_index (r_pkt_rob_index), .rs_pkt_daddr (r_pkt_daddr),
    .rs_pkt_num (r_pkt_num), .rs_pkt_last (r_pkt_last), .rs_pkt_slot (r_pkt_slot),
    .rs_pkt_offset (r_pkt_offset), .rs_pkt_narrow (r_pkt_narrow),
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
