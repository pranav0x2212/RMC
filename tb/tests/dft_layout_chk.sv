`timescale 1ns/1ps

// Layout checker: drives every dft input with distinct random values and requires
// dft_obs to equal their plain concatenation (no offsets), so a swapped, resized,
// dropped or duplicated field mismatches. iters proves the loop ran (TB requires 20).
module dft_layout_chk import rmc_cfg_pkg::*; #(
  parameter int AXI_IDW   = 8,
  parameter int AXI_AW    = 48,
  parameter int ROB_DEPTH = 32,
  parameter int MAX_PKTS  = 16,
  parameter int N_WDB     = 32,
  parameter int N_RDB     = 32
)(
  output int errors,
  output int bus_w,
  output int cat_w,
  output int iters
);
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1;
  localparam int PTR_W     = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1;
  localparam int OCC_W     = $clog2(ROB_DEPTH + 1);
  localparam int CNT_W     = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1;
  localparam int PKT_NUM_W = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS)   : 1;
  localparam int WDB_AW    = (N_WDB     > 1) ? $clog2(N_WDB)      : 1;
  localparam int RDB_AW    = (N_RDB     > 1) ? $clog2(N_RDB)      : 1;
  localparam int SLOT_W    = (WDB_AW > RDB_AW) ? WDB_AW : RDB_AW;
  localparam int SRAM_W    = BL * DQ_W;
  localparam int OBS_W     = rmc_cif_dft_pkg::dft_obs_w(AXI_IDW, AXI_AW, ROB_DEPTH,
                                                        MAX_PKTS, N_WDB, N_RDB);

  // One variable per DFT input, same widths as the DUT ports (generated list)
  logic [1-1:0] awvalid;
  logic [1-1:0] awready;
  logic [AXI_AW-1:0] awaddr;
  logic [8-1:0] awlen;
  logic [3-1:0] awsize;
  logic [AXI_IDW-1:0] awid;
  logic [1-1:0] arvalid;
  logic [1-1:0] arready;
  logic [AXI_AW-1:0] araddr;
  logic [8-1:0] arlen;
  logic [3-1:0] arsize;
  logic [AXI_IDW-1:0] arid;
  logic [1-1:0] wvalid;
  logic [1-1:0] wready;
  logic [AXI_DW/8-1:0] wstrb;
  logic [1-1:0] wlast;
  logic [1-1:0] bvalid;
  logic [1-1:0] bready;
  logic [1-1:0] rvalid;
  logic [1-1:0] rready;
  logic [DADDR_W-1:0] aw_daddr;
  logic [DADDR_W-1:0] ar_daddr;
  logic [CH_W-1:0] aw_ch;
  logic [CH_W-1:0] ar_ch;
  logic [PKT_OFF_W-1:0] aw_off;
  logic [PKT_OFF_W-1:0] ar_off;
  logic [DEC_W*MAP_SEL_W-1:0] addr_map;
  logic [1-1:0] aw_ok;
  logic [1-1:0] ar_ok;
  logic [ROB_IDX_W-1:0] rd_rob_index;
  logic [1-1:0] rd_idx_vld;
  logic [1-1:0] rd_alloc_vld;
  logic [PTR_W-1:0] rd_head;
  logic [PTR_W-1:0] rd_tail;
  logic [OCC_W-1:0] rd_occ;
  logic [1-1:0] rd_retire_vld;
  logic [1-1:0] rd_retire_fire;
  logic [ROB_DEPTH-1:0] rd_inflow_complt;
  logic [ROB_DEPTH*CNT_W-1:0] rd_outflow_cnt;
  logic [ROB_DEPTH*CNT_W-1:0] rd_n_pkts;
  logic [ROB_DEPTH-1:0] rd_ent_err;
  logic [ROB_DEPTH-1:0] rd_ent_derr;
  logic [ROB_DEPTH*RDB_AW-1:0] rd_slot_head;
  logic [AXI_IDW-1:0] rd_resp_id;
  logic [2-1:0] rd_resp_resp;
  logic [RDB_AW-1:0] rd_resp_head;
  logic [PKT_OFF_W-1:0] rd_resp_offset;
  logic [8-1:0] rd_resp_len;
  logic [3-1:0] rd_resp_size;
  logic [ROB_IDX_W-1:0] wr_rob_index;
  logic [1-1:0] wr_idx_vld;
  logic [1-1:0] wr_alloc_vld;
  logic [PTR_W-1:0] wr_head;
  logic [PTR_W-1:0] wr_tail;
  logic [OCC_W-1:0] wr_occ;
  logic [1-1:0] wr_retire_vld;
  logic [1-1:0] wr_retire_fire;
  logic [ROB_DEPTH-1:0] wr_inflow_complt;
  logic [ROB_DEPTH*CNT_W-1:0] wr_outflow_cnt;
  logic [ROB_DEPTH*CNT_W-1:0] wr_n_pkts;
  logic [ROB_DEPTH-1:0] wr_ent_err;
  logic [ROB_DEPTH-1:0] wr_ent_derr;
  logic [ROB_DEPTH*WDB_AW-1:0] wr_slot_head;
  logic [AXI_IDW-1:0] wr_resp_id;
  logic [2-1:0] wr_resp_resp;
  logic [WDB_AW-1:0] wr_resp_head;
  logic [1-1:0] rs_busy;
  logic [CNT_W-1:0] rs_pkt_cnt;
  logic [1-1:0] rs_seg_last;
  logic [CNT_W-1:0] rs_seg_npkts;
  logic [PKT_OFF_W-1:0] rs_seg_offset;
  logic [8-1:0] rs_seg_len;
  logic [3-1:0] rs_seg_size;
  logic [RDB_AW-1:0] rs_seg_head;
  logic [N_CH-1:0] rs_pkt_valid;
  logic [N_CH-1:0] rs_pkt_ready;
  logic [N_CH*ROB_IDX_W-1:0] rs_pkt_rob_index;
  logic [N_CH*DADDR_W-1:0] rs_pkt_daddr;
  logic [N_CH*PKT_NUM_W-1:0] rs_pkt_num;
  logic [N_CH-1:0] rs_pkt_last;
  logic [N_CH*RDB_AW-1:0] rs_pkt_slot;
  logic [N_CH*PKT_OFF_W-1:0] rs_pkt_offset;
  logic [N_CH-1:0] rs_pkt_narrow;
  logic [1-1:0] ws_seg_last;
  logic [CNT_W-1:0] ws_seg_npkts;
  logic [WDB_AW-1:0] ws_seg_head;
  logic [1-1:0] ws_free_slot_take;
  logic [N_CH-1:0] ws_pkt_valid;
  logic [N_CH-1:0] ws_pkt_ready;
  logic [N_CH*ROB_IDX_W-1:0] ws_pkt_rob_index;
  logic [N_CH*DADDR_W-1:0] ws_pkt_daddr;
  logic [N_CH*PKT_NUM_W-1:0] ws_pkt_num;
  logic [N_CH-1:0] ws_pkt_last;
  logic [N_CH*WDB_AW-1:0] ws_pkt_slot;
  logic [N_CH*PKT_OFF_W-1:0] ws_pkt_offset;
  logic [N_CH-1:0] ws_pkt_narrow;
  logic [1-1:0] async_mc_req_valid;
  logic [1-1:0] async_mc_req_ready;
  logic [ROB_IDX_W-1:0] async_mc_req_rob_index;
  logic [1-1:0] async_mc_req_op;
  logic [DADDR_W-1:0] async_mc_req_daddr;
  logic [PKT_NUM_W-1:0] async_mc_req_pkt_num;
  logic [1-1:0] async_mc_req_last_in_txn;
  logic [SLOT_W-1:0] async_mc_req_sram_slot;
  logic [1-1:0] async_mc_cmpl_valid;
  logic [1-1:0] async_mc_cmpl_ready;
  logic [ROB_IDX_W-1:0] async_mc_cmpl_rob_index;
  logic [1-1:0] wdb_we;
  logic [WDB_AW-1:0] wdb_addr;
  logic [SRAM_W-1:0] wdb_din;
  logic [1-1:0] rdb_re;
  logic [RDB_AW-1:0] rdb_addr;
  logic [SRAM_W-1:0] rdb_dout;
  logic [1-1:0] aclk;
  logic [1-1:0] aresetn;
  logic [OBS_W-1:0] dft_obs;

  rmc_cif_dft #(.AXI_IDW(AXI_IDW), .AXI_AW(AXI_AW), .ROB_DEPTH(ROB_DEPTH),
    .MAX_PKTS(MAX_PKTS), .N_WDB(N_WDB), .N_RDB(N_RDB)) dut (
    .awvalid(awvalid),
    .awready(awready),
    .awaddr(awaddr),
    .awlen(awlen),
    .awsize(awsize),
    .awid(awid),
    .arvalid(arvalid),
    .arready(arready),
    .araddr(araddr),
    .arlen(arlen),
    .arsize(arsize),
    .arid(arid),
    .wvalid(wvalid),
    .wready(wready),
    .wstrb(wstrb),
    .wlast(wlast),
    .bvalid(bvalid),
    .bready(bready),
    .rvalid(rvalid),
    .rready(rready),
    .aw_daddr(aw_daddr),
    .ar_daddr(ar_daddr),
    .aw_ch(aw_ch),
    .ar_ch(ar_ch),
    .aw_off(aw_off),
    .ar_off(ar_off),
    .addr_map(addr_map),
    .aw_ok(aw_ok),
    .ar_ok(ar_ok),
    .rd_rob_index(rd_rob_index),
    .rd_idx_vld(rd_idx_vld),
    .rd_alloc_vld(rd_alloc_vld),
    .rd_head(rd_head),
    .rd_tail(rd_tail),
    .rd_occ(rd_occ),
    .rd_retire_vld(rd_retire_vld),
    .rd_retire_fire(rd_retire_fire),
    .rd_inflow_complt(rd_inflow_complt),
    .rd_outflow_cnt(rd_outflow_cnt),
    .rd_n_pkts(rd_n_pkts),
    .rd_ent_err(rd_ent_err),
    .rd_ent_derr(rd_ent_derr),
    .rd_slot_head(rd_slot_head),
    .rd_resp_id(rd_resp_id),
    .rd_resp_resp(rd_resp_resp),
    .rd_resp_head(rd_resp_head),
    .rd_resp_offset(rd_resp_offset),
    .rd_resp_len(rd_resp_len),
    .rd_resp_size(rd_resp_size),
    .wr_rob_index(wr_rob_index),
    .wr_idx_vld(wr_idx_vld),
    .wr_alloc_vld(wr_alloc_vld),
    .wr_head(wr_head),
    .wr_tail(wr_tail),
    .wr_occ(wr_occ),
    .wr_retire_vld(wr_retire_vld),
    .wr_retire_fire(wr_retire_fire),
    .wr_inflow_complt(wr_inflow_complt),
    .wr_outflow_cnt(wr_outflow_cnt),
    .wr_n_pkts(wr_n_pkts),
    .wr_ent_err(wr_ent_err),
    .wr_ent_derr(wr_ent_derr),
    .wr_slot_head(wr_slot_head),
    .wr_resp_id(wr_resp_id),
    .wr_resp_resp(wr_resp_resp),
    .wr_resp_head(wr_resp_head),
    .rs_busy(rs_busy),
    .rs_pkt_cnt(rs_pkt_cnt),
    .rs_seg_last(rs_seg_last),
    .rs_seg_npkts(rs_seg_npkts),
    .rs_seg_offset(rs_seg_offset),
    .rs_seg_len(rs_seg_len),
    .rs_seg_size(rs_seg_size),
    .rs_seg_head(rs_seg_head),
    .rs_pkt_valid(rs_pkt_valid),
    .rs_pkt_ready(rs_pkt_ready),
    .rs_pkt_rob_index(rs_pkt_rob_index),
    .rs_pkt_daddr(rs_pkt_daddr),
    .rs_pkt_num(rs_pkt_num),
    .rs_pkt_last(rs_pkt_last),
    .rs_pkt_slot(rs_pkt_slot),
    .rs_pkt_offset(rs_pkt_offset),
    .rs_pkt_narrow(rs_pkt_narrow),
    .ws_seg_last(ws_seg_last),
    .ws_seg_npkts(ws_seg_npkts),
    .ws_seg_head(ws_seg_head),
    .ws_free_slot_take(ws_free_slot_take),
    .ws_pkt_valid(ws_pkt_valid),
    .ws_pkt_ready(ws_pkt_ready),
    .ws_pkt_rob_index(ws_pkt_rob_index),
    .ws_pkt_daddr(ws_pkt_daddr),
    .ws_pkt_num(ws_pkt_num),
    .ws_pkt_last(ws_pkt_last),
    .ws_pkt_slot(ws_pkt_slot),
    .ws_pkt_offset(ws_pkt_offset),
    .ws_pkt_narrow(ws_pkt_narrow),
    .async_mc_req_valid(async_mc_req_valid),
    .async_mc_req_ready(async_mc_req_ready),
    .async_mc_req_rob_index(async_mc_req_rob_index),
    .async_mc_req_op(async_mc_req_op),
    .async_mc_req_daddr(async_mc_req_daddr),
    .async_mc_req_pkt_num(async_mc_req_pkt_num),
    .async_mc_req_last_in_txn(async_mc_req_last_in_txn),
    .async_mc_req_sram_slot(async_mc_req_sram_slot),
    .async_mc_cmpl_valid(async_mc_cmpl_valid),
    .async_mc_cmpl_ready(async_mc_cmpl_ready),
    .async_mc_cmpl_rob_index(async_mc_cmpl_rob_index),
    .wdb_we(wdb_we),
    .wdb_addr(wdb_addr),
    .wdb_din(wdb_din),
    .rdb_re(rdb_re),
    .rdb_addr(rdb_addr),
    .rdb_dout(rdb_dout),
    .aclk(aclk),
    .aresetn(aresetn),
    .dft_obs(dft_obs)
  );

  // Fill tmp with fresh random bits (a $urandom per 32 bits, no repeated pattern)
  logic [4095:0] tmp;
  task automatic rnd(input int w);
    for (int b = 0; b < 4096; b += 32) tmp[b +: 32] = $urandom;
  endtask

  localparam int CAT_W = (1) + (1) + (AXI_AW) + (8) + (3) + (AXI_IDW) + (1) + (1) + (AXI_AW) + (8) + (3) + (AXI_IDW) + (1) + (1) + (AXI_DW/8) + (1) + (1) + (1) + (1) + (1) + (DADDR_W) + (DADDR_W) + (CH_W) + (CH_W) + (PKT_OFF_W) + (PKT_OFF_W) + (DEC_W*MAP_SEL_W) + (1) + (1) + (ROB_IDX_W) + (1) + (1) + (PTR_W) + (PTR_W) + (OCC_W) + (1) + (1) + (ROB_DEPTH) + (ROB_DEPTH*CNT_W) + (ROB_DEPTH*CNT_W) + (ROB_DEPTH) + (ROB_DEPTH) + (ROB_DEPTH*RDB_AW) + (AXI_IDW) + (2) + (RDB_AW) + (PKT_OFF_W) + (8) + (3) + (ROB_IDX_W) + (1) + (1) + (PTR_W) + (PTR_W) + (OCC_W) + (1) + (1) + (ROB_DEPTH) + (ROB_DEPTH*CNT_W) + (ROB_DEPTH*CNT_W) + (ROB_DEPTH) + (ROB_DEPTH) + (ROB_DEPTH*WDB_AW) + (AXI_IDW) + (2) + (WDB_AW) + (1) + (CNT_W) + (1) + (CNT_W) + (PKT_OFF_W) + (8) + (3) + (RDB_AW) + (N_CH) + (N_CH) + (N_CH*ROB_IDX_W) + (N_CH*DADDR_W) + (N_CH*PKT_NUM_W) + (N_CH) + (N_CH*RDB_AW) + (N_CH*PKT_OFF_W) + (N_CH) + (1) + (CNT_W) + (WDB_AW) + (1) + (N_CH) + (N_CH) + (N_CH*ROB_IDX_W) + (N_CH*DADDR_W) + (N_CH*PKT_NUM_W) + (N_CH) + (N_CH*WDB_AW) + (N_CH*PKT_OFF_W) + (N_CH) + (1) + (1) + (ROB_IDX_W) + (1) + (DADDR_W) + (PKT_NUM_W) + (1) + (SLOT_W) + (1) + (1) + (ROB_IDX_W) + (1) + (WDB_AW) + (SRAM_W) + (1) + (RDB_AW) + (SRAM_W) + (1) + (1);
  // expc: expected bus, built by concatenation in reverse field order (MSB first)
  logic [CAT_W-1:0] expc;
  // 20 iterations: new random inputs, then compare the bus with the concatenation.
  initial begin
    errors = 0;
    iters  = 0;
    bus_w = OBS_W;
    cat_w = CAT_W;
    for (int it = 0; it < 20; it++) begin
      rnd(0); awvalid = tmp[1-1:0];
      rnd(0); awready = tmp[1-1:0];
      rnd(0); awaddr = tmp[AXI_AW-1:0];
      rnd(0); awlen = tmp[8-1:0];
      rnd(0); awsize = tmp[3-1:0];
      rnd(0); awid = tmp[AXI_IDW-1:0];
      rnd(0); arvalid = tmp[1-1:0];
      rnd(0); arready = tmp[1-1:0];
      rnd(0); araddr = tmp[AXI_AW-1:0];
      rnd(0); arlen = tmp[8-1:0];
      rnd(0); arsize = tmp[3-1:0];
      rnd(0); arid = tmp[AXI_IDW-1:0];
      rnd(0); wvalid = tmp[1-1:0];
      rnd(0); wready = tmp[1-1:0];
      rnd(0); wstrb = tmp[AXI_DW/8-1:0];
      rnd(0); wlast = tmp[1-1:0];
      rnd(0); bvalid = tmp[1-1:0];
      rnd(0); bready = tmp[1-1:0];
      rnd(0); rvalid = tmp[1-1:0];
      rnd(0); rready = tmp[1-1:0];
      rnd(0); aw_daddr = tmp[DADDR_W-1:0];
      rnd(0); ar_daddr = tmp[DADDR_W-1:0];
      rnd(0); aw_ch = tmp[CH_W-1:0];
      rnd(0); ar_ch = tmp[CH_W-1:0];
      rnd(0); aw_off = tmp[PKT_OFF_W-1:0];
      rnd(0); ar_off = tmp[PKT_OFF_W-1:0];
      rnd(0); addr_map = tmp[DEC_W*MAP_SEL_W-1:0];
      rnd(0); aw_ok = tmp[1-1:0];
      rnd(0); ar_ok = tmp[1-1:0];
      rnd(0); rd_rob_index = tmp[ROB_IDX_W-1:0];
      rnd(0); rd_idx_vld = tmp[1-1:0];
      rnd(0); rd_alloc_vld = tmp[1-1:0];
      rnd(0); rd_head = tmp[PTR_W-1:0];
      rnd(0); rd_tail = tmp[PTR_W-1:0];
      rnd(0); rd_occ = tmp[OCC_W-1:0];
      rnd(0); rd_retire_vld = tmp[1-1:0];
      rnd(0); rd_retire_fire = tmp[1-1:0];
      rnd(0); rd_inflow_complt = tmp[ROB_DEPTH-1:0];
      rnd(0); rd_outflow_cnt = tmp[ROB_DEPTH*CNT_W-1:0];
      rnd(0); rd_n_pkts = tmp[ROB_DEPTH*CNT_W-1:0];
      rnd(0); rd_ent_err = tmp[ROB_DEPTH-1:0];
      rnd(0); rd_ent_derr = tmp[ROB_DEPTH-1:0];
      rnd(0); rd_slot_head = tmp[ROB_DEPTH*RDB_AW-1:0];
      rnd(0); rd_resp_id = tmp[AXI_IDW-1:0];
      rnd(0); rd_resp_resp = tmp[2-1:0];
      rnd(0); rd_resp_head = tmp[RDB_AW-1:0];
      rnd(0); rd_resp_offset = tmp[PKT_OFF_W-1:0];
      rnd(0); rd_resp_len = tmp[8-1:0];
      rnd(0); rd_resp_size = tmp[3-1:0];
      rnd(0); wr_rob_index = tmp[ROB_IDX_W-1:0];
      rnd(0); wr_idx_vld = tmp[1-1:0];
      rnd(0); wr_alloc_vld = tmp[1-1:0];
      rnd(0); wr_head = tmp[PTR_W-1:0];
      rnd(0); wr_tail = tmp[PTR_W-1:0];
      rnd(0); wr_occ = tmp[OCC_W-1:0];
      rnd(0); wr_retire_vld = tmp[1-1:0];
      rnd(0); wr_retire_fire = tmp[1-1:0];
      rnd(0); wr_inflow_complt = tmp[ROB_DEPTH-1:0];
      rnd(0); wr_outflow_cnt = tmp[ROB_DEPTH*CNT_W-1:0];
      rnd(0); wr_n_pkts = tmp[ROB_DEPTH*CNT_W-1:0];
      rnd(0); wr_ent_err = tmp[ROB_DEPTH-1:0];
      rnd(0); wr_ent_derr = tmp[ROB_DEPTH-1:0];
      rnd(0); wr_slot_head = tmp[ROB_DEPTH*WDB_AW-1:0];
      rnd(0); wr_resp_id = tmp[AXI_IDW-1:0];
      rnd(0); wr_resp_resp = tmp[2-1:0];
      rnd(0); wr_resp_head = tmp[WDB_AW-1:0];
      rnd(0); rs_busy = tmp[1-1:0];
      rnd(0); rs_pkt_cnt = tmp[CNT_W-1:0];
      rnd(0); rs_seg_last = tmp[1-1:0];
      rnd(0); rs_seg_npkts = tmp[CNT_W-1:0];
      rnd(0); rs_seg_offset = tmp[PKT_OFF_W-1:0];
      rnd(0); rs_seg_len = tmp[8-1:0];
      rnd(0); rs_seg_size = tmp[3-1:0];
      rnd(0); rs_seg_head = tmp[RDB_AW-1:0];
      rnd(0); rs_pkt_valid = tmp[N_CH-1:0];
      rnd(0); rs_pkt_ready = tmp[N_CH-1:0];
      rnd(0); rs_pkt_rob_index = tmp[N_CH*ROB_IDX_W-1:0];
      rnd(0); rs_pkt_daddr = tmp[N_CH*DADDR_W-1:0];
      rnd(0); rs_pkt_num = tmp[N_CH*PKT_NUM_W-1:0];
      rnd(0); rs_pkt_last = tmp[N_CH-1:0];
      rnd(0); rs_pkt_slot = tmp[N_CH*RDB_AW-1:0];
      rnd(0); rs_pkt_offset = tmp[N_CH*PKT_OFF_W-1:0];
      rnd(0); rs_pkt_narrow = tmp[N_CH-1:0];
      rnd(0); ws_seg_last = tmp[1-1:0];
      rnd(0); ws_seg_npkts = tmp[CNT_W-1:0];
      rnd(0); ws_seg_head = tmp[WDB_AW-1:0];
      rnd(0); ws_free_slot_take = tmp[1-1:0];
      rnd(0); ws_pkt_valid = tmp[N_CH-1:0];
      rnd(0); ws_pkt_ready = tmp[N_CH-1:0];
      rnd(0); ws_pkt_rob_index = tmp[N_CH*ROB_IDX_W-1:0];
      rnd(0); ws_pkt_daddr = tmp[N_CH*DADDR_W-1:0];
      rnd(0); ws_pkt_num = tmp[N_CH*PKT_NUM_W-1:0];
      rnd(0); ws_pkt_last = tmp[N_CH-1:0];
      rnd(0); ws_pkt_slot = tmp[N_CH*WDB_AW-1:0];
      rnd(0); ws_pkt_offset = tmp[N_CH*PKT_OFF_W-1:0];
      rnd(0); ws_pkt_narrow = tmp[N_CH-1:0];
      rnd(0); async_mc_req_valid = tmp[1-1:0];
      rnd(0); async_mc_req_ready = tmp[1-1:0];
      rnd(0); async_mc_req_rob_index = tmp[ROB_IDX_W-1:0];
      rnd(0); async_mc_req_op = tmp[1-1:0];
      rnd(0); async_mc_req_daddr = tmp[DADDR_W-1:0];
      rnd(0); async_mc_req_pkt_num = tmp[PKT_NUM_W-1:0];
      rnd(0); async_mc_req_last_in_txn = tmp[1-1:0];
      rnd(0); async_mc_req_sram_slot = tmp[SLOT_W-1:0];
      rnd(0); async_mc_cmpl_valid = tmp[1-1:0];
      rnd(0); async_mc_cmpl_ready = tmp[1-1:0];
      rnd(0); async_mc_cmpl_rob_index = tmp[ROB_IDX_W-1:0];
      rnd(0); wdb_we = tmp[1-1:0];
      rnd(0); wdb_addr = tmp[WDB_AW-1:0];
      rnd(0); wdb_din = tmp[SRAM_W-1:0];
      rnd(0); rdb_re = tmp[1-1:0];
      rnd(0); rdb_addr = tmp[RDB_AW-1:0];
      rnd(0); rdb_dout = tmp[SRAM_W-1:0];
      rnd(0); aclk = tmp[1-1:0];
      rnd(0); aresetn = tmp[1-1:0];
      #1;   // let the combinational bus settle
      expc = {aresetn, aclk, rdb_dout, rdb_addr, rdb_re, wdb_din, wdb_addr, wdb_we, async_mc_cmpl_rob_index, async_mc_cmpl_ready, async_mc_cmpl_valid, async_mc_req_sram_slot, async_mc_req_last_in_txn, async_mc_req_pkt_num, async_mc_req_daddr, async_mc_req_op, async_mc_req_rob_index, async_mc_req_ready, async_mc_req_valid, ws_pkt_narrow, ws_pkt_offset, ws_pkt_slot, ws_pkt_last, ws_pkt_num, ws_pkt_daddr, ws_pkt_rob_index, ws_pkt_ready, ws_pkt_valid, ws_free_slot_take, ws_seg_head, ws_seg_npkts, ws_seg_last, rs_pkt_narrow, rs_pkt_offset, rs_pkt_slot, rs_pkt_last, rs_pkt_num, rs_pkt_daddr, rs_pkt_rob_index, rs_pkt_ready, rs_pkt_valid, rs_seg_head, rs_seg_size, rs_seg_len, rs_seg_offset, rs_seg_npkts, rs_seg_last, rs_pkt_cnt, rs_busy, wr_resp_head, wr_resp_resp, wr_resp_id, wr_slot_head, wr_ent_derr, wr_ent_err, wr_n_pkts, wr_outflow_cnt, wr_inflow_complt, wr_retire_fire, wr_retire_vld, wr_occ, wr_tail, wr_head, wr_alloc_vld, wr_idx_vld, wr_rob_index, rd_resp_size, rd_resp_len, rd_resp_offset, rd_resp_head, rd_resp_resp, rd_resp_id, rd_slot_head, rd_ent_derr, rd_ent_err, rd_n_pkts, rd_outflow_cnt, rd_inflow_complt, rd_retire_fire, rd_retire_vld, rd_occ, rd_tail, rd_head, rd_alloc_vld, rd_idx_vld, rd_rob_index, ar_ok, aw_ok, addr_map, ar_off, aw_off, ar_ch, aw_ch, ar_daddr, aw_daddr, rready, rvalid, bready, bvalid, wlast, wstrb, wready, wvalid, arid, arsize, arlen, araddr, arready, arvalid, awid, awsize, awlen, awaddr, awready, awvalid};
      // a width disagreement counts as an error; !== so X on the bus is a mismatch
      if (OBS_W !== CAT_W || dft_obs !== expc) errors++;
      iters++;
    end
  end
endmodule : dft_layout_chk
