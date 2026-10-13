// tb_rmc_cif.sv: self-checking TB for rmc_cif and dft_obs; T1-T9 drive top-level ports only.
// IMPL = implemented logic/layout, SEGF = forced pins (T10/T11, unit level), TIED = constants,
// DFT = bus == source net (not functional), BLOCKED = RTL incomplete, never a pass.

`timescale 1ns/1ps

module tb_rmc_cif;
  import rmc_cfg_pkg::*;

  // ---- DUT configuration (defaults; no overrides) ----
  localparam int AXI_IDW   = 8;
  localparam int AXI_AWD   = 48;
  localparam int ROB_DEPTH = 32;
  localparam int MAX_PKTS  = 16;
  localparam int N_WDB     = 32;
  localparam int N_RDB     = 32;

  // TB's own width derivations (same guarded convention as the RTL)
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1;
  localparam int PTR_W     = ROB_IDX_W;
  localparam int QPTR_W    = PTR_W + 1;     // rseg queue pointer = index + wrap bit
  localparam int OCC_W     = $clog2(ROB_DEPTH + 1);
  localparam int CNT_W     = (MAX_PKTS > 1) ? $clog2(MAX_PKTS+1)  : 1;
  localparam int PKT_NUM_W = (MAX_PKTS > 1) ? $clog2(MAX_PKTS)    : 1;
  localparam int WDB_AW    = (N_WDB > 1) ? $clog2(N_WDB) : 1;
  localparam int RDB_AW    = (N_RDB > 1) ? $clog2(N_RDB) : 1;
  localparam int SLOT_W    = (WDB_AW > RDB_AW) ? WDB_AW : RDB_AW;
  localparam int SRAMW     = BL * DQ_W;

  localparam int NCH = N_CH;

  // expected total bus width (independent sum)
  function automatic int exp_obs_w(int idw, int aw, int depth, int pkts, int nw, int nr);
    int rob_idx, ptr, occ, cnt, pn, wa, ra, sl, t;
    rob_idx = (depth > 1) ? $clog2(depth) : 1;
    ptr     = rob_idx;
    occ     = $clog2(depth + 1);
    cnt     = (pkts > 1) ? $clog2(pkts+1) : 1;
    pn      = (pkts > 1) ? $clog2(pkts)   : 1;
    wa      = (nw > 1) ? $clog2(nw) : 1;
    ra      = (nr > 1) ? $clog2(nr) : 1;
    sl      = (wa > ra) ? wa : ra;
    t  = 2 * (2 + aw + 8 + 3 + idw);                  // AW, AR
    t += 2 + AXI_DW/8 + 1 + 4;                        // W, B/R
    t += 2*DADDR_W + 2*CH_W + 2*PKT_OFF_W + DEC_W*MAP_SEL_W;
    t += 2;                                           // aw_ok, ar_ok
    t += 2 * (rob_idx + 4 + 2*ptr + occ + depth + depth*cnt);
    t += 2 * (depth*cnt + 2*depth + idw + 2);          // n_pkts, ent_err/derr, resp_id/resp
    t += depth*ra + ra + 6 + 8 + 3;                    // read: slot_head, resp head/off/len/size
    t += depth*wa + wa;                                // write: slot_head, resp head
    t += 1 + 2*(ptr+1) + depth*NCH + NCH*(ptr+1) + NCH*cnt + ra;   // rseg: ready, q ptrs, ch_done, walkers, dbuf_base
    t += 1 + rob_idx + cnt + 6 + 8 + 3 + ra;          // rseg seg_* bundle
    t += NCH * (1 + 1 + 1 + rob_idx + DADDR_W + ra);   // rseg pkt_*: valid, ready, dbuf_rdy, rob_idx, daddr, dbuf_adr
    t += 1 + cnt + wa + 1;                             // write seg seg_*/free_slot_take
    t += NCH * (1 + 1 + rob_idx + DADDR_W + pn + 1 + wa + PKT_OFF_W + 1);
    t += 1 + 1 + rob_idx + 1 + DADDR_W + pn + 1 + sl + 1 + 1 + rob_idx;
    t += 1 + wa + BL*DQ_W + 1 + ra + BL*DQ_W;
    t += 2;                                           // aclk, aresetn
    return t;
  endfunction

  localparam int OBS_W_EXP = exp_obs_w(AXI_IDW, AXI_AWD, ROB_DEPTH, MAX_PKTS, N_WDB, N_RDB);

  // ---- clock / reset / DUT ----
  logic aclk = 1'b0;
  logic aresetn;
  always #5 aclk = ~aclk;

  logic [AXI_IDW-1:0]  awid, arid;
  logic [AXI_AWD-1:0]  awaddr, araddr;
  logic [7:0]          awlen, arlen;
  logic [2:0]          awsize, arsize;
  logic [1:0]          awburst, arburst;
  logic                awvalid, awready, arvalid, arready;
  logic [AXI_DW-1:0]   wdata;
  logic [AXI_DW/8-1:0] wstrb;
  logic                wlast, wvalid, wready;
  logic [AXI_IDW-1:0]  bid, rid;
  logic [1:0]          bresp, rresp;
  logic                bvalid, bready, rvalid, rready, rlast;
  logic [AXI_DW-1:0]   rdata;

  logic                  req_valid, req_ready, req_op, req_last;
  logic [ROB_IDX_W-1:0]  req_rob;
  logic [DADDR_W-1:0]    req_daddr;
  logic [PKT_NUM_W-1:0]  req_pkt;
  logic [SLOT_W-1:0]     req_slot;
  logic                  cmpl_valid, cmpl_ready;
  logic [ROB_IDX_W-1:0]  cmpl_rob;

  logic                  wdb_we, rdb_re;
  logic [WDB_AW-1:0]     wdb_addr;
  logic [RDB_AW-1:0]     rdb_addr;
  logic [SRAMW-1:0]      wdb_din, rdb_dout;

  logic [OBS_W_EXP-1:0]  dft_obs;

  rmc_cif dut (
    .aclk(aclk), .aresetn(aresetn),
    .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize), .awburst(awburst),
    .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
    .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst),
    .arvalid(arvalid), .arready(arready),
    .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast), .rvalid(rvalid), .rready(rready),
    .async_mc_req_valid(req_valid), .async_mc_req_ready(req_ready),
    .async_mc_req_rob_index(req_rob), .async_mc_req_op(req_op),
    .async_mc_req_daddr(req_daddr), .async_mc_req_pkt_num(req_pkt),
    .async_mc_req_last_in_txn(req_last), .async_mc_req_sram_slot(req_slot),
    .async_mc_cmpl_valid(cmpl_valid), .async_mc_cmpl_ready(cmpl_ready),
    .async_mc_cmpl_rob_index(cmpl_rob),
    .wdb_we(wdb_we), .wdb_addr(wdb_addr), .wdb_din(wdb_din),
    .rdb_re(rdb_re), .rdb_addr(rdb_addr), .rdb_dout(rdb_dout),
    .dft_obs(dft_obs)
  );

  // Elaboration-only min-corner instance (ROB_DEPTH=1, MAX_PKTS=1), width check only;
  // N_WDB/N_RDB stay 2 because the CIF's wdb/rdb_addr ports use an unguarded $clog2.
  localparam int MIN_DEPTH = 1;
  localparam int MIN_PKTS  = 1;
  localparam int MIN_OBS_W = exp_obs_w(AXI_IDW, AXI_AWD, MIN_DEPTH, MIN_PKTS, 2, 2);
  logic [MIN_OBS_W-1:0] dft_obs_min;
  rmc_cif #(.ROB_DEPTH(MIN_DEPTH), .MAX_PKTS(MIN_PKTS), .N_WDB(2), .N_RDB(2)) dut_min (
    .aclk(aclk), .aresetn(aresetn),
    .awid('0), .awaddr('0), .awlen('0), .awsize('0), .awburst('0), .awvalid(1'b0), .awready(),
    .wdata('0), .wstrb('0), .wlast(1'b0), .wvalid(1'b0), .wready(),
    .bid(), .bresp(), .bvalid(), .bready(1'b0),
    .arid('0), .araddr('0), .arlen('0), .arsize('0), .arburst('0), .arvalid(1'b0), .arready(),
    .rid(), .rdata(), .rresp(), .rlast(), .rvalid(), .rready(1'b0),
    .async_mc_req_valid(), .async_mc_req_ready(1'b0), .async_mc_req_rob_index(),
    .async_mc_req_op(), .async_mc_req_daddr(), .async_mc_req_pkt_num(),
    .async_mc_req_last_in_txn(), .async_mc_req_sram_slot(),
    .async_mc_cmpl_valid(1'b0), .async_mc_cmpl_ready(), .async_mc_cmpl_rob_index('0),
    .wdb_we(), .wdb_addr(), .wdb_din(), .rdb_re(), .rdb_addr(), .rdb_dout('0),
    .dft_obs(dft_obs_min)
  );

  // Alternate config: N_WDB != N_RDB, non-power-of-2 ROB depth. Elaboration, bus
  // width and the N_SLOTS pass-through widths are checked; not simulated.
  localparam int ALT_DEPTH = 3, ALT_PKTS = 3, ALT_WDB = 8, ALT_RDB = 64;
  localparam int ALT_OBS_W = exp_obs_w(AXI_IDW, AXI_AWD, ALT_DEPTH, ALT_PKTS, ALT_WDB, ALT_RDB);
  logic [ALT_OBS_W-1:0] dft_obs_alt;
  rmc_cif #(.ROB_DEPTH(ALT_DEPTH), .MAX_PKTS(ALT_PKTS), .N_WDB(ALT_WDB), .N_RDB(ALT_RDB)) dut_alt (
    .aclk(aclk), .aresetn(aresetn),
    .awid('0), .awaddr('0), .awlen('0), .awsize('0), .awburst('0), .awvalid(1'b0), .awready(),
    .wdata('0), .wstrb('0), .wlast(1'b0), .wvalid(1'b0), .wready(),
    .bid(), .bresp(), .bvalid(), .bready(1'b0),
    .arid('0), .araddr('0), .arlen('0), .arsize('0), .arburst('0), .arvalid(1'b0), .arready(),
    .rid(), .rdata(), .rresp(), .rlast(), .rvalid(), .rready(1'b0),
    .async_mc_req_valid(), .async_mc_req_ready(1'b0), .async_mc_req_rob_index(),
    .async_mc_req_op(), .async_mc_req_daddr(), .async_mc_req_pkt_num(),
    .async_mc_req_last_in_txn(), .async_mc_req_sram_slot(),
    .async_mc_cmpl_valid(1'b0), .async_mc_cmpl_ready(), .async_mc_cmpl_rob_index('0),
    .wdb_we(), .wdb_addr(), .wdb_din(), .rdb_re(), .rdb_addr(), .rdb_dout('0),
    .dft_obs(dft_obs_alt)
  );

  // concatenation-based layout checks of rmc_cif_dft at several configurations
  localparam int N_LCHK = 6;
  logic [N_LCHK*32-1:0] lc_err_v, lc_bus_v, lc_cat_v, lc_it_v;   // packed: array-element port hookup is unreliable in iverilog
  dft_layout_chk #(8, 48, 32, 16, 32, 32) lchk0 (lc_err_v[0*32 +: 32], lc_bus_v[0*32 +: 32], lc_cat_v[0*32 +: 32], lc_it_v[0*32 +: 32]);
  dft_layout_chk #(8, 48,  1,  1,  2,  2) lchk1 (lc_err_v[1*32 +: 32], lc_bus_v[1*32 +: 32], lc_cat_v[1*32 +: 32], lc_it_v[1*32 +: 32]);
  dft_layout_chk #(4, 48,  3,  3,  8, 64) lchk2 (lc_err_v[2*32 +: 32], lc_bus_v[2*32 +: 32], lc_cat_v[2*32 +: 32], lc_it_v[2*32 +: 32]);
  dft_layout_chk #(1, 40,  2,  2,  2,  2) lchk3 (lc_err_v[3*32 +: 32], lc_bus_v[3*32 +: 32], lc_cat_v[3*32 +: 32], lc_it_v[3*32 +: 32]);
  dft_layout_chk #(8, 48, 16, 64, 64,  4) lchk4 (lc_err_v[4*32 +: 32], lc_bus_v[4*32 +: 32], lc_cat_v[4*32 +: 32], lc_it_v[4*32 +: 32]);
  dft_layout_chk #(16,48,  5,  7,  2, 16) lchk5 (lc_err_v[5*32 +: 32], lc_bus_v[5*32 +: 32], lc_cat_v[5*32 +: 32], lc_it_v[5*32 +: 32]);

  // Walker state read straight from each per-channel engine (independent of rseg's flattening).
  logic [N_CH*QPTR_W-1:0] src_eng_rd;
  logic [N_CH*CNT_W-1:0]  src_eng_m;
  for (genvar c = 0; c < N_CH; c++) begin : g_src
    assign src_eng_rd[c*QPTR_W +: QPTR_W] = dut.u_rseg.g_ch[c].u_ch.rd_ptr;
    assign src_eng_m[c*CNT_W +: CNT_W]    = dut.u_rseg.g_ch[c].u_ch.m;
  end

  // ---- bookkeeping ----
  // per-class counters; BLOCKED items are only counted, never added to a pass total
  int n_impl_pass = 0, n_impl_fail = 0;
  int n_tied_pass = 0, n_tied_fail = 0;
  int n_dft_pass  = 0, n_dft_fail  = 0;
  int n_segf_pass = 0, n_segf_fail = 0;
  int n_blocked   = 0;

  // Implemented logic or bus layout: a failure is a real defect.
  task automatic chk_impl(input bit cond, input string msg);
    if (cond) n_impl_pass++;
    else begin n_impl_fail++; $display("[%0t] FAIL(IMPL) %s", $time, msg); end
  endtask

  // Output is currently tied off; passing only confirms the constant, not behaviour.
  task automatic chk_tied(input bit cond, input string msg);
    if (cond) n_tied_pass++;
    else begin n_tied_fail++; $display("[%0t] FAIL(TIED) %s", $time, msg); end
  endtask

  // Unit-level check that relies on TB-forced pins (T10/T11); not an integration check.
  task automatic chk_segf(input bit cond, input string msg);
    if (cond) n_segf_pass++;
    else begin n_segf_fail++; $display("[%0t] FAIL(SEGF) %s", $time, msg); end
  endtask

  // Records a path that cannot be tested yet; deliberately not a pass or a fail.
  task automatic blocked(input string msg);
    n_blocked++;
    $display("BLOCKED: %s", msg);
  endtask

  // ---- dft_obs layout: TB-side order and widths, built independently of the DFT module ----
  int cur;
  int o_awvalid, o_awready, o_awaddr, o_awlen, o_awsize, o_awid;
  int o_arvalid, o_arready, o_araddr, o_arlen, o_arsize, o_arid;
  int o_wvalid, o_wready, o_wstrb, o_wlast, o_bvalid, o_bready, o_rvalid, o_rready;
  int o_aw_daddr, o_ar_daddr, o_aw_ch, o_ar_ch, o_aw_off, o_ar_off, o_addr_map;
  int o_aw_ok, o_ar_ok;
  int o_rd_rob_index, o_rd_rob_ready, o_rd_alloc_vld, o_rd_head, o_rd_tail, o_rd_occ;
  int o_rd_retire_vld, o_rd_retire_fire, o_rd_inflow, o_rd_outflow;
  int o_wr_rob_index, o_wr_idx_vld, o_wr_alloc_vld, o_wr_head, o_wr_tail, o_wr_occ;
  int o_wr_retire_vld, o_wr_retire_fire, o_wr_inflow, o_wr_outflow;
  int o_rq_valid, o_rq_ready, o_rq_rob, o_rq_op, o_rq_daddr, o_rq_pkt, o_rq_last, o_rq_slot;
  int o_cm_valid, o_cm_ready, o_cm_rob;
  int o_wdb_we, o_wdb_addr, o_wdb_din, o_rdb_re, o_rdb_addr, o_rdb_dout;
  int o_rd_npkts, o_rd_eerr, o_rd_ederr, o_rd_shead, o_rd_rid, o_rd_rresp, o_rd_rhead;
  int o_rd_roff, o_rd_rlen, o_rd_rsize;
  int o_wr_npkts, o_wr_eerr, o_wr_ederr, o_wr_shead, o_wr_rid, o_wr_rresp, o_wr_rhead;
  int o_rs_ready, o_rs_qa, o_rs_qp, o_rs_chd, o_rs_erp, o_rs_em, o_rs_dbase;
  int o_rs_last, o_rs_srob, o_rs_npkts, o_rs_off, o_rs_len, o_rs_size, o_rs_head;
  int o_rs_pv, o_rs_pr, o_rs_pdr, o_rs_prob, o_rs_pdaddr, o_rs_pdbuf;
  int o_ws_last, o_ws_npkts, o_ws_head, o_ws_take;
  int o_ws_pv, o_ws_pr, o_ws_prob, o_ws_pdaddr, o_ws_pnum, o_ws_plast, o_ws_pslot, o_ws_poff, o_ws_pnar;
  int o_aclk, o_aresetn;

  task automatic fld(output int o, input int w);
    o = cur; cur += w;
  endtask

  task automatic build_layout();
    cur = 0;
    fld(o_awvalid,1); fld(o_awready,1); fld(o_awaddr,AXI_AWD); fld(o_awlen,8);
    fld(o_awsize,3); fld(o_awid,AXI_IDW);
    fld(o_arvalid,1); fld(o_arready,1); fld(o_araddr,AXI_AWD); fld(o_arlen,8);
    fld(o_arsize,3); fld(o_arid,AXI_IDW);
    fld(o_wvalid,1); fld(o_wready,1); fld(o_wstrb,AXI_DW/8); fld(o_wlast,1);
    fld(o_bvalid,1); fld(o_bready,1); fld(o_rvalid,1); fld(o_rready,1);
    fld(o_aw_daddr,DADDR_W); fld(o_ar_daddr,DADDR_W);
    fld(o_aw_ch,CH_W); fld(o_ar_ch,CH_W);
    fld(o_aw_off,PKT_OFF_W); fld(o_ar_off,PKT_OFF_W);
    fld(o_addr_map,DEC_W*MAP_SEL_W);
    fld(o_aw_ok,1); fld(o_ar_ok,1);
    fld(o_rd_rob_index,ROB_IDX_W); fld(o_rd_rob_ready,1); fld(o_rd_alloc_vld,1);
    fld(o_rd_head,PTR_W); fld(o_rd_tail,PTR_W); fld(o_rd_occ,OCC_W);
    fld(o_rd_retire_vld,1); fld(o_rd_retire_fire,1);
    fld(o_rd_inflow,ROB_DEPTH); fld(o_rd_outflow,ROB_DEPTH*CNT_W);
    fld(o_rd_npkts,ROB_DEPTH*CNT_W); fld(o_rd_eerr,ROB_DEPTH); fld(o_rd_ederr,ROB_DEPTH);
    fld(o_rd_shead,ROB_DEPTH*RDB_AW);
    fld(o_rd_rid,AXI_IDW); fld(o_rd_rresp,2); fld(o_rd_rhead,RDB_AW);
    fld(o_rd_roff,PKT_OFF_W); fld(o_rd_rlen,8); fld(o_rd_rsize,3);
    fld(o_wr_rob_index,ROB_IDX_W); fld(o_wr_idx_vld,1); fld(o_wr_alloc_vld,1);
    fld(o_wr_head,PTR_W); fld(o_wr_tail,PTR_W); fld(o_wr_occ,OCC_W);
    fld(o_wr_retire_vld,1); fld(o_wr_retire_fire,1);
    fld(o_wr_inflow,ROB_DEPTH); fld(o_wr_outflow,ROB_DEPTH*CNT_W);
    fld(o_wr_npkts,ROB_DEPTH*CNT_W); fld(o_wr_eerr,ROB_DEPTH); fld(o_wr_ederr,ROB_DEPTH);
    fld(o_wr_shead,ROB_DEPTH*WDB_AW);
    fld(o_wr_rid,AXI_IDW); fld(o_wr_rresp,2); fld(o_wr_rhead,WDB_AW);
    fld(o_rs_ready,1); fld(o_rs_qa,QPTR_W); fld(o_rs_qp,QPTR_W); fld(o_rs_chd,ROB_DEPTH*N_CH);
    fld(o_rs_erp,N_CH*QPTR_W); fld(o_rs_em,N_CH*CNT_W); fld(o_rs_dbase,RDB_AW);
    fld(o_rs_last,1); fld(o_rs_srob,ROB_IDX_W); fld(o_rs_npkts,CNT_W);
    fld(o_rs_off,PKT_OFF_W); fld(o_rs_len,8); fld(o_rs_size,3); fld(o_rs_head,RDB_AW);
    fld(o_rs_pv,N_CH); fld(o_rs_pr,N_CH); fld(o_rs_pdr,N_CH); fld(o_rs_prob,N_CH*ROB_IDX_W);
    fld(o_rs_pdaddr,N_CH*DADDR_W); fld(o_rs_pdbuf,N_CH*RDB_AW);
    fld(o_ws_last,1); fld(o_ws_npkts,CNT_W); fld(o_ws_head,WDB_AW); fld(o_ws_take,1);
    fld(o_ws_pv,N_CH); fld(o_ws_pr,N_CH); fld(o_ws_prob,N_CH*ROB_IDX_W);
    fld(o_ws_pdaddr,N_CH*DADDR_W); fld(o_ws_pnum,N_CH*PKT_NUM_W); fld(o_ws_plast,N_CH);
    fld(o_ws_pslot,N_CH*WDB_AW); fld(o_ws_poff,N_CH*PKT_OFF_W); fld(o_ws_pnar,N_CH);
    fld(o_rq_valid,1); fld(o_rq_ready,1); fld(o_rq_rob,ROB_IDX_W); fld(o_rq_op,1);
    fld(o_rq_daddr,DADDR_W); fld(o_rq_pkt,PKT_NUM_W); fld(o_rq_last,1); fld(o_rq_slot,SLOT_W);
    fld(o_cm_valid,1); fld(o_cm_ready,1); fld(o_cm_rob,ROB_IDX_W);
    fld(o_wdb_we,1); fld(o_wdb_addr,WDB_AW); fld(o_wdb_din,SRAMW);
    fld(o_rdb_re,1); fld(o_rdb_addr,RDB_AW); fld(o_rdb_dout,SRAMW);
    fld(o_aclk,1); fld(o_aresetn,1);
  endtask

  // Tap correspondence only: the bus slice must equal its source net (=== so X must
  // match X). Says nothing about whether the source value is functionally right.
  logic [4095:0] f_mask, f_got, f_exp;
  task automatic cmp(input string name, input int off, input int w, input logic [4095:0] exp);
    f_mask = {4096{1'b1}} >> (4096 - w);
    f_got  = ({{(4096-OBS_W_EXP){1'b0}}, dft_obs} >> off) & f_mask;
    f_exp  = exp & f_mask;
    if (f_got === f_exp) n_dft_pass++;
    else begin
      n_dft_fail++;
      $display("[%0t] FAIL(DFT) %s: bus[%0d+:%0d]=%0h expected %0h", $time, name, off, w, f_got, f_exp);
    end
  endtask

  // every requested tap vs. its actual source net (read-only hierarchical reads)
  task automatic check_dft_all();
    logic [4095:0] z;
    z = '0;
    cmp("awvalid", o_awvalid, 1, z | awvalid);   cmp("awready", o_awready, 1, z | awready);
    cmp("awaddr",  o_awaddr, AXI_AWD, z | awaddr); cmp("awlen", o_awlen, 8, z | awlen);
    cmp("awsize",  o_awsize, 3, z | awsize);     cmp("awid", o_awid, AXI_IDW, z | awid);
    cmp("arvalid", o_arvalid, 1, z | arvalid);   cmp("arready", o_arready, 1, z | arready);
    cmp("araddr",  o_araddr, AXI_AWD, z | araddr); cmp("arlen", o_arlen, 8, z | arlen);
    cmp("arsize",  o_arsize, 3, z | arsize);     cmp("arid", o_arid, AXI_IDW, z | arid);
    cmp("wvalid", o_wvalid, 1, z | wvalid);      cmp("wready", o_wready, 1, z | wready);
    cmp("wstrb", o_wstrb, AXI_DW/8, z | wstrb);  cmp("wlast", o_wlast, 1, z | wlast);
    cmp("bvalid", o_bvalid, 1, z | bvalid);      cmp("bready", o_bready, 1, z | bready);
    cmp("rvalid", o_rvalid, 1, z | rvalid);      cmp("rready", o_rready, 1, z | rready);
    // address map: sources are the nets inside the CIF
    cmp("aw_daddr", o_aw_daddr, DADDR_W, z | dut.aw_daddr);
    cmp("ar_daddr", o_ar_daddr, DADDR_W, z | dut.ar_daddr);
    cmp("aw_ch", o_aw_ch, CH_W, z | dut.aw_ch);  cmp("ar_ch", o_ar_ch, CH_W, z | dut.ar_ch);
    cmp("aw_off", o_aw_off, PKT_OFF_W, z | dut.aw_off);
    cmp("ar_off", o_ar_off, PKT_OFF_W, z | dut.ar_off);
    cmp("addr_map", o_addr_map, DEC_W*MAP_SEL_W, z | dut.addr_map);
    cmp("aw_ok", o_aw_ok, 1, z | dut.aw_ok);     cmp("ar_ok", o_ar_ok, 1, z | dut.ar_ok);
    // read ROB: sources are the ROB's own existing signals
    cmp("rd_rob_index", o_rd_rob_index, ROB_IDX_W, z | dut.u_rrob.ROB_IDX);
    cmp("rd_rob_ready", o_rd_rob_ready, 1, z | dut.u_rrob.A_ID_READY);
    cmp("rd_alloc_vld", o_rd_alloc_vld, 1, z | dut.u_rrob.admit_fire);
    cmp("rd_head", o_rd_head, PTR_W, z | dut.u_rrob.retire_ptr);
    cmp("rd_tail", o_rd_tail, PTR_W, z | dut.u_rrob.alloc_ptr);
    cmp("rd_occ", o_rd_occ, OCC_W, z | dut.u_rrob.occ);
    cmp("rd_retire_vld", o_rd_retire_vld, 1, z | dut.u_rrob.RESP_VALID);
    cmp("rd_retire_fire", o_rd_retire_fire, 1, z | dut.u_rrob.retire_fire);
    // write ROB
    cmp("wr_rob_index", o_wr_rob_index, ROB_IDX_W, z | dut.u_wrob.ROB_IDX);
    cmp("wr_idx_vld", o_wr_idx_vld, 1, z | dut.u_wrob.pending_split);
    cmp("wr_alloc_vld", o_wr_alloc_vld, 1, z | dut.u_wrob.admit_fire);
    cmp("wr_head", o_wr_head, PTR_W, z | dut.u_wrob.retire_ptr);
    cmp("wr_tail", o_wr_tail, PTR_W, z | dut.u_wrob.alloc_ptr);
    cmp("wr_occ", o_wr_occ, OCC_W, z | dut.u_wrob.occ);
    cmp("wr_retire_vld", o_wr_retire_vld, 1, z | dut.u_wrob.RESP_VALID);
    cmp("wr_retire_fire", o_wr_retire_fire, 1, z | dut.u_wrob.retire_fire);
    for (int i = 0; i < ROB_DEPTH; i++) begin
      cmp("rd_inflow_complt", o_rd_inflow  + i,         1,     z | dut.u_rrob.inflow_complt[i]);
      cmp("rd_outflow_cnt",   o_rd_outflow + i*CNT_W,   CNT_W, z | dut.u_rrob.outflow_cnt[i]);
      cmp("wr_inflow_complt", o_wr_inflow  + i,         1,     z | dut.u_wrob.inflow_complt[i]);
      cmp("wr_outflow_cnt",   o_wr_outflow + i*CNT_W,   CNT_W, z | dut.u_wrob.outflow_cnt[i]);
    end
    cmp("rd_resp_id", o_rd_rid, AXI_IDW, z | dut.u_rrob.RESP_ID);
    cmp("rd_resp_resp", o_rd_rresp, 2, z | dut.u_rrob.RESP_RESP);
    cmp("rd_resp_head", o_rd_rhead, RDB_AW, z | dut.u_rrob.RESP_HEAD_PTR);
    cmp("rd_resp_offset", o_rd_roff, PKT_OFF_W, z | dut.u_rrob.RESP_OFFSET);
    cmp("rd_resp_len", o_rd_rlen, 8, z | dut.u_rrob.RESP_LEN);
    cmp("rd_resp_size", o_rd_rsize, 3, z | dut.u_rrob.RESP_SIZE);
    cmp("wr_resp_id", o_wr_rid, AXI_IDW, z | dut.u_wrob.RESP_ID);
    cmp("wr_resp_resp", o_wr_rresp, 2, z | dut.u_wrob.RESP_RESP);
    cmp("wr_resp_head", o_wr_rhead, WDB_AW, z | dut.u_wrob.RESP_HEAD_PTR);
    for (int i = 0; i < ROB_DEPTH; i++) begin
      cmp("rd_n_pkts", o_rd_npkts + i*CNT_W, CNT_W, z | dut.u_rrob.n_pkts[i]);
      cmp("rd_ent_err", o_rd_eerr + i, 1, z | dut.u_rrob.ent_err[i]);
      cmp("rd_ent_derr", o_rd_ederr + i, 1, z | dut.u_rrob.ent_derr[i]);
      cmp("rd_slot_head", o_rd_shead + i*RDB_AW, RDB_AW, z | dut.u_rrob.mem_meta[i][RDB_AW-1:0]);
      cmp("wr_n_pkts", o_wr_npkts + i*CNT_W, CNT_W, z | dut.u_wrob.n_pkts[i]);
      cmp("wr_ent_err", o_wr_eerr + i, 1, z | dut.u_wrob.ent_err[i]);
      cmp("wr_ent_derr", o_wr_ederr + i, 1, z | dut.u_wrob.ent_derr[i]);
      cmp("wr_slot_head", o_wr_shead + i*WDB_AW, WDB_AW, z | dut.u_wrob.chain_head[i]);
    end
    // segmenters: sources are the instance's own ports / state
    cmp("rs_ready", o_rs_ready, 1, z | dut.u_rseg.ready);
    cmp("rs_q_alloc", o_rs_qa, QPTR_W, z | dut.u_rseg.alloc_ptr);
    cmp("rs_q_pop", o_rs_qp, QPTR_W, z | dut.u_rseg.pop_ptr);
    for (int e = 0; e < ROB_DEPTH; e++)
      cmp("rs_q_ch_done", o_rs_chd + e*N_CH, N_CH, z | dut.u_rseg.ch_done[e]);
    cmp("rs_eng_rd_ptr", o_rs_erp, N_CH*QPTR_W, z | src_eng_rd);
    cmp("rs_eng_m", o_rs_em, N_CH*CNT_W, z | src_eng_m);
    cmp("rs_req_dbuf_base", o_rs_dbase, RDB_AW, z | dut.u_rseg.req_dbuf_base);
    cmp("rs_seg_last", o_rs_last, 1, z | dut.u_rseg.seg_last);
    cmp("rs_seg_rob_idx", o_rs_srob, ROB_IDX_W, z | dut.u_rseg.seg_rob_idx);
    cmp("rs_seg_npkts", o_rs_npkts, CNT_W, z | dut.u_rseg.seg_npkts);
    cmp("rs_seg_offset", o_rs_off, PKT_OFF_W, z | dut.u_rseg.seg_offset);
    cmp("rs_seg_len", o_rs_len, 8, z | dut.u_rseg.seg_len);
    cmp("rs_seg_size", o_rs_size, 3, z | dut.u_rseg.seg_size);
    cmp("rs_seg_head", o_rs_head, RDB_AW, z | dut.u_rseg.seg_head_ptr);
    cmp("rs_pkt_valid", o_rs_pv, N_CH, z | dut.u_rseg.pkt_valid);
    cmp("rs_pkt_ready", o_rs_pr, N_CH, z | dut.u_rseg.pkt_ready);
    cmp("rs_pkt_dbuf_rdy", o_rs_pdr, N_CH, z | dut.u_rseg.pkt_dbuf_adr_ready);
    cmp("rs_pkt_rob_idx", o_rs_prob, N_CH*ROB_IDX_W, z | dut.u_rseg.pkt_rob_idx);
    cmp("rs_pkt_daddr", o_rs_pdaddr, N_CH*DADDR_W, z | dut.u_rseg.pkt_daddr);
    cmp("rs_pkt_dbuf_adr", o_rs_pdbuf, N_CH*RDB_AW, z | dut.u_rseg.pkt_dbuf_adr);
    cmp("ws_seg_last", o_ws_last, 1, z | dut.u_wseg.SEG_LAST);
    cmp("ws_seg_npkts", o_ws_npkts, CNT_W, z | dut.u_wseg.SEG_NPKTS);
    cmp("ws_seg_head", o_ws_head, WDB_AW, z | dut.u_wseg.SEG_HEAD_PTR);
    cmp("ws_free_slot_take", o_ws_take, 1, z | dut.u_wseg.free_slot_take);
    cmp("ws_pkt_valid", o_ws_pv, N_CH, z | dut.u_wseg.packet_valid);
    cmp("ws_pkt_ready", o_ws_pr, N_CH, z | dut.u_wseg.packet_ready);
    cmp("ws_pkt_rob_index", o_ws_prob, N_CH*ROB_IDX_W, z | dut.u_wseg.packet_rob_index);
    cmp("ws_pkt_daddr", o_ws_pdaddr, N_CH*DADDR_W, z | dut.u_wseg.packet_daddr);
    cmp("ws_pkt_num", o_ws_pnum, N_CH*PKT_NUM_W, z | dut.u_wseg.packet_pkt_num);
    cmp("ws_pkt_last", o_ws_plast, N_CH, z | dut.u_wseg.packet_last_in_txn);
    cmp("ws_pkt_slot", o_ws_pslot, N_CH*WDB_AW, z | dut.u_wseg.packet_slot);
    cmp("ws_pkt_offset", o_ws_poff, N_CH*PKT_OFF_W, z | dut.u_wseg.packet_offset);
    cmp("ws_pkt_narrow", o_ws_pnar, N_CH, z | dut.u_wseg.packet_narrow);
    // async FIFO boundary
    cmp("req_valid", o_rq_valid, 1, z | req_valid);   cmp("req_ready", o_rq_ready, 1, z | req_ready);
    cmp("req_rob", o_rq_rob, ROB_IDX_W, z | req_rob); cmp("req_op", o_rq_op, 1, z | req_op);
    cmp("req_daddr", o_rq_daddr, DADDR_W, z | req_daddr);
    cmp("req_pkt", o_rq_pkt, PKT_NUM_W, z | req_pkt); cmp("req_last", o_rq_last, 1, z | req_last);
    cmp("req_slot", o_rq_slot, SLOT_W, z | req_slot);
    cmp("cmpl_valid", o_cm_valid, 1, z | cmpl_valid); cmp("cmpl_ready", o_cm_ready, 1, z | cmpl_ready);
    cmp("cmpl_rob", o_cm_rob, ROB_IDX_W, z | cmpl_rob);
    // SRAM
    cmp("wdb_we", o_wdb_we, 1, z | wdb_we);          cmp("wdb_addr", o_wdb_addr, WDB_AW, z | wdb_addr);
    cmp("wdb_din", o_wdb_din, SRAMW, z | wdb_din);   cmp("rdb_re", o_rdb_re, 1, z | rdb_re);
    cmp("rdb_addr", o_rdb_addr, RDB_AW, z | rdb_addr);
    cmp("rdb_dout", o_rdb_dout, SRAMW, z | rdb_dout);
    // clock / reset
    // aclk is checked tick-aligned in tick(); sampling a clock as data races the edge
    cmp("aresetn", o_aresetn, 1, z | aresetn);
  endtask

  // bus slice helpers for semantic checks (defined values only)
  function automatic logic [4095:0] slice(input int off, input int w);
    logic [4095:0] m;
    m = {4096{1'b1}} >> (4096 - w);
    return ({{(4096-OBS_W_EXP){1'b0}}, dft_obs} >> off) & m;
  endfunction

  function automatic logic [4095:0] outflow_of(input bit wr, input int slot);
    return slice((wr ? o_wr_outflow : o_rd_outflow) + slot*CNT_W, CNT_W);
  endfunction

  // ---- address-map model from package bit positions, not the map function ----
  // daddr = {rank, bg_hi, bg_lo, bank, row, col_hi, col_lo}
  function automatic logic [DADDR_W-1:0] ref_daddr(input logic [AXI_AWD-1:0] a);
    logic [COL_LO_W-1:0]       col_lo;
    logic [COL_HI_W-1:0]       col_hi;
    logic [ROW_W-1:0]          row;
    logic [BANK_PER_BG_W-1:0]  bank;
    logic [BG_LO_W-1:0]        bg_lo;
    logic [BG_HI_W-1:0]        bg_hi;
    logic [RANK_W-1:0]         rank;
    col_lo = a[COLLO_POS +: COL_LO_W];
    col_hi = a[COLHI_POS +: COL_HI_W];
    row    = a[ROW_POS   +: ROW_W];
    bank   = a[BANK_POS  +: BANK_PER_BG_W];
    bg_lo  = a[BGLO_POS  +: BG_LO_W];
    bg_hi  = a[BGHI_POS  +: BG_HI_W];
    rank   = a[RANK_POS  +: RANK_W];
    return {rank, bg_hi, bg_lo, bank, row, col_hi, col_lo};
  endfunction
  function automatic logic [CH_W-1:0] ref_ch(input logic [AXI_AWD-1:0] a);
    return a[CH_POS +: CH_W];
  endfunction
  function automatic logic [PKT_OFF_W-1:0] ref_off(input logic [AXI_AWD-1:0] a);
    return a[OFF_POS +: PKT_OFF_W];
  endfunction
  function automatic bit ref_ok(input logic [AXI_AWD-1:0] a);
    return ((a >> MAP_ADDR_W) == '0);
  endfunction

  // ---- stimulus helpers: drive after posedge, sample combinational results at #1 ----
  task automatic tick();
    @(posedge aclk); #1;
    cmp("aclk (tick-aligned)", o_aclk, 1, 4096'(1));   // aclk is high 1 time unit after posedge
  endtask

  task automatic idle_inputs();
    awvalid = 0; awid = '0; awaddr = '0; awlen = '0; awsize = '0; awburst = '0;
    arvalid = 0; arid = '0; araddr = '0; arlen = '0; arsize = '0; arburst = '0;
    wvalid = 0; wdata = '0; wstrb = '0; wlast = 0;
    bready = 0; rready = 0;
    req_ready = 0; cmpl_valid = 0; cmpl_rob = '0;
    rdb_dout = '0;
  endtask

  task automatic do_reset(input int cycles);
    aresetn = 0; idle_inputs();
    repeat (cycles) tick();
    aresetn = 1; tick();
  endtask

  // constant-output / tied-off checks, called at many points
  task automatic check_tied(input string where);
    chk_tied(wready === 1'b0,            {where, ": wready tied 0"});
    chk_tied(bvalid === 1'b0,            {where, ": bvalid tied 0"});
    chk_tied(bid === '0,                 {where, ": bid tied 0"});
    chk_tied(bresp === 2'b00,            {where, ": bresp tied 0"});
    chk_tied(rvalid === 1'b0,            {where, ": rvalid tied 0"});
    chk_tied(rid === '0,                 {where, ": rid tied 0"});
    chk_tied(rdata === '0,               {where, ": rdata tied 0"});
    chk_tied(rresp === 2'b00,            {where, ": rresp tied 0"});
    chk_tied(rlast === 1'b0,             {where, ": rlast tied 0"});
    chk_tied(req_valid === 1'b0,         {where, ": async_mc_req_valid tied 0"});
    chk_tied(req_rob === '0,             {where, ": async_mc_req_rob_index tied 0"});
    chk_tied(req_op === 1'b0,            {where, ": async_mc_req_op tied 0"});
    chk_tied(req_daddr === '0,           {where, ": async_mc_req_daddr tied 0"});
    chk_tied(req_pkt === '0,             {where, ": async_mc_req_pkt_num tied 0"});
    chk_tied(req_last === 1'b0,          {where, ": async_mc_req_last_in_txn tied 0"});
    chk_tied(req_slot === '0,            {where, ": async_mc_req_sram_slot tied 0"});
    chk_tied(cmpl_ready === 1'b1,        {where, ": async_mc_cmpl_ready tied 1"});
    chk_tied(wdb_we === 1'b0,            {where, ": wdb_we tied 0"});
    chk_tied(wdb_addr === '0,            {where, ": wdb_addr tied 0"});
    chk_tied(wdb_din === '0,             {where, ": wdb_din tied 0"});
    chk_tied(rdb_re === 1'b0,            {where, ": rdb_re tied 0"});
    chk_tied(rdb_addr === '0,            {where, ": rdb_addr tied 0"});
    chk_tied(dut.u_wseg.SEG_LAST === 1'b0 && dut.u_wseg.SEG_NPKTS === '0 &&
             dut.u_wseg.SEG_HEAD_PTR === '0 && dut.u_wseg.free_slot_take === 1'b0,
             {where, ": wseg seg/free-slot outputs tied 0 (stub)"});
    chk_tied(dut.u_wseg.packet_valid === '0 && dut.u_wseg.packet_daddr === '0 &&
             dut.u_wseg.packet_pkt_num === '0 && dut.u_wseg.packet_slot === '0 &&
             dut.u_wseg.packet_last_in_txn === '0 && dut.u_wseg.packet_offset === '0 &&
             dut.u_wseg.packet_narrow === '0 && dut.u_wseg.packet_rob_index === '0,
             {where, ": wseg packet_* tied 0 (stub)"});
  endtask

  // ROB state after reset: pointers / occupancy / handoff cleared
  task automatic check_rob_reset(input string where);
    chk_impl(slice(o_rd_occ, OCC_W) === 0,      {where, ": read occ == 0"});
    chk_impl(slice(o_wr_occ, OCC_W) === 0,      {where, ": write occ == 0"});
    chk_impl(slice(o_rd_head, PTR_W) === 0,     {where, ": read head == 0"});
    chk_impl(slice(o_rd_tail, PTR_W) === 0,     {where, ": read tail == 0"});
    chk_impl(slice(o_wr_head, PTR_W) === 0,     {where, ": write head == 0"});
    chk_impl(slice(o_wr_tail, PTR_W) === 0,     {where, ": write tail == 0"});
    chk_impl(slice(o_rs_qa, QPTR_W) === 0 && slice(o_rs_qp, QPTR_W) === 0 && slice(o_rs_erp, N_CH*QPTR_W) === 0,
             {where, ": rseg queue and walker pointers == 0"});
    chk_impl(slice(o_rs_ready, 1) === 1,        {where, ": rseg queue ready"});
    chk_impl(slice(o_wr_idx_vld, 1) === 0,      {where, ": write idx_vld == 0"});
    chk_impl(slice(o_rd_retire_vld, 1) === 0,   {where, ": read retire_vld == 0"});
    chk_impl(slice(o_wr_retire_vld, 1) === 0,   {where, ": write retire_vld == 0"});
  endtask


  // ---- write-ROB sequence with segmenter-side pins forced (SEGF): wseg is a stub ----
  // Static tasks below: iverilog rejects automatic variables in force statements.
  task force_seg(input bit wr, input bit last, input int npk, input int hd,
                           input int off, input int len, input int size);
    // write ROB only: the read ROB's SEG_* are driven by rseg through the top now
    force dut.u_wrob.SEG_LAST = last;           force dut.u_wrob.SEG_NPKTS = CNT_W'(npk);
    force dut.u_wrob.SEG_HEAD_PTR = WDB_AW'(hd);
  endtask

  // Force RESP_READY and cmpl_err on one ROB (both unwired at the top, read and write).
  task force_rsp(input bit wr, input bit ready, input bit err);
    if (!wr) begin force dut.u_rrob.RESP_READY = ready; force dut.u_rrob.cmpl_err = err; end
    else     begin force dut.u_wrob.RESP_READY = ready; force dut.u_wrob.cmpl_err = err; end
  endtask

  // Fill, backpressure, out-of-order completion, stall and in-order drain (write ROB).
  task automatic rob_forced_test(input bit wr);
    string  nm;
    int     o_idx, o_alloc, o_head, o_tail, o_occ, o_rv, o_rf, o_infl, o_out, o_np, o_sh;
    int     o_rid, o_rrsp, o_rhd, o_roff, o_rlen, o_rsz;
    int     m_n [ROB_DEPTH];
    int     m_hd [ROB_DEPTH];
    int     nslot, retired;
    nm      = wr ? "wrob" : "rrob";
    o_idx   = o_wr_idx_vld;                       o_alloc = wr ? o_wr_alloc_vld : o_rd_alloc_vld;
    o_head  = wr ? o_wr_head : o_rd_head;         o_tail  = wr ? o_wr_tail : o_rd_tail;
    o_occ   = wr ? o_wr_occ : o_rd_occ;           o_rv    = wr ? o_wr_retire_vld : o_rd_retire_vld;
    o_rf    = wr ? o_wr_retire_fire : o_rd_retire_fire;
    o_infl  = wr ? o_wr_inflow : o_rd_inflow;     o_out   = wr ? o_wr_outflow : o_rd_outflow;
    o_np    = wr ? o_wr_npkts : o_rd_npkts;       o_sh    = wr ? o_wr_shead : o_rd_shead;
    o_rid   = wr ? o_wr_rid : o_rd_rid;           o_rrsp  = wr ? o_wr_rresp : o_rd_rresp;
    o_rhd   = wr ? o_wr_rhead : o_rd_rhead;
    nslot   = wr ? (1 << WDB_AW) : (1 << RDB_AW);
    o_roff = o_rd_roff; o_rlen = o_rd_rlen; o_rsz = o_rd_rsize;

    do_reset(2);
    force_seg(wr, 1'b0, 0, 0, 0, 0, 0);
    force_rsp(wr, 1'b1, 1'b0);
    // fill the ring: admit + seal ROB_DEPTH requests, nothing completes
    for (int i = 0; i < ROB_DEPTH; i++) begin
      m_n[i]  = 1 + (i % 3);
      m_hd[i] = (3*i + 1) % nslot;
      if (wr) begin awvalid = 1; awid = AXI_IDW'(i + 1); awaddr = '0; end
      else    begin arvalid = 1; arid = AXI_IDW'(i + 1); araddr = '0; end
      #1;
      chk_segf(slice(o_alloc,1) === 1 && slice(wr ? o_wr_rob_index : o_rd_rob_index, ROB_IDX_W) === i,
               $sformatf("%s fill %0d: admit pulse at rob_index %0d", nm, i, i));
      tick();
      if (wr) awvalid = 0; else arvalid = 0;
      #1;
      chk_segf(slice(o_occ,OCC_W) === i + 1 && slice(o_idx,1) === 1 && slice(o_tail,PTR_W) === i,
               $sformatf("%s fill %0d: occ/pending_split/tail after admit", nm, i));
      force_seg(wr, 1'b1, m_n[i], m_hd[i], (i*5) % PKT_BYTES, (i*7) % 256, i % 8);
      tick();
      force_seg(wr, 1'b0, 0, 0, 0, 0, 0);
      #1;
      chk_segf(slice(o_tail,PTR_W) === (i + 1) % ROB_DEPTH && slice(o_idx,1) === 0 &&
               slice(o_occ,OCC_W) === i + 1 && slice(o_head,PTR_W) === 0,
               $sformatf("%s fill %0d: tail advances on SEG_LAST, pending_split clears", nm, i));
      chk_segf(slice(o_infl + i,1) === 1 && slice(o_np + i*CNT_W,CNT_W) === m_n[i] &&
               slice(o_sh + i*(wr ? WDB_AW : RDB_AW), wr ? WDB_AW : RDB_AW) === m_hd[i],
               $sformatf("%s fill %0d: inflow_complt/n_pkts/slot_head latched", nm, i));
      check_dft_all();
    end
    // full: no further admit
    if (wr) begin awvalid = 1; awid = 8'hEE; end else begin arvalid = 1; arid = 8'hEE; end
    #1;
    chk_segf(slice(o_occ,OCC_W) === ROB_DEPTH && slice(o_alloc,1) === 0 &&
             (wr ? awready : arready) === 1'b0, $sformatf("%s full: ready low, no admit", nm));
    tick(); #1;
    chk_segf(slice(o_occ,OCC_W) === ROB_DEPTH && slice(o_tail,PTR_W) === 0, $sformatf("%s full: refused request leaves state", nm));
    if (wr) awvalid = 0; else arvalid = 0;
    // other direction untouched
    chk_segf(slice(wr ? o_rd_occ : o_wr_occ, OCC_W) === 0 && slice(wr ? o_rd_tail : o_wr_tail, PTR_W) === 0,
             $sformatf("%s: opposite ROB state untouched", nm));
    // complete out of order (31 .. 0), odd slots carry a downstream error
    for (int sl = ROB_DEPTH - 1; sl >= 0; sl--) begin
      for (int k = 0; k < m_n[sl]; k++) begin
        force_rsp(wr, 1'b1, (sl % 2) == 1);
        cmpl_rob = ROB_IDX_W'(sl); cmpl_valid = 1;
        tick();
        cmpl_valid = 0; force_rsp(wr, 1'b1, 1'b0); #1;
        chk_segf(slice(o_out + sl*CNT_W, CNT_W) === k + 1,
                 $sformatf("%s slot %0d outflow_cnt == %0d", nm, sl, k + 1));
        if (sl > 0)
          chk_segf(slice(o_rv,1) === 0 && slice(o_head,PTR_W) === 0,
                   $sformatf("%s slot %0d done but head (slot 0) not: no retire", nm, sl));
      end
    end
    // drain in program order; check one stall cycle with RESP_READY low
    force_rsp(wr, 1'b0, 1'b0); #1;
    chk_segf(slice(o_rv,1) === 1 && slice(o_rf,1) === 0 && slice(o_head,PTR_W) === 0,
             $sformatf("%s RESP_VALID held, no retire_fire while RESP_READY low", nm));
    tick(); #1;
    chk_segf(slice(o_head,PTR_W) === 0 && slice(o_occ,OCC_W) === ROB_DEPTH, $sformatf("%s stall holds head/occ", nm));
    force_rsp(wr, 1'b1, 1'b0);
    for (retired = 0; retired < ROB_DEPTH; retired++) begin
      #1;
      chk_segf(slice(o_rv,1) === 1 && slice(o_rf,1) === 1, $sformatf("%s retire %0d: valid & fire", nm, retired));
      chk_segf(slice(o_rid,AXI_IDW) === retired + 1, $sformatf("%s retire %0d: RESP_ID in program order", nm, retired));
      chk_segf(slice(o_rrsp,2) === ((retired % 2) ? 2'b10 : 2'b00),
               $sformatf("%s retire %0d: RESP_RESP (SLVERR iff downstream error)", nm, retired));
      chk_segf(slice(o_rhd, wr ? WDB_AW : RDB_AW) === m_hd[retired], $sformatf("%s retire %0d: head ptr", nm, retired));
      if (!wr)
        chk_segf(slice(o_roff,PKT_OFF_W) === (retired*5) % PKT_BYTES && slice(o_rlen,8) === (retired*7) % 256 &&
                 slice(o_rsz,3) === retired % 8, $sformatf("%s retire %0d: offset/len/size", nm, retired));
      check_dft_all();
      tick();
    end
    #1;
    chk_segf(slice(o_occ,OCC_W) === 0 && slice(o_head,PTR_W) === 0 && slice(o_tail,PTR_W) === 0 &&
             slice(o_rv,1) === 0, $sformatf("%s drained: occ 0, head == tail == 0 after one full wrap", nm));
    release dut.u_wrob.SEG_LAST; release dut.u_wrob.SEG_NPKTS; release dut.u_wrob.SEG_HEAD_PTR;
    release dut.u_wrob.RESP_READY; release dut.u_wrob.cmpl_err;
  endtask

  // ---- integrated read path (T10): AR -> rseg queue -> walkers -> rrob via the top wiring ----
  // Downstream stubs are forced because they are unfinished upstream: ar_pkt_ready (req FIFOs),
  // ar_dbuf_base (RD-SRAM allocator) and rrob RESP_READY. Static task: force needs static vars.
  logic [N_CH-1:0]   f_rdy;
  logic [RDB_AW-1:0] f_base;
  int                rb_cnt = 0;          // batches run, makes request ids unique

  task rd_batch(input int R, input int rdy_mode, input bit ooo, input bit one_err);
    int  i, k, c, cyc, cnt, idx0, occ0, kk, nxt, nret, seg_cnt, sl, sz, ri, nslot;
    logic [AXI_AWD-1:0] sa, la, last_byte;
    logic [AXI_AWD-1:0] sa_a  [ROB_DEPTH];
    int                 sl_a  [ROB_DEPTH];
    int                 sz_a  [ROB_DEPTH];
    int                 bs_a  [ROB_DEPTH];
    int                 np_a  [ROB_DEPTH];
    bit                 ok_a  [ROB_DEPTH];
    logic [63:0]        seen  [ROB_DEPTH];
    int                 lastk [ROB_DEPTH][N_CH];
    nslot = 1 << RDB_AW;
    rb_cnt++;
    f_rdy = '0;
    force dut.ar_pkt_ready = f_rdy;
    force dut.u_rrob.RESP_READY = 1'b1;
    idx0 = int'(slice(o_rd_tail, PTR_W));
    occ0 = int'(slice(o_rd_occ, OCC_W));
    // admit R requests back-to-back with the downstream blocked
    for (i = 0; i < R; i++) begin
      sa = {$urandom, $urandom} & ((AXI_AWD'(1) << MAP_ADDR_W) - 1);
      sl = $urandom % 32;  sz = $urandom % 5;
      ok_a[i] = !(one_err && i == 1);
      if (!ok_a[i]) sa = sa | (AXI_AWD'(1) << MAP_ADDR_W);
      last_byte = ((sa >> sz) << sz) + AXI_AWD'((sl + 1) << sz) - 1;   // INCR burst byte range
      np_a[i] = int'((last_byte >> PKT_OFF_W) - (sa >> PKT_OFF_W)) + 1;
      sa_a[i] = sa;  sl_a[i] = sl;  sz_a[i] = sz;  bs_a[i] = $urandom % nslot;
      seen[i] = '0;
      for (c = 0; c < N_CH; c++) lastk[i][c] = -1;
      f_base = RDB_AW'(bs_a[i]);
      force dut.ar_dbuf_base = f_base;
      arvalid = 1; arid = AXI_IDW'(16 + i + 64*rb_cnt); araddr = sa; arlen = 8'(sl); arsize = 3'(sz);
      #1;
      chk_segf(arready === 1'b1, $sformatf("b%0d req%0d: arready high with room", rb_cnt, i));
      tick();
      chk_segf(slice(o_rd_occ, OCC_W) === occ0 + i + 1 &&
               slice(o_rd_tail, PTR_W) === (idx0 + i + 1) % ROB_DEPTH,
               $sformatf("b%0d req%0d: occ/alloc_ptr advance at admit", rb_cnt, i));
      chk_segf(slice(o_rd_npkts + ((idx0 + i) % ROB_DEPTH)*CNT_W, CNT_W) === np_a[i] &&
               slice(o_rd_inflow + ((idx0 + i) % ROB_DEPTH), 1) === 0,
               $sformatf("b%0d req%0d: n_pkts %0d written at admit, not yet sealed", rb_cnt, i, np_a[i]));
    end
    arvalid = 0;
    if (R == ROB_DEPTH) begin       // ring full: no further admit
      arvalid = 1; arid = 8'hEE; #1;
      chk_segf(arready === 1'b0 && slice(o_rd_alloc_vld, 1) === 0, $sformatf("b%0d full: arready low, no admit", rb_cnt));
      tick(); arvalid = 0; #1;
    end
    chk_segf((slice(o_rs_qa, QPTR_W) - slice(o_rs_qp, QPTR_W)) % (2*ROB_DEPTH) === R &&
             slice(o_rs_last, 1) === 0, $sformatf("b%0d: queue holds %0d, no seg_last while blocked", rb_cnt, R));
    // drain: per-channel ready pattern; check every consumed packet and every seg_last
    seg_cnt = 0;
    for (cyc = 0; cyc < 800 && seg_cnt < R; cyc++) begin
      case (rdy_mode)
        0: f_rdy = '1;
        1: f_rdy = (cyc % 2) ? '1 : N_CH'(1);
        default: f_rdy = N_CH'($urandom);
      endcase
      force dut.ar_pkt_ready = f_rdy;
      #1;
      for (c = 0; c < N_CH; c++) begin
        if (dut.u_rseg.pkt_valid[c] === 1'b1 && f_rdy[c]) begin
          ri = (int'(dut.u_rseg.pkt_rob_idx[c]) - idx0 + ROB_DEPTH) % ROB_DEPTH;
          if (ri >= R) chk_segf(0, $sformatf("b%0d: packet tagged with unexpected rob_idx %0d", rb_cnt, dut.u_rseg.pkt_rob_idx[c]));
          else begin
            kk = (int'(dut.u_rseg.pkt_dbuf_adr[c]) - bs_a[ri] + nslot) % nslot;
            la = (((sa_a[ri] >> PKT_OFF_W) + AXI_AWD'(kk)) << PKT_OFF_W);
            chk_segf(kk < np_a[ri], $sformatf("b%0d req%0d: packet index %0d < n_pkts %0d", rb_cnt, ri, kk, np_a[ri]));
            chk_segf(c == int'(ref_ch(la)), $sformatf("b%0d req%0d pkt%0d on its mapped channel", rb_cnt, ri, kk));
            chk_segf(dut.u_rseg.pkt_daddr[c] === ref_daddr(la), $sformatf("b%0d req%0d pkt%0d daddr", rb_cnt, ri, kk));
            chk_segf(kk < 64 && !seen[ri][kk], $sformatf("b%0d req%0d pkt%0d issued once", rb_cnt, ri, kk));
            // channel order: no skipped packet of this channel
            nxt = lastk[ri][c] + 1;
            while (nxt < np_a[ri] && int'(ref_ch(((sa_a[ri] >> PKT_OFF_W) + AXI_AWD'(nxt)) << PKT_OFF_W)) != c) nxt++;
            chk_segf(kk == nxt, $sformatf("b%0d req%0d ch%0d in order (got %0d want %0d)", rb_cnt, ri, c, kk, nxt));
            if (kk < 64) seen[ri][kk] = 1'b1;
            lastk[ri][c] = kk;
          end
        end
      end
      if (dut.u_rseg.seg_last === 1'b1) begin
        ri = (int'(dut.u_rseg.seg_rob_idx) - idx0 + ROB_DEPTH) % ROB_DEPTH;
        chk_segf(ri == seg_cnt, $sformatf("b%0d: seg_last #%0d is request %0d (program order)", rb_cnt, seg_cnt, ri));
        if (ri < R) begin
          chk_segf(seen[ri] === ((64'd1 << np_a[ri]) - 1), $sformatf("b%0d req%0d: all packets issued before seg_last", rb_cnt, ri));
          chk_segf(dut.u_rseg.seg_npkts === CNT_W'(np_a[ri]) && dut.u_rseg.seg_offset === PKT_OFF_W'(sa_a[ri][PKT_OFF_W-1:0]) &&
                   dut.u_rseg.seg_len === 8'(sl_a[ri]) && dut.u_rseg.seg_size === 3'(sz_a[ri]) &&
                   dut.u_rseg.seg_head_ptr === RDB_AW'(bs_a[ri]), $sformatf("b%0d req%0d: seg_* bundle", rb_cnt, ri));
        end
        seg_cnt++;
      end
      check_dft_all();
      tick();
    end
    chk_segf(seg_cnt == R, $sformatf("b%0d: all %0d requests segmented", rb_cnt, R));
    chk_segf(slice(o_rs_qa, QPTR_W) === slice(o_rs_qp, QPTR_W), $sformatf("b%0d: queue drained", rb_cnt));
    if (R == ROB_DEPTH) begin       // queue has room but the ROB is full: ROB must backpressure
      arvalid = 1; arid = 8'hED; #1;
      chk_segf(slice(o_rs_ready, 1) === 1 && arready === 1'b0 && slice(o_rd_alloc_vld, 1) === 0,
               $sformatf("b%0d: ROB-full backpressure with an empty queue", rb_cnt));
      tick(); arvalid = 0; #1;
    end
    // completions from the MC side through the real port; out of order if requested
    nret = 0;
    for (i = 0; i < R; i++) begin
      ri = ooo ? (R - 1 - i) : i;
      for (k = 0; k < np_a[ri]; k++) begin
        cmpl_rob = ROB_IDX_W'((idx0 + ri) % ROB_DEPTH); cmpl_valid = 1;
        tick(); cmpl_valid = 0; #1;
        if (slice(o_rd_retire_vld, 1) === 1 && slice(o_rd_retire_fire, 1) === 1) begin
          chk_segf(slice(o_rd_rid, AXI_IDW) === (16 + nret + 64*rb_cnt) % 256, $sformatf("b%0d retire %0d: RESP_ID in program order", rb_cnt, nret));
          chk_segf(slice(o_rd_rresp, 2) === (ok_a[nret] ? 2'b00 : 2'b10), $sformatf("b%0d retire %0d: RESP_RESP", rb_cnt, nret));
          chk_segf(slice(o_rd_rhead, RDB_AW) === bs_a[nret] && slice(o_rd_roff, PKT_OFF_W) === sa_a[nret][PKT_OFF_W-1:0] &&
                   slice(o_rd_rlen, 8) === sl_a[nret] && slice(o_rd_rsize, 3) === sz_a[nret],
                   $sformatf("b%0d retire %0d: resp meta", rb_cnt, nret));
          nret++;
        end
      end
    end
    // let the remaining retirements ripple out (one per cycle)
    for (cyc = 0; cyc < ROB_DEPTH + 4 && nret < R; cyc++) begin
      tick();   // the entry sampled last cycle retires at this edge; look at the next head
      if (slice(o_rd_retire_vld, 1) === 1 && slice(o_rd_retire_fire, 1) === 1) begin
        chk_segf(slice(o_rd_rid, AXI_IDW) === (16 + nret + 64*rb_cnt) % 256, $sformatf("b%0d retire %0d: RESP_ID in program order", rb_cnt, nret));
        chk_segf(slice(o_rd_rresp, 2) === (ok_a[nret] ? 2'b00 : 2'b10), $sformatf("b%0d retire %0d: RESP_RESP", rb_cnt, nret));
        chk_segf(slice(o_rd_rhead, RDB_AW) === bs_a[nret] && slice(o_rd_roff, PKT_OFF_W) === sa_a[nret][PKT_OFF_W-1:0] &&
                 slice(o_rd_rlen, 8) === sl_a[nret] && slice(o_rd_rsize, 3) === sz_a[nret],
                 $sformatf("b%0d retire %0d: resp meta", rb_cnt, nret));
        nret++;
      end
      check_dft_all();
    end
    chk_segf(nret == R, $sformatf("b%0d: all %0d requests retired", rb_cnt, R));
    tick();   // the last retire fires at this edge
    chk_segf(slice(o_rd_occ, OCC_W) === occ0 && slice(o_rd_tail, PTR_W) === (idx0 + R) % ROB_DEPTH &&
             slice(o_rd_head, PTR_W) === (idx0 + R) % ROB_DEPTH, $sformatf("b%0d: ROB back to idle", rb_cnt));
    release dut.ar_pkt_ready; release dut.ar_dbuf_base; release dut.u_rrob.RESP_READY;
  endtask

  // ---- main ----
  logic [AXI_AWD-1:0] a_w, a_r;
  int i;

  initial begin : watchdog
    #2000000;
    $display("TB RESULT: FAIL (watchdog timeout)");
    $fatal(1, "watchdog");
  end

  initial begin : main
    build_layout();

    // ---- bus layout / width -------------------------------------------------
    // Independent width sums, corner/alt elaboration, concatenation checker; all IMPL.
    chk_impl(cur == OBS_W_EXP,             "TB layout total == expected sum");
    chk_impl($bits(dft_obs) == OBS_W_EXP,  "rmc_cif dft_obs width == expected sum");
    chk_impl($bits(dft_obs_min) == MIN_OBS_W,
             "guard corner (ROB_DEPTH=1, MAX_PKTS=1) elaborates with expected dft_obs width");
    chk_impl($bits(dft_obs_alt) == ALT_OBS_W, "alt config (D=3,P=3,WDB=8,RDB=64) dft_obs width");
    chk_impl($bits(dut_alt.u_rrob.RESP_HEAD_PTR) == 6 && $bits(dut_alt.u_rseg.seg_head_ptr) == 6 &&
             $bits(dut_alt.u_rseg.pkt_dbuf_adr) == N_CH*6,
             "alt config: read slot widths follow N_RDB (6)");
    chk_impl($bits(dut_alt.u_wrob.RESP_HEAD_PTR) == 3 && $bits(dut_alt.u_wseg.SEG_HEAD_PTR) == 3 &&
             $bits(dut_alt.u_wseg.packet_slot) == N_CH*3,
             "alt config: write slot widths follow N_WDB (3)");
    chk_impl($bits(dut_alt.async_mc_req_sram_slot) == 6, "alt config: req sram_slot is max(WDB,RDB) wide");
    #30;   // layout checkers run at t=0..20
    for (int c = 0; c < N_LCHK; c++)
      chk_impl(^lc_err_v[c*32 +: 32] !== 1'bx && lc_err_v[c*32 +: 32] == 0 &&
               lc_it_v[c*32 +: 32] == 20 && lc_bus_v[c*32 +: 32] == lc_cat_v[c*32 +: 32],
               $sformatf("layout checker %0d: bus == concat of inputs (bus %0d, concat %0d, %0d mismatches)",
                         c, lc_bus_v[c*32 +: 32], lc_cat_v[c*32 +: 32], lc_err_v[c*32 +: 32]));
    chk_impl(lc_bus_v[0 +: 32] == OBS_W_EXP && lc_bus_v[32 +: 32] == MIN_OBS_W,
             "layout checker widths agree with the TB's independent width sum");
    chk_impl(OBS_W_EXP == 3196, "default-config dft_obs width is 3196 (3115 before the read-path rework)");
    // default-config widths named in the task
    chk_impl(DADDR_W == 34 && CH_W == 1 && PKT_OFF_W == 6 && DEC_W == 41 && MAP_SEL_W == 6,
             "package widths DADDR_W/CH_W/PKT_OFF_W/DEC_W/MAP_SEL_W");
    chk_impl(AXI_DW == 128 && SRAMW == 512 && ROB_IDX_W == 5 && PKT_NUM_W == 4 && SLOT_W == 5,
             "AXI_DW/SRAM_W/ROB_IDX_W/PKT_NUM_W/SLOT_W");
    chk_impl(SRAMW == PKT_BYTES*8, "SRAM_W: BL*DQ_W == PKT_BYTES*8");

    // ---- T1 reset -----------------------------------------------------------
    // Reset state of both ROBs, tied-off outputs and taps while inputs are active.
    aresetn = 0; idle_inputs();
    // activity while in reset: random-ish AXI and completion inputs
    awid = 8'h5A; awaddr = 48'h1234; arid = 8'hC3; araddr = 48'h8888;
    cmpl_rob = 5'd3; rdb_dout = {SRAMW{1'b1}};
    repeat (3) tick();
    #1;
    check_rob_reset("in reset");
    check_tied("in reset");
    chk_impl(awready === 1'b1 && arready === 1'b1,
             "in reset: awready/arready high (ROB not full, not mid-split)");
    check_dft_all();
    aresetn = 1; idle_inputs(); tick();
    check_rob_reset("after reset release");
    check_tied("after reset release");
    chk_impl(slice(o_aresetn,1) === 1, "dft aresetn bit high after release");
    check_dft_all();

    // ---- T2 address map: IMPL ----------------------------------------------
    // daddr/ch/off and range check vs. a package-position model (shares package constants).
    chk_impl(dut.addr_map === rmc_cfg_pkg::default_addr_map(),
             "addr_map equals default_addr_map() (map is constant; CSR drive is TODO)");
    // single-bit sweeps, AW and AR carry different addresses to catch cross-wiring
    for (i = 0; i < AXI_AWD; i++) begin
      a_w = AXI_AWD'(1) << i;
      a_r = AXI_AWD'(1) << (AXI_AWD-1-i);
      awaddr = a_w; araddr = a_r; #1;
      chk_impl(dut.aw_daddr === ref_daddr(a_w), $sformatf("aw_daddr single bit %0d", i));
      chk_impl(dut.aw_ch    === ref_ch(a_w),    $sformatf("aw_ch single bit %0d", i));
      chk_impl(dut.aw_off   === ref_off(a_w),   $sformatf("aw_off single bit %0d", i));
      chk_impl(dut.ar_daddr === ref_daddr(a_r), $sformatf("ar_daddr single bit %0d", AXI_AWD-1-i));
      chk_impl(dut.ar_ch    === ref_ch(a_r),    $sformatf("ar_ch single bit %0d", AXI_AWD-1-i));
      chk_impl(dut.ar_off   === ref_off(a_r),   $sformatf("ar_off single bit %0d", AXI_AWD-1-i));
      check_dft_all();
    end
    // boundaries + random; AW == AR for the same address
    begin
      logic [AXI_AWD-1:0] t [0:8];
      t[0] = '0;
      t[1] = (AXI_AWD'(1) << MAP_ADDR_W) - 1;       // last mapped byte
      t[2] =  AXI_AWD'(1) << MAP_ADDR_W;            // first unmapped
      t[3] = '1;
      t[4] = (AXI_AWD'(1) << MAP_ADDR_W) - PKT_BYTES;
      t[5] =  AXI_AWD'(PKT_BYTES) - 1;              // last byte of packet 0
      t[6] =  AXI_AWD'(PKT_BYTES);                  // first byte of packet 1
      t[7] =  AXI_AWD'(1) << ROW_POS;               // lowest row bit
      t[8] =  AXI_AWD'(1) << (MAP_ADDR_W-1);        // highest row bit
      for (i = 0; i < 9 + 200; i++) begin
        a_w = (i < 9) ? t[i] : {$urandom, $urandom};
        awaddr = a_w; araddr = a_w; #1;
        chk_impl(dut.aw_daddr === ref_daddr(a_w), $sformatf("aw_daddr %h", a_w));
        chk_impl(dut.ar_daddr === ref_daddr(a_w), $sformatf("ar_daddr %h", a_w));
        chk_impl(dut.aw_daddr === dut.ar_daddr && dut.aw_ch === dut.ar_ch &&
                 dut.aw_off === dut.ar_off, $sformatf("AW/AR map paths agree %h", a_w));
        chk_impl(dut.aw_ch === ref_ch(a_w) && dut.aw_off === ref_off(a_w),
                 $sformatf("aw_ch/aw_off %h", a_w));
        // ---- request validation (range check only) ----
        chk_impl(dut.aw_ok === ref_ok(a_w), $sformatf("aw_ok %h", a_w));
        chk_impl(dut.ar_ok === ref_ok(a_w), $sformatf("ar_ok %h", a_w));
        if (i < 9) check_dft_all();
      end
      // validator ignores len/size today: ok must not depend on them
      awaddr = '0; awlen = 8'hFF; awsize = 3'd7; #1;
      chk_impl(dut.aw_ok === 1'b1, "aw_ok ignores len/size (size/burst-end checks are TODO)");
      awaddr = (AXI_AWD'(1) << MAP_ADDR_W) - 1; awlen = 8'hFF; awsize = 3'd7; #1;
      chk_impl(dut.aw_ok === 1'b1,
               "aw_ok stays 1 when burst crosses the range end (burst-end check not implemented)");
      awlen = '0; awsize = '0; awaddr = '0; araddr = '0;
    end
    blocked("request validation: illegal size and burst-end-out-of-range checks are TODO in rmc_cif_reqvalidator");

    // ---- T3 AXI admission (AW and AR independent) ---------------------------
    // One admit per ROB through the real AXI ports. Write: pending_split holds ready low;
    // read: admit is decoupled from the split (ready stays high).
    do_reset(2);
    check_rob_reset("T3 start");
    // ready does not depend on valid: high with valid low
    chk_impl(awready === 1'b1 && arready === 1'b1, "ready high with valid low (not valid-dependent)");
    // present an AW request; admission is combinational on (valid & ready)
    awvalid = 1; awid = 8'hA5; awaddr = 48'h0000_0000_0040; awlen = 8'd3; awsize = 3'd2; #1;
    chk_impl(slice(o_wr_alloc_vld,1) === 1, "write alloc_vld (admit_fire) pulses when AW accepted");
    chk_impl(slice(o_rd_alloc_vld,1) === 0, "read alloc_vld stays 0 on an AW request");
    chk_impl(slice(o_awready,1) === 1,      "dft awready == 1 on admit cycle");
    tick(); awvalid = 0; #1;
    chk_impl(slice(o_wr_occ, OCC_W) === 1,  "write occ == 1 after AW admit");
    chk_impl(slice(o_wr_idx_vld,1) === 1,   "write idx_vld (pending_split) set after admit");
    chk_impl(slice(o_wr_tail, PTR_W) === 0 && slice(o_wr_head, PTR_W) === 0,
             "write head/tail unchanged (alloc_ptr only advances on SEG_LAST)");
    chk_impl(dut.u_wrob.ent_id[0] === 8'hA5, "write ROB slot0 holds the AW id");
    chk_impl(dut.u_wrob.ent_err[0] === 1'b0, "write ROB slot0 err == 0 for an in-range address");
    chk_impl(awready === 1'b0,              "awready low while a request is mid-split");
    chk_impl(slice(o_rd_occ, OCC_W) === 0 && arready === 1'b1,
             "read ROB untouched by an AW admit; arready still high");
    // keep offering AW: no second admission is possible without SEG_LAST
    awvalid = 1; awid = 8'h11; #1;
    chk_impl(slice(o_wr_alloc_vld,1) === 0, "no second write admit while pending_split");
    tick(); tick(); #1;
    chk_impl(slice(o_wr_occ, OCC_W) === 1, "write occ stays 1 under held AW valid");
    chk_impl(dut.u_wrob.ent_id[0] === 8'hA5, "slot0 id not overwritten by blocked AW");
    awvalid = 0;
    // read side admits independently
    arvalid = 1; arid = 8'h3C; araddr = 48'h0000_0000_1000; arlen = 8'd7; arsize = 3'd3; #1;
    chk_impl(slice(o_rd_alloc_vld,1) === 1, "read alloc_vld pulses when AR accepted");
    chk_impl(slice(o_wr_alloc_vld,1) === 0, "write alloc_vld low on an AR request");
    tick(); arvalid = 0; #1;
    chk_impl(slice(o_rd_occ, OCC_W) === 1 && slice(o_wr_occ, OCC_W) === 1,
             "read and write occ both 1 (separate state)");
    chk_impl(dut.u_rrob.ent_id[0] === 8'h3C, "read ROB slot0 holds the AR id");
    chk_impl(arready === 1'b1 && slice(o_rd_tail, PTR_W) === 1 && slice(o_rd_head, PTR_W) === 0,
             "read: arready stays high, alloc_ptr advanced at admit (multi-outstanding)");
    chk_impl(slice(o_rd_npkts, CNT_W) === 1, "read n_pkts[0] written at admit (64B request == 1 packet)");
    check_dft_all();
    check_tied("after admits");
    blocked("write ROB full backpressure / slot allocation: wseg is a stub so SEG_LAST never arrives (covered only with forced pins, T11)");

    // ---- T4 err-tagged admission -------------------------------------------
    // Out-of-range requests are still admitted and carry the error flag.
    do_reset(2);
    awvalid = 1; awid = 8'h77; awaddr = AXI_AWD'(1) << MAP_ADDR_W; #1;
    chk_impl(dut.aw_ok === 1'b0, "T4 aw_ok low for out-of-range address");
    chk_impl(slice(o_wr_alloc_vld,1) === 1, "out-of-range request is still admitted (rides through with err)");
    tick(); awvalid = 0; #1;
    chk_impl(dut.u_wrob.ent_err[0] === 1'b1, "write ROB slot0 err set for out-of-range request");
    chk_impl(dut.u_wrob.RESP_RESP === 2'b10,
             "write ROB RESP_RESP == SLVERR for the err entry at head (RESP_VALID itself stays blocked)");
    arvalid = 1; arid = 8'h78; araddr = {AXI_AWD{1'b1}}; #1;
    tick(); arvalid = 0; #1;
    chk_impl(dut.u_rrob.ent_err[0] === 1'b1, "read ROB slot0 err set for out-of-range request");
    check_dft_all();

    // ---- T5 completion observations (current, shared behaviour) -------------
    // Completion pulses bump outflow_cnt in both ROBs (no direction routing yet).
    do_reset(2);
    awvalid = 1; awid = 8'h01; awaddr = '0; #1; tick(); awvalid = 0;
    arvalid = 1; arid = 8'h02; araddr = '0; #1; tick(); arvalid = 0; #1;
    chk_impl(outflow_of(1,0) === 0 && outflow_of(0,0) === 0,
             "slot0 outflow_cnt cleared on admit (both ROBs)");
    cmpl_rob = '0; cmpl_valid = 1; #1;
    chk_impl(cmpl_ready === 1'b1, "completion ready high");
    tick(); cmpl_valid = 0; #1;
    chk_impl(outflow_of(0,0) === 1, "read ROB outflow_cnt[0] == 1 after one completion pulse");
    chk_impl(outflow_of(1,0) === 1,
             "write ROB outflow_cnt[0] == 1 after one completion pulse (OBSERVED: both ROBs count; direction routing is TODO)");
    cmpl_valid = 1; tick(); cmpl_valid = 0; #1;
    chk_impl(outflow_of(0,0) === 2 && outflow_of(1,0) === 2, "outflow_cnt[0] == 2 after second pulse");
    tick(); tick(); #1;
    chk_impl(outflow_of(0,0) === 2 && outflow_of(1,0) === 2, "outflow_cnt holds with no completion pulse");
    chk_impl(slice(o_rd_retire_vld,1) === 0 && slice(o_wr_retire_vld,1) === 0,
             "OBSERVED: retire_vld stays 0 (nothing seals an entry: read packets never drain, wseg stub)");
    chk_impl(slice(o_rd_inflow,1) === 0 && slice(o_wr_inflow,1) === 0,
             "OBSERVED: inflow_complt[0] == 0 (never set)");
    chk_impl(slice(o_rd_occ, OCC_W) === 1 && slice(o_wr_occ, OCC_W) === 1, "occ stays 1: nothing retires");
    chk_impl(slice(o_rd_head, PTR_W) === 0 && slice(o_wr_head, PTR_W) === 0, "head does not move");
    check_dft_all();
    check_tied("after completions");
    blocked("retire at the integrated top: read needs pkt_ready/dbuf_base/RESP_READY (forced in T10), write needs wseg (forced in T11)");
    blocked("completion routing by direction (R vs W ROB): not implemented, both ROBs count every completion");
    blocked("completion error tagging: cmpl_err floating at the CIF (not part of the pulse+rob_index interface)");

    // ---- T6 reset during activity (asynchronous assert mid-cycle) ------------
    // Pointers/occupancy/pending clear asynchronously; entry arrays are not reset.
    awvalid = 1; arvalid = 1; cmpl_valid = 1; awid = 8'hEE;
    #3;  // mid-cycle, away from the clock edge
    aresetn = 0; #1;
    check_rob_reset("async reset mid-cycle");
    chk_impl(awready === 1'b1 && arready === 1'b1, "ready recovers immediately on async reset");
    tick(); tick();
    awvalid = 0; arvalid = 0; cmpl_valid = 0;
    aresetn = 1; tick();
    check_rob_reset("after reset during activity");
    check_tied("after reset during activity");
    blocked("outflow_cnt/inflow state is NOT cleared by reset (no reset on ROB entry arrays); only ptr/occ/pending are");

    // ---- T7 SRAM ports (tied-off only) --------------------------------------
    // Taps mirror rdb_dout; wdb/rdb outputs are only confirmed constant.
    rdb_dout = {SRAMW{1'b1}}; tick();
    check_tied("rdb_dout driven high");
    cmp("rdb_dout mirror", o_rdb_dout, SRAMW, {4096{1'b1}});
    rdb_dout = $urandom; tick();
    check_dft_all();
    wvalid = 1; wstrb = '1; wlast = 1; wdata = {AXI_DW{1'b1}}; tick();
    chk_tied(wready === 1'b0 && wdb_we === 1'b0, "W beat offered: wready/wdb_we stay 0 (W path not implemented)");
    wvalid = 0; wlast = 0;
    blocked("WD_SRAM/RD_SRAM functionality: SRAM is not instantiated in the CIF; wdb_*/rdb_* are tied off");
    blocked("W channel accept, B response, R response: wready/bvalid/rvalid tied low");

    // ---- T8 CIF-to-MC request path (tied-off only) --------------------------
    // The request builder is TODO, so this only confirms constant outputs.
    req_ready = 1; awvalid = 1; arvalid = 1; tick(); tick();
    check_tied("req_ready high with requests offered");
    awvalid = 0; arvalid = 0; req_ready = 0;
    blocked("request generation (daddr/pkt_num/last_in_txn/sram_slot/op on async_mc_req_*): builder not implemented");
    blocked("write segmenter: rmc_cif_wseg is a stub (all outputs tied 0), no free-list/chain logic to test");
    blocked("order_q / inflow_cnt / ent_valid: no such signals in rmc_cif_rrob/wrob (not tapped; n_pkts/inflow_complt/slot_head are)");
    blocked("runtime address-map programming: addr_map is a constant (CSR has no outputs)");

    // ---- T9 randomized tap-correspondence + functional non-interference ------
    // Random top-level stimulus, every tap vs. source; the ready check mirrors the RTL (sanity only).
    do_reset(2);
    for (i = 0; i < 400; i++) begin
      awvalid = ($urandom % 4) == 0;  arvalid = ($urandom % 4) == 0;
      awid = $urandom; arid = $urandom;
      awaddr = {$urandom, $urandom}; araddr = {$urandom, $urandom};
      awlen = $urandom; arlen = $urandom; awsize = $urandom; arsize = $urandom;
      wvalid = $urandom; wstrb = {$urandom, $urandom, $urandom, $urandom};
      wlast = $urandom; bready = $urandom; rready = $urandom;
      cmpl_valid = ($urandom % 3) == 0; cmpl_rob = $urandom;
      req_ready = $urandom; rdb_dout = {16{$urandom}};
      #1;
      check_dft_all();
      check_tied("random");
      // functional non-interference: ready comes only from the ROBs' own state
      chk_impl(awready === (~dut.u_wrob.rob_full & ~dut.u_wrob.pending_split),
               "awready equals the write ROB admit condition");
      chk_impl(arready === (dut.u_rseg.ready & ~dut.u_rrob.rob_full),
               "arready equals rseg ready & read ROB not full");
      // occupancy bounded and defined
      chk_impl(^slice(o_rd_occ, OCC_W) !== 1'bx && ^slice(o_wr_occ, OCC_W) !== 1'bx, "occ defined");
      tick();
      if (i == 150 || i == 300) begin aresetn = 0; #2; aresetn = 1; end
    end

    // ---- T10 integrated read path, downstream stubs forced (SEGF) ----------------
    // Real AR -> queue -> walkers -> rrob; only pkt_ready, dbuf_base, RESP_READY forced.
    // First, unforced: pkt_ready is undriven upstream, so the packet is never consumed.
    do_reset(2);
    check_rob_reset("T10 start");
    arvalid = 1; arid = 8'h5A; araddr = 48'h40; arlen = 8'd3; arsize = 3'd4; #1;
    tick(); arvalid = 0;
    repeat (8) tick();
    #1;
    chk_impl(slice(o_rd_occ, OCC_W) === 1 && (slice(o_rs_qa, QPTR_W) - slice(o_rs_qp, QPTR_W)) === 1,
             "OBSERVED: unforced AR admitted into the ROB and the rseg queue");
    chk_impl(slice(o_rs_pv, N_CH) === (N_CH'(1) << int'(ref_ch(48'h40))),
             "OBSERVED: the owning channel offers its packet");
    chk_impl(slice(o_rs_last, 1) === 0 && slice(o_rd_inflow, 1) === 0 && slice(o_rs_em, N_CH*CNT_W) === 0 &&
             slice(o_rd_retire_vld, 1) === 0,
             "OBSERVED: packet never consumed (pkt_ready undriven): no seg_last, no seal, no retire");
    do_reset(2);
    rd_batch(1, 0, 0, 0);
    rd_batch(1, 1, 0, 0);
    rd_batch(1, 2, 0, 0);
    rd_batch(6, 0, 0, 1);       // multi-outstanding, one out-of-range (SLVERR)
    rd_batch(6, 2, 1, 0);       // random ready, out-of-order completion
    rd_batch(ROB_DEPTH, 1, 1, 0);  // fill the ring (backpressure), drain, wrap
    rd_batch(5, 2, 0, 1);
    rd_batch(ROB_DEPTH, 2, 0, 0);
    blocked("read path to MC: pkt_* has no req FIFO/arbiter, so rseg packets are consumed only by the forced ar_pkt_ready");
    blocked("read dbuf: RD-SRAM allocator absent (ar_dbuf_base forced) and dbuf_adr_ready is tied high upstream");
    blocked("requests longer than MAX_PKTS: SEG_NPKTS truncates silently, validator burst-end check is TODO (not tested)");

    // ---- T11 write ROB sequence with forced SEG_*/RESP_READY (SEGF) ------------------
    // Gives the write-ROB taps non-trivial values (wseg is a stub); unit level, not integration.
    rob_forced_test(1'b1);
    blocked("write ROB<->segmenter handoff and B response: exercised only with TB-forced pins; not an integration test");

    idle_inputs();

    // ---- summary ------------------------------------------------------------
    // PASS needs zero failures in IMPL/TIED/SEGF/DFT; BLOCKED items never count as pass.
    tick();
    $display("---------------------------------------------------------------");
    $display("IMPL  checks : %0d pass, %0d fail", n_impl_pass, n_impl_fail);
    $display("TIED  checks : %0d pass, %0d fail  (confirm tied-off behaviour only)", n_tied_pass, n_tied_fail);
    $display("SEGF  checks : %0d pass, %0d fail  (read path + write ROB, forced pins)", n_segf_pass, n_segf_fail);
    $display("DFT   checks : %0d pass, %0d fail  (bus slice == source net)", n_dft_pass, n_dft_fail);
    $display("BLOCKED items: %0d  (RTL incomplete; NOT counted as pass)", n_blocked);
    if (n_impl_fail + n_tied_fail + n_dft_fail + n_segf_fail == 0) begin
      $display("TB RESULT: PASS");
      $finish;
    end else begin
      $display("TB RESULT: FAIL");
      $fatal(1, "testbench failures");
    end
  end

endmodule
