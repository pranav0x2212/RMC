// rmc_cif_dft.sv: passive DFT tap block, no state; each field is one slice assign.
// Packed LSB-first in listed order (O_* offsets); per-slot / per-channel fields are
// slot- / channel-major. Width: rmc_cif_dft_pkg::dft_obs_w(). wseg fields are stub zeros.

module rmc_cif_dft import rmc_cfg_pkg::*; #(
  parameter int AXI_IDW   = 8,
  parameter int AXI_AW    = 48,
  parameter int ROB_DEPTH = 32,
  parameter int MAX_PKTS  = 16,
  parameter int N_WDB     = 32,
  parameter int N_RDB     = 32,

  // derived widths: guarded so a parameter of 1 never gives a zero-width field
  localparam int ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int PTR_W     = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1,
  localparam int QPTR_W    = PTR_W + 1,
  localparam int OCC_W     = $clog2(ROB_DEPTH + 1),
  localparam int CNT_W     = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1,
  localparam int PKT_NUM_W = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS)   : 1,
  localparam int WDB_AW    = (N_WDB     > 1) ? $clog2(N_WDB)      : 1,
  localparam int RDB_AW    = (N_RDB     > 1) ? $clog2(N_RDB)      : 1,
  localparam int SLOT_W    = (WDB_AW > RDB_AW) ? WDB_AW : RDB_AW,   // widest SRAM slot id
  localparam int SRAM_W    = BL * DQ_W,
  localparam int OBS_W     = rmc_cif_dft_pkg::dft_obs_w(AXI_IDW, AXI_AW, ROB_DEPTH,
                                                        MAX_PKTS, N_WDB, N_RDB)
)(
  // Tap inputs, grouped and ordered as in the bus.
  // AXI ingress
  input  logic [1-1:0] awvalid,
  input  logic [1-1:0] awready,
  input  logic [AXI_AW-1:0] awaddr,
  input  logic [8-1:0] awlen,
  input  logic [3-1:0] awsize,
  input  logic [AXI_IDW-1:0] awid,
  input  logic [1-1:0] arvalid,
  input  logic [1-1:0] arready,
  input  logic [AXI_AW-1:0] araddr,
  input  logic [8-1:0] arlen,
  input  logic [3-1:0] arsize,
  input  logic [AXI_IDW-1:0] arid,
  input  logic [1-1:0] wvalid,
  input  logic [1-1:0] wready,
  input  logic [AXI_DW/8-1:0] wstrb,
  input  logic [1-1:0] wlast,
  input  logic [1-1:0] bvalid,
  input  logic [1-1:0] bready,
  input  logic [1-1:0] rvalid,
  input  logic [1-1:0] rready,
  // Address map
  input  logic [DADDR_W-1:0] aw_daddr,
  input  logic [DADDR_W-1:0] ar_daddr,
  input  logic [CH_W-1:0] aw_ch,
  input  logic [CH_W-1:0] ar_ch,
  input  logic [PKT_OFF_W-1:0] aw_off,
  input  logic [PKT_OFF_W-1:0] ar_off,
  input  logic [DEC_W*MAP_SEL_W-1:0] addr_map,
  // Request validation
  input  logic [1-1:0] aw_ok,
  input  logic [1-1:0] ar_ok,
  // Read ROB (u_rrob)
  input  logic [ROB_IDX_W-1:0] rd_rob_index,
  input  logic [1-1:0] rd_rob_ready,
  input  logic [1-1:0] rd_alloc_vld,
  input  logic [PTR_W-1:0] rd_head,
  input  logic [PTR_W-1:0] rd_tail,
  input  logic [OCC_W-1:0] rd_occ,
  input  logic [1-1:0] rd_retire_vld,
  input  logic [1-1:0] rd_retire_fire,
  input  logic [ROB_DEPTH-1:0] rd_inflow_complt,
  input  logic [ROB_DEPTH*CNT_W-1:0] rd_outflow_cnt,
  input  logic [ROB_DEPTH*CNT_W-1:0] rd_n_pkts,
  input  logic [ROB_DEPTH-1:0] rd_ent_err,
  input  logic [ROB_DEPTH-1:0] rd_ent_derr,
  input  logic [ROB_DEPTH*RDB_AW-1:0] rd_slot_head,
  input  logic [AXI_IDW-1:0] rd_resp_id,
  input  logic [2-1:0] rd_resp_resp,
  input  logic [RDB_AW-1:0] rd_resp_head,
  input  logic [PKT_OFF_W-1:0] rd_resp_offset,
  input  logic [8-1:0] rd_resp_len,
  input  logic [3-1:0] rd_resp_size,
  // Write ROB (u_wrob)
  input  logic [ROB_IDX_W-1:0] wr_rob_index,
  input  logic [1-1:0] wr_idx_vld,
  input  logic [1-1:0] wr_alloc_vld,
  input  logic [PTR_W-1:0] wr_head,
  input  logic [PTR_W-1:0] wr_tail,
  input  logic [OCC_W-1:0] wr_occ,
  input  logic [1-1:0] wr_retire_vld,
  input  logic [1-1:0] wr_retire_fire,
  input  logic [ROB_DEPTH-1:0] wr_inflow_complt,
  input  logic [ROB_DEPTH*CNT_W-1:0] wr_outflow_cnt,
  input  logic [ROB_DEPTH*CNT_W-1:0] wr_n_pkts,
  input  logic [ROB_DEPTH-1:0] wr_ent_err,
  input  logic [ROB_DEPTH-1:0] wr_ent_derr,
  input  logic [ROB_DEPTH*WDB_AW-1:0] wr_slot_head,
  input  logic [AXI_IDW-1:0] wr_resp_id,
  input  logic [2-1:0] wr_resp_resp,
  input  logic [WDB_AW-1:0] wr_resp_head,
  // Read segmenter (u_rseg)
  input  logic [1-1:0] rs_ready,
  input  logic [QPTR_W-1:0] rs_q_alloc,
  input  logic [QPTR_W-1:0] rs_q_pop,
  input  logic [ROB_DEPTH*N_CH-1:0] rs_q_ch_done,
  input  logic [N_CH*QPTR_W-1:0] rs_eng_rd_ptr,
  input  logic [N_CH*CNT_W-1:0] rs_eng_m,
  input  logic [RDB_AW-1:0] rs_req_dbuf_base,
  input  logic [1-1:0] rs_seg_last,
  input  logic [ROB_IDX_W-1:0] rs_seg_rob_idx,
  input  logic [CNT_W-1:0] rs_seg_npkts,
  input  logic [PKT_OFF_W-1:0] rs_seg_offset,
  input  logic [8-1:0] rs_seg_len,
  input  logic [3-1:0] rs_seg_size,
  input  logic [RDB_AW-1:0] rs_seg_head,
  input  logic [N_CH-1:0] rs_pkt_valid,
  input  logic [N_CH-1:0] rs_pkt_ready,
  input  logic [N_CH-1:0] rs_pkt_dbuf_rdy,
  input  logic [N_CH*ROB_IDX_W-1:0] rs_pkt_rob_idx,
  input  logic [N_CH*DADDR_W-1:0] rs_pkt_daddr,
  input  logic [N_CH*RDB_AW-1:0] rs_pkt_dbuf_adr,
  // Write segmenter (u_wseg, stub)
  input  logic [1-1:0] ws_seg_last,
  input  logic [CNT_W-1:0] ws_seg_npkts,
  input  logic [WDB_AW-1:0] ws_seg_head,
  input  logic [1-1:0] ws_free_slot_take,
  input  logic [N_CH-1:0] ws_pkt_valid,
  input  logic [N_CH-1:0] ws_pkt_ready,
  input  logic [N_CH*ROB_IDX_W-1:0] ws_pkt_rob_index,
  input  logic [N_CH*DADDR_W-1:0] ws_pkt_daddr,
  input  logic [N_CH*PKT_NUM_W-1:0] ws_pkt_num,
  input  logic [N_CH-1:0] ws_pkt_last,
  input  logic [N_CH*WDB_AW-1:0] ws_pkt_slot,
  input  logic [N_CH*PKT_OFF_W-1:0] ws_pkt_offset,
  input  logic [N_CH-1:0] ws_pkt_narrow,
  // CIF-to-MC async FIFO boundary
  input  logic [1-1:0] async_mc_req_valid,
  input  logic [1-1:0] async_mc_req_ready,
  input  logic [ROB_IDX_W-1:0] async_mc_req_rob_index,
  input  logic [1-1:0] async_mc_req_op,
  input  logic [DADDR_W-1:0] async_mc_req_daddr,
  input  logic [PKT_NUM_W-1:0] async_mc_req_pkt_num,
  input  logic [1-1:0] async_mc_req_last_in_txn,
  input  logic [SLOT_W-1:0] async_mc_req_sram_slot,
  input  logic [1-1:0] async_mc_cmpl_valid,
  input  logic [1-1:0] async_mc_cmpl_ready,
  input  logic [ROB_IDX_W-1:0] async_mc_cmpl_rob_index,
  // Data SRAM client ports
  input  logic [1-1:0] wdb_we,
  input  logic [WDB_AW-1:0] wdb_addr,
  input  logic [SRAM_W-1:0] wdb_din,
  input  logic [1-1:0] rdb_re,
  input  logic [RDB_AW-1:0] rdb_addr,
  input  logic [SRAM_W-1:0] rdb_dout,
  // Clock and reset
  input  logic [1-1:0] aclk,
  input  logic [1-1:0] aresetn,

  output logic [OBS_W-1:0] dft_obs
);

  // Each offset = previous offset + previous width (contiguous); O_END is the total.
  // AXI ingress
  localparam int O_awvalid = 0;
  localparam int O_awready = O_awvalid + (1);
  localparam int O_awaddr = O_awready + (1);
  localparam int O_awlen = O_awaddr + (AXI_AW);
  localparam int O_awsize = O_awlen + (8);
  localparam int O_awid = O_awsize + (3);
  localparam int O_arvalid = O_awid + (AXI_IDW);
  localparam int O_arready = O_arvalid + (1);
  localparam int O_araddr = O_arready + (1);
  localparam int O_arlen = O_araddr + (AXI_AW);
  localparam int O_arsize = O_arlen + (8);
  localparam int O_arid = O_arsize + (3);
  localparam int O_wvalid = O_arid + (AXI_IDW);
  localparam int O_wready = O_wvalid + (1);
  localparam int O_wstrb = O_wready + (1);
  localparam int O_wlast = O_wstrb + (AXI_DW/8);
  localparam int O_bvalid = O_wlast + (1);
  localparam int O_bready = O_bvalid + (1);
  localparam int O_rvalid = O_bready + (1);
  localparam int O_rready = O_rvalid + (1);
  // Address map
  localparam int O_aw_daddr = O_rready + (1);
  localparam int O_ar_daddr = O_aw_daddr + (DADDR_W);
  localparam int O_aw_ch = O_ar_daddr + (DADDR_W);
  localparam int O_ar_ch = O_aw_ch + (CH_W);
  localparam int O_aw_off = O_ar_ch + (CH_W);
  localparam int O_ar_off = O_aw_off + (PKT_OFF_W);
  localparam int O_addr_map = O_ar_off + (PKT_OFF_W);
  // Request validation
  localparam int O_aw_ok = O_addr_map + (DEC_W*MAP_SEL_W);
  localparam int O_ar_ok = O_aw_ok + (1);
  // Read ROB (u_rrob)
  localparam int O_rd_rob_index = O_ar_ok + (1);
  localparam int O_rd_rob_ready = O_rd_rob_index + (ROB_IDX_W);
  localparam int O_rd_alloc_vld = O_rd_rob_ready + (1);
  localparam int O_rd_head = O_rd_alloc_vld + (1);
  localparam int O_rd_tail = O_rd_head + (PTR_W);
  localparam int O_rd_occ = O_rd_tail + (PTR_W);
  localparam int O_rd_retire_vld = O_rd_occ + (OCC_W);
  localparam int O_rd_retire_fire = O_rd_retire_vld + (1);
  localparam int O_rd_inflow_complt = O_rd_retire_fire + (1);
  localparam int O_rd_outflow_cnt = O_rd_inflow_complt + (ROB_DEPTH);
  localparam int O_rd_n_pkts = O_rd_outflow_cnt + (ROB_DEPTH*CNT_W);
  localparam int O_rd_ent_err = O_rd_n_pkts + (ROB_DEPTH*CNT_W);
  localparam int O_rd_ent_derr = O_rd_ent_err + (ROB_DEPTH);
  localparam int O_rd_slot_head = O_rd_ent_derr + (ROB_DEPTH);
  localparam int O_rd_resp_id = O_rd_slot_head + (ROB_DEPTH*RDB_AW);
  localparam int O_rd_resp_resp = O_rd_resp_id + (AXI_IDW);
  localparam int O_rd_resp_head = O_rd_resp_resp + (2);
  localparam int O_rd_resp_offset = O_rd_resp_head + (RDB_AW);
  localparam int O_rd_resp_len = O_rd_resp_offset + (PKT_OFF_W);
  localparam int O_rd_resp_size = O_rd_resp_len + (8);
  // Write ROB (u_wrob)
  localparam int O_wr_rob_index = O_rd_resp_size + (3);
  localparam int O_wr_idx_vld = O_wr_rob_index + (ROB_IDX_W);
  localparam int O_wr_alloc_vld = O_wr_idx_vld + (1);
  localparam int O_wr_head = O_wr_alloc_vld + (1);
  localparam int O_wr_tail = O_wr_head + (PTR_W);
  localparam int O_wr_occ = O_wr_tail + (PTR_W);
  localparam int O_wr_retire_vld = O_wr_occ + (OCC_W);
  localparam int O_wr_retire_fire = O_wr_retire_vld + (1);
  localparam int O_wr_inflow_complt = O_wr_retire_fire + (1);
  localparam int O_wr_outflow_cnt = O_wr_inflow_complt + (ROB_DEPTH);
  localparam int O_wr_n_pkts = O_wr_outflow_cnt + (ROB_DEPTH*CNT_W);
  localparam int O_wr_ent_err = O_wr_n_pkts + (ROB_DEPTH*CNT_W);
  localparam int O_wr_ent_derr = O_wr_ent_err + (ROB_DEPTH);
  localparam int O_wr_slot_head = O_wr_ent_derr + (ROB_DEPTH);
  localparam int O_wr_resp_id = O_wr_slot_head + (ROB_DEPTH*WDB_AW);
  localparam int O_wr_resp_resp = O_wr_resp_id + (AXI_IDW);
  localparam int O_wr_resp_head = O_wr_resp_resp + (2);
  // Read segmenter (u_rseg)
  localparam int O_rs_ready = O_wr_resp_head + (WDB_AW);
  localparam int O_rs_q_alloc = O_rs_ready + (1);
  localparam int O_rs_q_pop = O_rs_q_alloc + (QPTR_W);
  localparam int O_rs_q_ch_done = O_rs_q_pop + (QPTR_W);
  localparam int O_rs_eng_rd_ptr = O_rs_q_ch_done + (ROB_DEPTH*N_CH);
  localparam int O_rs_eng_m = O_rs_eng_rd_ptr + (N_CH*QPTR_W);
  localparam int O_rs_req_dbuf_base = O_rs_eng_m + (N_CH*CNT_W);
  localparam int O_rs_seg_last = O_rs_req_dbuf_base + (RDB_AW);
  localparam int O_rs_seg_rob_idx = O_rs_seg_last + (1);
  localparam int O_rs_seg_npkts = O_rs_seg_rob_idx + (ROB_IDX_W);
  localparam int O_rs_seg_offset = O_rs_seg_npkts + (CNT_W);
  localparam int O_rs_seg_len = O_rs_seg_offset + (PKT_OFF_W);
  localparam int O_rs_seg_size = O_rs_seg_len + (8);
  localparam int O_rs_seg_head = O_rs_seg_size + (3);
  localparam int O_rs_pkt_valid = O_rs_seg_head + (RDB_AW);
  localparam int O_rs_pkt_ready = O_rs_pkt_valid + (N_CH);
  localparam int O_rs_pkt_dbuf_rdy = O_rs_pkt_ready + (N_CH);
  localparam int O_rs_pkt_rob_idx = O_rs_pkt_dbuf_rdy + (N_CH);
  localparam int O_rs_pkt_daddr = O_rs_pkt_rob_idx + (N_CH*ROB_IDX_W);
  localparam int O_rs_pkt_dbuf_adr = O_rs_pkt_daddr + (N_CH*DADDR_W);
  // Write segmenter (u_wseg, stub)
  localparam int O_ws_seg_last = O_rs_pkt_dbuf_adr + (N_CH*RDB_AW);
  localparam int O_ws_seg_npkts = O_ws_seg_last + (1);
  localparam int O_ws_seg_head = O_ws_seg_npkts + (CNT_W);
  localparam int O_ws_free_slot_take = O_ws_seg_head + (WDB_AW);
  localparam int O_ws_pkt_valid = O_ws_free_slot_take + (1);
  localparam int O_ws_pkt_ready = O_ws_pkt_valid + (N_CH);
  localparam int O_ws_pkt_rob_index = O_ws_pkt_ready + (N_CH);
  localparam int O_ws_pkt_daddr = O_ws_pkt_rob_index + (N_CH*ROB_IDX_W);
  localparam int O_ws_pkt_num = O_ws_pkt_daddr + (N_CH*DADDR_W);
  localparam int O_ws_pkt_last = O_ws_pkt_num + (N_CH*PKT_NUM_W);
  localparam int O_ws_pkt_slot = O_ws_pkt_last + (N_CH);
  localparam int O_ws_pkt_offset = O_ws_pkt_slot + (N_CH*WDB_AW);
  localparam int O_ws_pkt_narrow = O_ws_pkt_offset + (N_CH*PKT_OFF_W);
  // CIF-to-MC async FIFO boundary
  localparam int O_async_mc_req_valid = O_ws_pkt_narrow + (N_CH);
  localparam int O_async_mc_req_ready = O_async_mc_req_valid + (1);
  localparam int O_async_mc_req_rob_index = O_async_mc_req_ready + (1);
  localparam int O_async_mc_req_op = O_async_mc_req_rob_index + (ROB_IDX_W);
  localparam int O_async_mc_req_daddr = O_async_mc_req_op + (1);
  localparam int O_async_mc_req_pkt_num = O_async_mc_req_daddr + (DADDR_W);
  localparam int O_async_mc_req_last_in_txn = O_async_mc_req_pkt_num + (PKT_NUM_W);
  localparam int O_async_mc_req_sram_slot = O_async_mc_req_last_in_txn + (1);
  localparam int O_async_mc_cmpl_valid = O_async_mc_req_sram_slot + (SLOT_W);
  localparam int O_async_mc_cmpl_ready = O_async_mc_cmpl_valid + (1);
  localparam int O_async_mc_cmpl_rob_index = O_async_mc_cmpl_ready + (1);
  // Data SRAM client ports
  localparam int O_wdb_we = O_async_mc_cmpl_rob_index + (ROB_IDX_W);
  localparam int O_wdb_addr = O_wdb_we + (1);
  localparam int O_wdb_din = O_wdb_addr + (WDB_AW);
  localparam int O_rdb_re = O_wdb_din + (SRAM_W);
  localparam int O_rdb_addr = O_rdb_re + (1);
  localparam int O_rdb_dout = O_rdb_addr + (RDB_AW);
  // Clock and reset
  localparam int O_aclk = O_rdb_dout + (SRAM_W);
  localparam int O_aresetn = O_aclk + (1);
  localparam int O_END = O_aresetn + (1);

  // Elaboration-time check: the field list here must add up to dft_obs_w().
  if (O_END != OBS_W) begin : g_obs_w_chk
    $error("rmc_cif_dft: field list (%0d) != dft_obs_w (%0d)", O_END, OBS_W);
  end

  // AXI ingress
  assign dft_obs[O_awvalid +: 1] = awvalid;
  assign dft_obs[O_awready +: 1] = awready;
  assign dft_obs[O_awaddr +: AXI_AW] = awaddr;
  assign dft_obs[O_awlen +: 8] = awlen;
  assign dft_obs[O_awsize +: 3] = awsize;
  assign dft_obs[O_awid +: AXI_IDW] = awid;
  assign dft_obs[O_arvalid +: 1] = arvalid;
  assign dft_obs[O_arready +: 1] = arready;
  assign dft_obs[O_araddr +: AXI_AW] = araddr;
  assign dft_obs[O_arlen +: 8] = arlen;
  assign dft_obs[O_arsize +: 3] = arsize;
  assign dft_obs[O_arid +: AXI_IDW] = arid;
  assign dft_obs[O_wvalid +: 1] = wvalid;
  assign dft_obs[O_wready +: 1] = wready;
  assign dft_obs[O_wstrb +: AXI_DW/8] = wstrb;
  assign dft_obs[O_wlast +: 1] = wlast;
  assign dft_obs[O_bvalid +: 1] = bvalid;
  assign dft_obs[O_bready +: 1] = bready;
  assign dft_obs[O_rvalid +: 1] = rvalid;
  assign dft_obs[O_rready +: 1] = rready;

  // Address map
  assign dft_obs[O_aw_daddr +: DADDR_W] = aw_daddr;
  assign dft_obs[O_ar_daddr +: DADDR_W] = ar_daddr;
  assign dft_obs[O_aw_ch +: CH_W] = aw_ch;
  assign dft_obs[O_ar_ch +: CH_W] = ar_ch;
  assign dft_obs[O_aw_off +: PKT_OFF_W] = aw_off;
  assign dft_obs[O_ar_off +: PKT_OFF_W] = ar_off;
  assign dft_obs[O_addr_map +: DEC_W*MAP_SEL_W] = addr_map;

  // Request validation
  assign dft_obs[O_aw_ok +: 1] = aw_ok;
  assign dft_obs[O_ar_ok +: 1] = ar_ok;

  // Read ROB (u_rrob). rob_ready = A_ID_READY; head = retire_ptr, tail = alloc_ptr (next
  // slot to admit); n_pkts is written at admit; slot_head = RD-SRAM head slot per entry.
  assign dft_obs[O_rd_rob_index +: ROB_IDX_W] = rd_rob_index;
  assign dft_obs[O_rd_rob_ready +: 1] = rd_rob_ready;
  assign dft_obs[O_rd_alloc_vld +: 1] = rd_alloc_vld;
  assign dft_obs[O_rd_head +: PTR_W] = rd_head;
  assign dft_obs[O_rd_tail +: PTR_W] = rd_tail;
  assign dft_obs[O_rd_occ +: OCC_W] = rd_occ;
  assign dft_obs[O_rd_retire_vld +: 1] = rd_retire_vld;
  assign dft_obs[O_rd_retire_fire +: 1] = rd_retire_fire;
  assign dft_obs[O_rd_inflow_complt +: ROB_DEPTH] = rd_inflow_complt;
  assign dft_obs[O_rd_outflow_cnt +: ROB_DEPTH*CNT_W] = rd_outflow_cnt;
  assign dft_obs[O_rd_n_pkts +: ROB_DEPTH*CNT_W] = rd_n_pkts;
  assign dft_obs[O_rd_ent_err +: ROB_DEPTH] = rd_ent_err;
  assign dft_obs[O_rd_ent_derr +: ROB_DEPTH] = rd_ent_derr;
  assign dft_obs[O_rd_slot_head +: ROB_DEPTH*RDB_AW] = rd_slot_head;
  assign dft_obs[O_rd_resp_id +: AXI_IDW] = rd_resp_id;
  assign dft_obs[O_rd_resp_resp +: 2] = rd_resp_resp;
  assign dft_obs[O_rd_resp_head +: RDB_AW] = rd_resp_head;
  assign dft_obs[O_rd_resp_offset +: PKT_OFF_W] = rd_resp_offset;
  assign dft_obs[O_rd_resp_len +: 8] = rd_resp_len;
  assign dft_obs[O_rd_resp_size +: 3] = rd_resp_size;

  // Write ROB (u_wrob). idx_vld = pending_split; slot_head = first slot of the write chain.
  assign dft_obs[O_wr_rob_index +: ROB_IDX_W] = wr_rob_index;
  assign dft_obs[O_wr_idx_vld +: 1] = wr_idx_vld;
  assign dft_obs[O_wr_alloc_vld +: 1] = wr_alloc_vld;
  assign dft_obs[O_wr_head +: PTR_W] = wr_head;
  assign dft_obs[O_wr_tail +: PTR_W] = wr_tail;
  assign dft_obs[O_wr_occ +: OCC_W] = wr_occ;
  assign dft_obs[O_wr_retire_vld +: 1] = wr_retire_vld;
  assign dft_obs[O_wr_retire_fire +: 1] = wr_retire_fire;
  assign dft_obs[O_wr_inflow_complt +: ROB_DEPTH] = wr_inflow_complt;
  assign dft_obs[O_wr_outflow_cnt +: ROB_DEPTH*CNT_W] = wr_outflow_cnt;
  assign dft_obs[O_wr_n_pkts +: ROB_DEPTH*CNT_W] = wr_n_pkts;
  assign dft_obs[O_wr_ent_err +: ROB_DEPTH] = wr_ent_err;
  assign dft_obs[O_wr_ent_derr +: ROB_DEPTH] = wr_ent_derr;
  assign dft_obs[O_wr_slot_head +: ROB_DEPTH*WDB_AW] = wr_slot_head;
  assign dft_obs[O_wr_resp_id +: AXI_IDW] = wr_resp_id;
  assign dft_obs[O_wr_resp_resp +: 2] = wr_resp_resp;
  assign dft_obs[O_wr_resp_head +: WDB_AW] = wr_resp_head;

  // Read segmenter (u_rseg): admit queue + per-channel walkers. q_* = ring pointers and
  // per-slot done bits; eng_* = each walker's rd_ptr / issued count; pkt_* are per channel.
  assign dft_obs[O_rs_ready +: 1] = rs_ready;
  assign dft_obs[O_rs_q_alloc +: QPTR_W] = rs_q_alloc;
  assign dft_obs[O_rs_q_pop +: QPTR_W] = rs_q_pop;
  assign dft_obs[O_rs_q_ch_done +: ROB_DEPTH*N_CH] = rs_q_ch_done;
  assign dft_obs[O_rs_eng_rd_ptr +: N_CH*QPTR_W] = rs_eng_rd_ptr;
  assign dft_obs[O_rs_eng_m +: N_CH*CNT_W] = rs_eng_m;
  assign dft_obs[O_rs_req_dbuf_base +: RDB_AW] = rs_req_dbuf_base;
  assign dft_obs[O_rs_seg_last +: 1] = rs_seg_last;
  assign dft_obs[O_rs_seg_rob_idx +: ROB_IDX_W] = rs_seg_rob_idx;
  assign dft_obs[O_rs_seg_npkts +: CNT_W] = rs_seg_npkts;
  assign dft_obs[O_rs_seg_offset +: PKT_OFF_W] = rs_seg_offset;
  assign dft_obs[O_rs_seg_len +: 8] = rs_seg_len;
  assign dft_obs[O_rs_seg_size +: 3] = rs_seg_size;
  assign dft_obs[O_rs_seg_head +: RDB_AW] = rs_seg_head;
  assign dft_obs[O_rs_pkt_valid +: N_CH] = rs_pkt_valid;
  assign dft_obs[O_rs_pkt_ready +: N_CH] = rs_pkt_ready;
  assign dft_obs[O_rs_pkt_dbuf_rdy +: N_CH] = rs_pkt_dbuf_rdy;
  assign dft_obs[O_rs_pkt_rob_idx +: N_CH*ROB_IDX_W] = rs_pkt_rob_idx;
  assign dft_obs[O_rs_pkt_daddr +: N_CH*DADDR_W] = rs_pkt_daddr;
  assign dft_obs[O_rs_pkt_dbuf_adr +: N_CH*RDB_AW] = rs_pkt_dbuf_adr;

  // Write segmenter (u_wseg, stub): taps are wired for when logic lands.
  assign dft_obs[O_ws_seg_last +: 1] = ws_seg_last;
  assign dft_obs[O_ws_seg_npkts +: CNT_W] = ws_seg_npkts;
  assign dft_obs[O_ws_seg_head +: WDB_AW] = ws_seg_head;
  assign dft_obs[O_ws_free_slot_take +: 1] = ws_free_slot_take;
  assign dft_obs[O_ws_pkt_valid +: N_CH] = ws_pkt_valid;
  assign dft_obs[O_ws_pkt_ready +: N_CH] = ws_pkt_ready;
  assign dft_obs[O_ws_pkt_rob_index +: N_CH*ROB_IDX_W] = ws_pkt_rob_index;
  assign dft_obs[O_ws_pkt_daddr +: N_CH*DADDR_W] = ws_pkt_daddr;
  assign dft_obs[O_ws_pkt_num +: N_CH*PKT_NUM_W] = ws_pkt_num;
  assign dft_obs[O_ws_pkt_last +: N_CH] = ws_pkt_last;
  assign dft_obs[O_ws_pkt_slot +: N_CH*WDB_AW] = ws_pkt_slot;
  assign dft_obs[O_ws_pkt_offset +: N_CH*PKT_OFF_W] = ws_pkt_offset;
  assign dft_obs[O_ws_pkt_narrow +: N_CH] = ws_pkt_narrow;

  // CIF-to-MC async FIFO boundary
  assign dft_obs[O_async_mc_req_valid +: 1] = async_mc_req_valid;
  assign dft_obs[O_async_mc_req_ready +: 1] = async_mc_req_ready;
  assign dft_obs[O_async_mc_req_rob_index +: ROB_IDX_W] = async_mc_req_rob_index;
  assign dft_obs[O_async_mc_req_op +: 1] = async_mc_req_op;
  assign dft_obs[O_async_mc_req_daddr +: DADDR_W] = async_mc_req_daddr;
  assign dft_obs[O_async_mc_req_pkt_num +: PKT_NUM_W] = async_mc_req_pkt_num;
  assign dft_obs[O_async_mc_req_last_in_txn +: 1] = async_mc_req_last_in_txn;
  assign dft_obs[O_async_mc_req_sram_slot +: SLOT_W] = async_mc_req_sram_slot;
  assign dft_obs[O_async_mc_cmpl_valid +: 1] = async_mc_cmpl_valid;
  assign dft_obs[O_async_mc_cmpl_ready +: 1] = async_mc_cmpl_ready;
  assign dft_obs[O_async_mc_cmpl_rob_index +: ROB_IDX_W] = async_mc_cmpl_rob_index;

  // Data SRAM client ports
  assign dft_obs[O_wdb_we +: 1] = wdb_we;
  assign dft_obs[O_wdb_addr +: WDB_AW] = wdb_addr;
  assign dft_obs[O_wdb_din +: SRAM_W] = wdb_din;
  assign dft_obs[O_rdb_re +: 1] = rdb_re;
  assign dft_obs[O_rdb_addr +: RDB_AW] = rdb_addr;
  assign dft_obs[O_rdb_dout +: SRAM_W] = rdb_dout;

  // Clock and reset (aclk is sampled as data)
  assign dft_obs[O_aclk +: 1] = aclk;
  assign dft_obs[O_aresetn +: 1] = aresetn;

endmodule : rmc_cif_dft
