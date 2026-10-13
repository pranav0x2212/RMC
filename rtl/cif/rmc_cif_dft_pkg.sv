// rmc_cif_dft_pkg.sv
// dft_obs width: one term per field in bus order; rmc_cif_dft checks it at elaboration.

package rmc_cif_dft_pkg;
  import rmc_cfg_pkg::*;

  function automatic int dft_obs_w(int AXI_IDW, int AXI_AW, int ROB_DEPTH,
                                   int MAX_PKTS, int N_WDB, int N_RDB);
    // same guarded width expressions as rmc_cif_dft
    int ROB_IDX_W, PTR_W, QPTR_W, OCC_W, CNT_W, PKT_NUM_W, SLOT_W, WDB_AW, RDB_AW, SRAM_W, t;
    ROB_IDX_W = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)   : 1;
    PTR_W     = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)   : 1;
    QPTR_W    = PTR_W + 1;
    OCC_W     = $clog2(ROB_DEPTH + 1);
    CNT_W     = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1)  : 1;
    PKT_NUM_W = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS)    : 1;
    WDB_AW    = (N_WDB     > 1) ? $clog2(N_WDB)       : 1;
    RDB_AW    = (N_RDB     > 1) ? $clog2(N_RDB)       : 1;
    SLOT_W    = (WDB_AW > RDB_AW) ? WDB_AW : RDB_AW;
    SRAM_W    = BL * DQ_W;
    t = 0;
    // AXI ingress
    t += 1;  // awvalid
    t += 1;  // awready
    t += AXI_AW;  // awaddr
    t += 8;  // awlen
    t += 3;  // awsize
    t += AXI_IDW;  // awid
    t += 1;  // arvalid
    t += 1;  // arready
    t += AXI_AW;  // araddr
    t += 8;  // arlen
    t += 3;  // arsize
    t += AXI_IDW;  // arid
    t += 1;  // wvalid
    t += 1;  // wready
    t += AXI_DW/8;  // wstrb
    t += 1;  // wlast
    t += 1;  // bvalid
    t += 1;  // bready
    t += 1;  // rvalid
    t += 1;  // rready
    // Address map
    t += DADDR_W;  // aw_daddr
    t += DADDR_W;  // ar_daddr
    t += CH_W;  // aw_ch
    t += CH_W;  // ar_ch
    t += PKT_OFF_W;  // aw_off
    t += PKT_OFF_W;  // ar_off
    t += DEC_W*MAP_SEL_W;  // addr_map
    // Request validation
    t += 1;  // aw_ok
    t += 1;  // ar_ok
    // Read ROB (u_rrob)
    t += ROB_IDX_W;  // rd_rob_index
    t += 1;  // rd_rob_ready
    t += 1;  // rd_alloc_vld
    t += PTR_W;  // rd_head
    t += PTR_W;  // rd_tail
    t += OCC_W;  // rd_occ
    t += 1;  // rd_retire_vld
    t += 1;  // rd_retire_fire
    t += ROB_DEPTH;  // rd_inflow_complt
    t += ROB_DEPTH*CNT_W;  // rd_outflow_cnt
    t += ROB_DEPTH*CNT_W;  // rd_n_pkts
    t += ROB_DEPTH;  // rd_ent_err
    t += ROB_DEPTH;  // rd_ent_derr
    t += ROB_DEPTH*RDB_AW;  // rd_slot_head
    t += AXI_IDW;  // rd_resp_id
    t += 2;  // rd_resp_resp
    t += RDB_AW;  // rd_resp_head
    t += PKT_OFF_W;  // rd_resp_offset
    t += 8;  // rd_resp_len
    t += 3;  // rd_resp_size
    // Write ROB (u_wrob)
    t += ROB_IDX_W;  // wr_rob_index
    t += 1;  // wr_idx_vld
    t += 1;  // wr_alloc_vld
    t += PTR_W;  // wr_head
    t += PTR_W;  // wr_tail
    t += OCC_W;  // wr_occ
    t += 1;  // wr_retire_vld
    t += 1;  // wr_retire_fire
    t += ROB_DEPTH;  // wr_inflow_complt
    t += ROB_DEPTH*CNT_W;  // wr_outflow_cnt
    t += ROB_DEPTH*CNT_W;  // wr_n_pkts
    t += ROB_DEPTH;  // wr_ent_err
    t += ROB_DEPTH;  // wr_ent_derr
    t += ROB_DEPTH*WDB_AW;  // wr_slot_head
    t += AXI_IDW;  // wr_resp_id
    t += 2;  // wr_resp_resp
    t += WDB_AW;  // wr_resp_head
    // Read segmenter (u_rseg)
    t += 1;  // rs_ready
    t += QPTR_W;  // rs_q_alloc
    t += QPTR_W;  // rs_q_pop
    t += ROB_DEPTH*N_CH;  // rs_q_ch_done
    t += N_CH*QPTR_W;  // rs_eng_rd_ptr
    t += N_CH*CNT_W;  // rs_eng_m
    t += RDB_AW;  // rs_req_dbuf_base
    t += 1;  // rs_seg_last
    t += ROB_IDX_W;  // rs_seg_rob_idx
    t += CNT_W;  // rs_seg_npkts
    t += PKT_OFF_W;  // rs_seg_offset
    t += 8;  // rs_seg_len
    t += 3;  // rs_seg_size
    t += RDB_AW;  // rs_seg_head
    t += N_CH;  // rs_pkt_valid
    t += N_CH;  // rs_pkt_ready
    t += N_CH;  // rs_pkt_dbuf_rdy
    t += N_CH*ROB_IDX_W;  // rs_pkt_rob_idx
    t += N_CH*DADDR_W;  // rs_pkt_daddr
    t += N_CH*RDB_AW;  // rs_pkt_dbuf_adr
    // Write segmenter (u_wseg, stub)
    t += 1;  // ws_seg_last
    t += CNT_W;  // ws_seg_npkts
    t += WDB_AW;  // ws_seg_head
    t += 1;  // ws_free_slot_take
    t += N_CH;  // ws_pkt_valid
    t += N_CH;  // ws_pkt_ready
    t += N_CH*ROB_IDX_W;  // ws_pkt_rob_index
    t += N_CH*DADDR_W;  // ws_pkt_daddr
    t += N_CH*PKT_NUM_W;  // ws_pkt_num
    t += N_CH;  // ws_pkt_last
    t += N_CH*WDB_AW;  // ws_pkt_slot
    t += N_CH*PKT_OFF_W;  // ws_pkt_offset
    t += N_CH;  // ws_pkt_narrow
    // CIF-to-MC async FIFO boundary
    t += 1;  // async_mc_req_valid
    t += 1;  // async_mc_req_ready
    t += ROB_IDX_W;  // async_mc_req_rob_index
    t += 1;  // async_mc_req_op
    t += DADDR_W;  // async_mc_req_daddr
    t += PKT_NUM_W;  // async_mc_req_pkt_num
    t += 1;  // async_mc_req_last_in_txn
    t += SLOT_W;  // async_mc_req_sram_slot
    t += 1;  // async_mc_cmpl_valid
    t += 1;  // async_mc_cmpl_ready
    t += ROB_IDX_W;  // async_mc_cmpl_rob_index
    // Data SRAM client ports
    t += 1;  // wdb_we
    t += WDB_AW;  // wdb_addr
    t += SRAM_W;  // wdb_din
    t += 1;  // rdb_re
    t += RDB_AW;  // rdb_addr
    t += SRAM_W;  // rdb_dout
    // Clock and reset
    t += 1;  // aclk
    t += 1;  // aresetn
    return t;
  endfunction
endpackage : rmc_cif_dft_pkg
