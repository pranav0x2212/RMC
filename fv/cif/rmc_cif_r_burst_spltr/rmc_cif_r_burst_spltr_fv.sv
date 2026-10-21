// rmc_cif_r_burst_spltr_fv.sv
// Formal checker for rmc_cif_r_burst_spltr (read burst splitter top). Bound into the DUT
// by rmc_cif_r_burst_spltr_bind.sv, so the RTL carries no verification code.
//
// What this proves (safety, open-source slang flow - boolean properties only):
//   1. precompute   : an independent re-derivation of the 4KB-bounded line math and the
//                     per-channel popcount matches the DUT's admit descriptor. Ties
//                     admit_npkts / admit_ch_mask to the address + map.
//   2. shared queue : the alloc/pop ring tracks a reference ring; admit only on a free
//                     slot + a free ROB slot; the ring never overflows.
//   3. head pop      : seg_last fires exactly when the oldest live entry has every channel
//                     done, carries that entry's rob_idx, and pops in admit order.
// The engines run inside the DUT with pkt_ready / pkt_dbuf_adr_ready left free, so the
// proof holds for any back-pressure. Liveness (every admit eventually pops) needs a
// fairness assumption on those and is left as future work (see props NOT DONE).
//
// Property style matches rmc_cif_rob_fv: boolean only (no |->, |=>, sequences).
//   a -> b  written  !a || b ;  a => b  written via `NEXT (skips first post-reset cycle).

import rmc_cfg_pkg::*;

module rmc_cif_r_burst_spltr_fv #(
  parameter int ROB_DEPTH   = 32,
  parameter int MAX_PKTS    = 64,
  parameter int AXI_AW      = SYS_ADDR_W,
  parameter int AXI_IDW     = 8,
  parameter int QUEUE_DEPTH = 32,
  parameter int N_SLOTS     = 64,
  // DUT-derived widths, passed straight from the DUT localparams by the bind
  parameter int ROB_IDX_W   = 5,
  parameter int CNT_W       = 7,
  parameter int SLOT_W      = 6,
  parameter int QIDX_W      = 5,
  parameter int QPTR_W      = 6,
  parameter int PAGE_LSB    = rmc_cfg_pkg::PAGE_LSB,
  parameter int LN_W        = 6,
  parameter int LEN_W       = 9,
  parameter int SIZE_W      = 3,
  parameter int PG_LINES    = rmc_cfg_pkg::PG_LINES,
  parameter int NCH_LOG_W   = 1
)(
  input logic                              aclk,
  input logic                              aresetn,

  // DUT config + AR inputs (free, constrained by assumptions)
  input logic [DEC_W-1:0][MAP_SEL_W-1:0]   addr_map,
  input logic [NCH_LOG_W-1:0]              cfg_n_ch_log2,
  input logic                              ar_valid,
  input logic                              ar_ready,

  // DUT admit descriptor (the real outputs under test)
  input logic                              admit_valid,
  input logic [AXI_IDW-1:0]                admit_id,
  input logic                              admit_validated,
  input logic [CNT_W-1:0]                  admit_npkts,
  input logic [PAGE_LSB-1:0]               admit_addr_lo,
  input logic [CH_W-1:0]                   admit_base_ch,
  input logic [N_CH-1:0]                   admit_ch_mask,

  // ROB handshake + head pop
  input logic                              rob_free,
  input logic [ROB_IDX_W-1:0]              rob_idx,
  input logic                              seg_last,
  input logic [ROB_IDX_W-1:0]              seg_rob_idx,
  input logic                              idle,

  // DUT internals (reachable through the bind)
  input logic                              s1_valid,
  input logic [AXI_IDW-1:0]                s1_id,
  input logic [AXI_AW-1:0]                 s1_addr,
  input logic [LEN_W-1:0]                  s1_len,
  input logic [SIZE_W-1:0]                 s1_size,
  input logic                              s1_validated,
  input logic [CNT_W-1:0]                  n_pkts,
  input logic [N_CH-1:0][CNT_W-1:0]        ch_pkts_eff,
  input logic [LN_W-1:0]                   l0,
  input logic [LN_W-1:0]                   l_last,
  input logic [15:0]                       end16,
  input logic [QPTR_W-1:0]                 alloc_ptr,
  input logic [QPTR_W-1:0]                 pop_ptr,
  input logic [ROB_IDX_W-1:0]              q_rob  [QUEUE_DEPTH],
  input logic [N_CH-1:0]                   q_done [QUEUE_DEPTH]
);

`define ASSERT(name, expr) name: assert property (@(posedge aclk) disable iff (!aresetn) (expr))
`define COVER(name, expr)  name: cover  property (@(posedge aclk) disable iff (!aresetn) (expr))
`define ASSUME(name, expr) name: assume property (@(posedge aclk) (expr))
`define NEXT(a, b)         (!$past(aresetn) || !$past(a) || (b))

  localparam int N_CH_L = 1 << CH_W;

  wire [QIDX_W-1:0] alloc_idx = alloc_ptr[QIDX_W-1:0];
  wire [QIDX_W-1:0] pop_idx   = pop_ptr[QIDX_W-1:0];
  wire              admit     = admit_valid;   // DUT asserts this on the admitted request
  wire              pop       = seg_last;

  // ------------------------------------------------------------------------
  // 1. reference ring (mirrors alloc_ptr / pop_ptr)
  // ------------------------------------------------------------------------
  function automatic logic [QPTR_W-1:0] ring_inc(input logic [QPTR_W-1:0] p);
    // QUEUE_DEPTH is a power of 2, so a free running counter wraps the index and
    // toggles the wrap bit on its own.
    return p + 1'b1;
  endfunction

  logic [QPTR_W-1:0]      fv_alloc, fv_pop;
  logic [QUEUE_DEPTH-1:0] fv_live;
  logic [ROB_IDX_W-1:0]   fv_rob [QUEUE_DEPTH];

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      fv_alloc <= '0;
      fv_pop   <= '0;
      fv_live  <= '0;
    end else begin
      if (admit) begin
        fv_live[fv_alloc[QIDX_W-1:0]] <= 1'b1;
        fv_rob [fv_alloc[QIDX_W-1:0]] <= rob_idx;
        fv_alloc <= ring_inc(fv_alloc);
      end
      if (pop) begin
        fv_live[fv_pop[QIDX_W-1:0]] <= 1'b0;
        fv_pop <= ring_inc(fv_pop);
      end
    end
  end

  wire fv_empty = (fv_alloc == fv_pop);
  wire fv_full  = (fv_alloc[QIDX_W-1:0] == fv_pop[QIDX_W-1:0])
                & (fv_alloc[QIDX_W] != fv_pop[QIDX_W]);

  // ------------------------------------------------------------------------
  // 2. independent precompute reference (from s1, compared to the DUT descriptor)
  // ------------------------------------------------------------------------
  logic [PAGE_LSB-1:0] ref_amask, ref_aal;
  logic [15:0]         ref_nbytes, ref_end;
  logic [CNT_W-1:0]    ref_npkts;
  assign ref_amask  = PAGE_LSB'((32'd1 << s1_size) - 32'd1);
  assign ref_aal    = s1_addr[PAGE_LSB-1:0] & ~ref_amask;
  assign ref_nbytes = (16'(s1_len) + 16'd1) << s1_size;
  assign ref_end    = 16'(ref_aal) + ref_nbytes - 16'd1;
  wire [LN_W-1:0] ref_l0_b    = s1_addr[PAGE_LSB-1 -: LN_W];      // ar_addr[11:6]
  wire [LN_W-1:0] ref_llast_b = ref_end [PAGE_LSB-1 -: LN_W];     // end[11:6]
  assign ref_npkts = CNT_W'(ref_llast_b) - CNT_W'(ref_l0_b) + CNT_W'(1);

  // independent per-channel popcount over the page lines
  function automatic logic [AXI_AW-1:0] ref_line_byte(input logic [LN_W-1:0] i);
    ref_line_byte = {s1_addr[AXI_AW-1:PAGE_LSB], i, {LN_W{1'b0}}};
  endfunction
  logic [N_CH_L-1:0][CNT_W-1:0] ref_chpkts;
  always_comb begin
    for (int c = 0; c < N_CH_L; c++) ref_chpkts[c] = '0;
    for (int i = 0; i < PG_LINES; i++)
      if ((LN_W'(i) >= ref_l0_b) && (LN_W'(i) <= ref_llast_b))
        ref_chpkts[rmc_cfg_pkg::line_channel(ref_line_byte(LN_W'(i)), addr_map, cfg_n_ch_log2)]
          = ref_chpkts[rmc_cfg_pkg::line_channel(ref_line_byte(LN_W'(i)), addr_map, cfg_n_ch_log2)] + CNT_W'(1);
  end

  // ------------------------------------------------------------------------
  // assumptions: a legal AR does not cross the 4KB page (rejected upstream)
  // ------------------------------------------------------------------------
  logic fv_started = 1'b0;
  always_ff @(posedge aclk) fv_started <= 1'b1;
  `ASSUME(m_reset_first, fv_started || !aresetn);
  // the registered request, when it is going to admit, stays inside its 4KB page
  `ASSUME(m_no_cross, !s1_valid || (ref_end[15:PAGE_LSB] == '0));
  // AXI: a beat is never wider than the data bus
  `ASSUME(m_req_size, !s1_valid || (32'(1) << s1_size) <= AXI_DW / 8);
  // the request fits the packet counter. Automatic at MAX_PKTS = 64 (a 4KB page holds 64
  // lines); the small proof configs use a smaller MAX_PKTS and need it stated.
  `ASSUME(m_req_fits, !s1_valid || 32'(ref_llast_b) - 32'(ref_l0_b) + 1 <= MAX_PKTS);

  // legal map: each CH source bit sits at or above the line offset (so a 64B line has one
  // channel) and inside the address, and only the active channels are selected. A CSR can
  // place the CH bits anywhere subject to this - the proof then holds for every such map.
  for (genvar k = 0; k < CH_W; k++) begin : g_map_legal
    `ASSUME(m_map_ch_lo, addr_map[PKT_OFF_W + k] >= MAP_SEL_W'(PKT_OFF_W));
    `ASSUME(m_map_ch_hi, addr_map[PKT_OFF_W + k] <  MAP_SEL_W'(AXI_AW));
  end
  `ASSUME(m_cfg_nch, cfg_n_ch_log2 == NCH_LOG_W'(CH_W));
  // boot-time CSR: the map does not change while requests are in flight
  `ASSUME(m_map_stable, !fv_started || $stable(addr_map));

  // ------------------------------------------------------------------------
  // 3. assertions
  // ------------------------------------------------------------------------
  // precompute vs DUT
  `ASSERT(a_npkts_math, !s1_valid || n_pkts == ref_npkts);
  `ASSERT(a_l0,         !s1_valid || l0 == ref_l0_b);
  `ASSERT(a_llast,      !s1_valid || l_last == ref_llast_b);
  `ASSERT(a_admit_npkts, !admit || admit_npkts == (s1_validated ? ref_npkts : CNT_W'(0)));
  for (genvar c = 0; c < N_CH_L; c++) begin : g_ch
    `ASSERT(a_chpkts, !s1_valid ||
            ch_pkts_eff[c] == (s1_validated ? ref_chpkts[c] : CNT_W'(0)));
    `ASSERT(a_mask,   !admit || admit_ch_mask[c] == (ch_pkts_eff[c] != CNT_W'(0)));
  end
  // sum of per-channel counts equals the total (completeness of the split)
  logic [CNT_W+CH_W:0] ref_sum;
  always_comb begin
    ref_sum = '0;
    for (int c = 0; c < N_CH_L; c++) ref_sum = ref_sum + (CNT_W+CH_W+1)'(ref_chpkts[c]);
  end
  `ASSERT(a_sum, !admit || !s1_validated || ref_sum == (CNT_W+CH_W+1)'(ref_npkts));
  `ASSERT(a_id,  !admit || admit_id == s1_id);
  `ASSERT(a_lo,  !admit || admit_addr_lo == s1_addr[PAGE_LSB-1:0]);

  // admit handshake
  `ASSERT(a_admit_free,    !admit || rob_free);                 // only with a ROB slot
  `ASSERT(a_admit_notfull, !admit || !fv_full);                 // and queue room
  `ASSERT(a_ar_ready,      ar_ready == (~s1_valid | admit));    // AR backpressure

  // ring tracking
  `ASSERT(a_alloc,     alloc_ptr == fv_alloc);
  `ASSERT(a_pop,       pop_ptr   == fv_pop);
  `ASSERT(a_no_ovf,    !fv_full || !admit);                     // never admit into a full ring
  `ASSERT(a_alloc_mv,  `NEXT(!admit, $stable(alloc_ptr)));
  `ASSERT(a_pop_mv,    `NEXT(!pop,   $stable(pop_ptr)));
  // induction helpers: the live slots are exactly [pop, alloc), and each live slot still
  // holds the rob_idx it was admitted with
  for (genvar i = 0; i < QUEUE_DEPTH; i++) begin : g_slot
    wire [QIDX_W-1:0] off = QIDX_W'(i) - fv_pop[QIDX_W-1:0];
    `ASSERT(a_live,  fv_live[i] == (QPTR_W'(off) < QPTR_W'(fv_alloc - fv_pop)));
    `ASSERT(a_q_rob, !fv_live[i] || q_rob[i] == fv_rob[i]);
  end
  `ASSERT(a_occ,       QPTR_W'(fv_alloc - fv_pop) <= QPTR_W'(QUEUE_DEPTH));

  // head pop
  `ASSERT(a_pop_cond,  pop == (~fv_empty & (&q_done[pop_idx]))); // fires exactly on all-done
  `ASSERT(a_pop_rob,   !pop || seg_rob_idx == q_rob[pop_idx]);   // carries the oldest entry
  `ASSERT(a_pop_live,  !pop || !fv_empty);                       // never pop an empty ring
  `ASSERT(a_pop_order, !pop || seg_rob_idx == fv_rob[fv_pop[QIDX_W-1:0]]); // in admit order

  // reset
  `ASSERT(a_reset, $past(aresetn) || (
       alloc_ptr == '0 && pop_ptr == '0 && !admit && !seg_last && ar_ready && idle));

  // ------------------------------------------------------------------------
  // 4. packets, one watched request
  // fv_e is a queue slot the solver picks (a flop that only holds its value, so its
  // start value is free). The request admitted into that slot is followed through the
  // engines: every packet it gets is checked on its own, and at pop every channel must
  // have sent exactly its share. Since fv_e is any slot, this covers every request.
  // Packet line = the engine's selected page line (sel_i); everything else is checked
  // against the address and map independently of the engine.
  // ------------------------------------------------------------------------
  localparam int SLOT_W_L = SLOT_W;

  // every map select points at a real address bit (the daddr bits are decoded too)
  for (genvar b = 0; b < DEC_W; b++) begin : g_map_rng
    `ASSUME(m_map_range, 32'(addr_map[b]) < AXI_AW);
  end

  // the map as the spec states it: decoded bit b = sys_addr[addr_map[b]]
  function automatic logic [DEC_W-1:0] exp_dec(input logic [AXI_AW-1:0] a);
    for (int b = 0; b < DEC_W; b++) exp_dec[b] = a[addr_map[b]];
  endfunction

  // channel field of exp_dec. Selected from a local variable: a part-select on a function
  // call result is not accepted by every parser.
  function automatic logic [CH_W-1:0] exp_ch(input logic [AXI_AW-1:0] a);
    logic [DEC_W-1:0] d;
    d = exp_dec(a);
    return d[PKT_OFF_W +: CH_W];
  endfunction

  logic [QIDX_W-1:0] fv_e;
  always_ff @(posedge aclk) fv_e <= fv_e;

  wire w_enq = admit & (fv_alloc[QIDX_W-1:0] == fv_e);
  wire w_pop = pop   & (fv_pop  [QIDX_W-1:0] == fv_e);

  logic                         w_live;
  logic                         w_val;            // request was validated
  logic [QPTR_W-1:0]            w_ptr;
  logic [AXI_AW-1:PAGE_LSB]     w_page;           // page of the request
  logic [LN_W-1:0]              w_l0, w_llast;    // first / last line in the page
  logic [ROB_IDX_W-1:0]         w_rob;
  logic [N_CH-1:0][CNT_W-1:0]   w_cnt;            // lines each channel owes
  logic [N_CH-1:0][CNT_W-1:0]   w_m;              // lines each channel has sent
  logic [N_CH-1:0][LN_W-1:0]    w_lasti;          // last line each channel sent
  logic [N_CH-1:0][SLOT_W_L-1:0] w_head;          // slot of each channel's first packet
  logic [N_CH-1:0][SLOT_W_L-1:0] fv_slot;         // each channel's RD-SRAM ring pointer

  wire [N_CH-1:0] pv   = rmc_cif_r_burst_spltr.pkt_valid;
  wire [N_CH-1:0] prdy = rmc_cif_r_burst_spltr.pkt_ready;
  wire [N_CH-1:0] drdy = rmc_cif_r_burst_spltr.pkt_dbuf_adr_ready;
  wire [N_CH-1:0] fire = pv & prdy;

  // byte address of page line i of the watched request, and its channel / DRAM coords
  function automatic logic [AXI_AW-1:0] w_line_byte(input logic [LN_W-1:0] i);
    w_line_byte = {w_page, i, {PKT_OFF_W{1'b0}}};
  endfunction
  logic [PG_LINES-1:0][CH_W-1:0] w_chan;
  always_comb for (int i = 0; i < PG_LINES; i++)
    w_chan[i] = exp_ch(w_line_byte(LN_W'(i)));

  logic [N_CH-1:0] at_e;
  always_comb for (int c = 0; c < N_CH; c++)
    at_e[c] = w_live & (rmc_cif_r_burst_spltr.eng_rd[c] == w_ptr);

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      w_live  <= 1'b0;
    end else begin
      if (w_pop) w_live <= 1'b0;
      if (w_enq) begin
        w_live  <= 1'b1;
        w_ptr   <= fv_alloc;
        w_page  <= s1_addr[AXI_AW-1:PAGE_LSB];
        w_l0    <= ref_l0_b;
        w_llast <= ref_llast_b;
        w_rob   <= rob_idx;
        w_val   <= s1_validated;
        for (int c = 0; c < N_CH; c++) w_cnt[c] <= s1_validated ? ref_chpkts[c] : '0;
      end
    end
  end

  for (genvar c = 0; c < N_CH; c++) begin : g_pkt
    wire [LN_W-1:0]  i    = rmc_cif_r_burst_spltr.g_eng[c].u_ch.sel_i;
    always_ff @(posedge aclk or negedge aresetn) begin
      if (!aresetn) begin
        fv_slot[c] <= '0;
      end else begin
        if (fire[c]) fv_slot[c] <= (fv_slot[c] == SLOT_W_L'(N_SLOTS-1)) ? '0 : fv_slot[c] + 1'b1;
        if (w_enq) begin
          w_m[c] <= '0;
        end else if (at_e[c] & fire[c]) begin
          w_m[c]     <= w_m[c] + 1'b1;
          w_lasti[c] <= i;
          if (w_m[c] == '0) w_head[c] <= fv_slot[c];
        end
      end
    end
    wire [DEC_W-1:0] dec  = exp_dec(w_line_byte(i));
    wire             v    = at_e[c] & pv[c];

    // the packet is a line of this request, on this channel, after the previous one
    `ASSERT(a_pkt_range, !v || (i >= w_l0 && i <= w_llast));
    `ASSERT(a_pkt_route, !v || w_chan[i] == CH_W'(c));
    `ASSERT(a_pkt_order, !v || w_m[c] == '0 || i > w_lasti[c]);
    `ASSERT(a_pkt_owed,  !v || w_m[c] < w_cnt[c]);
    `ASSERT(a_pkt_daddr, !v || rmc_cif_r_burst_spltr.pkt_daddr[c] == dec[PKT_OFF_W + CH_W +: DADDR_W]);
    `ASSERT(a_pkt_rob,   !v || rmc_cif_r_burst_spltr.pkt_rob_idx[c] == w_rob);
    // RD-SRAM: a packet only with a slot, slots handed out in ring order per channel
    `ASSERT(a_pkt_slot,  !pv[c] || (drdy[c] && rmc_cif_r_burst_spltr.pkt_dbuf_adr[c] == fv_slot[c]));
    // a packet only from an occupied queue slot
    `ASSERT(a_no_stray,  !pv[c] || rmc_cif_r_burst_spltr.eng_rd[c] != alloc_ptr);
    // no stall of its own: owing a line and given a slot, the engine offers it
    `ASSERT(a_progress,  !(at_e[c] && w_m[c] < w_cnt[c] && drdy[c]) || pv[c]);
    // no drop: at pop every channel sent its whole share; head pointer = its first slot
    `ASSERT(a_no_drop,   !w_pop || w_m[c] == w_cnt[c]);
    `ASSERT(a_seg_head,  !w_pop || w_cnt[c] == '0
                         || rmc_cif_r_burst_spltr.seg_head_ptr[c] == w_head[c]);

    // ---- induction helpers: engine c against the watched request
    wire [QPTR_W-1:0] d_c = rmc_cif_r_burst_spltr.eng_rd[c] - fv_pop;
    wire [QPTR_W-1:0] d_e = w_ptr - fv_pop;
    wire              act = rmc_cif_r_burst_spltr.g_eng[c].u_ch.active;
    wire              dn  = rmc_cif_r_burst_spltr.q_done[fv_e][c];
    `ASSERT(a_h_slot,  rmc_cif_r_burst_spltr.g_eng[c].u_ch.dbuf_ptr == fv_slot[c]);
    `ASSERT(a_h_range, d_c <= QPTR_W'(fv_alloc - fv_pop));
    `ASSERT(a_h_idle,  rmc_cif_r_burst_spltr.eng_rd[c] != alloc_ptr || !act);
    // mid-request means the engine's own done bit for that entry is still clear (whatever
    // entry it is on), so head-pop cannot pass an engine that is still issuing
    // between requests the engine's count sits at zero (the first fire does m+1, not m_eff+1)
    `ASSERT(a_h_m0,    act || rmc_cif_r_burst_spltr.g_eng[c].u_ch.m == '0);
    `ASSERT(a_h_busy,  !act || !rmc_cif_r_burst_spltr.q_done[rmc_cif_r_burst_spltr.eng_ri[c]][c]);
    `ASSERT(a_h_entry, !w_live || (rmc_cif_r_burst_spltr.q_base[fv_e] == {w_page, w_l0}
                                && rmc_cif_r_burst_spltr.q_ch_pkts[fv_e][c] == w_cnt[c]
                                && w_cnt[c] <= CNT_W'(MAX_PKTS)));
    `ASSERT(a_h_head,  !(w_live && w_m[c] != '0) || rmc_cif_r_burst_spltr.q_head[fv_e][c] == w_head[c]);
    `ASSERT(a_h_pre,   !(w_live && d_c <  d_e) || (w_m[c] == '0 && dn == (w_cnt[c] == '0)));
    `ASSERT(a_h_at,    !(w_live && d_c == d_e) || (
                          act == (w_m[c] != '0) && (!act || rmc_cif_r_burst_spltr.g_eng[c].u_ch.m == w_m[c])
                       && dn == (w_cnt[c] == '0) && (w_cnt[c] == '0 || w_m[c] < w_cnt[c])));
    `ASSERT(a_h_post,  !(w_live && d_c >  d_e) || (dn && w_m[c] == w_cnt[c]));
    // lines sent so far = the first w_m lines of this channel in the request, and the
    // engine's remaining-line mask holds exactly the later lines of this channel
    logic [CNT_W:0] sent;
    always_comb begin
      sent = '0;
      for (int j = 0; j < PG_LINES; j++)
        if (LN_W'(j) >= w_l0 && LN_W'(j) <= w_lasti[c] && w_chan[j] == CH_W'(c)) sent = sent + 1'b1;
    end
    // the share recorded at admit is this channel's lines of the request
    logic [CNT_W:0] share;
    always_comb begin
      share = '0;
      for (int j = 0; j < PG_LINES; j++)
        if (LN_W'(j) >= w_l0 && LN_W'(j) <= w_llast && w_chan[j] == CH_W'(c)) share = share + 1'b1;
    end
    `ASSERT(a_h_share, !w_live || (CNT_W+1)'(w_cnt[c]) == (w_val ? share : '0));
    `ASSERT(a_h_sent,  !(w_live && w_m[c] != '0) || ((CNT_W+1)'(w_m[c]) == sent
                          && w_chan[w_lasti[c]] == CH_W'(c) && w_lasti[c] >= w_l0 && w_lasti[c] <= w_llast));
    for (genvar j = 0; j < PG_LINES; j++) begin : g_mine
      `ASSERT(a_h_mine, !(w_live && d_c == d_e && act) ||
              rmc_cif_r_burst_spltr.g_eng[c].u_ch.mine_reg[j] ==
              (LN_W'(j) > w_lasti[c] && w_chan[j] == CH_W'(c)));
    end
  end
  // the watched slot is live exactly when it lies in [pop, alloc)
  `ASSERT(a_w_live, w_live == fv_live[fv_e]);
  `ASSERT(a_w_ptr,  !w_live || w_ptr == fv_pop + QPTR_W'(QIDX_W'(fv_e - fv_pop[QIDX_W-1:0])));
  `ASSERT(a_w_rob,  !w_live || w_rob == fv_rob[fv_e]);
  `ASSERT(a_w_fit,  !w_live || (w_l0 <= w_llast && 32'(w_llast) - 32'(w_l0) + 1 <= MAX_PKTS));

  // ------------------------------------------------------------------------
  // 5. read-response rebuild (claim of the ROB page-tag change)
  // The ROB keeps only ar_addr[PAGE_LSB-1:0] and base_ch (channel of the first line).
  // The R path must recover the channel of EVERY line of the request from those two:
  // channel bit k comes from the in-page line address when the map takes it from below
  // PAGE_LSB, else from base_ch (bits at or above PAGE_LSB are constant in the page).
  // Checked at admit, for every line of the page, against the full-address decode.
  // ------------------------------------------------------------------------
  // base_ch is the channel of the request's first line
  `ASSERT(a_base_ch, !admit || admit_base_ch == exp_ch(ref_line_byte(ref_l0_b)));
  for (genvar i = 0; i < PG_LINES; i++) begin : g_rebuild
    wire [PAGE_LSB-1:0] lo  = {LN_W'(i), {PKT_OFF_W{1'b0}}};      // what the R path knows
    wire [DEC_W-1:0]    dec = exp_dec(ref_line_byte(LN_W'(i)));   // the truth
    for (genvar k = 0; k < CH_W; k++) begin : g_k
      wire [MAP_SEL_W-1:0] src = addr_map[PKT_OFF_W + k];
      wire rebuilt = (32'(src) < PAGE_LSB) ? lo[src[$clog2(PAGE_LSB)-1:0]] : admit_base_ch[k];
      `ASSERT(a_rebuild, !admit || rebuilt == dec[PKT_OFF_W + k]);
    end
  end

  `COVER(c_w_two_ch,  w_pop && w_cnt[0] != '0 && w_cnt[N_CH-1] != '0);
  `COVER(c_w_skew,    w_pop && w_cnt[0] == '0 && 32'(w_cnt[N_CH-1]) >= 3);  // map piles lines on one channel
  `COVER(c_w_stall,   at_e[0] && !drdy[0] && pv[N_CH-1]);                 // one channel waits for a slot

  // ------------------------------------------------------------------------
  // covers (guard against a vacuous proof)
  // ------------------------------------------------------------------------
  `COVER(c_admit,     admit);
  `COVER(c_pop,       pop);
  `COVER(c_full,      fv_full);
  `COVER(c_wrap,      pop && pop_idx == QIDX_W'(QUEUE_DEPTH-1));
  `COVER(c_npkts_max, admit && admit_npkts == CNT_W'(MAX_PKTS));
  `COVER(c_slverr,    admit && !admit_validated);
  // a split that actually spreads across more than one channel
  `COVER(c_multich,   admit && $countones(admit_ch_mask) > 1);

`undef ASSERT
`undef COVER
`undef ASSUME
`undef NEXT

endmodule : rmc_cif_r_burst_spltr_fv
