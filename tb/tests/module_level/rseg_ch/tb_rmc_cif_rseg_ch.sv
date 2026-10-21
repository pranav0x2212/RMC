`timescale 1ns/1ps
// Unit testbench for rmc_cif_r_burst_spltr_ch (one per-channel engine). The TB plays the
// wrapper (rmc_cif_r_burst_spltr): it owns the shared queue entries and the per-channel done
// bits, presents the entry at the engine's rd_ptr, and acts on the engine's set_done.
//
// Contract checked (sources: the DUT header + theory block, rmc_cif_r_burst_spltr.sv worked
// examples A and B, rmc_cfg_pkg::line_channel, rmc_cif_addrmap.sv bus layout):
//   * A queue entry is a request {base line number, lines L0..L_last inside one 4 KB page}. The
//     wrapper's precompute gives each engine e_ch_pkts = the number of request lines that map to
//     its channel; if that is 0 the wrapper presets the entry's done bit and the engine skips it.
//   * entry_valid & e_ch_done & idle-engine : SKIP, rd_ptr++ in that cycle, never gated by
//     pkt_ready / pkt_dbuf_adr_ready.
//   * entry_valid & !e_ch_done : the engine owes e_ch_pkts packets, issued in ascending line
//     order, one per fire. pkt_valid = owes & pkt_dbuf_adr_ready (it never waits on pkt_ready);
//     fire = pkt_valid & pkt_ready. The packet carries the entry's rob idx, daddr =
//     addrmap(page base, line, 0) and pkt_dbuf_adr = the engine's own RD-SRAM ring pointer, which
//     advances by one (mod N_SLOTS) on every fire, across entries.
//   * first fire of an entry : head_ptr_we, head_ptr = the slot used by that packet.
//     last fire (the e_ch_pkts-th) : set_done and rd_ptr++ in the same cycle; a 1-packet entry
//     is first and last at once. The next entry is presented the following cycle (no bubble).
//   * idle = no unfinished entry and the queue has caught up.
//   * Channel of a line = the CH source bits of addr_map (dec bus layout offset | ch | daddr, so
//     the select for ch bit k is addr_map[PKT_OFF_W + k]), with channel bits >= cfg_n_ch_log2
//     forced 0. The reference here is written from that layout and the TB's own record of which
//     address bit it put in each CH select; it does NOT call rmc_cfg_pkg::line_channel (the DUT
//     and the parent splitter share that function, so using it would hide a bug in it).
// The expected packet stream is built from the REQUEST (lines L0 .. L0+n-1 that map to this
// channel), not from the DUT's "first e_ch_pkts lines >= L0" rule, so that rule is tested too.
//
// Config contract respected: N_SLOTS >= MAX_PKTS, MAX_PKTS / N_SLOTS / QUEUE_DEPTH powers of two,
// a request has at most MAX_PKTS lines inside one page (L0 + n <= 64), e_ch_pkts <= MAX_PKTS.
// The engine has no parameter check; the TB asserts its own configuration.
//
// Not driven (outside the contract, would need an RTL/protocol decision): e_ch_pkts == 0 with
// done clear (the DUT $errors), e_ch_done rising while an entry is mid-issue, request crossing a
// 4 KB page, addr_map / cfg_n_ch_log2 changing while requests are in flight (they are static
// after boot; the TB changes them only together with a reset).

module rseg_ch_chk #(
  parameter int CH_ID       = 0,
  parameter int QUEUE_DEPTH = 4,     // power of two
  parameter int ROB_DEPTH   = 4,
  parameter int MAX_PKTS    = 16,    // power of two, <= N_SLOTS
  parameter int N_SLOTS     = 16,    // power of two
  parameter int SEED        = 1,
  parameter int NRAND       = 3000
)(
  output int n_pass,
  output int n_fail,
  output int n_pkts_seen,
  output int n_cov,                  // coverage markers reached (of 16)
  output bit done
);
  import rmc_cfg_pkg::*;
  localparam int AXI_AW     = SYS_ADDR_W;
  localparam int ROB_IDX_W  = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1;
  localparam int CNT_W      = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1;
  localparam int SLOT_W     = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1;
  localparam int LINE_W     = AXI_AW - PKT_OFF_W;
  localparam int QIDX_W     = (QUEUE_DEPTH > 1) ? $clog2(QUEUE_DEPTH) : 1;
  localparam int QPTR_W     = QIDX_W + 1;
  localparam int NCH_LOG_W  = $clog2(CH_W + 1);
  localparam int LN_W       = (PG_LINES    > 1) ? $clog2(PG_LINES)    : 1;   // line-in-page index
  localparam int NFLAV      = 6;                                               // channel-bit placements
  // line-number bits that exist in the mapped address space (requests stay in range)
  localparam logic [LINE_W-1:0] LINE_MASK = LINE_W'(((48'd1 << MAP_ADDR_W) - 48'd1) >> PKT_OFF_W);

  logic aclk = 1'b0, aresetn = 1'b1;
  always #5 aclk = ~aclk;

  logic [NCH_LOG_W-1:0] cfg_ncl;
  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map;
  logic                 entry_valid;
  logic [LINE_W-1:0]    e_base;
  logic [CNT_W-1:0]     e_ch_pkts;
  logic [ROB_IDX_W-1:0] e_rob_idx;
  logic                 e_ch_done;
  logic                 pkt_ready = 0, dbuf_rdy = 0;
  logic [QPTR_W-1:0]    rd_ptr;
  logic                 set_done, head_ptr_we;
  logic [SLOT_W-1:0]    head_ptr;
  logic                 pkt_valid;
  logic [ROB_IDX_W-1:0] pkt_rob_idx;
  logic [DADDR_W-1:0]   pkt_daddr;
  logic [SLOT_W-1:0]    pkt_dbuf_adr;
  logic                 idle;

  rmc_cif_r_burst_spltr_ch #(.ROB_DEPTH(ROB_DEPTH), .MAX_PKTS(MAX_PKTS), .AXI_AW(AXI_AW),
                             .N_SLOTS(N_SLOTS), .QUEUE_DEPTH(QUEUE_DEPTH)) dut (
    .aclk, .aresetn, .ch_id(CH_W'(CH_ID)), .addr_map, .cfg_n_ch_log2(cfg_ncl),
    .rd_ptr, .entry_valid, .e_base, .e_ch_pkts, .e_rob_idx, .e_ch_done,
    .set_done, .head_ptr_we, .head_ptr,
    .pkt_valid, .pkt_ready, .pkt_dbuf_adr_ready(dbuf_rdy),
    .pkt_rob_idx, .pkt_daddr, .pkt_dbuf_adr, .idle);

  // ---------------- TB wrapper: queue storage + done bits, presented at the engine's rd_ptr ----
  logic [LINE_W-1:0]    q_base [QUEUE_DEPTH];
  int                   q_n    [QUEUE_DEPTH];     // lines in the request
  int                   q_cnt  [QUEUE_DEPTH];     // of those, the lines of THIS channel (= e_ch_pkts)
  int                   q_rob  [QUEUE_DEPTH];
  bit                   q_done [QUEUE_DEPTH];
  int                   alloc_cnt = 0;            // entries enqueued since reset (unwrapped)
  int                   exp_rd    = 0;            // entries the engine has finished (unwrapped)
  int                   m_m       = 0;            // packets already issued for the current entry
  int                   dbuf_m    = 0;            // model of the engine's RD-SRAM ring pointer
  logic [QPTR_W-1:0]    alloc_q;
  assign alloc_q     = QPTR_W'(alloc_cnt % (2*QUEUE_DEPTH));
  assign entry_valid = (rd_ptr !== alloc_q);      // the wrapper's rd_ptr != alloc_ptr

  wire [QIDX_W-1:0] rdi = rd_ptr[QIDX_W-1:0];
  always_comb begin
    e_base    = q_base[rdi];
    e_ch_pkts = CNT_W'(q_cnt[rdi]);
    e_rob_idx = ROB_IDX_W'(q_rob[rdi]);
    e_ch_done = q_done[rdi];
  end

  // ---------------- current address-map configuration (set only together with a reset) --------
  int ch_src [CH_W];            // address bit the TB put into CH select k
  int cur_ncl = 0;              // active channels = 2^cur_ncl

  function automatic int flav_pos(input int f);
    if (f == 0)      return CH_POS;     // the default map (channel = line LSB)
    else if (f == 1) return 7;
    else if (f == 2) return 9;          // RTL example B (channel = addr[9])
    else if (f == 3) return 11;         // top in-page line bit
    else if (f == 4) return 12;         // above the page: constant for a whole request
    else             return 15;
  endfunction

  // ---------------- independent references ----------------
  function automatic logic [AXI_AW-1:0] line_addr(input logic [LINE_W-1:0] base, input int i);
    return {base[LINE_W-1:LN_W], LN_W'(i), {PKT_OFF_W{1'b0}}};
  endfunction

  function automatic int ref_ch(input logic [AXI_AW-1:0] a);   // channel of a byte address
    int c;
    c = 0;
    for (int k = 0; k < CH_W; k++)
      if (k < cur_ncl && a[ch_src[k]]) c = c | (1 << k);
    return c;
  endfunction

  // DRAM coords from the package field positions (independent of rmc_cif_addrmap / default_addr_map)
  function automatic logic [DADDR_W-1:0] ref_daddr(input logic [AXI_AW-1:0] a);
    return {a[RANK_POS +: RANK_W], a[BGHI_POS +: BG_HI_W], a[BGLO_POS +: BG_LO_W],
            a[BANK_POS +: BANK_PER_BG_W], a[ROW_POS +: ROW_W], a[COLHI_POS +: COL_HI_W],
            a[COLLO_POS +: COL_LO_W]};
  endfunction

  // number of lines of request {base, n} that this channel owns
  function automatic int count_ch(input logic [LINE_W-1:0] base, input int n);
    int c, l0;
    c = 0; l0 = int'(base[LN_W-1:0]);
    for (int i = l0; i < l0 + n; i++)
      if (ref_ch(line_addr(base, i)) == CH_ID) c++;
    return c;
  endfunction

  // page-line index of the m-th line of queue entry idx owned by this channel, -1 if none
  function automatic int kth_line(input int idx, input int m);
    int c, l0;
    c = 0; l0 = int'(q_base[idx][LN_W-1:0]);
    for (int i = l0; i < l0 + q_n[idx]; i++)
      if (ref_ch(line_addr(q_base[idx], i)) == CH_ID) begin
        if (c == m) return i;
        c++;
      end
    return -1;
  endfunction

  // ---------------- bookkeeping ----------------
  int  cyc = 0, tot_pkts = 0, n_skips = 0;
  bit  prev_sd = 0, prev_idle = 1;
  bit  rec_en = 0;
  int  obs_n = 0;
  logic [DADDR_W-1:0] obs_da [80];
  logic [SLOT_W-1:0]  obs_sl [80];
  bit                 obs_we [80];
  int  lit [8];
  int  lit_n = 0;
  logic [LINE_W-1:0] last_base;
  bit  cov_skip = 0, cov_skip_stalled = 0, cov_stall_mid = 0, cov_valid_noready = 0, cov_dbuf_stall = 0,
       cov_first_last = 0, cov_b2b = 0, cov_wrap = 0, cov_ptr_wrap = 0, cov_idle_rise = 0,
       cov_reset_mid = 0, cov_unaligned = 0, cov_pageedge = 0, cov_maxcnt = 0, cov_cfg0 = 0, cov_coarse = 0;

  task automatic chk(input bit c, input string msg);
    if (c) n_pass++;
    else begin n_fail++; $display("[%0t] FAIL rseg_ch(ch=%0d,Q=%0d,P=%0d,S=%0d) %s", $time, CH_ID, QUEUE_DEPTH, MAX_PKTS, N_SLOTS, msg); end
  endtask

  task model_reset();
    alloc_cnt = 0; exp_rd = 0; m_m = 0; dbuf_m = 0; prev_sd = 0; prev_idle = 1;
    for (int i = 0; i < QUEUE_DEPTH; i++) q_done[i] = 0;
  endtask

  function automatic int room();
    return QUEUE_DEPTH - (alloc_cnt - exp_rd);
  endfunction

  // an entry is finished (done or skipped): the engine moves to the next one
  task consume(input int idx);
    int l0;
    l0 = int'(q_base[idx][LN_W-1:0]);
    if (q_cnt[idx] > 0) begin
      if (l0 != 0) cov_unaligned = 1;
      if (l0 + q_n[idx] - 1 == PG_LINES - 1) cov_pageedge = 1;
      if (q_cnt[idx] == MAX_PKTS) cov_maxcnt = 1;
    end
    exp_rd = exp_rd + 1;
    if (exp_rd >= 2*QUEUE_DEPTH) cov_ptr_wrap = 1;
  endtask

  // One clock cycle: the caller has set pkt_ready / dbuf_rdy and enqueued. Compare every DUT
  // output with the model, then advance the model with what happens at the edge.
  task tick();
    int idx, l;
    bit have, ex_pv, ex_sd, ex_we, ex_skip, ex_fire, ex_idle;
    #1;
    cyc++;
    idx  = exp_rd % QUEUE_DEPTH;
    have = (exp_rd != alloc_cnt);
    ex_pv = 0; ex_sd = 0; ex_we = 0; ex_skip = 0; ex_fire = 0;
    l = -1;
    if (have) begin
      if (q_done[idx]) begin
        if (m_m == 0) ex_skip = 1;
        else begin n_fail++; $display("[%0t] FAIL rseg_ch TB: done entry while mid-issue", $time); end
      end else begin
        ex_pv   = dbuf_rdy;
        ex_fire = ex_pv && pkt_ready;
        ex_we   = ex_fire && (m_m == 0);
        ex_sd   = ex_fire && (m_m == q_cnt[idx] - 1);
        l       = kth_line(idx, m_m);
      end
    end
    ex_idle = (m_m == 0) && !have;

    chk(rd_ptr === QPTR_W'(exp_rd % (2*QUEUE_DEPTH)), $sformatf("c%0d rd_ptr %0d exp %0d", cyc, rd_ptr, exp_rd % (2*QUEUE_DEPTH)));
    chk(pkt_valid === ex_pv,     $sformatf("c%0d pkt_valid %b exp %b (m=%0d)", cyc, pkt_valid, ex_pv, m_m));
    chk(set_done === ex_sd,      $sformatf("c%0d set_done %b exp %b", cyc, set_done, ex_sd));
    chk(head_ptr_we === ex_we,   $sformatf("c%0d head_ptr_we %b exp %b", cyc, head_ptr_we, ex_we));
    chk(idle === ex_idle,        $sformatf("c%0d idle %b exp %b", cyc, idle, ex_idle));
    if (ex_pv) begin
      chk(l >= 0, $sformatf("c%0d TB model: no line left for packet m=%0d", cyc, m_m));
      chk(pkt_rob_idx === ROB_IDX_W'(q_rob[idx]), $sformatf("c%0d pkt_rob_idx %0d exp %0d", cyc, pkt_rob_idx, q_rob[idx]));
      chk(pkt_dbuf_adr === SLOT_W'(dbuf_m), $sformatf("c%0d pkt_dbuf_adr %0d exp %0d", cyc, pkt_dbuf_adr, dbuf_m));
      if (l >= 0)
        chk(pkt_daddr === ref_daddr(line_addr(q_base[idx], l)),
            $sformatf("c%0d pkt_daddr %h exp %h (line %0d, m=%0d)", cyc, pkt_daddr, ref_daddr(line_addr(q_base[idx], l)), l, m_m));
    end
    if (ex_we)
      chk(head_ptr === SLOT_W'(dbuf_m), $sformatf("c%0d head_ptr %0d exp %0d", cyc, head_ptr, dbuf_m));

    // coverage of the situations exercised (pre-edge view)
    if (aresetn) begin
      if (ex_skip) begin cov_skip = 1; n_skips++; if (!pkt_ready || !dbuf_rdy) cov_skip_stalled = 1; end
      if (have && m_m > 0 && (!pkt_ready || !dbuf_rdy)) cov_stall_mid = 1;
      if (ex_pv && !pkt_ready) cov_valid_noready = 1;
      if (have && !q_done[idx] && !dbuf_rdy) cov_dbuf_stall = 1;
      if (ex_we && ex_sd) cov_first_last = 1;
      if (ex_fire && prev_sd) cov_b2b = 1;
      if (ex_idle && !prev_idle) cov_idle_rise = 1;
      if (ex_fire && rec_en && obs_n < 80) begin
        obs_da[obs_n] = pkt_daddr; obs_sl[obs_n] = pkt_dbuf_adr; obs_we[obs_n] = head_ptr_we; obs_n++;
      end
    end

    // advance the model 1 ns after the edge, never at it: the wrapper's done bit must not change
    // in the time step in which the DUT samples e_ch_done
    @(posedge aclk); #1;
    if (!aresetn) model_reset();
    else begin
      if (ex_skip) consume(idx);
      else if (ex_fire) begin
        tot_pkts++;
        if (dbuf_m == N_SLOTS - 1) cov_wrap = 1;
        dbuf_m = (dbuf_m + 1) % N_SLOTS;
        if (ex_sd) begin q_done[idx] = 1; consume(idx); m_m = 0; end
        else m_m = m_m + 1;
      end
      prev_sd = ex_sd; prev_idle = ex_idle;
    end
  endtask

  task idle_n(input int n);
    repeat (n) tick();
  endtask

  // power-on / boot reset: map and config are set before this call
  task do_reset();
    aresetn = 1; #1; aresetn = 0; model_reset(); tick(); tick(); aresetn = 1; tick();
  endtask

  // asynchronous reset between clock edges, with whatever is in flight
  task midrun_reset();
    if (m_m > 0) cov_reset_mid = 1;
    #2 aresetn = 0; model_reset(); #1;
    chk(rd_ptr === '0 && idle === 1'b1 && pkt_valid === 1'b0 && set_done === 1'b0 && head_ptr_we === 1'b0,
        "async reset clears rd_ptr / active, engine idle");
    tick(); aresetn = 1; tick();
  endtask

  // program the channel-bit source and the active-channel count (call before do_reset)
  task set_map(input int flavor, input int ncl);
    logic [DEC_W*MAP_SEL_W-1:0] mf;
    mf = default_addr_map();
    for (int k = 0; k < CH_W; k++) begin
      ch_src[k] = flav_pos(flavor) + k;
      mf[(PKT_OFF_W + k)*MAP_SEL_W +: MAP_SEL_W] = MAP_SEL_W'(ch_src[k]);
    end
    addr_map = mf;
    cur_ncl  = ncl;
    cfg_ncl  = NCH_LOG_W'(ncl);
  endtask

  // ---------------- stimulus helpers ----------------
  // queue one request: l0 = first line in its page, n = lines (l0 + n <= 64, n <= MAX_PKTS)
  task enq_req(input int l0, input int n);
    int idx;
    logic [63:0] raw;
    idx = alloc_cnt % QUEUE_DEPTH;
    raw = {$urandom, $urandom};
    q_base[idx] = (LINE_W'(raw) & LINE_MASK & ~LINE_W'(PG_LINES - 1)) | LINE_W'(l0);
    q_n[idx]    = n;
    q_rob[idx]  = $urandom % ROB_DEPTH;
    q_cnt[idx]  = count_ch(q_base[idx], n);
    q_done[idx] = (q_cnt[idx] == 0);              // the wrapper presets done where nothing is owed
    if (n < 1 || l0 + n > PG_LINES || n > MAX_PKTS || q_cnt[idx] > MAX_PKTS || room() <= 0) begin
      n_fail++; $display("[%0t] FAIL rseg_ch TB STIMULUS ILLEGAL: l0=%0d n=%0d cnt=%0d room=%0d", $time, l0, n, q_cnt[idx], room());
    end
    alloc_cnt = alloc_cnt + 1;
  endtask

  task rand_req(output int l0, output int n);
    int lim, cls;
    cls = $urandom % 8;
    if (cls == 0)      l0 = 0;
    else if (cls == 1) l0 = PG_LINES - 1;
    else               l0 = $urandom % PG_LINES;
    lim = PG_LINES - l0;
    if (lim > MAX_PKTS) lim = MAX_PKTS;
    cls = $urandom % 5;
    if (cls == 0)      n = 1;
    else if (cls == 1) n = lim;
    else               n = 1 + ($urandom % lim);
  endtask

  // queue a random request whose count for this channel is in [lo, hi]; big = longest request
  // from line 0. ok = 0 if none was found (e.g. the map gives this channel nothing).
  task enq_try(input int lo, input int hi, input bit big, output bit ok);
    int l0, n, t, idx;
    ok = 0;
    for (t = 0; t < 400 && !ok; t++) begin
      if (big) begin l0 = 0; n = MAX_PKTS; end
      else rand_req(l0, n);
      enq_req(l0, n);
      idx = (alloc_cnt - 1) % QUEUE_DEPTH;
      if (q_cnt[idx] >= lo && q_cnt[idx] <= hi) ok = 1;
      else alloc_cnt = alloc_cnt - 1;             // not what was asked for: drop it again
    end
  endtask

  function automatic bit rdy_bit(input int mode, input int g);
    if (mode == 0)      return 1;
    else if (mode == 1) return (g % 3) == 0;
    else if (mode == 2) return ($urandom % 2) == 1;
    else if (mode == 3) return 0;
    else                return (g % 5) == 0;
  endfunction
  function automatic bit dbf_bit(input int mode, input int g);
    if (mode == 0)      return 1;
    else if (mode == 1) return (g % 4) != 3;
    else if (mode == 2) return ($urandom % 2) == 1;
    else if (mode == 3) return 0;
    else                return (g % 3) == 0;
  endfunction

  // run until the engine has consumed everything queued; modes pick the ready/slot patterns
  task run_until_empty(input int rdy_mode, input int dbuf_mode, input int cap_cycles);
    int g;
    g = 0;
    while ((exp_rd != alloc_cnt || m_m != 0) && g < cap_cycles) begin
      pkt_ready = rdy_bit(rdy_mode, g);
      dbuf_rdy  = dbf_bit(dbuf_mode, g);
      tick(); g++;
    end
    chk(exp_rd == alloc_cnt && m_m == 0, $sformatf("run_until_empty timeout (rdy %0d dbuf %0d)", rdy_mode, dbuf_mode));
  endtask

  // literal anchor: queue {l0,n} on an engine whose ring is at slot 0 (just reset) and compare
  // the observed packet stream with lit[0..lit_n-1] (page-line indices), not with the model
  task lit_run(input string tag, input int l0, input int n);
    obs_n = 0; rec_en = 1;
    enq_req(l0, n);
    last_base = q_base[(alloc_cnt - 1) % QUEUE_DEPTH];
    pkt_ready = 1; dbuf_rdy = 1;
    run_until_empty(0, 0, 400);
    rec_en = 0;
    chk(obs_n == lit_n, $sformatf("literal %s: %0d packets, expected %0d", tag, obs_n, lit_n));
    for (int j = 0; j < lit_n && j < obs_n; j++) begin
      chk(obs_da[j] === ref_daddr(line_addr(last_base, lit[j])), $sformatf("literal %s packet %0d: daddr for line %0d", tag, j, lit[j]));
      chk(obs_sl[j] === SLOT_W'(j), $sformatf("literal %s packet %0d: slot %0d exp %0d", tag, j, obs_sl[j], j));
      chk(obs_we[j] === (j == 0),   $sformatf("literal %s packet %0d: head_ptr_we %b", tag, j, obs_we[j]));
    end
  endtask

  function automatic int l0_of(input int a);
    if (a == 0) return 0; else if (a == 1) return 1; else if (a == 2) return 2; else if (a == 3) return 5;
    else if (a == 4) return 31; else if (a == 5) return 32; else if (a == 6) return 62; else return 63;
  endfunction
  function automatic int n_of(input int b);
    if (b == 0) return 1; else if (b == 1) return 2; else if (b == 2) return 3;
    else if (b == 3) return 5; else if (b == 4) return 8; else return MAX_PKTS;
  endfunction

  // ---------------- one configuration phase ----------------
  task run_phase(input int flavor, input int ncl, input int ncyc);
    bit okb;
    int l0, n, lim, ne, ph_pkts0, ph_skips0, p_rdy, p_dbf, p_enq, kind;
    set_map(flavor, ncl);
    do_reset();
    ph_pkts0 = tot_pkts; ph_skips0 = n_skips;
    if (ncl == 0) cov_cfg0 = 1;

    // D1 idle after reset: nothing owed, engine idle, no packets
    pkt_ready = 1; dbuf_rdy = 1; idle_n(4);
    chk(idle === 1'b1 && pkt_valid === 1'b0 && set_done === 1'b0 && rd_ptr === '0, "D1 idle after reset");

    // D2 literal anchors (engine ring at slot 0). RTL examples: lines 2..6, default map (A),
    //    channel = addr[9] (B); and with one active channel every line is channel 0.
    ne = (MAX_PKTS >= 5) ? 5 : 3;
    if (N_CH == 2 && ncl == CH_W && flavor == 0) begin
      lit_n = 0;
      for (int i = 2; i < 2 + ne; i++) if ((i % 2) == CH_ID) begin lit[lit_n] = i; lit_n++; end
      lit_run("A(fine interleave)", 2, ne);
    end else if (N_CH == 2 && ncl == CH_W && flavor == 2) begin
      lit_n = 0;
      for (int i = 2; i < 2 + ne; i++) if (CH_ID == 0) begin lit[lit_n] = i; lit_n++; end
      lit_run("B(channel=addr[9])", 2, ne);
    end else if (ncl == 0) begin
      lit_n = 0;
      for (int i = 2; i < 2 + ne; i++) if (CH_ID == 0) begin lit[lit_n] = i; lit_n++; end
      lit_run("one active channel", 2, ne);
    end

    // D3 single requests: every start alignment x several lengths, ready/slot patterns varied
    for (int a = 0; a < 8; a++)
      for (int b = 0; b < 6; b++) begin
        l0 = l0_of(a);
        lim = PG_LINES - l0; if (lim > MAX_PKTS) lim = MAX_PKTS;
        n = n_of(b); if (n > lim) n = lim;
        enq_req(l0, n);
        run_until_empty(a % 3, b % 3, 2000);
      end

    // D4 back-to-back one-packet entries (no bubble between entries)
    pkt_ready = 1; dbuf_rdy = 1;
    okb = 1;
    for (int i = 0; i < QUEUE_DEPTH && okb; i++)
      if (room() > 0) enq_try(1, 1, 0, okb);
    run_until_empty(0, 0, 400);

    // D5 entries with nothing for this channel (done preset): skipped at 1/cycle no matter what
    okb = 1;
    for (int i = 0; i < QUEUE_DEPTH && okb; i++)
      if (room() > 0) enq_try(0, 0, 0, okb);
    pkt_ready = 0; dbuf_rdy = 0;
    repeat (QUEUE_DEPTH + 2) tick();
    chk(exp_rd == alloc_cnt, "D5 skip entries drain with pkt_ready and dbuf both low");
    run_until_empty(0, 0, 100);

    // D6 handshake: valid without ready, stall mid-entry, no free slot, then resume
    enq_try(3, MAX_PKTS, 0, okb);
    if (okb) begin
      pkt_ready = 0; dbuf_rdy = 1; repeat (3) tick();
      chk(pkt_valid === 1'b1 && head_ptr_we === 1'b0 && set_done === 1'b0, "D6 first packet offered but not accepted: no head_ptr_we");
      pkt_ready = 1; tick();                               // first fire
      pkt_ready = 0; repeat (5) tick();                    // paused mid-entry
      dbuf_rdy = 0;  repeat (4) tick();                    // no RD-SRAM slot: pkt_valid drops
      chk(pkt_valid === 1'b0, "D6 pkt_valid low without a free slot");
      dbuf_rdy = 1;  repeat (3) tick();                    // slot back, command FIFO still full
      chk(pkt_valid === 1'b1, "D6 pkt_valid does not wait for pkt_ready");
      run_until_empty(0, 0, 400);
    end

    // D7 longest request from line 0 (all of a coarse-mapped request lands on one channel)
    enq_try(MAX_PKTS, MAX_PKTS, 1, okb);
    if (okb) run_until_empty(1, 1, 4000);

    // D8 queue filled to depth, several laps (rd_ptr and RD-SRAM ring wrap), random pacing
    for (int lap = 0; lap < 2; lap++) begin
      while (room() > 0) begin rand_req(l0, n); enq_req(l0, n); end
      run_until_empty(2, 2, 40000);
    end

    // D9 reset in the middle of an entry and in the middle of a skip run
    enq_try(3, MAX_PKTS, 0, okb);
    if (okb) begin
      pkt_ready = 1; dbuf_rdy = 1; tick(); tick();         // two packets issued: mid-entry
      pkt_ready = 0; tick();
      midrun_reset();                                      // rd_ptr, active, m and ring pointer clear
      pkt_ready = 1; dbuf_rdy = 1;
      enq_try(1, MAX_PKTS, 0, okb);
      run_until_empty(0, 0, 400);                          // model: first packet back at slot 0, head_ptr_we
    end
    okb = 1;
    for (int i = 0; i < 3 && okb; i++)
      if (room() > 0) enq_try(0, 0, 0, okb);
    pkt_ready = 0; dbuf_rdy = 0; tick();
    midrun_reset();
    chk(idle === 1'b1, "D9 idle after a reset during a skip run");

    // D10 constrained random: requests whenever there is room (mix of owed / skipped / 1-packet),
    //     random ready and slot availability in phases of 150 cycles
    p_rdy = 70; p_dbf = 70; p_enq = 40;
    for (int c = 0; c < ncyc; c++) begin
      if (c % 150 == 0) begin
        case ((c / 150) % 4)
          0: begin p_rdy = 70; p_dbf = 70; p_enq = 40; end
          1: begin p_rdy = 10; p_dbf = 90; p_enq = 60; end     // command FIFO mostly full
          2: begin p_rdy = 90; p_dbf = 15; p_enq = 60; end     // RD-SRAM mostly full
          default: begin p_rdy = 95; p_dbf = 95; p_enq = 90; end
        endcase
      end
      if (room() > 0 && ($urandom % 100) < p_enq) begin
        kind = $urandom % 10;
        if (kind < 6)      begin rand_req(l0, n); enq_req(l0, n); end
        else if (kind < 8) enq_try(0, 0, 0, okb);
        else               enq_try(1, 1, 0, okb);
      end
      pkt_ready = (($urandom % 100) < p_rdy);
      dbuf_rdy  = (($urandom % 100) < p_dbf);
      tick();
    end
    pkt_ready = 1; dbuf_rdy = 1;
    run_until_empty(0, 0, 40000);
    idle_n(2);
    chk(idle === 1'b1, "D10 idle once everything is consumed");

    // phase-level expectations that do not come from the model
    if (flavor >= 2) cov_coarse = 1;
    if (ncl == 0) begin
      if (CH_ID == 0) chk(tot_pkts > ph_pkts0, "one active channel: channel 0 issues the lines");
      else            chk(tot_pkts == ph_pkts0 && n_skips > ph_skips0, "one active channel: other channels issue nothing and skip");
    end else begin
      chk(tot_pkts > ph_pkts0, "phase issued packets (stimulus vacuity guard)");
    end
  endtask

  // ---------------- main ----------------
  initial begin : main
    int seed_v;
    n_pass = 0; n_fail = 0; n_pkts_seen = 0; n_cov = 0; done = 0;
    seed_v = SEED; void'($urandom(seed_v));
    chk(N_SLOTS >= MAX_PKTS && (N_SLOTS & (N_SLOTS-1)) == 0 && (MAX_PKTS & (MAX_PKTS-1)) == 0 &&
        (QUEUE_DEPTH & (QUEUE_DEPTH-1)) == 0, "TB configuration violates N_SLOTS >= MAX_PKTS / power-of-two contract");
    for (int f = 0; f < NFLAV; f++) run_phase(f, CH_W, NRAND / (NFLAV + 1));   // channel bits at six places
    run_phase(0, 0, NRAND / (NFLAV + 1));                                       // one active channel

    n_pkts_seen = tot_pkts;
    chk(tot_pkts > 20 * NFLAV, $sformatf("stimulus vacuity guard: only %0d packets issued", tot_pkts));
    chk(cov_skip && cov_skip_stalled && cov_stall_mid && cov_valid_noready && cov_dbuf_stall && cov_first_last &&
        cov_b2b && cov_wrap && cov_ptr_wrap && cov_idle_rise && cov_reset_mid && cov_unaligned && cov_pageedge &&
        cov_maxcnt && cov_cfg0 && cov_coarse,
        $sformatf("coverage skip=%b skipstall=%b stallmid=%b validnoready=%b dbufstall=%b firstlast=%b b2b=%b wrap=%b ptrwrap=%b idlerise=%b rstmid=%b unaligned=%b pageedge=%b maxcnt=%b cfg0=%b coarse=%b",
                  cov_skip, cov_skip_stalled, cov_stall_mid, cov_valid_noready, cov_dbuf_stall, cov_first_last,
                  cov_b2b, cov_wrap, cov_ptr_wrap, cov_idle_rise, cov_reset_mid, cov_unaligned, cov_pageedge,
                  cov_maxcnt, cov_cfg0, cov_coarse));
    n_cov = cov_skip + cov_skip_stalled + cov_stall_mid + cov_valid_noready + cov_dbuf_stall + cov_first_last +
            cov_b2b + cov_wrap + cov_ptr_wrap + cov_idle_rise + cov_reset_mid + cov_unaligned + cov_pageedge +
            cov_maxcnt + cov_cfg0 + cov_coarse;
    done = 1;
  end

  initial begin : wd
    #(10 * (NRAND * 40 + 3000000));
    if (!done) begin n_fail++; $display("[%0t] FAIL rseg_ch(ch=%0d) watchdog: test did not finish", $time, CH_ID); done = 1; end
  end
endmodule

module tb_rmc_cif_r_burst_spltr_ch;
  int p0, f0, s0, c0, p1, f1, s1, c1, p2, f2, s2, c2, p3, f3, s3, c3, p4, f4, s4, c4, p5, f5, s5, c5;
  bit d0, d1, d2, d3, d4, d5;
  //           CH_ID Q   ROB MAX SLOTS SEED NRAND      (N_SLOTS >= MAX_PKTS, all powers of two)
  rseg_ch_chk #(0,   4,   4, 16, 16,   1,  3500) u0 (p0, f0, s0, c0, d0);   // N_SLOTS == MAX_PKTS
  rseg_ch_chk #(1,   4,   4, 16, 32,   2,  3500) u1 (p1, f1, s1, c1, d1);
  rseg_ch_chk #(0,  32,  32, 64, 64,   3,  3500) u2 (p2, f2, s2, c2, d2);   // RTL defaults, whole-page requests
  rseg_ch_chk #(1,   8,  32, 64, 64,   4,  3500) u3 (p3, f3, s3, c3, d3);   // queue shallower than the ROB
  rseg_ch_chk #(0,   2,   2,  4,  4,   5,  2800) u4 (p4, f4, s4, c4, d4);   // tiny queue and ring
  rseg_ch_chk #(1,   2,   2,  4,  4,   6,  2800) u5 (p5, f5, s5, c5, d5);
  initial begin
    wait (d0 && d1 && d2 && d3 && d4 && d5);
    #20;
    $display("rseg_ch configs: %0d/%0d  %0d/%0d  %0d/%0d  %0d/%0d  %0d/%0d  %0d/%0d (pass/fail), packets issued %0d %0d %0d %0d %0d %0d",
             p0, f0, p1, f1, p2, f2, p3, f3, p4, f4, p5, f5, s0, s1, s2, s3, s4, s5);
    $display("coverage markers (of 16): %0d %0d %0d %0d %0d %0d", c0, c1, c2, c3, c4, c5);
    if (f0 + f1 + f2 + f3 + f4 + f5 == 0) $display("TB RESULT: PASS");
    else begin $display("TB RESULT: FAIL"); $fatal(1, "r_burst_spltr_ch unit test failures"); end
    $finish;
  end
endmodule
