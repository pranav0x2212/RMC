`timescale 1ns/1ps
// Unit testbench for rmc_cif_rrob (simulation only).
//
// Checker: a program-order scoreboard built from the documented contract, compared with the
// DUT's ports every cycle. It never reads DUT internals (the DUT has no DFT taps any more).
// Requests are tracked by a running sequence number; the ROB slot a request must be given is
// seq mod ROB_DEPTH (slots are handed out sequentially, see ROB_IDX). Per request it records
// what the stimulus supplied: id, validated, n_pkts and the trim meta at ADMIT, the per-channel
// head pointers at SEG_LAST, and the completion count / error flag from cmpl_*.
//
// Contract used (sources: rmc_cif_rrob.sv header, fv/cif/rmc_cif_rrob/rmc_cif_rrob_fv.sv):
//   * A_ID_READY = free slot; it does not depend on RESP_READY (a retire frees a slot only
//     from the next cycle on).
//   * Admit meta (id, validated, n_pkts, len, size, addr_lo, base_ch) is latched at admit;
//     the per-channel head pointers are latched at that slot's SEG_LAST.
//   * RESP_VALID iff the oldest request is sealed (SEG_LAST seen) and has seen n_pkts
//     completions; retire is strictly in program order; RESP_* hold while RESP_READY is low.
//   * RESP_RESP = SLVERR if the request was not validated or any completion carried cmpl_err.
//   * Legal environment (formal ENV assumptions): validated <=> ADMIT_NPKTS != 0 and
//     ADMIT_NPKTS <= MAX_PKTS (the splitter reports 0 packets for a rejected request, which
//     retires SLVERR after SEG_LAST with no completions); SEG_LAST names a live, unsealed
//     slot; a completion names a live slot and never exceeds its n_pkts. The TB checks its
//     own stimulus against these rules and counts a violation as a failure, so a TB bug
//     cannot silently drive an illegal protocol.
// Not covered here (not in the contract, would need an RTL/protocol decision): validated with
// n_pkts == 0, unvalidated with n_pkts != 0, extra/duplicate completions, SEG_LAST re-pulses.

module rrob_chk #(
  parameter int ROB_DEPTH = 32,
  parameter int MAX_PKTS  = 16,
  parameter int AXI_IDW   = 8,
  parameter int N_SLOTS   = 32,
  parameter int SEED      = 1,
  parameter int NRAND     = 4000
)(
  output int  n_pass,
  output int  n_fail,
  output int  n_cov,      // coverage markers reached (of 14)
  output bit  done
);
  import rmc_cfg_pkg::*;
  localparam int ROB_IDX_W    = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1;
  localparam int CNT_W        = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1;
  localparam int SLOT_PTR_W   = (N_SLOTS   > 1) ? $clog2(N_SLOTS)    : 1;
  localparam int HEAD_W       = N_CH * SLOT_PTR_W;
  localparam int RESP_CAT_W   = AXI_IDW + 2 + HEAD_W + CH_W + PAGE_LSB + 8 + 3;

  logic aclk = 1'b0, aresetn = 1'b1;   // starts high so the first reset is a real negedge
  always #5 aclk = ~aclk;

  logic                  cmpl_valid = 0, cmpl_err = 0;
  logic [ROB_IDX_W-1:0]  cmpl_rob_index = '0;
  logic [AXI_IDW:0]      AW_REQUEST = '0;
  logic [CNT_W-1:0]      ADMIT_NPKTS = '0;
  logic [PAGE_LSB-1:0]   ADMIT_ADDR_LO = '0;
  logic [7:0]            ADMIT_LEN = '0;
  logic [2:0]            ADMIT_SIZE = '0;
  logic [CH_W-1:0]       ADMIT_BASE_CH = '0;
  logic                  A_ID_VALID = 0, A_ID_READY;
  logic [ROB_IDX_W-1:0]  ROB_IDX;
  logic                  SEG_LAST = 0;
  logic [ROB_IDX_W-1:0]  SEG_ROB_IDX = '0;
  logic [N_CH-1:0][SLOT_PTR_W-1:0] SEG_HEAD_PTR = '0;
  logic                  RESP_VALID, RESP_READY = 0;
  logic [AXI_IDW-1:0]    RESP_ID;
  logic [1:0]            RESP_RESP;
  logic [N_CH-1:0][SLOT_PTR_W-1:0] RESP_HEAD_PTR;
  logic [CH_W-1:0]       RESP_BASE_CH;
  logic [PAGE_LSB-1:0]   RESP_ADDR_LO;
  logic [7:0]            RESP_LEN;
  logic [2:0]            RESP_SIZE;

  rmc_cif_rrob #(.ROB_DEPTH(ROB_DEPTH), .MAX_PKTS(MAX_PKTS), .AXI_IDW(AXI_IDW), .N_SLOTS(N_SLOTS)) dut (
    .aclk, .aresetn, .cmpl_valid, .cmpl_rob_index, .cmpl_err,
    .AW_REQUEST, .ADMIT_NPKTS, .ADMIT_ADDR_LO, .ADMIT_LEN, .ADMIT_SIZE, .ADMIT_BASE_CH,
    .A_ID_VALID, .A_ID_READY, .ROB_IDX,
    .SEG_LAST, .SEG_ROB_IDX, .SEG_HEAD_PTR,
    .RESP_VALID, .RESP_READY, .RESP_ID, .RESP_RESP, .RESP_HEAD_PTR, .RESP_BASE_CH,
    .RESP_ADDR_LO, .RESP_LEN, .RESP_SIZE);

  wire [RESP_CAT_W-1:0] resp_cat = {RESP_ID, RESP_RESP, RESP_HEAD_PTR, RESP_BASE_CH,
                                    RESP_ADDR_LO, RESP_LEN, RESP_SIZE};

  // ---------------- scoreboard ----------------
  // Records are keyed by program-order sequence number (index = seq % ROB_DEPTH; at most
  // ROB_DEPTH requests are live, so an index never holds two live requests).
  int  adm_seq, ret_seq;                    // requests admitted / retired since reset
  logic [AXI_IDW-1:0]  s_id     [ROB_DEPTH];
  bit                  s_ok     [ROB_DEPTH];   // validated
  bit                  s_derr   [ROB_DEPTH];   // a completion carried cmpl_err
  bit                  s_sealed [ROB_DEPTH];   // SEG_LAST seen
  int                  s_n      [ROB_DEPTH];   // n_pkts given at admit
  int                  s_cnt    [ROB_DEPTH];   // completions seen
  logic [7:0]          s_len    [ROB_DEPTH];
  logic [2:0]          s_size   [ROB_DEPTH];
  logic [PAGE_LSB-1:0] s_alo    [ROB_DEPTH];
  logic [CH_W-1:0]     s_bch    [ROB_DEPTH];
  logic [N_CH-1:0][SLOT_PTR_W-1:0] s_hd [ROB_DEPTH];

  int  cyc = 0, n_ret = 0, n_adm = 0;
  bit                    stall_prev = 0;     // previous cycle: RESP_VALID held by RESP_READY low
  logic [RESP_CAT_W-1:0] resp_saved;
  int  p_adm = 50, p_seal = 33, p_cmpl = 50, p_rdy = 75;   // random-phase percentages
  bit  ord = 0;                                            // random phase: seal in program order

  // coverage / vacuity markers (each is asserted at the end, so a missing scenario fails)
  bit cov_full = 0, cov_adm_wrap = 0, cov_ret_wrap = 0, cov_early = 0, cov_err_val = 0,
      cov_err_ds = 0, cov_zero = 0, cov_stall = 0, cov_simul = 0, cov_ooo = 0,
      cov_multi = 0, cov_refused = 0, cov_maxn = 0, cov_reset_busy = 0;

  task automatic chk(input bit c, input string msg);
    if (c) n_pass++;
    else begin n_fail++; $display("[%0t] FAIL rrob(D=%0d,P=%0d) %s", $time, ROB_DEPTH, MAX_PKTS, msg); end
  endtask

  task automatic stim_chk(input bit c, input string msg);
    if (!c) begin n_fail++; $display("[%0t] FAIL rrob(D=%0d,P=%0d) TB STIMULUS ILLEGAL: %s", $time, ROB_DEPTH, MAX_PKTS, msg); end
  endtask

  task model_reset();
    adm_seq = 0; ret_seq = 0; stall_prev = 0;
    for (int i = 0; i < ROB_DEPTH; i++) begin
      s_sealed[i] = 0; s_cnt[i] = 0; s_n[i] = 0; s_derr[i] = 0; s_ok[i] = 1;
    end
  endtask

  function automatic int cap(input int n);
    return (n > MAX_PKTS) ? MAX_PKTS : n;
  endfunction

  // live request (admitted, not retired) owning ROB slot idx, as a sequence number; -1 if none
  function automatic int find_live(input int idx);
    for (int k = 0; k < adm_seq - ret_seq; k++)
      if (((ret_seq + k) % ROB_DEPTH) == idx) return ret_seq + k;
    return -1;
  endfunction

  function automatic bit rec_done(input int s);
    return s_sealed[s % ROB_DEPTH] && (s_cnt[s % ROB_DEPTH] == s_n[s % ROB_DEPTH]);
  endfunction

  // random live request: mode 0 = not yet sealed, 1 = still owes completions. Returns its ROB
  // slot or -1. in_order (mode 0 only) picks the oldest unsealed one, as the splitter does.
  function automatic int pick(input int mode, input bit in_order);
    int sel, s, cnt;
    cnt = adm_seq - ret_seq;
    if (cnt == 0) return -1;
    if (in_order && mode == 0) begin
      for (int k = 0; k < cnt; k++) begin
        s = ret_seq + k;
        if (!s_sealed[s % ROB_DEPTH]) return s % ROB_DEPTH;
      end
      return -1;
    end
    sel = $urandom % cnt;
    for (int j = 0; j < cnt; j++) begin
      s = ret_seq + ((sel + j) % cnt);
      if (mode == 0) begin
        if (!s_sealed[s % ROB_DEPTH]) return s % ROB_DEPTH;
      end else begin
        if (s_cnt[s % ROB_DEPTH] < s_n[s % ROB_DEPTH]) return s % ROB_DEPTH;
      end
    end
    return -1;
  endfunction

  // One clock cycle: the caller has set the inputs. Compare the DUT outputs with the
  // scoreboard, advance the scoreboard with this cycle's inputs, then clock the DUT.
  task tick();
    bit exp_rdy, adm, rv, ret;
    int hs, s, sl, nl, nun;
    bit young_done;
    #1;
    cyc++;
    nl      = adm_seq - ret_seq;
    hs      = ret_seq % ROB_DEPTH;
    exp_rdy = (nl != ROB_DEPTH);
    rv      = (nl != 0) && s_sealed[hs] && (s_cnt[hs] == s_n[hs]);
    adm     = aresetn && A_ID_VALID && exp_rdy;
    ret     = aresetn && rv && RESP_READY;

    // the stimulus itself must obey the environment contract
    if (aresetn) begin
      if (adm) begin
        stim_chk(AW_REQUEST[0] == (ADMIT_NPKTS != '0), "admit: validated must equal (n_pkts != 0)");
        stim_chk(int'(ADMIT_NPKTS) <= MAX_PKTS, "admit: n_pkts > MAX_PKTS");
      end
      if (SEG_LAST) begin
        s = find_live(int'(SEG_ROB_IDX));
        if (s < 0) stim_chk(0, $sformatf("SEG_LAST for non-live slot %0d", SEG_ROB_IDX));
        else       stim_chk(!s_sealed[s % ROB_DEPTH], $sformatf("second SEG_LAST for slot %0d", SEG_ROB_IDX));
      end
      if (cmpl_valid) begin
        s = find_live(int'(cmpl_rob_index));
        if (s < 0) stim_chk(0, $sformatf("completion for non-live slot %0d", cmpl_rob_index));
        else       stim_chk(s_cnt[s % ROB_DEPTH] < s_n[s % ROB_DEPTH], $sformatf("completion beyond n_pkts for slot %0d", cmpl_rob_index));
      end
    end

    // DUT outputs vs the scoreboard
    chk(A_ID_READY === exp_rdy, $sformatf("c%0d A_ID_READY %b exp %b (live %0d)", cyc, A_ID_READY, exp_rdy, nl));
    chk(ROB_IDX === ROB_IDX_W'(adm_seq % ROB_DEPTH), $sformatf("c%0d ROB_IDX %0d exp %0d", cyc, ROB_IDX, adm_seq % ROB_DEPTH));
    chk(RESP_VALID === rv, $sformatf("c%0d RESP_VALID %b exp %b", cyc, RESP_VALID, rv));
    if (rv) begin
      chk(RESP_ID === s_id[hs], $sformatf("c%0d RESP_ID %0h exp %0h", cyc, RESP_ID, s_id[hs]));
      chk(RESP_RESP === ((!s_ok[hs] || s_derr[hs]) ? 2'b10 : 2'b00), $sformatf("c%0d RESP_RESP %b (ok=%b derr=%b)", cyc, RESP_RESP, s_ok[hs], s_derr[hs]));
      chk(RESP_LEN === s_len[hs] && RESP_SIZE === s_size[hs], $sformatf("c%0d RESP_LEN/SIZE %0d/%0d exp %0d/%0d", cyc, RESP_LEN, RESP_SIZE, s_len[hs], s_size[hs]));
      chk(RESP_ADDR_LO === s_alo[hs] && RESP_BASE_CH === s_bch[hs], $sformatf("c%0d RESP_ADDR_LO/BASE_CH %0h/%0d exp %0h/%0d", cyc, RESP_ADDR_LO, RESP_BASE_CH, s_alo[hs], s_bch[hs]));
      chk(RESP_HEAD_PTR === s_hd[hs], $sformatf("c%0d RESP_HEAD_PTR %0h exp %0h", cyc, RESP_HEAD_PTR, s_hd[hs]));
    end
    if (stall_prev && aresetn)
      chk(RESP_VALID === 1'b1 && resp_cat === resp_saved, $sformatf("c%0d response changed while RESP_READY was low", cyc));

    // scoreboard update (completion, SEG_LAST, retire, admit target distinct requests)
    if (!aresetn) model_reset();
    else begin
      if (cmpl_valid) begin
        s = find_live(int'(cmpl_rob_index));
        if (s >= 0) begin
          if (!s_sealed[s % ROB_DEPTH]) cov_early = 1;       // completion beat SEG_LAST
          s_cnt[s % ROB_DEPTH] = s_cnt[s % ROB_DEPTH] + 1;
          if (cmpl_err) s_derr[s % ROB_DEPTH] = 1;
        end
      end
      if (SEG_LAST) begin
        s = find_live(int'(SEG_ROB_IDX));
        if (s >= 0) begin
          s_sealed[s % ROB_DEPTH] = 1;
          s_hd[s % ROB_DEPTH]     = SEG_HEAD_PTR;
        end
      end
      // state predicates worth having seen: a younger request complete while the oldest is not;
      // two or more requests live and unsealed at once
      nun = 0; young_done = 0;
      for (int k = 0; k < adm_seq - ret_seq; k++) begin
        s = ret_seq + k;
        if (!s_sealed[s % ROB_DEPTH]) nun = nun + 1;
        if (k > 0 && rec_done(s) && !rec_done(ret_seq)) young_done = 1;
      end
      if (young_done) cov_ooo = 1;
      if (nun >= 2)   cov_multi = 1;

      if (ret) begin
        if (!s_ok[hs])        cov_err_val = 1;
        else if (s_derr[hs])  cov_err_ds  = 1;
        if (s_n[hs] == 0)         cov_zero    = 1;
        if (s_n[hs] == MAX_PKTS)  cov_maxn    = 1;
        if (hs == ROB_DEPTH-1)    cov_ret_wrap = 1;
        ret_seq = ret_seq + 1; n_ret++;
      end
      if (adm) begin
        sl = adm_seq % ROB_DEPTH;
        s_id[sl] = AW_REQUEST[AXI_IDW:1]; s_ok[sl] = AW_REQUEST[0];
        s_n[sl] = int'(ADMIT_NPKTS); s_cnt[sl] = 0; s_sealed[sl] = 0; s_derr[sl] = 0;
        s_len[sl] = ADMIT_LEN; s_size[sl] = ADMIT_SIZE; s_alo[sl] = ADMIT_ADDR_LO; s_bch[sl] = ADMIT_BASE_CH;
        if (adm_seq >= ROB_DEPTH && sl == 0) cov_adm_wrap = 1;   // second lap reached slot 0
        adm_seq = adm_seq + 1; n_adm++;
      end
      if (ret && adm)              cov_simul   = 1;
      if (rv && !RESP_READY)       cov_stall   = 1;
      if (A_ID_VALID && !exp_rdy)  cov_refused = 1;
      if (adm_seq - ret_seq == ROB_DEPTH) cov_full = 1;
    end
    stall_prev = aresetn && rv && !RESP_READY;
    resp_saved = resp_cat;
    @(posedge aclk); #1;
    A_ID_VALID = 0; SEG_LAST = 0; cmpl_valid = 0; cmpl_err = 0;
  endtask

  // ---- input helpers (set; the caller then calls tick) ----
  task set_admit_meta(input int id, input bit ok, input int n, input int len, input int sz,
                      input logic [PAGE_LSB-1:0] alo, input logic [CH_W-1:0] bch);
    A_ID_VALID = 1; AW_REQUEST = {AXI_IDW'(id), ok}; ADMIT_NPKTS = CNT_W'(n);
    ADMIT_LEN = 8'(len); ADMIT_SIZE = 3'(sz); ADMIT_ADDR_LO = alo; ADMIT_BASE_CH = bch;
  endtask
  // validated <=> n != 0 is the splitter's contract; callers pass a consistent pair
  task set_admit(input int id, input bit ok, input int n);
    set_admit_meta(id, ok, n, $urandom, $urandom, PAGE_LSB'($urandom), CH_W'($urandom));
  endtask
  task set_seal_hp(input int idx, input logic [HEAD_W-1:0] hp);
    SEG_LAST = 1; SEG_ROB_IDX = ROB_IDX_W'(idx); SEG_HEAD_PTR = hp;
  endtask
  task set_seal(input int idx);
    logic [HEAD_W-1:0] hp;
    for (int c = 0; c < N_CH; c++) hp[c*SLOT_PTR_W +: SLOT_PTR_W] = SLOT_PTR_W'($urandom);
    set_seal_hp(idx, hp);
  endtask
  task set_cmpl(input int idx, input bit err);
    cmpl_valid = 1; cmpl_rob_index = ROB_IDX_W'(idx); cmpl_err = err;
  endtask
  task rand_admit();
    bit ok; int n;
    ok = ($urandom % 6) != 0;
    if (!ok) n = 0;
    else if (($urandom % 8) == 0) n = MAX_PKTS;
    else n = 1 + ($urandom % MAX_PKTS);
    set_admit($urandom, ok, n);
  endtask
  task idle(input int n);
    repeat (n) tick();
  endtask
  task do_reset();
    aresetn = 1; #1; aresetn = 0; RESP_READY = 0; tick(); tick(); aresetn = 1; tick();
  endtask

  // asynchronous reset between clock edges, with whatever is in flight
  task midrun_reset();
    if (adm_seq != ret_seq) cov_reset_busy = 1;
    #2 aresetn = 0; #1;
    chk(A_ID_READY === 1'b1 && RESP_VALID === 1'b0 && ROB_IDX === '0,
        "async reset clears occupancy/pointers immediately");
    RESP_READY = 0;
    model_reset();
    tick(); aresetn = 1; tick();
    chk(A_ID_READY === 1'b1 && RESP_VALID === 1'b0 && ROB_IDX === '0, "usable after reset");
  endtask

  // seal and complete everything outstanding and retire it (with a timeout)
  task drain();
    int guard, e, s;
    RESP_READY = 1;
    e = adm_seq;
    for (s = ret_seq; s < e; s++)
      if (s >= ret_seq && !s_sealed[s % ROB_DEPTH]) begin set_seal(s % ROB_DEPTH); tick(); end
    for (s = ret_seq; s < e; s++)
      while (s >= ret_seq && s_cnt[s % ROB_DEPTH] < s_n[s % ROB_DEPTH]) begin
        set_cmpl(s % ROB_DEPTH, 0); tick();
      end
    guard = 0;
    while (adm_seq != ret_seq && guard < 4*ROB_DEPTH + 8) begin tick(); guard++; end
    chk(adm_seq == ret_seq, "drain timeout (deadlock)");
    RESP_READY = 0;
  endtask

  // admit + retire in the same cycle, repeatedly, with `prefill` completed requests parked
  // behind RESP_READY so the retire and the admit hit different slots and the pointers lap
  task simul_lap(input int prefill, input int iters);
    int idx, n;
    bit ok;
    drain();
    for (int p = 0; p < prefill; p++) begin
      idx = adm_seq % ROB_DEPTH;
      set_admit(100 + p, 1, 1); tick();
      set_seal(idx); tick();
      set_cmpl(idx, 0); tick();
    end
    chk(RESP_VALID === 1'b1, "T6 prefill: oldest is complete and waiting");
    for (int r = 0; r < iters; r++) begin
      idx = adm_seq % ROB_DEPTH;
      ok = ($urandom % 5) != 0; n = ok ? 1 : 0;
      RESP_READY = 1; set_admit(r, ok, n); tick();     // retire head + admit together
      chk(adm_seq - ret_seq == prefill, $sformatf("T6 occupancy after simultaneous admit/retire (iter %0d)", r));
      RESP_READY = 0;
      set_seal(idx); tick();
      if (n > 0) begin set_cmpl(idx, 0); tick(); end
    end
    drain();
  endtask

  // full-width / all-bits-set / alternating metadata, with literal expected values (not taken
  // from the scoreboard) so stuck or swapped fields and truncation are caught
  task run_extreme(input int mode);
    int idx, n;
    logic [AXI_IDW-1:0]  eid;
    logic [7:0]          el;
    logic [2:0]          es;
    logic [PAGE_LSB-1:0] ea;
    logic [CH_W-1:0]     eb;
    logic [HEAD_W-1:0]   eh;
    if (mode == 1) begin eid = '1; el = 8'hFF; es = 3'b111; ea = '1; eb = '1; eh = '1; n = MAX_PKTS; end
    else if (mode == 0) begin eid = '0; el = 8'h00; es = 3'b000; ea = '0; eb = '0; eh = '0; n = 1; end
    else begin
      eid = AXI_IDW'(8'hA5); el = 8'h5A; es = 3'b101; ea = PAGE_LSB'(12'hA5A); eb = CH_W'(1);
      for (int c = 0; c < N_CH; c++) eh[c*SLOT_PTR_W +: SLOT_PTR_W] = SLOT_PTR_W'(c + 1);
      n = cap(3);
    end
    idx = adm_seq % ROB_DEPTH;
    set_admit_meta(int'(eid), 1, n, int'(el), int'(es), ea, eb); tick();
    set_seal_hp(idx, eh); tick();
    for (int k = 0; k < n; k++) begin
      chk(RESP_VALID === 1'b0, "T9 not valid before the last completion");
      set_cmpl(idx, 0); tick();
    end
    chk(RESP_VALID === 1'b1 && RESP_RESP === 2'b00 && RESP_ID === eid && RESP_LEN === el && RESP_SIZE === es &&
        RESP_ADDR_LO === ea && RESP_BASE_CH === eb && RESP_HEAD_PTR === eh,
        $sformatf("T9 extreme meta mode %0d: id %0h len %0h size %0h alo %0h bch %0h head %0h", mode,
                  RESP_ID, RESP_LEN, RESP_SIZE, RESP_ADDR_LO, RESP_BASE_CH, RESP_HEAD_PTR));
    RESP_READY = 1; tick(); RESP_READY = 0;
  endtask

  // ---------------- tests ----------------
  initial begin : main
    int idx, idx2, n, guard, seed_v, pos;
    n_pass = 0; n_fail = 0; n_cov = 0; done = 0;
    model_reset();
    seed_v = SEED;
    void'($urandom(seed_v));
    do_reset();

    // --- T1 reset state
    chk(A_ID_READY === 1'b1 && RESP_VALID === 1'b0 && ROB_IDX === '0, "T1 reset state");
    idle(3);

    // --- T2 one request, full lifecycle, RESP_READY stall
    drain();
    n = cap(3); idx = adm_seq % ROB_DEPTH;
    set_admit(8'hA1, 1, n); tick();
    idle(2);
    chk(RESP_VALID === 1'b0, "T2 no response before SEG_LAST / completions");
    for (int k = 0; k < n - 1; k++) begin set_cmpl(idx, 0); tick(); end
    set_seal(idx); tick();
    chk(RESP_VALID === 1'b0, "T2 sealed with n-1 of n completions: not valid");
    set_cmpl(idx, 0); tick();
    chk(RESP_VALID === 1'b1 && RESP_RESP === 2'b00 && RESP_ID === AXI_IDW'(8'hA1), "T2 valid at n/n, OKAY, own id");
    RESP_READY = 0; idle(3);
    chk(RESP_VALID === 1'b1, "T2 response held under RESP_READY low");
    RESP_READY = 1; tick();
    chk(RESP_VALID === 1'b0 && A_ID_READY === 1'b1, "T2 retired, empty");
    RESP_READY = 0;

    // --- T3 completions before SEG_LAST (early) must still count
    n = cap(2); idx = adm_seq % ROB_DEPTH;
    set_admit(8'hB2, 1, n); tick();
    for (int k = 0; k < n; k++) begin set_cmpl(idx, 0); tick(); end
    chk(RESP_VALID === 1'b0, "T3 count reached but not sealed: no response");
    RESP_READY = 1; set_seal(idx); tick();
    chk(RESP_VALID === 1'b1, "T3 valid once sealed");
    tick(); RESP_READY = 0;
    chk(adm_seq == ret_seq, "T3 retired");

    // --- T4 error responses
    // (a) rejected request: zero packets, validated = 0. No completions ever; retires SLVERR
    //     only after SEG_LAST.
    idx = adm_seq % ROB_DEPTH;
    set_admit(8'hC3, 0, 0); tick();
    idle(2);
    chk(RESP_VALID === 1'b0, "T4a zero-packet request: not valid before SEG_LAST");
    set_seal(idx); tick();
    chk(RESP_VALID === 1'b1 && RESP_RESP === 2'b10 && RESP_ID === AXI_IDW'(8'hC3), "T4a zero-packet request: SLVERR after SEG_LAST");
    RESP_READY = 1; tick(); RESP_READY = 0;
    // (b) downstream error on the first / last completion
    for (int e = 0; e < 2; e++) begin
      n = cap(3); idx = adm_seq % ROB_DEPTH; pos = (e == 0) ? 0 : n - 1;
      set_admit(8'hD4 + e, 1, n); tick();
      set_seal(idx); tick();
      for (int k = 0; k < n; k++) begin set_cmpl(idx, k == pos); tick(); end
      chk(RESP_VALID === 1'b1 && RESP_RESP === 2'b10, $sformatf("T4b downstream error at completion %0d: SLVERR", pos));
      RESP_READY = 1; tick(); RESP_READY = 0;
    end
    // (c) cmpl_err without cmpl_valid is not a completion and must not poison the entry
    idx = adm_seq % ROB_DEPTH;
    set_admit(8'hE5, 1, 1); tick();
    set_seal(idx); cmpl_err = 1; tick();
    set_cmpl(idx, 0); tick();
    chk(RESP_VALID === 1'b1 && RESP_RESP === 2'b00, "T4c cmpl_err without cmpl_valid ignored");
    RESP_READY = 1; tick(); RESP_READY = 0;
    // (d) errors must not leak into later users of the same slots: a full lap of clean requests
    for (int r = 0; r < ROB_DEPTH + 1; r++) begin
      idx = adm_seq % ROB_DEPTH;
      set_admit(r, 1, 1); tick();
      set_seal(idx); tick();
      set_cmpl(idx, 0); tick();
      chk(RESP_VALID === 1'b1 && RESP_RESP === 2'b00, $sformatf("T4d clean request %0d after errors: OKAY", r));
      RESP_READY = 1; tick(); RESP_READY = 0;
    end
    // (e) in-order: a sealed zero-packet request waits behind an older request still owing packets
    if (ROB_DEPTH >= 2) begin
      idx = adm_seq % ROB_DEPTH;
      set_admit(1, 1, cap(2)); tick();
      idx2 = adm_seq % ROB_DEPTH;
      set_admit(2, 0, 0); tick();
      set_seal(idx2); tick();
      idle(2);
      chk(RESP_VALID === 1'b0, "T4e zero-packet request must not overtake the older request");
      RESP_READY = 1; set_seal(idx); tick();
      for (int k = 0; k < cap(2); k++) begin set_cmpl(idx, 0); tick(); end
      chk(RESP_VALID === 1'b1 && RESP_ID === AXI_IDW'(1), "T4e older request retires first");
      tick();
      chk(RESP_VALID === 1'b1 && RESP_ID === AXI_IDW'(2) && RESP_RESP === 2'b10, "T4e then the zero-packet request, SLVERR");
      tick(); RESP_READY = 0;
      chk(adm_seq == ret_seq, "T4e drained");
    end

    // --- T5 fill to capacity, backpressure, reverse seal/complete, in-order retire
    drain();
    for (int i = 0; i < ROB_DEPTH; i++) begin set_admit(i + 1, 1, 1 + (i % MAX_PKTS)); tick(); end
    chk(A_ID_READY === 1'b0, "T5 full: ready low");
    set_admit(8'hEE, 1, 1); tick();                                  // offered while full: refused
    chk(A_ID_READY === 1'b0 && adm_seq - ret_seq == ROB_DEPTH, "T5 refused admit leaves occupancy");
    for (int i = ROB_DEPTH - 1; i >= 0; i--) begin set_seal((ret_seq + i) % ROB_DEPTH); tick(); end
    for (int i = ROB_DEPTH - 1; i >= 0; i--) begin
      idx = (ret_seq + i) % ROB_DEPTH;
      for (int k = 0; k < s_n[idx]; k++) begin set_cmpl(idx, 0); tick(); end
      if (i > 0) chk(RESP_VALID === 1'b0, $sformatf("T5 request %0d from the oldest is done but the oldest is not: no retire", i));
    end
    chk(RESP_VALID === 1'b1, "T5 oldest done: valid");
    // T5b: the oldest retires while the ROB is full; the offered admit is refused in that cycle
    RESP_READY = 1; set_admit(8'hF1, 1, 1); tick();
    chk(adm_seq - ret_seq == ROB_DEPTH - 1 && A_ID_READY === 1'b1, "T5b retire frees a slot only for the next cycle");
    guard = 0; while (adm_seq != ret_seq && guard < 2*ROB_DEPTH + 8) begin tick(); guard++; end
    chk(adm_seq == ret_seq, "T5 drained in order");
    RESP_READY = 0;

    // --- T6 admit and retire in the same cycle (different slots), lapping the ring
    if (ROB_DEPTH >= 2) begin
      simul_lap(1, 2*ROB_DEPTH + 3);
      if (ROB_DEPTH >= 3) simul_lap(ROB_DEPTH - 1, 2*ROB_DEPTH + 3);   // one below full
    end

    // --- T9 field widths: all-ones, all-zeros and a channel-distinct pattern in every slot
    drain();
    for (int mode = 1; mode >= 0; mode--)
      for (int r = 0; r < ROB_DEPTH; r++) run_extreme(mode);
    for (int r = 0; r < ROB_DEPTH; r++) run_extreme(2);

    // --- T10 a stalled response is not disturbed by younger activity
    drain();
    idx = adm_seq % ROB_DEPTH;
    set_admit(8'h31, 1, 1); tick(); set_seal(idx); tick(); set_cmpl(idx, 0); tick();
    chk(RESP_VALID === 1'b1, "T10 head valid and stalled");
    for (int e = 1; e < ROB_DEPTH && e <= 3; e++) begin
      idx = adm_seq % ROB_DEPTH;
      set_admit(8'h40 + e, 1, 1); tick();
      set_seal(idx); tick(); set_cmpl(idx, 0); tick();
      chk(RESP_VALID === 1'b1 && RESP_ID === AXI_IDW'(8'h31), "T10 stalled head unchanged by younger activity");
    end
    drain();

    // --- T11 several events in one cycle
    idx = adm_seq % ROB_DEPTH;                                      // (a) last completion + SEG_LAST together
    set_admit(8'h51, 1, 1); tick();
    set_seal(idx); set_cmpl(idx, 0); tick();
    chk(RESP_VALID === 1'b1 && RESP_RESP === 2'b00, "T11a SEG_LAST and last completion in one cycle");
    RESP_READY = 1; tick(); RESP_READY = 0;
    idx = adm_seq % ROB_DEPTH;                                      // (b) same, with the error flag
    set_admit(8'h52, 1, 1); tick();
    set_seal(idx); set_cmpl(idx, 1); tick();
    chk(RESP_VALID === 1'b1 && RESP_RESP === 2'b10, "T11b error on the completion that arrives with SEG_LAST");
    RESP_READY = 1; tick(); RESP_READY = 0;
    if (ROB_DEPTH >= 3) begin                                       // (c) admit + seal(older) + complete(other)
      idx = adm_seq % ROB_DEPTH;
      set_admit(8'h61, 1, cap(2)); tick();
      idx2 = adm_seq % ROB_DEPTH;
      set_admit(8'h62, 1, 1); tick();
      set_admit(8'h63, 1, 1); set_seal(idx); set_cmpl(idx2, 0); tick();
      drain();
    end

    // --- T7 constrained random, in phases (balanced / fill / drain / stalled response / in-order)
    for (int c = 0; c < NRAND; c++) begin
      int cand;
      if (c % 400 == 0) begin
        case ((c / 400) % 5)
          0: begin p_adm = 50; p_seal = 33; p_cmpl = 50; p_rdy = 75; ord = 0; end
          1: begin p_adm = 90; p_seal = 20; p_cmpl = 20; p_rdy = 10; ord = 0; end
          2: begin p_adm = 10; p_seal = 60; p_cmpl = 80; p_rdy = 90; ord = 0; end
          3: begin p_adm = 40; p_seal = 50; p_cmpl = 50; p_rdy = 5;  ord = 1; end
          default: begin p_adm = 60; p_seal = 70; p_cmpl = 70; p_rdy = 60; ord = 1; end
        endcase
      end
      if (c == NRAND / 2) midrun_reset();                           // reset with a busy ROB
      if (($urandom % 100) < p_seal) begin
        cand = pick(0, ord);
        if (cand >= 0) set_seal(cand);
      end
      if (($urandom % 100) < p_cmpl) begin
        cand = pick(1, 0);
        if (cand >= 0) set_cmpl(cand, ($urandom % 8) == 0);
      end
      if (($urandom % 100) < p_adm) rand_admit();
      RESP_READY = (($urandom % 100) < p_rdy);
      tick();
    end
    drain();

    // --- T8 asynchronous reset in the middle of activity; no state may survive it
    idx = adm_seq % ROB_DEPTH;
    set_admit(8'h71, 1, cap(2)); tick();
    set_cmpl(idx, 1); tick();                                       // one completion, with an error
    set_seal(idx); tick();
    if (ROB_DEPTH > 1) begin set_admit(8'h72, 0, 0); tick(); end
    midrun_reset();
    idx = adm_seq % ROB_DEPTH;
    n = cap(2);
    set_admit(8'h73, 1, n); tick();                                 // reuses slot 0 (old state was partial + errored)
    idle(2);
    chk(RESP_VALID === 1'b0, "T8 no inherited complete/sealed state after reset");
    set_seal(idx); tick();
    for (int k = 0; k < n; k++) begin
      chk(RESP_VALID === 1'b0, "T8 not valid before all new completions");
      set_cmpl(idx, 0); tick();
    end
    chk(RESP_VALID === 1'b1 && RESP_RESP === 2'b00 && RESP_ID === AXI_IDW'(8'h73), "T8 fresh request OKAY, own id (no stale error)");
    RESP_READY = 1; tick(); RESP_READY = 0; idle(2);

    // coverage / vacuity guards
    chk(n_ret > 3*ROB_DEPTH && n_adm > 3*ROB_DEPTH, $sformatf("stress retired %0d / admitted %0d", n_ret, n_adm));
    if (ROB_DEPTH == 1) begin cov_simul = 1; cov_ooo = 1; cov_multi = 1; end   // need two live slots
    chk(cov_full && cov_adm_wrap && cov_ret_wrap && cov_early && cov_err_val && cov_err_ds && cov_zero &&
        cov_stall && cov_simul && cov_ooo && cov_multi && cov_refused && cov_maxn && cov_reset_busy,
        $sformatf("coverage full=%b admwrap=%b retwrap=%b early=%b errval=%b errds=%b zero=%b stall=%b simul=%b ooo=%b multi=%b refused=%b maxn=%b rstbusy=%b",
                  cov_full, cov_adm_wrap, cov_ret_wrap, cov_early, cov_err_val, cov_err_ds, cov_zero,
                  cov_stall, cov_simul, cov_ooo, cov_multi, cov_refused, cov_maxn, cov_reset_busy));
    n_cov = cov_full + cov_adm_wrap + cov_ret_wrap + cov_early + cov_err_val + cov_err_ds + cov_zero +
            cov_stall + cov_simul + cov_ooo + cov_multi + cov_refused + cov_maxn + cov_reset_busy;
    done = 1;
  end

  // deadlock guard
  initial begin : wd
    #(10 * (NRAND * 8 + 200000));
    if (!done) begin n_fail++; $display("[%0t] FAIL rrob(D=%0d) watchdog: test did not finish", $time, ROB_DEPTH); done = 1; end
  end
endmodule

module tb_rmc_cif_rrob;
  int p0, f0, c0, p1, f1, c1, p2, f2, c2, p3, f3, c3, p4, f4, c4, p5, f5, c5;
  bit d0, d1, d2, d3, d4, d5;
  //        ROB_DEPTH MAX_PKTS AXI_IDW N_SLOTS SEED NRAND
  rrob_chk #(32, 16, 8, 32, 1, 4000) u0 (p0, f0, c0, d0);   // pow2 depth
  rrob_chk #( 4,  8, 4, 16, 2, 3000) u1 (p1, f1, c1, d1);   // small ring, wraps quickly
  rrob_chk #( 5,  7, 3,  8, 3, 3000) u2 (p2, f2, c2, d2);   // non-power-of-two depth
  rrob_chk #( 1,  2, 2,  2, 4, 1000) u3 (p3, f3, c3, d3);   // single-entry corner
  rrob_chk #(32, 64, 8, 32, 5, 4000) u4 (p4, f4, c4, d4);   // RTL defaults (MAX_PKTS = 64)
  rrob_chk #( 2,  1, 1,  2, 6, 1500) u5 (p5, f5, c5, d5);   // 1 packet max, 1-bit id
  initial begin
    wait (d0 && d1 && d2 && d3 && d4 && d5);
    #20;
    $display("rrob configs: D=32/P16 %0d pass/%0d fail  D=4 %0d/%0d  D=5 %0d/%0d  D=1 %0d/%0d  D=32/P64 %0d/%0d  D=2/P1 %0d/%0d",
             p0, f0, p1, f1, p2, f2, p3, f3, p4, f4, p5, f5);
    $display("coverage markers (of 14): %0d %0d %0d %0d %0d %0d", c0, c1, c2, c3, c4, c5);
    if (f0 + f1 + f2 + f3 + f4 + f5 == 0) $display("TB RESULT: PASS");
    else begin $display("TB RESULT: FAIL"); $fatal(1, "rrob unit test failures"); end
    $finish;
  end
endmodule
