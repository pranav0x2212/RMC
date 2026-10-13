`timescale 1ns/1ps
// Unit testbench for rmc_cif_rrob (simulation only). A cycle-accurate TB model of the
// documented contract (admit / SEG_LAST / completion / in-order retire) is compared with
// the DUT every cycle, plus directed and constrained-random sequences at several
// parameter sets. Expected values come from the model, never from DUT internals
// (dft_* taps are cross-checked as observation ports only).

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
  output int  n_cov,      // coverage markers reached (full, wrap, early cmpl, err, ...)
  output bit  done
);
  import rmc_cfg_pkg::*;
  localparam int ROB_IDX_W  = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH)  : 1;
  localparam int CNT_W      = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1;
  localparam int SLOT_PTR_W = (N_SLOTS   > 1) ? $clog2(N_SLOTS)    : 1;
  localparam int OCC_W      = $clog2(ROB_DEPTH + 1);
  localparam int PTR_W      = ROB_IDX_W;

  logic aclk = 1'b0, aresetn = 1'b1;   // starts high so the first reset is a real negedge
  always #5 aclk = ~aclk;

  logic                  cmpl_valid = 0, cmpl_err = 0;
  logic [ROB_IDX_W-1:0]  cmpl_rob_index = '0;
  logic [AXI_IDW:0]      AW_REQUEST = '0;
  logic [CNT_W-1:0]      ADMIT_NPKTS = '0;
  logic                  A_ID_VALID = 0, A_ID_READY;
  logic [ROB_IDX_W-1:0]  ROB_IDX;
  logic                  SEG_LAST = 0;
  logic [ROB_IDX_W-1:0]  SEG_ROB_IDX = '0;
  logic [PKT_OFF_W-1:0]  SEG_OFFSET = '0;
  logic [7:0]            SEG_LEN = '0;
  logic [2:0]            SEG_SIZE = '0;
  logic [SLOT_PTR_W-1:0] SEG_HEAD_PTR = '0;
  logic                  RESP_VALID, RESP_READY = 0;
  logic [AXI_IDW-1:0]    RESP_ID;
  logic [1:0]            RESP_RESP;
  logic [SLOT_PTR_W-1:0] RESP_HEAD_PTR;
  logic [PKT_OFF_W-1:0]  RESP_OFFSET;
  logic [7:0]            RESP_LEN;
  logic [2:0]            RESP_SIZE;
  logic [PTR_W-1:0]      d_alloc, d_retire;
  logic [OCC_W-1:0]      d_occ;
  logic                  d_admit, d_retfire;
  logic [ROB_DEPTH-1:0]  d_infl, d_err, d_derr;
  logic [ROB_DEPTH*CNT_W-1:0] d_out, d_np;
  logic [ROB_DEPTH*SLOT_PTR_W-1:0] d_head;

  rmc_cif_rrob #(.ROB_DEPTH(ROB_DEPTH), .MAX_PKTS(MAX_PKTS), .AXI_IDW(AXI_IDW), .N_SLOTS(N_SLOTS)) dut (
    .aclk, .aresetn, .cmpl_valid, .cmpl_rob_index, .cmpl_err,
    .AW_REQUEST, .ADMIT_NPKTS, .A_ID_VALID, .A_ID_READY, .ROB_IDX,
    .SEG_LAST, .SEG_ROB_IDX, .SEG_OFFSET, .SEG_LEN, .SEG_SIZE, .SEG_HEAD_PTR,
    .RESP_VALID, .RESP_READY, .RESP_ID, .RESP_RESP, .RESP_HEAD_PTR, .RESP_OFFSET, .RESP_LEN, .RESP_SIZE,
    .dft_alloc_ptr(d_alloc), .dft_retire_ptr(d_retire), .dft_occ(d_occ),
    .dft_admit_fire(d_admit), .dft_retire_fire(d_retfire), .dft_inflow_complt(d_infl),
    .dft_ent_err(d_err), .dft_ent_derr(d_derr), .dft_outflow_cnt(d_out), .dft_n_pkts(d_np),
    .dft_slot_head(d_head));

  // ---------------- TB model ----------------
  int  m_alloc, m_ret, m_occ;
  bit  m_live  [ROB_DEPTH];
  bit  m_ok    [ROB_DEPTH];
  bit  m_derr  [ROB_DEPTH];
  bit  m_infl  [ROB_DEPTH];
  int  m_n     [ROB_DEPTH];
  int  m_cnt   [ROB_DEPTH];
  int  m_id    [ROB_DEPTH];
  int  m_hd    [ROB_DEPTH];
  int  m_off   [ROB_DEPTH];
  int  m_len   [ROB_DEPTH];
  int  m_sz    [ROB_DEPTH];
  int  cyc = 0, max_occ = 0, n_ret = 0, n_adm = 0;
  bit  cov_full = 0, cov_wrap = 0, cov_early = 0, cov_err = 0, cov_stall = 0, cov_simul = 0, cov_ooo = 0;

  task automatic chk(input bit c, input string msg);
    if (c) n_pass++;
    else begin n_fail++; $display("[%0t] FAIL rrob(D=%0d,P=%0d) %s", $time, ROB_DEPTH, MAX_PKTS, msg); end
  endtask

  task model_reset();
    m_alloc = 0; m_ret = 0; m_occ = 0;
    for (int i = 0; i < ROB_DEPTH; i++) begin m_live[i] = 0; m_infl[i] = 0; m_cnt[i] = 0; m_n[i] = 0; end
  endtask

  function automatic bit exp_rv();
    return (m_occ != 0) && m_infl[m_ret] && (m_cnt[m_ret] == m_n[m_ret]);
  endfunction

  // One clock cycle: inputs were set by the caller. Check combinational outputs against the
  // model, advance the model with this cycle's inputs, then clock the DUT.
  task tick();
    bit exp_rdy, adm, rv, ret;
    #1;
    cyc++;
    exp_rdy = (m_occ != ROB_DEPTH);
    rv      = exp_rv();
    adm     = A_ID_VALID && exp_rdy;
    ret     = rv && RESP_READY;
    chk(A_ID_READY === exp_rdy, $sformatf("c%0d A_ID_READY %b exp %b (occ %0d)", cyc, A_ID_READY, exp_rdy, m_occ));
    chk(ROB_IDX === ROB_IDX_W'(m_alloc), $sformatf("c%0d ROB_IDX %0d exp %0d", cyc, ROB_IDX, m_alloc));
    chk(RESP_VALID === rv, $sformatf("c%0d RESP_VALID %b exp %b", cyc, RESP_VALID, rv));
    if (rv) begin
      chk(RESP_ID === AXI_IDW'(m_id[m_ret]), $sformatf("c%0d RESP_ID %0h exp %0h", cyc, RESP_ID, m_id[m_ret]));
      chk(RESP_RESP === ((!m_ok[m_ret] || m_derr[m_ret]) ? 2'b10 : 2'b00), $sformatf("c%0d RESP_RESP %b", cyc, RESP_RESP));
      chk(RESP_HEAD_PTR === SLOT_PTR_W'(m_hd[m_ret]) && RESP_OFFSET === PKT_OFF_W'(m_off[m_ret]) &&
          RESP_LEN === 8'(m_len[m_ret]) && RESP_SIZE === 3'(m_sz[m_ret]), $sformatf("c%0d resp meta", cyc));
    end
    // observation taps (cross-check only)
    chk(d_occ === OCC_W'(m_occ) && d_alloc === PTR_W'(m_alloc) && d_retire === PTR_W'(m_ret),
        $sformatf("c%0d dft occ/ptrs %0d/%0d/%0d exp %0d/%0d/%0d", cyc, d_occ, d_alloc, d_retire, m_occ, m_alloc, m_ret));
    chk(d_admit === adm && d_retfire === ret, $sformatf("c%0d dft admit/retire fire", cyc));
    for (int i = 0; i < ROB_DEPTH; i++) if (m_live[i]) begin
      chk(d_infl[i] === m_infl[i] && d_np[i*CNT_W +: CNT_W] === CNT_W'(m_n[i]) &&
          d_out[i*CNT_W +: CNT_W] === CNT_W'(m_cnt[i]) && d_err[i] === !m_ok[i] && d_derr[i] === m_derr[i],
          $sformatf("c%0d dft slot %0d state", cyc, i));
      if (m_infl[i]) chk(d_head[i*SLOT_PTR_W +: SLOT_PTR_W] === SLOT_PTR_W'(m_hd[i]), $sformatf("c%0d dft slot_head %0d", cyc, i));
    end
    // model update, same order as the RTL (admit, SEG_LAST, completion, retire)
    if (!aresetn) model_reset();
    else begin
      if (adm) begin
        m_live[m_alloc] = 1; m_ok[m_alloc] = AW_REQUEST[0]; m_id[m_alloc] = int'(AW_REQUEST[AXI_IDW:1]);
        m_n[m_alloc] = int'(ADMIT_NPKTS); m_cnt[m_alloc] = 0; m_infl[m_alloc] = 0; m_derr[m_alloc] = 0;
        if (m_alloc == ROB_DEPTH-1) cov_wrap = 1;
        m_alloc = (m_alloc + 1) % ROB_DEPTH; n_adm++;
      end
      if (SEG_LAST) begin
        m_infl[SEG_ROB_IDX] = 1; m_hd[SEG_ROB_IDX] = int'(SEG_HEAD_PTR); m_off[SEG_ROB_IDX] = int'(SEG_OFFSET);
        m_len[SEG_ROB_IDX] = int'(SEG_LEN); m_sz[SEG_ROB_IDX] = int'(SEG_SIZE);
      end
      if (cmpl_valid) begin
        if (!m_infl[cmpl_rob_index]) cov_early = 1;
        m_cnt[cmpl_rob_index]++;
        if (cmpl_err) m_derr[cmpl_rob_index] = 1;
      end
      if (ret) begin
        if (!m_ok[m_ret] || m_derr[m_ret]) cov_err = 1;
        m_live[m_ret] = 0; m_ret = (m_ret + 1) % ROB_DEPTH; n_ret++;
      end
      if (rv && !RESP_READY) cov_stall = 1;
      if (adm && ret) cov_simul = 1;
      m_occ = m_occ + (adm ? 1 : 0) - (ret ? 1 : 0);
      if (m_occ > max_occ) max_occ = m_occ;
      if (m_occ == ROB_DEPTH) cov_full = 1;
    end
    @(posedge aclk); #1;
    A_ID_VALID = 0; SEG_LAST = 0; cmpl_valid = 0; cmpl_err = 0;
  endtask

  // ---- input helpers (set; the caller then calls tick) ----
  task set_admit(input int id, input bit ok, input int n);
    A_ID_VALID = 1; AW_REQUEST = {AXI_IDW'(id), ok}; ADMIT_NPKTS = CNT_W'(n);
  endtask
  task set_seal(input int idx, input int hd, input int off, input int len, input int sz);
    SEG_LAST = 1; SEG_ROB_IDX = ROB_IDX_W'(idx); SEG_HEAD_PTR = SLOT_PTR_W'(hd);
    SEG_OFFSET = PKT_OFF_W'(off); SEG_LEN = 8'(len); SEG_SIZE = 3'(sz);
  endtask
  task set_cmpl(input int idx, input bit err);
    cmpl_valid = 1; cmpl_rob_index = ROB_IDX_W'(idx); cmpl_err = err;
  endtask
  task idle(input int n);
    repeat (n) tick();
  endtask
  task do_reset();
    aresetn = 1; #1; aresetn = 0; RESP_READY = 0; tick(); tick(); aresetn = 1; tick();
  endtask

  // drain: complete/seal everything outstanding and retire (with a timeout)
  task drain();
    int guard;
    RESP_READY = 1;
    for (int i = 0; i < ROB_DEPTH; i++) if (m_live[i] && !m_infl[i]) begin set_seal(i, i, i, i, i % 8); tick(); end
    for (int i = 0; i < ROB_DEPTH; i++) if (m_live[i])
      while (m_cnt[i] < m_n[i]) begin set_cmpl(i, 0); tick(); end
    guard = 0;
    while (m_occ != 0 && guard < 4*ROB_DEPTH + 8) begin tick(); guard++; end
    chk(m_occ == 0, "drain timeout (deadlock)");
    RESP_READY = 0;
  endtask

  // ---------------- tests ----------------
  initial begin : main
    int idx, n, guard, id, seed_v;
    n_pass = 0; n_fail = 0; n_cov = 0; done = 0;
    model_reset();
    seed_v = SEED;
    void'($urandom(seed_v));
    do_reset();

    // --- T1 reset state
    chk(A_ID_READY === 1'b1 && RESP_VALID === 1'b0 && ROB_IDX === '0 && d_occ === '0, "T1 reset state");

    // --- T2 one request, full lifecycle, RESP_READY stall
    idx = m_alloc;
    set_admit(8'hA1, 1, 3); tick();
    chk(m_occ == 1, "T2 occupancy after admit");
    idle(2);                                           // no response before SEG_LAST / completions
    set_cmpl(idx, 0); tick(); set_cmpl(idx, 0); tick();
    set_seal(idx, 5 % N_SLOTS, 13, 7, 3); tick();       // sealed but 2 of 3 completions
    chk(RESP_VALID === 1'b0, "T2 not valid with 2/3 completions");
    set_cmpl(idx, 0); tick();                          // 3/3: RESP_VALID (checked in tick via model)
    chk(RESP_VALID === 1'b1, "T2 valid at 3/3");
    RESP_READY = 0; idle(3);                           // stall: must hold, no retire
    chk(RESP_VALID === 1'b1 && m_occ == 1, "T2 response held under RESP_READY low");
    RESP_READY = 1; tick();
    chk(m_occ == 0 && RESP_VALID === 1'b0, "T2 retired, empty");
    RESP_READY = 0;

    // --- T3 completions before SEG_LAST (early) must still count
    idx = m_alloc;
    set_admit(8'hB2, 1, 2); tick();
    set_cmpl(idx, 0); tick(); set_cmpl(idx, 0); tick();
    chk(RESP_VALID === 1'b0, "T3 count reached but not sealed: no response");
    RESP_READY = 1; set_seal(idx, 1 % N_SLOTS, 0, 1, 0); tick();
    chk(RESP_VALID === 1'b1, "T3 valid once sealed");
    tick(); RESP_READY = 0;
    chk(m_occ == 0, "T3 retired");

    // --- T4 error responses: failed validation and downstream error
    idx = m_alloc; set_admit(8'hC3, 0, 1); tick();     // validated = 0
    set_seal(idx, 0, 0, 0, 0); tick(); set_cmpl(idx, 0); RESP_READY = 1; tick();
    chk(RESP_RESP === 2'b10 || m_occ == 0, "T4 validation error response");
    tick();
    idx = m_alloc; set_admit(8'hD4, 1, 2); tick();
    set_seal(idx, 0, 0, 0, 0); tick(); set_cmpl(idx, 0); tick(); set_cmpl(idx, 1); tick();   // cmpl_err on 2nd
    tick(); RESP_READY = 0;
    chk(m_occ == 0, "T4 downstream-error entry retired after all completions");

    // --- T5 fill to capacity, backpressure, out-of-order seal/complete, in-order retire
    drain();
    for (int i = 0; i < ROB_DEPTH; i++) begin
      set_admit(i + 1, 1, 1 + (i % MAX_PKTS)); tick();
    end
    chk(m_occ == ROB_DEPTH && A_ID_READY === 1'b0, "T5 full: ready low");
    set_admit(8'hEE, 1, 1); tick();                    // refused (model keeps state)
    chk(m_occ == ROB_DEPTH, "T5 refused admit leaves occupancy");
    for (int i = ROB_DEPTH-1; i >= 0; i--) begin set_seal((m_ret + i) % ROB_DEPTH, i % N_SLOTS, i % 64, i, i % 8); tick(); end   // reverse order
    RESP_READY = 1;
    for (int i = ROB_DEPTH-1; i >= 0; i--) begin
      idx = (m_ret + i) % ROB_DEPTH;
      for (int k = 0; k < m_n[idx]; k++) begin set_cmpl(idx, 0); tick(); end
      if (i > 0) chk(RESP_VALID === 1'b0, $sformatf("T5 slot %0d done but head not: no retire", i));
    end
    cov_ooo = 1;
    guard = 0; while (m_occ != 0 && guard < 2*ROB_DEPTH + 8) begin tick(); guard++; end
    chk(m_occ == 0, "T5 drained in order");
    RESP_READY = 0;

    // --- T6 admit and retire in the same cycle, one below full and wrapping the ring
    drain();
    for (int r = 0; r < 2*ROB_DEPTH + 3; r++) begin
      idx = m_alloc;
      if (m_occ == 0) begin set_admit(r, 1, 1); tick(); end
      set_seal(m_ret, 0, 0, 0, 0); tick();
      set_cmpl(m_ret, 0); tick();                      // head now complete
      RESP_READY = 1;
      set_admit(r + 100, 1, 1); tick();                // retire + admit together
      RESP_READY = 0;
      drain();
    end

    // --- T7 constrained-random: random admit/seal/complete/ready/err, model compared per cycle
    for (int c = 0; c < NRAND; c++) begin
      int cand, sel;
      // seal a random live unsealed entry (not the slot being admitted this cycle)
      if (($urandom % 3) == 0) begin
        cand = -1; sel = $urandom % ROB_DEPTH;
        for (int j = 0; j < ROB_DEPTH; j++) begin
          int s2; s2 = (sel + j) % ROB_DEPTH;
          if (cand < 0 && m_live[s2] && !m_infl[s2]) cand = s2;
        end
        if (cand >= 0) set_seal(cand, $urandom % N_SLOTS, $urandom % 64, $urandom % 256, $urandom % 8);
      end
      // complete a random live entry that still owes packets (also before it is sealed)
      if (($urandom % 2) == 0) begin
        cand = -1; sel = $urandom % ROB_DEPTH;
        for (int j = 0; j < ROB_DEPTH; j++) begin
          int s2; s2 = (sel + j) % ROB_DEPTH;
          if (cand < 0 && m_live[s2] && m_cnt[s2] < m_n[s2]) cand = s2;
        end
        if (cand >= 0) set_cmpl(cand, ($urandom % 8) == 0);
      end
      if (($urandom % 2) == 0) set_admit($urandom, ($urandom % 6) != 0, 1 + ($urandom % MAX_PKTS));
      RESP_READY = (($urandom % 4) != 0);
      // legal protocol: never complete/seal the slot that is being admitted this cycle
      if (A_ID_VALID && m_occ != ROB_DEPTH) begin
        if (SEG_LAST && int'(SEG_ROB_IDX) == m_alloc) SEG_LAST = 0;
        if (cmpl_valid && int'(cmpl_rob_index) == m_alloc) cmpl_valid = 0;
      end
      tick();
    end
    drain();

    // --- T8 asynchronous reset in the middle of activity
    set_admit(1, 1, 2); tick(); set_admit(2, 1, 2); tick();
    #2 aresetn = 0; #1;
    chk(d_occ === '0 && d_alloc === '0 && d_retire === '0 && A_ID_READY === 1'b1 && RESP_VALID === 1'b0,
        "T8 async reset clears pointers/occupancy immediately");
    model_reset();
    tick(); aresetn = 1; tick();
    chk(m_occ == 0 && ROB_IDX === '0, "T8 usable after reset");
    set_admit(9, 1, 1); tick();
    set_seal(0, 0, 0, 0, 0); tick(); set_cmpl(0, 0); RESP_READY = 1; tick(); tick();
    chk(m_occ == 0, "T8 request completes after reset");
    RESP_READY = 0; idle(2);

    // coverage / vacuity guards for the random phase
    chk(n_ret > 3*ROB_DEPTH && n_adm > 3*ROB_DEPTH, $sformatf("stress retired %0d / admitted %0d", n_ret, n_adm));
    if (ROB_DEPTH == 1) cov_simul = 1;   // admit needs a free slot, so it cannot coincide with a retire
    chk(cov_full && cov_wrap && cov_early && cov_err && cov_stall && cov_simul && cov_ooo,
        $sformatf("coverage full=%b wrap=%b early=%b err=%b stall=%b simul=%b ooo=%b",
                  cov_full, cov_wrap, cov_early, cov_err, cov_stall, cov_simul, cov_ooo));
    n_cov = cov_full + cov_wrap + cov_early + cov_err + cov_stall + cov_simul + cov_ooo;
    done = 1;
  end

  // deadlock guard
  initial begin : wd
    #(10 * (NRAND * 8 + 200000));
    if (!done) begin n_fail++; $display("[%0t] FAIL rrob(D=%0d) watchdog: test did not finish", $time, ROB_DEPTH); done = 1; end
  end
endmodule

module tb_rmc_cif_rrob;
  int p0, f0, c0, p1, f1, c1, p2, f2, c2, p3, f3, c3;
  bit d0, d1, d2, d3;
  rrob_chk #(32, 16, 8, 32, 1, 4000) u0 (p0, f0, c0, d0);   // defaults, power of two
  rrob_chk #( 4,  8, 4, 16, 2, 3000) u1 (p1, f1, c1, d1);   // small ring, wraps quickly
  rrob_chk #( 5,  7, 3,  8, 3, 3000) u2 (p2, f2, c2, d2);   // non-power-of-two depth
  rrob_chk #( 1,  2, 2,  2, 4, 1000) u3 (p3, f3, c3, d3);   // single-entry corner
  initial begin
    wait (d0 && d1 && d2 && d3);
    #20;
    $display("rrob configs: D=32 %0d pass/%0d fail  D=4 %0d/%0d  D=5 %0d/%0d  D=1 %0d/%0d",
             p0, f0, p1, f1, p2, f2, p3, f3);
    $display("coverage markers (of 7): %0d %0d %0d %0d", c0, c1, c2, c3);
    if (f0 + f1 + f2 + f3 == 0) $display("TB RESULT: PASS");
    else begin $display("TB RESULT: FAIL"); $fatal(1, "rrob unit test failures"); end
    $finish;
  end
endmodule
