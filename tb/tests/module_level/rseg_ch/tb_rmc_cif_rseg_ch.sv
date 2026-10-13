`timescale 1ns/1ps
// Unit testbench for rmc_cif_r_burst_spltr_ch (one per-channel ring walker). The TB plays the
// wrapper: it owns the queue entries and per-channel done bits and presents the entry at
// the engine's rd_ptr. Expected packets are derived from the contract in the RTL header:
// the engine issues exactly the packets k in [0, n_pkts) whose line (base + k) maps to its
// channel (low CH_W bits of the line number), in ascending order, with
// daddr = map(line << 6) and dbuf slot = dbuf_base + k.

module rseg_ch_chk #(
  parameter int CH_ID       = 0,
  parameter int QUEUE_DEPTH = 4,     // power of two
  parameter int ROB_DEPTH   = 4,
  parameter int MAX_PKTS    = 16,
  parameter int N_SLOTS     = 32,
  parameter int SEED        = 1,
  parameter int NRAND       = 3000
)(
  output int n_pass,
  output int n_fail,
  output int n_pkts_seen,
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

  logic aclk = 1'b0, aresetn = 1'b1;
  always #5 aclk = ~aclk;

  logic                 entry_valid;
  logic [LINE_W-1:0]    e_aligned_base;
  logic [CNT_W-1:0]     e_n_pkts;
  logic [SLOT_W-1:0]    e_dbuf_base;
  logic [ROB_IDX_W-1:0] e_rob_idx;
  logic                 e_ch_done;
  logic                 pkt_ready = 0, dbuf_rdy = 0;
  logic [QPTR_W-1:0]    rd_ptr;
  logic                 set_done, pkt_valid;
  logic [ROB_IDX_W-1:0] pkt_rob_idx;
  logic [DADDR_W-1:0]   pkt_daddr;
  logic [SLOT_W-1:0]    pkt_dbuf_adr;
  logic [CNT_W-1:0]     dft_m;
  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map;
  assign addr_map = default_addr_map();

  rmc_cif_r_burst_spltr_ch #(.ROB_DEPTH(ROB_DEPTH), .MAX_PKTS(MAX_PKTS), .AXI_AW(AXI_AW),
                    .N_SLOTS(N_SLOTS), .QUEUE_DEPTH(QUEUE_DEPTH)) dut (
    .aclk, .aresetn, .ch_id(CH_W'(CH_ID)), .addr_map,
    .rd_ptr, .entry_valid, .e_aligned_base, .e_n_pkts, .e_dbuf_base, .e_rob_idx, .e_ch_done,
    .set_done, .pkt_valid, .pkt_ready, .pkt_dbuf_adr_ready(dbuf_rdy),
    .pkt_rob_idx, .pkt_daddr, .pkt_dbuf_adr, .dft_m);

  // ---- TB wrapper: queue storage + done bits, presented at the engine's rd_ptr ----
  logic [LINE_W-1:0]    q_base [QUEUE_DEPTH];
  int                   q_n    [QUEUE_DEPTH];
  int                   q_db   [QUEUE_DEPTH];
  int                   q_rob  [QUEUE_DEPTH];
  bit                   q_done [QUEUE_DEPTH];
  int                   alloc_cnt = 0, exp_cnt = 0, exp_m = 0;    // unwrapped counters
  int                   cyc = 0;
  logic [QPTR_W-1:0]    alloc_q;
  assign alloc_q = QPTR_W'(alloc_cnt % (2*QUEUE_DEPTH));
  assign entry_valid = (rd_ptr !== alloc_q);

  wire [QIDX_W-1:0] rdi = rd_ptr[QIDX_W-1:0];
  always_comb begin
    e_aligned_base = q_base[rdi];
    e_n_pkts       = CNT_W'(q_n[rdi]);
    e_dbuf_base    = SLOT_W'(q_db[rdi]);
    e_rob_idx      = ROB_IDX_W'(q_rob[rdi]);
    e_ch_done      = q_done[rdi];
  end

  // package-position reference (independent of rmc_cif_addrmap / default_addr_map)
  function automatic logic [DADDR_W-1:0] ref_daddr(input logic [47:0] a);
    return {a[RANK_POS +: RANK_W], a[BGHI_POS +: BG_HI_W], a[BGLO_POS +: BG_LO_W],
            a[BANK_POS +: BANK_PER_BG_W], a[ROW_POS +: ROW_W], a[COLHI_POS +: COL_HI_W],
            a[COLLO_POS +: COL_LO_W]};
  endfunction

  int  tot_pkts = 0;
  task automatic chk(input bit c, input string msg);
    if (c) n_pass++;
    else begin n_fail++; $display("[%0t] FAIL rseg_ch(ch=%0d,Q=%0d) %s", $time, CH_ID, QUEUE_DEPTH, msg); end
  endtask

  // k of the m-th packet this channel owns in an entry, or -1 if none left (model only)
  function automatic int kth(input int base_lo, input int n, input int m);
    int cnt;
    cnt = 0;
    for (int k = 0; k < n; k++)
      if (((base_lo + k) % N_CH) == CH_ID) begin
        if (cnt == m) return k;
        cnt++;
      end
    return -1;
  endfunction

  task do_reset();
    aresetn = 1; #1; aresetn = 0; #7; aresetn = 1; #3;
    alloc_cnt = 0; exp_cnt = 0; exp_m = 0;
    for (int i = 0; i < QUEUE_DEPTH; i++) q_done[i] = 0;
  endtask

  // one cycle: compare DUT with the expected behaviour for the presented entry, then advance
  task tick();
    int  idx, k, bl;
    bit  have, skip, ex_valid, ex_sd, adv;
    logic [47:0] la;
    #1;
    cyc++;
    idx  = exp_cnt % QUEUE_DEPTH;
    have = (exp_cnt != alloc_cnt);
    ex_valid = 0; ex_sd = 0; skip = 0; adv = 0;
    chk(rd_ptr === QPTR_W'(exp_cnt % (2*QUEUE_DEPTH)), $sformatf("c%0d rd_ptr %0d exp %0d", cyc, rd_ptr, exp_cnt % (2*QUEUE_DEPTH)));
    chk(dft_m === CNT_W'(exp_m), $sformatf("c%0d dft_m %0d exp %0d", cyc, dft_m, exp_m));
    if (have) begin
      bl = int'(q_base[idx] % N_CH);
      k  = kth(bl, q_n[idx], exp_m);
      if (q_done[idx]) skip = 1;                          // already done for this channel: pass over
      else if (k >= 0) ex_valid = dbuf_rdy;               // packet owed, needs a dbuf slot
      else ex_sd = 1;                                     // share fully issued (or none): mark done
      if (skip || ex_sd) adv = 1;
    end
    chk(pkt_valid === ex_valid, $sformatf("c%0d pkt_valid %b exp %b (m=%0d)", cyc, pkt_valid, ex_valid, exp_m));
    chk(set_done === ex_sd, $sformatf("c%0d set_done %b exp %b", cyc, set_done, ex_sd));
    if (ex_valid) begin
      la = 48'((q_base[idx] + LINE_W'(k)) << PKT_OFF_W);
      chk(pkt_rob_idx === ROB_IDX_W'(q_rob[idx]), $sformatf("c%0d pkt_rob_idx %0d exp %0d", cyc, pkt_rob_idx, q_rob[idx]));
      chk(pkt_daddr === ref_daddr(la), $sformatf("c%0d pkt_daddr %h exp %h (line k=%0d)", cyc, pkt_daddr, ref_daddr(la), k));
      chk(pkt_dbuf_adr === SLOT_W'(q_db[idx] + k), $sformatf("c%0d pkt_dbuf_adr %0d exp %0d", cyc, pkt_dbuf_adr, (q_db[idx]+k) % N_SLOTS));
    end
    // advance model at the edge
    @(posedge aclk);
    if (adv) begin
      if (ex_sd) q_done[idx] = 1;        // wrapper sets the done bit; engine already moved on
      exp_cnt++; exp_m = 0;
    end else if (ex_valid && pkt_ready) begin
      exp_m++; tot_pkts++;
    end
    #1;
  endtask

  task enq(input int base_lo_bits, input int n, input int dbase, input int rob);
    int idx;
    idx = alloc_cnt % QUEUE_DEPTH;
    q_base[idx] = LINE_W'({$urandom, $urandom}) & ~LINE_W'(N_CH-1) | LINE_W'(base_lo_bits % N_CH);
    q_n[idx] = n; q_db[idx] = dbase; q_rob[idx] = rob; q_done[idx] = 0;
    alloc_cnt++;
  endtask

  function automatic int room();
    return QUEUE_DEPTH - (alloc_cnt - exp_cnt);
  endfunction

  // run until the engine has consumed everything queued; ready/dbuf per mode
  task run_until_empty(input int rdy_mode, input int dbuf_mode, input int cap);
    int g;
    g = 0;
    while (exp_cnt != alloc_cnt && g < cap) begin
      case (rdy_mode)  0: pkt_ready = 1;  1: pkt_ready = (g % 3) == 0;  default: pkt_ready = ($urandom % 2);  endcase
      case (dbuf_mode) 0: dbuf_rdy  = 1;  1: dbuf_rdy  = (g % 4) != 3;  default: dbuf_rdy  = ($urandom % 2);  endcase
      tick(); g++;
    end
    chk(exp_cnt == alloc_cnt, $sformatf("run_until_empty timeout (rdy_mode %0d dbuf_mode %0d)", rdy_mode, dbuf_mode));
  endtask

  initial begin : main
    int seed_v, nn, idx;
    n_pass = 0; n_fail = 0; n_pkts_seen = 0; done = 0;
    seed_v = SEED; void'($urandom(seed_v));
    do_reset();

    // D1 idle engine: no entry -> no packets, no done pulse, rd_ptr holds
    pkt_ready = 1; dbuf_rdy = 1;
    repeat (4) tick();

    // D2 single entry, every base alignment, short and long, all ready
    for (int a = 0; a < 2*N_CH; a++)
      for (int n = 0; n <= MAX_PKTS; n += (n < 3 ? 1 : 5)) begin
        enq(a, n, a*3 % N_SLOTS, a % ROB_DEPTH);
        run_until_empty(0, 0, 4*MAX_PKTS + 20);
      end

    // D2b literal sequence: base line even, n=5, dbuf base 10. Channel c owns k = c, c+2, c+4...
    //   ch0: k = 0,2,4  -> dbuf 10,12,14      ch1: k = 1,3 -> dbuf 11,13   (compared mod N_SLOTS)
    if (N_CH == 2) begin
      int  exp_k [3];
      int  got   [3];
      int  ng;
      ng = 0;
      if (CH_ID == 0) begin exp_k[0] = 0; exp_k[1] = 2; exp_k[2] = 4; end
      else            begin exp_k[0] = 1; exp_k[1] = 3; exp_k[2] = -1; end
      enq(0, 5, 10, 0);                              // line number even -> line 0 lands on ch0
      pkt_ready = 1; dbuf_rdy = 1;
      for (int g = 0; g < 12 && exp_cnt != alloc_cnt; g++) begin
        #1;   // let the combinational outputs settle after enq
        if (pkt_valid === 1'b1 && ng < 3) begin got[ng] = (int'(pkt_dbuf_adr) - (10 % N_SLOTS) + N_SLOTS) % N_SLOTS; ng++; end
        tick();
      end
      for (int j = 0; j < 3; j++)
        chk(exp_k[j] < 0 ? (ng <= j) : (ng > j && got[j] == exp_k[j] % N_SLOTS),
            $sformatf("D2b literal packet %0d: got k=%0d exp k=%0d", j, (ng > j) ? got[j] : -99, exp_k[j]));
    end

    // D3 n_pkts == 0 and == 1 explicitly (done pulse without packets / single packet)
    enq(0, 0, 0, 0); run_until_empty(0, 0, 20);
    enq(CH_ID, 1, 1, 1 % ROB_DEPTH); run_until_empty(0, 0, 20);
    enq(CH_ID + 1, 1, 2, 2 % ROB_DEPTH); run_until_empty(0, 0, 20);   // packet owned by the other channel

    // D4 stalls: pkt_ready low and dbuf not ready, then released; outputs must hold
    enq(1, MAX_PKTS, 7, 1 % ROB_DEPTH);
    pkt_ready = 0; dbuf_rdy = 0; repeat (5) tick();        // dbuf-stalled: no pkt_valid
    dbuf_rdy = 1; repeat (5) tick();                         // valid but ready low: held, m unchanged
    run_until_empty(1, 1, 400);

    // D5 several queued entries (to depth), ring wrap of rd_ptr over >2 laps
    for (int lap = 0; lap < 4; lap++) begin
      while (room() > 0) enq($urandom, 1 + ($urandom % MAX_PKTS), $urandom % N_SLOTS, $urandom % ROB_DEPTH);
      run_until_empty(2, 2, 40000);
    end

    // D6 defensive skip path: done bit already set for this channel when the engine arrives
    enq(0, 4, 0, 0); q_done[(alloc_cnt-1) % QUEUE_DEPTH] = 1;
    enq(CH_ID, 3, 5, 1 % ROB_DEPTH);
    run_until_empty(0, 0, 100);
    // note: the first entry's packets must NOT have been issued (checked by the model: skip)

    // D7 constrained random: entries appended whenever there is room, random ready/dbuf
    for (int c = 0; c < NRAND; c++) begin
      if (room() > 0 && ($urandom % 3) == 0)
        enq($urandom, $urandom % (MAX_PKTS + 1), $urandom % N_SLOTS, $urandom % ROB_DEPTH);
      pkt_ready = ($urandom % 3) != 0;
      dbuf_rdy  = ($urandom % 4) != 0;
      tick();
    end
    pkt_ready = 1; dbuf_rdy = 1;
    run_until_empty(0, 0, 40000);

    // D8 asynchronous reset mid-entry: pointer and m clear, engine usable afterwards
    enq(0, MAX_PKTS, 3, 0); pkt_ready = 1; dbuf_rdy = 1; repeat (3) tick();
    #2 aresetn = 0; #1;
    chk(rd_ptr === '0 && dft_m === '0, "D8 reset clears rd_ptr and m");
    alloc_cnt = 0; exp_cnt = 0; exp_m = 0;      // model cleared while the engine is held in reset
    for (int i = 0; i < QUEUE_DEPTH; i++) q_done[i] = 0;
    #4 aresetn = 1; #3;
    chk(entry_valid === 1'b0 && pkt_valid === 1'b0 && set_done === 1'b0, "D8 idle after reset");
    enq(1, 5, 9, 1 % ROB_DEPTH); run_until_empty(0, 0, 60);

    chk(tot_pkts > 20, $sformatf("stimulus vacuity guard: only %0d packets issued", tot_pkts));
    n_pkts_seen = tot_pkts;
    done = 1;
  end

  initial begin : wd
    #(10 * (NRAND * 6 + 100000));
    if (!done) begin n_fail++; $display("[%0t] FAIL rseg_ch(ch=%0d) watchdog", $time, CH_ID); done = 1; end
  end
endmodule

module tb_rmc_cif_r_burst_spltr_ch;
  int p0, f0, s0, p1, f1, s1, p2, f2, s2, p3, f3, s3;
  bit d0, d1, d2, d3;
  rseg_ch_chk #(0, 4,  4, 16, 32, 1, 3000) u0 (p0, f0, s0, d0);
  rseg_ch_chk #(1, 4,  4, 16, 32, 2, 3000) u1 (p1, f1, s1, d1);
  rseg_ch_chk #(1, 32, 32, 64, 64, 3, 3000) u2 (p2, f2, s2, d2);   // deep queue, 64-packet counts
  rseg_ch_chk #(0, 2,  2,  4,  4, 4, 2000) u3 (p3, f3, s3, d3);   // tiny queue, small slot space (wraps)
  initial begin
    wait (d0 && d1 && d2 && d3);
    #20;
    $display("rseg_ch configs: %0d/%0d  %0d/%0d  %0d/%0d  %0d/%0d (pass/fail), packets issued %0d %0d %0d %0d",
             p0, f0, p1, f1, p2, f2, p3, f3, s0, s1, s2, s3);
    if (f0 + f1 + f2 + f3 == 0) $display("TB RESULT: PASS");
    else begin $display("TB RESULT: FAIL"); $fatal(1, "r_burst_spltr_ch unit test failures"); end
    $finish;
  end
endmodule
