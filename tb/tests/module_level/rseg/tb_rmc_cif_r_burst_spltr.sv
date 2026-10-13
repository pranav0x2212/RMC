`timescale 1ns/1ps
// Unit testbench for rmc_cif_r_burst_spltr (pre-ROB queue + N_CH per-channel walkers).
// Everything is checked against a TB scoreboard built from the interface contract:
//   * requests enqueue in order; ready == (live entries != QUEUE_DEPTH)
//   * packet k of a request is line (addr>>6)+k, owned by channel line%N_CH (package CH_POS)
//   * daddr = map(line<<6) (package bit positions), dbuf slot = dbuf_base + k, rob tag = req_rob_idx
//   * each channel issues its packets in order, oldest request first; channels run independently
//   * seg_last pops the head only once every channel has issued its share, in program order,
//     carrying {rob_idx, n_pkts, offset, len, size, head slot} of that request
// No hierarchical force; all stimulus is through the module ports.

module rbs_chk #(
  parameter int ROB_DEPTH   = 8,
  parameter int QUEUE_DEPTH = 8,     // power of two, <= ROB_DEPTH so rob tags stay unique
  parameter int MAX_PKTS    = 16,
  parameter int N_SLOTS     = 32,
  parameter int SEED        = 1,
  parameter int NRAND       = 4000
)(
  output int  n_pass,
  output int  n_fail,
  output int  n_pkts_seen,
  output int  n_cov,
  output bit  done
);
  import rmc_cfg_pkg::*;
  localparam int AXI_AW     = SYS_ADDR_W;
  localparam int ROB_IDX_W  = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1;
  localparam int CNT_W      = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1;
  localparam int SLOT_W     = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1;
  localparam int QIDX_W     = (QUEUE_DEPTH > 1) ? $clog2(QUEUE_DEPTH) : 1;
  localparam int QPTR_W     = QIDX_W + 1;
  localparam int DATA_BYTES = AXI_DW / 8;
  localparam int SIZE_MAX   = (DATA_BYTES > 1) ? $clog2(DATA_BYTES) : 0;
  localparam int MAX_BEATS  = MAX_REQ_BYTES / DATA_BYTES;
  localparam int LEN_W      = (MAX_BEATS > 1) ? $clog2(MAX_BEATS) : 1;
  localparam int SIZE_W     = (SIZE_MAX > 0) ? $clog2(SIZE_MAX+1) : 1;

  logic aclk = 1'b0, aresetn = 1'b1;
  always #5 aclk = ~aclk;

  logic                 start_req = 0;
  logic [AXI_AW-1:0]    req_addr = '0;
  logic [LEN_W-1:0]     req_len = '0;
  logic [SIZE_W-1:0]    req_size = '0;
  logic [SLOT_W-1:0]    req_dbuf_base = '0;
  logic [ROB_IDX_W-1:0] req_rob_idx = '0;
  logic                 ready;
  logic [N_CH-1:0]      pkt_valid, pkt_ready = '0, dbuf_rdy = '0;
  logic [N_CH-1:0][ROB_IDX_W-1:0] pkt_rob_idx;
  logic [N_CH-1:0][DADDR_W-1:0]   pkt_daddr;
  logic [N_CH-1:0][SLOT_W-1:0]    pkt_dbuf_adr;
  logic                 seg_last;
  logic [ROB_IDX_W-1:0] seg_rob_idx;
  logic [CNT_W-1:0]     seg_npkts;
  logic [PKT_OFF_W-1:0] seg_offset;
  logic [LEN_W-1:0]     seg_len;
  logic [SIZE_W-1:0]    seg_size;
  logic [SLOT_W-1:0]    seg_head_ptr;
  logic [QPTR_W-1:0]    d_alloc, d_pop;
  logic [QUEUE_DEPTH*N_CH-1:0] d_chdone;
  logic [N_CH*QPTR_W-1:0] d_erp;
  logic [N_CH*CNT_W-1:0]  d_em;
  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map;
  assign addr_map = default_addr_map();

  rmc_cif_r_burst_spltr #(.ROB_DEPTH(ROB_DEPTH), .MAX_PKTS(MAX_PKTS), .AXI_AW(AXI_AW),
                          .N_SLOTS(N_SLOTS), .QUEUE_DEPTH(QUEUE_DEPTH)) dut (
    .aclk, .aresetn, .addr_map, .start_req, .req_addr, .req_len, .req_size, .req_dbuf_base,
    .req_rob_idx, .ready, .pkt_valid, .pkt_ready, .pkt_dbuf_adr_ready(dbuf_rdy),
    .pkt_rob_idx, .pkt_daddr, .pkt_dbuf_adr, .seg_last, .seg_rob_idx, .seg_npkts, .seg_offset,
    .seg_len, .seg_size, .seg_head_ptr,
    .dft_alloc_ptr(d_alloc), .dft_pop_ptr(d_pop), .dft_ch_done(d_chdone),
    .dft_eng_rd_ptr(d_erp), .dft_eng_m(d_em));

  // ---------------- scoreboard (indexed by ring slot = count % QUEUE_DEPTH) ----------------
  logic [AXI_AW-1:0] r_addr [QUEUE_DEPTH];
  int  r_len [QUEUE_DEPTH];
  int  r_size[QUEUE_DEPTH];
  int  r_db  [QUEUE_DEPTH];
  int  r_rob [QUEUE_DEPTH];
  int  r_n   [QUEUE_DEPTH];
  int  r_nxt [QUEUE_DEPTH][N_CH];     // next packet index owed on that channel (-1 = share done)
  int  enq_cnt = 0, pop_cnt = 0, cyc = 0, rob_ctr = 0;
  int  bubble [N_CH];
  int  idle_head = 0;
  int  tot_pkts = 0, tot_pops = 0, ahead_cnt = 0;
  bit  cov_full = 0, cov_ahead = 0, cov_dstall = 0, cov_wrap = 0, cov_refused = 0;

  task automatic chk(input bit c, input string msg);
    if (c) n_pass++;
    else begin n_fail++; $display("[%0t] FAIL r_burst_spltr(R=%0d,Q=%0d,P=%0d) %s", $time, ROB_DEPTH, QUEUE_DEPTH, MAX_PKTS, msg); end
  endtask

  // ---- independent reference functions ----
  function automatic logic [DADDR_W-1:0] ref_daddr(input logic [47:0] a);
    return {a[RANK_POS +: RANK_W], a[BGHI_POS +: BG_HI_W], a[BGLO_POS +: BG_LO_W],
            a[BANK_POS +: BANK_PER_BG_W], a[ROW_POS +: ROW_W], a[COLHI_POS +: COL_HI_W],
            a[COLLO_POS +: COL_LO_W]};
  endfunction
  // packets of an INCR burst: first beat at addr, later beats size-aligned; one per 64B line touched
  function automatic int npk_of(input logic [AXI_AW-1:0] a, input int len, input int sz);
    logic [AXI_AW-1:0] last;
    last = ((a >> sz) << sz) + AXI_AW'((len + 1) << sz) - 1;
    return int'((last >> PKT_OFF_W) - (a >> PKT_OFF_W)) + 1;
  endfunction
  function automatic int chan_of(input logic [AXI_AW-1:0] a, input int k);   // channel of packet k
    logic [AXI_AW-1:0] line_addr;
    line_addr = ((a >> PKT_OFF_W) + AXI_AW'(k)) << PKT_OFF_W;
    return int'((line_addr >> CH_POS) & AXI_AW'(N_CH - 1));
  endfunction
  function automatic int next_k(input logic [AXI_AW-1:0] a, input int n, input int c, input int from);
    for (int k = from; k < n; k++) if (chan_of(a, k) == c) return k;
    return -1;
  endfunction

  function automatic int head_for_chan(input int c);    // oldest live request this channel still owes
    for (int i = pop_cnt; i < enq_cnt; i++) if (r_nxt[i % QUEUE_DEPTH][c] >= 0) return i;
    return -1;
  endfunction
  function automatic bit head_issued();
    if (enq_cnt == pop_cnt) return 0;
    for (int c = 0; c < N_CH; c++) if (r_nxt[pop_cnt % QUEUE_DEPTH][c] >= 0) return 0;
    return 1;
  endfunction

  // ---------------- one clock cycle ----------------
  task tick();
    bit exp_rdy, enq_ok;
    bit cons [N_CH];
    int h, s, k, c, popped_slot;
    bit pop_now;
    #1;
    cyc++;
    exp_rdy = ((enq_cnt - pop_cnt) != QUEUE_DEPTH);
    chk(ready === exp_rdy, $sformatf("c%0d ready %b exp %b (live %0d)", cyc, ready, exp_rdy, enq_cnt - pop_cnt));
    chk(d_alloc === QPTR_W'(enq_cnt % (2*QUEUE_DEPTH)) && d_pop === QPTR_W'(pop_cnt % (2*QUEUE_DEPTH)),
        $sformatf("c%0d dft ptrs alloc %0d pop %0d exp %0d/%0d", cyc, d_alloc, d_pop, enq_cnt % (2*QUEUE_DEPTH), pop_cnt % (2*QUEUE_DEPTH)));
    for (c = 0; c < N_CH; c++) cons[c] = 0;
    // ---- per-channel packet checks ----
    for (c = 0; c < N_CH; c++) begin
      h = head_for_chan(c);
      if (pkt_valid[c] === 1'b1) begin
        bubble[c] = 0;
        chk(dbuf_rdy[c], $sformatf("c%0d ch%0d pkt_valid while dbuf not ready", cyc, c));
        chk(h >= 0, $sformatf("c%0d ch%0d pkt_valid with no packet owed", cyc, c));
        if (h >= 0) begin
          s = h % QUEUE_DEPTH; k = r_nxt[s][c];
          chk(pkt_rob_idx[c] === ROB_IDX_W'(r_rob[s]), $sformatf("c%0d ch%0d rob_idx %0d exp %0d", cyc, c, pkt_rob_idx[c], r_rob[s]));
          chk(pkt_daddr[c] === ref_daddr(((r_addr[s] >> PKT_OFF_W) + AXI_AW'(k)) << PKT_OFF_W),
              $sformatf("c%0d ch%0d daddr %h (req line k=%0d)", cyc, c, pkt_daddr[c], k));
          chk(pkt_dbuf_adr[c] === SLOT_W'(r_db[s] + k), $sformatf("c%0d ch%0d dbuf %0d exp %0d", cyc, c, pkt_dbuf_adr[c], (r_db[s]+k) % N_SLOTS));
          if (pkt_ready[c]) begin
            cons[c] = 1;
            if (h != pop_cnt) begin ahead_cnt++; cov_ahead = 1; end    // ran ahead of the head request
          end
        end
      end else begin
        if (h >= 0 && dbuf_rdy[c]) begin
          bubble[c]++;
          chk(bubble[c] <= QUEUE_DEPTH + 3, $sformatf("c%0d ch%0d stuck: work owed, dbuf ready, no pkt_valid for %0d cycles", cyc, c, bubble[c]));
        end else bubble[c] = 0;
        if (h >= 0 && !dbuf_rdy[c]) cov_dstall = 1;
      end
    end
    // ---- seg_last / in-order pop ----
    pop_now = 0;
    if (seg_last === 1'b1) begin
      chk(enq_cnt != pop_cnt && head_issued(), $sformatf("c%0d seg_last before the head request was fully issued", cyc));
      if (enq_cnt != pop_cnt) begin
        s = pop_cnt % QUEUE_DEPTH;
        chk(seg_rob_idx === ROB_IDX_W'(r_rob[s]), $sformatf("c%0d seg_rob_idx %0d exp %0d", cyc, seg_rob_idx, r_rob[s]));
        chk(seg_npkts === CNT_W'(r_n[s]), $sformatf("c%0d seg_npkts %0d exp %0d", cyc, seg_npkts, r_n[s]));
        chk(seg_offset === PKT_OFF_W'(r_addr[s][PKT_OFF_W-1:0]), $sformatf("c%0d seg_offset %0d", cyc, seg_offset));
        chk(seg_len === LEN_W'(r_len[s]) && seg_size === SIZE_W'(r_size[s]) && seg_head_ptr === SLOT_W'(r_db[s]),
            $sformatf("c%0d seg_len/size/head", cyc));
        pop_now = 1;
      end
      idle_head = 0;
    end else if (head_issued()) begin
      idle_head++;
      chk(idle_head <= 6, $sformatf("c%0d head fully issued but no seg_last for %0d cycles", cyc, idle_head));
    end else idle_head = 0;
    // ---- enqueue ----
    enq_ok = start_req && exp_rdy;
    if (start_req && !exp_rdy) cov_refused = 1;
    // ---- edge ----
    @(posedge aclk);
    for (c = 0; c < N_CH; c++) if (cons[c]) begin
      h = head_for_chan(c); s = h % QUEUE_DEPTH;
      r_nxt[s][c] = next_k(r_addr[s], r_n[s], c, r_nxt[s][c] + 1);
      tot_pkts++;
    end
    if (pop_now) begin pop_cnt++; tot_pops++; if (pop_cnt % QUEUE_DEPTH == 0) cov_wrap = 1; end
    if (enq_ok) begin
      s = enq_cnt % QUEUE_DEPTH;
      r_addr[s] = req_addr; r_len[s] = int'(req_len); r_size[s] = int'(req_size);
      r_db[s] = int'(req_dbuf_base); r_rob[s] = int'(req_rob_idx);
      r_n[s] = npk_of(req_addr, int'(req_len), int'(req_size));
      for (c = 0; c < N_CH; c++) r_nxt[s][c] = next_k(req_addr, r_n[s], c, 0);
      enq_cnt++;
      if (enq_cnt - pop_cnt == QUEUE_DEPTH) cov_full = 1;
    end
    #1;
    start_req = 0;
  endtask

  // ---------------- stimulus helpers ----------------
  task set_req(input logic [AXI_AW-1:0] a, input int len, input int sz, input int db);
    start_req = 1; req_addr = a; req_len = LEN_W'(len); req_size = SIZE_W'(sz);
    req_dbuf_base = SLOT_W'(db); req_rob_idx = ROB_IDX_W'(rob_ctr % ROB_DEPTH);
  endtask
  // offer a request; the scoreboard accepts it only if the queue has room, rob tag advances then
  task send(input logic [AXI_AW-1:0] a, input int len, input int sz, input int db);
    int n0;
    n0 = enq_cnt;
    set_req(a, len, sz, db);
    tick();
    if (enq_cnt != n0) rob_ctr++;
  endtask
  task set_modes(input logic [N_CH-1:0] pr, input logic [N_CH-1:0] dr);
    pkt_ready = pr; dbuf_rdy = dr;
  endtask
  task do_reset();
    aresetn = 1; #1; aresetn = 0; #7; aresetn = 1; #3;
    enq_cnt = 0; pop_cnt = 0; rob_ctr = 0; idle_head = 0;
    for (int c = 0; c < N_CH; c++) bubble[c] = 0;
  endtask
  task drain(input int cap);
    int g;
    g = 0;
    while (enq_cnt != pop_cnt && g < cap) begin tick(); g++; end
    chk(enq_cnt == pop_cnt, "drain timeout: requests still live (deadlock)");
  endtask
  // random request within the per-config packet cap; flavour picks alignment / size classes
  task rand_req(output logic [AXI_AW-1:0] a, output int len, output int sz);
    int tries;
    tries = 0;
    do begin
      a   = AXI_AW'({$urandom, $urandom}) & ((AXI_AW'(1) << MAP_ADDR_W) - 1);
      sz  = $urandom % (SIZE_MAX + 1);
      len = $urandom % MAX_BEATS;
      if (($urandom % 3) == 0) len = $urandom % 8;
      if (($urandom % 4) == 0) a[PKT_OFF_W-1:0] = '0;
      tries++;
    end while (npk_of(a, len, sz) > MAX_PKTS && tries < 100);
    if (npk_of(a, len, sz) > MAX_PKTS) begin len = 0; sz = 0; end
  endtask

  initial begin : main
    logic [AXI_AW-1:0] a;
    int len, sz, seed_v, n, pops0;
    n_pass = 0; n_fail = 0; n_pkts_seen = 0; n_cov = 0; done = 0;
    seed_v = SEED; void'($urandom(seed_v));
    for (int c = 0; c < N_CH; c++) bubble[c] = 0;
    do_reset();

    // S1 reset state
    set_modes('1, '1);
    tick();
    chk(ready === 1'b1 && seg_last === 1'b0 && pkt_valid === '0 && d_alloc === '0 && d_pop === '0, "S1 reset state");

    // S2 directed single requests: packet count, owning channel(s), unaligned / crossing / byte
    //    (addr, len, size) -> packets: 0x00,3,4 =1 (line0)  0x40,3,4 =1 (line1)  0x30,3,4 =2
    //    0x3F,0,0 =1  0x3F,1,0 =2  0x7F,1,0 =2  0x1000,63,4 = 16 lines
    send(48'h0000_0000_0000, 3, 4, 0);     drain(100);
    send(48'h0000_0000_0040, 3, 4, 5);     drain(100);
    send(48'h0000_0000_0030, 3, 4, 7);     drain(100);
    send(48'h0000_0000_003F, 0, 0, 9);     drain(100);
    send(48'h0000_0000_003F, 1, 0, 11);    drain(100);
    send(48'h0000_0000_007F, 1, 0, 13);    drain(100);
    if (MAX_PKTS >= 16) begin send(48'h0000_0000_1000, 63, 4, 17); drain(400); end
    // maximum packet count for this config, at an odd line (starts on channel 1)
    send(AXI_AW'(64) , MAX_PKTS*4 - 1, 4, 3);  drain(40*MAX_PKTS);
    // high address bits set (map ignores bits above the mapped range; tag/ptr unaffected)
    send(48'hFFFF_FFFF_FFC0 & ((48'd1 << MAP_ADDR_W) - 1), 7, 4, 1); drain(200);
    chk(tot_pkts >= 8, "S2 vacuity guard: packets issued");

    // S3 queue full: nothing consumed, over-offer; extra requests must be refused
    set_modes('0, '1);
    for (int i = 0; i < QUEUE_DEPTH + 3; i++) send(AXI_AW'(i * 64 * 3), 3 + (i % 4), 4, i % N_SLOTS);
    chk(ready === 1'b0 && (enq_cnt - pop_cnt) == QUEUE_DEPTH, "S3 queue full: ready low, exactly QUEUE_DEPTH live");
    repeat (6) tick();
    chk(seg_last === 1'b0 && cov_refused, "S3 no pop without packets issued; surplus requests were refused");
    set_modes('1, '1);
    drain(60 * QUEUE_DEPTH);                          // ready returns as the head pops (checked every cycle)
    chk(ready === 1'b1, "S3 ready high again after drain");

    // S4 channel independence: channel 1 stalled on the request FIFO, channel 0 free-runs onto later requests
    if (N_CH >= 2) begin
      pops0 = tot_pops;
      set_modes(N_CH'(1), '1);                                          // only ch0 ready
      for (int i = 0; i < QUEUE_DEPTH; i++) send(AXI_AW'(i * 4096), 15, 4, (i * 4) % N_SLOTS);   // 4 packets each
      repeat (4 * QUEUE_DEPTH + 10) tick();
      chk(cov_ahead, "S4 channel 0 ran ahead of the head request while channel 1 was stalled");
      chk(seg_last === 1'b0 && tot_pops == pops0, "S4 no pop while channel 1 owes its share");
      set_modes('1, '1);
      drain(60 * QUEUE_DEPTH);
    end

    // S5 per-channel data-buffer stall: stalled channel offers nothing, the other keeps flowing
    set_modes('1, N_CH'(1) << (N_CH-1));              // dbuf ready only on the last channel
    for (int i = 0; i < QUEUE_DEPTH; i++) send(AXI_AW'(i * 640 + 64), 7, 4, (i * 3) % N_SLOTS);
    repeat (40) tick();
    chk(pkt_valid[0] === 1'b0, "S5 dbuf-stalled channel offers no packet");
    set_modes('1, '1);
    drain(60 * QUEUE_DEPTH);

    // S6 constrained random: random requests, random per-channel ready/dbuf, many ring wraps
    for (int c = 0; c < NRAND; c++) begin
      for (int j = 0; j < N_CH; j++) begin pkt_ready[j] = (($urandom % 3) != 0); dbuf_rdy[j] = (($urandom % 4) != 0); end
      if (($urandom % 3) == 0) begin
        rand_req(a, len, sz);
        send(a, len, sz, $urandom % N_SLOTS);
      end else tick();
    end
    set_modes('1, '1);
    drain(200 * QUEUE_DEPTH);

    // S7 asynchronous reset with requests in flight
    set_modes('0, '1);
    send(48'h40, 15, 4, 2); send(48'h80, 15, 4, 4);
    repeat (3) tick();
    #2 aresetn = 0; #1;
    chk(d_alloc === '0 && d_pop === '0 && ready === 1'b1 && pkt_valid === '0 && seg_last === 1'b0,
        "S7 async reset clears queue pointers and outputs");
    enq_cnt = 0; pop_cnt = 0; rob_ctr = 0; idle_head = 0;
    #4 aresetn = 1; #3;
    set_modes('1, '1);
    send(48'h40, 3, 4, 6); drain(60);
    chk(tot_pops > 0, "S7 request flows after reset");

    chk(tot_pkts > 10*QUEUE_DEPTH && tot_pops > 4*QUEUE_DEPTH,
        $sformatf("stress vacuity guard: %0d packets, %0d pops", tot_pkts, tot_pops));
    chk(cov_full && cov_ahead && cov_dstall && cov_wrap && cov_refused,
        $sformatf("coverage full=%b ahead=%b dstall=%b wrap=%b refused=%b", cov_full, cov_ahead, cov_dstall, cov_wrap, cov_refused));
    n_cov = cov_full + cov_ahead + cov_dstall + cov_wrap + cov_refused;
    n_pkts_seen = tot_pkts;
    done = 1;
  end

  initial begin : wd
    #(10 * (NRAND * 40 + 400000));
    if (!done) begin n_fail++; $display("[%0t] FAIL r_burst_spltr watchdog", $time); done = 1; end
  end
endmodule

// -------------------------------------------------------------------------------------------
// Read-path flow composite: rmc_cif_rrob + rmc_cif_r_burst_spltr wired exactly as rmc_cif.sv
// does it (admit gating, ROB_IDX -> req_rob_idx, seg_* -> SEG_*). The TB plays the downstream:
// a request sink (pkt_ready) and an MC that completes each consumed packet by the rob tag the
// splitter put on it. Checks the index/address/buffer mappings and in-order response end to end.
// -------------------------------------------------------------------------------------------
module flow_chk #(
  parameter int ROB_DEPTH = 8,
  parameter int MAX_PKTS  = 16,
  parameter int AXI_IDW   = 8,
  parameter int N_SLOTS   = 32,
  parameter int NREQ      = 600
)(
  output int  n_pass,
  output int  n_fail,
  output int  n_ret,
  output bit  done
);
  import rmc_cfg_pkg::*;
  localparam int AXI_AW     = SYS_ADDR_W;
  localparam int ROB_IDX_W  = (ROB_DEPTH > 1) ? $clog2(ROB_DEPTH) : 1;
  localparam int CNT_W      = (MAX_PKTS  > 1) ? $clog2(MAX_PKTS+1) : 1;
  localparam int SLOT_W     = (N_SLOTS   > 1) ? $clog2(N_SLOTS) : 1;
  localparam int DATA_BYTES = AXI_DW / 8;
  localparam int SIZE_MAX   = (DATA_BYTES > 1) ? $clog2(DATA_BYTES) : 0;
  localparam int MAX_BEATS  = MAX_REQ_BYTES / DATA_BYTES;
  localparam int LEN_W      = (MAX_BEATS > 1) ? $clog2(MAX_BEATS) : 1;
  localparam int SIZE_W     = (SIZE_MAX > 0) ? $clog2(SIZE_MAX+1) : 1;

  logic aclk = 0, aresetn = 1;
  always #5 aclk = ~aclk;

  // TB-driven AR channel + downstream stubs
  logic                 arvalid = 0;
  logic [AXI_IDW-1:0]   arid = '0;
  logic [AXI_AW-1:0]    araddr = '0;
  logic [LEN_W-1:0]     arlen = '0;
  logic [SIZE_W-1:0]    arsize = '0;
  logic                 ar_ok = 1;
  logic [SLOT_W-1:0]    dbase = '0;
  logic [CNT_W-1:0]     admit_npkts = '0;
  logic [N_CH-1:0]      pkt_ready = '0, dbuf_rdy = '0;
  logic                 resp_ready = 0;
  logic                 cmpl_valid = 0, cmpl_err = 0;
  logic [ROB_IDX_W-1:0] cmpl_idx = '0;

  // wiring identical to rmc_cif.sv
  logic spl_ready, rrob_rdy, ar_admit;
  logic [ROB_IDX_W-1:0] rob_idx;
  logic seg_last; logic [ROB_IDX_W-1:0] seg_rob_idx; logic [CNT_W-1:0] seg_npkts;
  logic [PKT_OFF_W-1:0] seg_offset; logic [LEN_W-1:0] seg_len; logic [SIZE_W-1:0] seg_size;
  logic [SLOT_W-1:0] seg_head;
  logic [N_CH-1:0] pkt_valid;
  logic [N_CH-1:0][ROB_IDX_W-1:0] pkt_rob_idx;
  logic [N_CH-1:0][DADDR_W-1:0]   pkt_daddr;
  logic [N_CH-1:0][SLOT_W-1:0]    pkt_dbuf_adr;
  logic resp_valid; logic [AXI_IDW-1:0] resp_id; logic [1:0] resp_resp;
  logic [SLOT_W-1:0] resp_head; logic [PKT_OFF_W-1:0] resp_off; logic [7:0] resp_len; logic [2:0] resp_size;
  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map;
  assign addr_map = default_addr_map();
  assign ar_admit = arvalid & spl_ready & rrob_rdy;

  rmc_cif_r_burst_spltr #(.ROB_DEPTH(ROB_DEPTH), .MAX_PKTS(MAX_PKTS), .AXI_AW(AXI_AW), .N_SLOTS(N_SLOTS)) u_spl (
    .aclk, .aresetn, .addr_map, .start_req(ar_admit), .req_addr(araddr), .req_len(arlen),
    .req_size(arsize), .req_dbuf_base(dbase), .req_rob_idx(rob_idx), .ready(spl_ready),
    .pkt_valid, .pkt_ready, .pkt_dbuf_adr_ready(dbuf_rdy), .pkt_rob_idx, .pkt_daddr, .pkt_dbuf_adr,
    .seg_last, .seg_rob_idx, .seg_npkts, .seg_offset, .seg_len, .seg_size, .seg_head_ptr(seg_head),
    .dft_alloc_ptr(), .dft_pop_ptr(), .dft_ch_done(), .dft_eng_rd_ptr(), .dft_eng_m());
  rmc_cif_rrob #(.ROB_DEPTH(ROB_DEPTH), .MAX_PKTS(MAX_PKTS), .AXI_IDW(AXI_IDW), .N_SLOTS(N_SLOTS)) u_rrob (
    .aclk, .aresetn, .cmpl_valid, .cmpl_rob_index(cmpl_idx), .cmpl_err,
    .AW_REQUEST({arid, ar_ok}), .ADMIT_NPKTS(admit_npkts), .A_ID_VALID(arvalid & spl_ready),
    .A_ID_READY(rrob_rdy), .ROB_IDX(rob_idx),
    .SEG_LAST(seg_last), .SEG_ROB_IDX(seg_rob_idx), .SEG_OFFSET(seg_offset),
    .SEG_LEN(8'(seg_len)), .SEG_SIZE(3'(seg_size)), .SEG_HEAD_PTR(seg_head),
    .RESP_VALID(resp_valid), .RESP_READY(resp_ready), .RESP_ID(resp_id), .RESP_RESP(resp_resp),
    .RESP_HEAD_PTR(resp_head), .RESP_OFFSET(resp_off), .RESP_LEN(resp_len), .RESP_SIZE(resp_size),
    .dft_alloc_ptr(), .dft_retire_ptr(), .dft_occ(), .dft_admit_fire(), .dft_retire_fire(),
    .dft_inflow_complt(), .dft_ent_err(), .dft_ent_derr(), .dft_outflow_cnt(), .dft_n_pkts(), .dft_slot_head());

  // ---- request book (program order) ----
  localparam int NB = 1024;
  logic [AXI_AW-1:0] b_addr [NB];
  int  b_len [NB], b_sz [NB], b_db [NB], b_np [NB], b_id [NB], b_rob [NB];
  bit  b_ok  [NB];
  logic [63:0] b_seen [NB];
  int  n_adm = 0, n_seg = 0, n_resp = 0, cyc = 0, pkts_total = 0, pkts_cmpl = 0, in_flight = 0;
  // completion latency queue
  int  cq_idx [256], cq_due [256];
  int  cq_h = 0, cq_t = 0;

  task automatic chk(input bit c, input string msg);
    if (c) n_pass++;
    else begin n_fail++; $display("[%0t] FAIL read_flow(R=%0d,P=%0d) %s", $time, ROB_DEPTH, MAX_PKTS, msg); end
  endtask
  function automatic logic [DADDR_W-1:0] ref_daddr(input logic [47:0] a);
    return {a[RANK_POS +: RANK_W], a[BGHI_POS +: BG_HI_W], a[BGLO_POS +: BG_LO_W],
            a[BANK_POS +: BANK_PER_BG_W], a[ROW_POS +: ROW_W], a[COLHI_POS +: COL_HI_W],
            a[COLLO_POS +: COL_LO_W]};
  endfunction
  function automatic int npk_of(input logic [AXI_AW-1:0] a, input int len, input int sz);
    logic [AXI_AW-1:0] last;
    last = ((a >> sz) << sz) + AXI_AW'((len + 1) << sz) - 1;
    return int'((last >> PKT_OFF_W) - (a >> PKT_OFF_W)) + 1;
  endfunction
  function automatic int chan_of(input logic [AXI_AW-1:0] a, input int k);
    logic [AXI_AW-1:0] line_addr;
    line_addr = ((a >> PKT_OFF_W) + AXI_AW'(k)) << PKT_OFF_W;
    return int'((line_addr >> CH_POS) & AXI_AW'(N_CH - 1));
  endfunction

  int  cur_req;        // request being offered on AR (not yet admitted)
  task new_req();
    int tries, len, sz;
    logic [AXI_AW-1:0] a;
    tries = 0;
    do begin
      a   = AXI_AW'({$urandom, $urandom}) & ((AXI_AW'(1) << MAP_ADDR_W) - 1);
      sz  = $urandom % (SIZE_MAX + 1);
      len = (($urandom % 2) == 0) ? ($urandom % 8) : ($urandom % MAX_BEATS);
      tries++;
    end while (npk_of(a, len, sz) > MAX_PKTS && tries < 100);
    if (npk_of(a, len, sz) > MAX_PKTS) begin len = 0; sz = 0; end
    arvalid = 1; araddr = a; arlen = LEN_W'(len); arsize = SIZE_W'(sz);
    arid = AXI_IDW'($urandom); ar_ok = (($urandom % 8) != 0);
    dbase = SLOT_W'($urandom % N_SLOTS);
    admit_npkts = CNT_W'(npk_of(a, len, sz));
  endtask

  initial begin : main
    int idle_guard, k, s, ri, c, pn;
    bit want_new, adm_now;
    n_pass = 0; n_fail = 0; n_ret = 0; done = 0;
    for (int i = 0; i < NB; i++) b_seen[i] = '0;
    #1 aresetn = 0; #12 aresetn = 1; #3;
    want_new = 1; idle_guard = 0;
    while (n_resp < NREQ && idle_guard < 400) begin
      // downstream behaviour this cycle
      for (int j = 0; j < N_CH; j++) begin pkt_ready[j] = (($urandom % 3) != 0); dbuf_rdy[j] = (($urandom % 5) != 0); end
      resp_ready = (($urandom % 3) != 0);
      if (!arvalid && n_adm < NREQ && ($urandom % 3) != 0) new_req();
      // MC: complete the oldest due packet (by the tag the splitter issued)
      cmpl_valid = 0; cmpl_err = 0;
      if (cq_h != cq_t && cq_due[cq_h % 256] <= cyc) begin
        cmpl_valid = 1; cmpl_idx = ROB_IDX_W'(cq_idx[cq_h % 256]); cq_h++; pkts_cmpl++;
      end
      #1;
      cyc++;
      // ---- admit ----
      chk(rob_idx === ROB_IDX_W'(n_adm % ROB_DEPTH) || ROB_DEPTH == 1, $sformatf("c%0d ROB_IDX %0d exp %0d (admit #%0d)", cyc, rob_idx, n_adm % ROB_DEPTH, n_adm));
      if (ar_admit) begin
        s = n_adm % NB;
        b_addr[s] = araddr; b_len[s] = int'(arlen); b_sz[s] = int'(arsize); b_db[s] = int'(dbase);
        b_np[s] = npk_of(araddr, int'(arlen), int'(arsize)); b_id[s] = int'(arid); b_ok[s] = ar_ok;
        b_rob[s] = int'(rob_idx); b_seen[s] = '0;
        chk(b_rob[s] == n_adm % ROB_DEPTH, $sformatf("c%0d admitted request %0d got rob idx %0d", cyc, n_adm, b_rob[s]));
        n_adm++; in_flight++; idle_guard = 0;
      end
      // ---- packets leaving the splitter: tag -> request, address and buffer mappings ----
      for (c = 0; c < N_CH; c++) if (pkt_valid[c] === 1'b1 && pkt_ready[c]) begin
        ri = -1;
        for (int q = n_resp; q < n_adm; q++)           // live requests, oldest first
          if (b_rob[q % NB] == int'(pkt_rob_idx[c])) begin ri = q; break; end
        chk(ri >= 0, $sformatf("c%0d ch%0d packet tagged rob %0d matches no live request", cyc, c, pkt_rob_idx[c]));
        if (ri >= 0) begin
          s = ri % NB;
          k = (int'(pkt_dbuf_adr[c]) - b_db[s] + N_SLOTS) % N_SLOTS;
          chk(k < b_np[s], $sformatf("c%0d req%0d packet index %0d >= n_pkts %0d", cyc, ri, k, b_np[s]));
          chk(c == chan_of(b_addr[s], k), $sformatf("c%0d req%0d pkt%0d on ch%0d, owner ch%0d", cyc, ri, k, c, chan_of(b_addr[s], k)));
          chk(pkt_daddr[c] === ref_daddr(((b_addr[s] >> PKT_OFF_W) + AXI_AW'(k)) << PKT_OFF_W),
              $sformatf("c%0d req%0d pkt%0d daddr", cyc, ri, k));
          chk(k < 64 && !b_seen[s][k], $sformatf("c%0d req%0d pkt%0d issued twice", cyc, ri, k));
          if (k < 64) b_seen[s][k] = 1'b1;
          cq_idx[cq_t % 256] = int'(pkt_rob_idx[c]);
          cq_due[cq_t % 256] = cyc + 1 + ($urandom % 6);
          cq_t++; pkts_total++;
        end
      end
      // ---- seg_last: program order, bundle, all packets seen ----
      if (seg_last === 1'b1) begin
        s = n_seg % NB;
        chk(n_seg < n_adm, $sformatf("c%0d seg_last with no admitted request", cyc));
        chk(int'(seg_rob_idx) == b_rob[s], $sformatf("c%0d seg_rob_idx %0d exp %0d", cyc, seg_rob_idx, b_rob[s]));
        chk(int'(seg_npkts) == b_np[s] && b_np[s] == npk_of(b_addr[s], b_len[s], b_sz[s]),
            $sformatf("c%0d seg_npkts %0d exp %0d", cyc, seg_npkts, b_np[s]));
        chk(b_seen[s] === ((64'd1 << b_np[s]) - 1), $sformatf("c%0d req%0d sealed before all packets issued", cyc, n_seg));
        n_seg++;
      end
      // ---- responses: program order, id, resp, meta ----
      if (resp_valid === 1'b1 && resp_ready) begin
        s = n_resp % NB;
        chk(n_resp < n_adm, "resp with no request");
        chk(resp_id === AXI_IDW'(b_id[s]), $sformatf("c%0d resp #%0d id %0h exp %0h", cyc, n_resp, resp_id, b_id[s]));
        chk(resp_resp === (b_ok[s] ? 2'b00 : 2'b10), $sformatf("c%0d resp #%0d resp %b", cyc, n_resp, resp_resp));
        chk(resp_head === SLOT_W'(b_db[s]) && resp_off === PKT_OFF_W'(b_addr[s][PKT_OFF_W-1:0]) &&
            resp_len === 8'(b_len[s]) && resp_size === 3'(b_sz[s]), $sformatf("c%0d resp #%0d meta", cyc, n_resp));
        chk(b_seen[s] === ((64'd1 << b_np[s]) - 1), $sformatf("c%0d resp #%0d before all packets issued", cyc, n_resp));
        n_resp++; in_flight--; idle_guard = 0;
      end
      idle_guard++;
      adm_now = ar_admit;           // sampled before the edge; the request leaves AR after it
      @(posedge aclk); #1;
      if (adm_now) arvalid = 0;
    end
    chk(n_resp == NREQ, $sformatf("flow timeout: %0d of %0d responses (deadlock?)", n_resp, NREQ));
    chk(pkts_total == pkts_cmpl || cq_h != cq_t, "packet/completion accounting");
    n_ret = n_resp;
    done = 1;
  end

  initial begin : wd
    #(10 * (NREQ * 400 + 100000));
    if (!done) begin n_fail++; $display("[%0t] FAIL read_flow watchdog", $time); done = 1; end
  end
endmodule

module tb_rmc_cif_r_burst_spltr;
  int p0, f0, k0, c0, p1, f1, k1, c1, p2, f2, k2, c2, p3, f3, k3, c3;
  int fp0, ff0, fr0, fp1, ff1, fr1;
  bit d0, d1, d2, d3, e0, e1;
  rbs_chk #( 4,  4, 16, 32, 1, 4000) u0 (p0, f0, k0, c0, d0);
  rbs_chk #( 8,  8, 16, 16, 2, 4000) u1 (p1, f1, k1, c1, d1);   // small slot space: dbuf wraps
  rbs_chk #(32, 32, 64, 64, 3, 4000) u2 (p2, f2, k2, c2, d2);   // deep queue, 64-packet requests
  rbs_chk #( 8,  2,  8,  8, 4, 3000) u3 (p3, f3, k3, c3, d3);   // QUEUE_DEPTH < ROB_DEPTH
  flow_chk #( 8, 16, 8, 32, 600) fl0 (fp0, ff0, fr0, e0);
  flow_chk #(32, 16, 8, 32, 600) fl1 (fp1, ff1, fr1, e1);
  initial begin
    wait (d0 && d1 && d2 && d3 && e0 && e1);
    #20;
    $display("r_burst_spltr unit: %0d/%0d  %0d/%0d  %0d/%0d  %0d/%0d (pass/fail); packets %0d %0d %0d %0d; coverage markers %0d %0d %0d %0d of 5",
             p0, f0, p1, f1, p2, f2, p3, f3, k0, k1, k2, k3, c0, c1, c2, c3);
    $display("read flow (rrob + splitter): %0d/%0d (R=8, %0d resp)  %0d/%0d (R=32, %0d resp)", fp0, ff0, fr0, fp1, ff1, fr1);
    if (f0 + f1 + f2 + f3 + ff0 + ff1 == 0) $display("TB RESULT: PASS");
    else begin $display("TB RESULT: FAIL"); $fatal(1, "r_burst_spltr unit test failures"); end
    $finish;
  end
endmodule
