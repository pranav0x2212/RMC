`timescale 1ns/1ps
// Testbench for rmc_cif_r_burst_spltr (top-level read burst splitter), two checkers:
//
//  rbs_chk   unit test of the splitter alone. The TB plays the AXI AR master, the ROB's
//            admit side (rob_free / rob_idx) and the per-channel command FIFO / RD-SRAM
//            handshakes. A transaction-level scoreboard built from the request, plus exact
//            checks for the things the contract pins to a cycle, is compared with the ports.
//  flow_chk  integration composite: the real splitter wired to the real rmc_cif_rrob exactly
//            as rmc_cif.sv does it (splitter owns arready; admit -> rrob admit; seg_* ->
//            SEG_*), with a modelled memory controller completing each packet by its rob tag.
//            Checks the in-order R-response descriptor end to end.
//
// Contract used (sources: rmc_cif_r_burst_spltr.sv header / precompute / theory block,
// rmc_cif_r_burst_spltr_ch.sv, rmc_cfg_pkg::line_channel, rmc_cif_addrmap.sv, rmc_cif.sv):
//   * Accept: ar_valid & ar_ready, ar_ready = S1 empty or draining (admit) - one request is held
//     in the S1 register; a new AR is accepted every cycle while S1 drains.
//   * Admit (cycle after accept at the earliest) = S1 occupied & rob_free & queue not full.
//     admit_* describe the S1 request: id, validated, npkts (0 if rejected), addr_lo = addr[11:0],
//     len, size, base_ch = channel of its first line, ch_mask = channels with >= 1 packet (0 if
//     rejected). The request is enqueued at that edge; rob_idx is sampled in that cycle.
//   * A request is a run of 64 B lines L0..L_last inside ONE 4 KB page (a crossing is illegal
//     and $errors in the RTL). n_pkts = L_last - L0 + 1. Burst end follows AXI INCR: the first
//     beat is at addr, later beats are size-aligned.
//   * Channel of a line = CH source bits of addr_map (dec bus layout offset | ch | daddr, select
//     for ch bit k at addr_map[PKT_OFF_W + k]), channel bits >= cfg_n_ch_log2 forced 0. The
//     reference below is written from that layout and the TB's own record of the address bit it
//     put in each CH select; it never calls rmc_cfg_pkg::line_channel.
//   * Each channel issues its lines of every request in ascending line order, oldest request
//     first, one packet per fire (pkt_valid & pkt_ready; pkt_valid = owes & dbuf ready, never
//     waiting for pkt_ready). pkt_dbuf_adr is that channel's own RD-SRAM ring pointer: it
//     advances by one (mod N_SLOTS) per fire across requests and restarts at 0 after reset.
//   * seg_last pulses once per request, in program order, in the cycle after the last packet of
//     the oldest request has fired on every channel (immediately after enqueue for a request
//     with no packets). It carries that request's rob_idx and, for each channel that owns packets
//     of it, the RD-SRAM slot of the first one (seg_head_ptr for other channels is not defined).
//   * idle = nothing in S1 and the queue empty.
// Stimulus constraints (illegal otherwise): requests stay in one page, <= MAX_PKTS lines, size <=
// log2(AXI_DW/8); N_SLOTS >= MAX_PKTS, MAX_PKTS / N_SLOTS / QUEUE_DEPTH powers of two,
// QUEUE_DEPTH <= ROB_DEPTH; addr_map / cfg_n_ch_log2 are static after boot (changed only with a
// reset); once ar_valid is raised it stays up with a stable payload until accepted (AXI).
// Not driven: rejected requests that would cross a page, overrun of the RD-SRAM ring (the splitter
// does not track occupancy; pkt_dbuf_adr_ready is the only throttle), size > log2(AXI_DW/8).
// Deliberately varied: AR payload carries garbage while ar_valid is low; rob_idx carries garbage
// while rob_free is low (it is only meaningful when the ROB offers a slot).

module rbs_chk #(
  parameter int ROB_DEPTH   = 8,
  parameter int QUEUE_DEPTH = 8,     // power of two, <= ROB_DEPTH
  parameter int MAX_PKTS    = 16,    // power of two, <= 64
  parameter int N_SLOTS     = 32,    // power of two, >= MAX_PKTS
  parameter int AXI_IDW     = 8,
  parameter int SEED        = 1,
  parameter int NRAND       = 3500
)(
  output int  n_pass,
  output int  n_fail,
  output int  n_pkts_seen,
  output int  n_cov,                // coverage markers reached (of 18)
  output bit  done
);
  import rmc_cfg_pkg::*;
  localparam int AXI_AW     = SYS_ADDR_W;
  localparam int ROB_IDX_W  = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1;
  localparam int CNT_W      = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1;
  localparam int SLOT_W     = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1;
  localparam int DATA_BYTES = AXI_DW / 8;
  localparam int SIZE_MAX   = (DATA_BYTES > 1) ? $clog2(DATA_BYTES) : 0;
  localparam int MAX_BEATS  = MAX_REQ_BYTES / DATA_BYTES;
  localparam int LEN_W      = (MAX_BEATS  > 1) ? $clog2(MAX_BEATS)  : 1;
  localparam int SIZE_W     = (SIZE_MAX   > 0) ? $clog2(SIZE_MAX+1) : 1;
  localparam int NCH_LOG_W  = $clog2(CH_W + 1);
  localparam int LN_W       = (PG_LINES   > 1) ? $clog2(PG_LINES)   : 1;
  localparam int NFLAV      = 6;                                        // channel-bit placements
  localparam int REC        = QUEUE_DEPTH + 2;                          // scoreboard window
  localparam int NPQ        = (QUEUE_DEPTH + 2) * MAX_PKTS;             // per-channel packet FIFO
  localparam logic [AXI_AW-1:0] ADDR_MASK = AXI_AW'((48'd1 << MAP_ADDR_W) - 48'd1);
  localparam logic [AXI_AW-1:0] BAD_BIT   = AXI_AW'(48'd1) << (AXI_AW - 1);   // out of the mapped range

  logic aclk = 1'b0, aresetn = 1'b1;
  always #5 aclk = ~aclk;

  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map;
  logic [NCH_LOG_W-1:0] cfg_ncl;
  logic                 ar_valid = 0, ar_ready;
  logic [AXI_IDW-1:0]   ar_id = '0;
  logic [AXI_AW-1:0]    ar_addr = '0;
  logic [LEN_W-1:0]     ar_len = '0;
  logic [SIZE_W-1:0]    ar_size = '0;
  logic                 ar_validated = 0;
  logic                 rob_free = 0;
  logic [ROB_IDX_W-1:0] rob_idx = '0;
  logic                 admit_valid;
  logic [AXI_IDW-1:0]   admit_id;
  logic                 admit_validated;
  logic [CNT_W-1:0]     admit_npkts;
  logic [PAGE_LSB-1:0]  admit_addr_lo;
  logic [LEN_W-1:0]     admit_len;
  logic [SIZE_W-1:0]    admit_size;
  logic [CH_W-1:0]      admit_base_ch;
  logic [N_CH-1:0]      admit_ch_mask;
  logic [N_CH-1:0]      pkt_valid, pkt_ready = '0, dbuf_rdy = '0;
  logic [N_CH-1:0][ROB_IDX_W-1:0] pkt_rob_idx;
  logic [N_CH-1:0][DADDR_W-1:0]   pkt_daddr;
  logic [N_CH-1:0][SLOT_W-1:0]    pkt_dbuf_adr;
  logic                 seg_last;
  logic [ROB_IDX_W-1:0] seg_rob_idx;
  logic [N_CH-1:0][SLOT_W-1:0]    seg_head_ptr;
  logic                 idle;

  rmc_cif_r_burst_spltr #(.ROB_DEPTH(ROB_DEPTH), .MAX_PKTS(MAX_PKTS), .AXI_AW(AXI_AW),
                          .AXI_IDW(AXI_IDW), .N_SLOTS(N_SLOTS), .QUEUE_DEPTH(QUEUE_DEPTH)) dut (
    .aclk, .aresetn, .addr_map, .cfg_n_ch_log2(cfg_ncl),
    .ar_valid, .ar_ready, .ar_id, .ar_addr, .ar_len, .ar_size, .ar_validated,
    .rob_free, .rob_idx,
    .admit_valid, .admit_id, .admit_validated, .admit_npkts, .admit_addr_lo, .admit_len,
    .admit_size, .admit_base_ch, .admit_ch_mask,
    .pkt_valid, .pkt_ready, .pkt_dbuf_adr_ready(dbuf_rdy), .pkt_rob_idx, .pkt_daddr, .pkt_dbuf_adr,
    .seg_last, .seg_rob_idx, .seg_head_ptr, .idle);

  // ---------------- address-map configuration (changed only together with a reset) ----------------
  int ch_src [CH_W];            // address bit the TB put into CH select k
  int cur_ncl  = 0;             // active channels = 2^cur_ncl
  int cur_flav = 0;

  function automatic int flav_pos(input int f);
    if (f == 0)      return CH_POS;     // default map: channel = line LSB
    else if (f == 1) return 7;
    else if (f == 2) return 9;
    else if (f == 3) return 11;         // top in-page line bit
    else if (f == 4) return 12;         // above the page: one channel for a whole request
    else             return 15;
  endfunction

  task set_map(input int flavor, input int ncl);
    logic [DEC_W*MAP_SEL_W-1:0] mf;
    mf = default_addr_map();
    for (int k = 0; k < CH_W; k++) begin
      ch_src[k] = flav_pos(flavor) + k;
      mf[(PKT_OFF_W + k)*MAP_SEL_W +: MAP_SEL_W] = MAP_SEL_W'(ch_src[k]);
    end
    addr_map = mf;
    cur_ncl = ncl; cur_flav = flavor; cfg_ncl = NCH_LOG_W'(ncl);
  endtask

  // ---------------- independent references ----------------
  function automatic logic [AXI_AW-1:0] line_addr(input logic [AXI_AW-1:0] a, input int i);
    return {a[AXI_AW-1:PAGE_LSB], LN_W'(i), {PKT_OFF_W{1'b0}}};
  endfunction

  function automatic int ref_ch(input logic [AXI_AW-1:0] a);
    int c;
    c = 0;
    for (int k = 0; k < CH_W; k++)
      if (k < cur_ncl && a[ch_src[k]]) c = c | (1 << k);
    return c;
  endfunction

  function automatic logic [DADDR_W-1:0] ref_daddr(input logic [AXI_AW-1:0] a);
    return {a[RANK_POS +: RANK_W], a[BGHI_POS +: BG_HI_W], a[BGLO_POS +: BG_LO_W],
            a[BANK_POS +: BANK_PER_BG_W], a[ROW_POS +: ROW_W], a[COLHI_POS +: COL_HI_W],
            a[COLLO_POS +: COL_LO_W]};
  endfunction

  // AXI INCR burst: first beat at a, later beats aligned to the beat size
  function automatic logic [AXI_AW-1:0] last_byte(input logic [AXI_AW-1:0] a, input int len, input int sz);
    logic [AXI_AW-1:0] fa;
    fa = (a >> sz) << sz;
    return fa + AXI_AW'((len + 1) << sz) - AXI_AW'(1);
  endfunction

  // legal stimulus: one page, <= MAX_PKTS lines
  function automatic bit legal(input logic [AXI_AW-1:0] a, input int len, input int sz);
    logic [AXI_AW-1:0] lb;
    lb = last_byte(a, len, sz);
    return ((a >> PAGE_LSB) == (lb >> PAGE_LSB)) && (int'((lb >> PKT_OFF_W) - (a >> PKT_OFF_W)) < MAX_PKTS);
  endfunction

  // ---------------- scoreboard state ----------------
  int  s1_occ = 0;                                 // a request is held in S1 (accepted, not admitted)
  logic [AXI_IDW-1:0] s1_id;
  logic [AXI_AW-1:0]  s1_addr;
  int  s1_len, s1_size, s1_l0, s1_ll, s1_nl, s1_bch;
  bit  s1_val;
  int  s1_cnt [N_CH];                              // lines per channel (0 if rejected)

  int  adm_cnt = 0, pop_cnt = 0, rob_seq = 0;      // admitted / popped / ROB slots handed out since reset
  int  b_rob  [REC];
  int  b_cnt  [REC*N_CH];
  int  b_cum  [REC*N_CH];                          // packets per channel up to and including this request
  int  b_head [REC*N_CH];                          // ring slot of its first packet on the channel
  logic [AXI_AW-1:0]    pq_addr [N_CH*NPQ];        // per-channel queue of packets owed, in order
  logic [ROB_IDX_W-1:0] pq_rob  [N_CH*NPQ];
  int                   pq_seq  [N_CH*NPQ];
  int  pq_h [N_CH], pq_t [N_CH], fired [N_CH], chan_total [N_CH], bub [N_CH];
  bit  fire_v [N_CH];
  int  pk_ch [N_CH];                               // packets fired per channel (since time 0)

  int  cyc = 0, tot_pkts = 0, tot_gen = 0, tot_acc = 0, tot_pop = 0, lost_pend = 0, lost_acc = 0;
  bit  prev_accept = 0, prev_idle = 1;

  bit cov_b2b = 0, cov_robstall = 0, cov_qfull = 0, cov_ahead = 0, cov_dstall = 0, cov_rdystall = 0,
      cov_zero_behind = 0, cov_one_ch = 0, cov_all_ch = 0, cov_wrap = 0, cov_maxpk = 0, cov_unaligned = 0,
      cov_pageedge = 0, cov_idle_rise = 0, cov_reset_busy = 0, cov_cfg0 = 0, cov_coarse = 0, cov_reject = 0;

  task automatic chk(input bit c, input string msg);
    if (c) n_pass++;
    else begin n_fail++; $display("[%0t] FAIL r_burst_spltr(R=%0d,Q=%0d,P=%0d,S=%0d) %s", $time, ROB_DEPTH, QUEUE_DEPTH, MAX_PKTS, N_SLOTS, msg); end
  endtask
  task automatic stim_chk(input bit c, input string msg);
    if (!c) begin n_fail++; $display("[%0t] FAIL r_burst_spltr TB STIMULUS ILLEGAL: %s", $time, msg); end
  endtask

  // ---------------- master / environment state ----------------
  bit  pending = 0;                                // an AR is being offered (held until accepted)
  logic [AXI_IDW-1:0] cur_id;
  logic [AXI_AW-1:0]  cur_addr;
  int  cur_len, cur_size;
  bit  cur_val;
  int  rand_left = 0, scr_n = 0, scr_i = 0;
  logic [AXI_AW-1:0]  scr_a [64];
  int  scr_l [64], scr_s [64];
  bit  scr_v [64];
  logic [AXI_AW-1:0]  dir_a [64];
  int  dir_l [64], dir_s [64];
  bit  dir_v [64];
  int  dir_n = 0;
  int  p_issue = 100, p_rob = 100;
  int  p_rdy [N_CH], p_dbf [N_CH];

  task model_reset();
    lost_pend = lost_pend + (pending ? 1 : 0);
    lost_acc  = lost_acc + s1_occ + (adm_cnt - pop_cnt);
    pending = 0; s1_occ = 0; adm_cnt = 0; pop_cnt = 0; rob_seq = 0;
    for (int c = 0; c < N_CH; c++) begin pq_h[c] = 0; pq_t[c] = 0; fired[c] = 0; chan_total[c] = 0; bub[c] = 0; end
    prev_accept = 0; prev_idle = 1;
  endtask

  // reference description of the request accepted into S1
  task s1_record();
    logic [AXI_AW-1:0] lb;
    s1_id = ar_id; s1_addr = ar_addr; s1_len = int'(ar_len); s1_size = int'(ar_size); s1_val = ar_validated;
    lb = last_byte(ar_addr, s1_len, s1_size);
    s1_l0 = int'(ar_addr[PAGE_LSB-1:PKT_OFF_W]);
    s1_ll = int'(lb[PAGE_LSB-1:PKT_OFF_W]);
    s1_nl = s1_ll - s1_l0 + 1;
    s1_bch = ref_ch(line_addr(ar_addr, s1_l0));
    for (int c = 0; c < N_CH; c++) s1_cnt[c] = 0;
    if (s1_val)
      for (int i = s1_l0; i <= s1_ll; i++) s1_cnt[ref_ch(line_addr(ar_addr, i))] = s1_cnt[ref_ch(line_addr(ar_addr, i))] + 1;
  endtask

  // the S1 request is admitted: enter it in the program-order scoreboard and the per-channel queues
  task book_push();
    int bi, c;
    bi = adm_cnt % REC;
    b_rob[bi] = int'(rob_idx);                     // the value the ROB offered in the admit cycle
    for (c = 0; c < N_CH; c++) begin
      b_cnt [bi*N_CH + c] = s1_cnt[c];
      b_head[bi*N_CH + c] = chan_total[c] % N_SLOTS;
      chan_total[c] = chan_total[c] + s1_cnt[c];
      b_cum [bi*N_CH + c] = chan_total[c];
    end
    if (s1_val)
      for (int i = s1_l0; i <= s1_ll; i++) begin
        c = ref_ch(line_addr(s1_addr, i));
        stim_chk(pq_t[c] - pq_h[c] < NPQ, "per-channel packet queue overflow");
        pq_addr[c*NPQ + (pq_t[c] % NPQ)] = line_addr(s1_addr, i);
        pq_rob [c*NPQ + (pq_t[c] % NPQ)] = rob_idx;
        pq_seq [c*NPQ + (pq_t[c] % NPQ)] = adm_cnt;
        pq_t[c] = pq_t[c] + 1;
      end
  endtask

  // One clock cycle: the caller has set every input. Compare all outputs with the scoreboard,
  // then advance the scoreboard with what happens at the clock edge (1 ns after it, never at it).
  task tick();
    bit exp_adm, exp_rdy, exp_seg, accept, exp_idle;
    int live, hidx, c, pf, npop;
    logic [N_CH-1:0] mask_e;
    #1;
    cyc++;
    live    = adm_cnt - pop_cnt;
    exp_adm = (s1_occ == 1) && rob_free && (live < QUEUE_DEPTH);
    exp_rdy = (s1_occ == 0) || exp_adm;
    exp_idle = (s1_occ == 0) && (live == 0);
    accept  = aresetn && ar_valid && exp_rdy;

    // AR handshake and admission (exact, from the contract)
    chk(ar_ready === exp_rdy,       $sformatf("c%0d ar_ready %b exp %b (s1=%0d live=%0d rob_free=%b)", cyc, ar_ready, exp_rdy, s1_occ, live, rob_free));
    chk(admit_valid === exp_adm,    $sformatf("c%0d admit_valid %b exp %b (s1=%0d live=%0d rob_free=%b)", cyc, admit_valid, exp_adm, s1_occ, live, rob_free));
    chk(idle === exp_idle,          $sformatf("c%0d idle %b exp %b", cyc, idle, exp_idle));
    if (exp_adm) begin
      mask_e = '0;
      for (c = 0; c < N_CH; c++) mask_e[c] = (s1_cnt[c] != 0);
      chk(admit_id === s1_id && admit_validated === s1_val, $sformatf("c%0d admit id/validated %0h/%b exp %0h/%b", cyc, admit_id, admit_validated, s1_id, s1_val));
      chk(admit_npkts === CNT_W'(s1_val ? s1_nl : 0), $sformatf("c%0d admit_npkts %0d exp %0d", cyc, admit_npkts, s1_val ? s1_nl : 0));
      chk(admit_addr_lo === s1_addr[PAGE_LSB-1:0], $sformatf("c%0d admit_addr_lo %0h exp %0h", cyc, admit_addr_lo, s1_addr[PAGE_LSB-1:0]));
      chk(admit_len === LEN_W'(s1_len) && admit_size === SIZE_W'(s1_size), $sformatf("c%0d admit len/size %0d/%0d exp %0d/%0d", cyc, admit_len, admit_size, s1_len, s1_size));
      chk(admit_base_ch === CH_W'(s1_bch), $sformatf("c%0d admit_base_ch %0d exp %0d", cyc, admit_base_ch, s1_bch));
      chk(admit_ch_mask === mask_e, $sformatf("c%0d admit_ch_mask %b exp %b", cyc, admit_ch_mask, mask_e));
      // channel bit above the page: every line of a request is on the same channel
      if (cur_flav >= 4 && cur_ncl == CH_W && s1_val)
        chk($countones(admit_ch_mask) == 1, $sformatf("c%0d page-constant channel map but mask %b", cyc, admit_ch_mask));
    end

    // packets: each channel offers the next owed packet, in order, only with a free RD-SRAM slot
    for (c = 0; c < N_CH; c++) begin
      fire_v[c] = 0;
      pf = c*NPQ + (pq_h[c] % NPQ);
      if (pkt_valid[c] === 1'b1) begin
        chk(dbuf_rdy[c] === 1'b1, $sformatf("c%0d ch%0d pkt_valid while no RD-SRAM slot", cyc, c));
        if (pq_t[c] > pq_h[c]) begin
          chk(pkt_rob_idx[c] === pq_rob[pf], $sformatf("c%0d ch%0d pkt_rob_idx %0d exp %0d", cyc, c, pkt_rob_idx[c], pq_rob[pf]));
          chk(pkt_dbuf_adr[c] === SLOT_W'(fired[c] % N_SLOTS), $sformatf("c%0d ch%0d pkt_dbuf_adr %0d exp %0d", cyc, c, pkt_dbuf_adr[c], fired[c] % N_SLOTS));
          chk(pkt_daddr[c] === ref_daddr(pq_addr[pf]), $sformatf("c%0d ch%0d pkt_daddr %h exp %h (packet %0d)", cyc, c, pkt_daddr[c], ref_daddr(pq_addr[pf]), pq_h[c]));
          if (aresetn && pkt_ready[c] === 1'b1) begin
            fire_v[c] = 1;
            if (pq_seq[pf] > pop_cnt) cov_ahead = 1;      // working on a request younger than the oldest
          end
          if (pkt_ready[c] !== 1'b1) cov_rdystall = 1;
        end else chk(0, $sformatf("c%0d ch%0d pkt_valid with nothing owed", cyc, c));
      end else chk(pkt_valid[c] === 1'b0, $sformatf("c%0d ch%0d pkt_valid %b", cyc, c, pkt_valid[c]));
      // an owed packet with a free slot must be offered within the time it takes to skip the queue
      if (pq_t[c] > pq_h[c] && dbuf_rdy[c] === 1'b1 && pkt_valid[c] !== 1'b1) bub[c] = bub[c] + 1; else bub[c] = 0;
      chk(bub[c] <= QUEUE_DEPTH + 3, $sformatf("c%0d ch%0d stuck: packet owed, slot free, not offered for %0d cycles", cyc, c, bub[c]));
      if (pq_t[c] > pq_h[c] && dbuf_rdy[c] !== 1'b1) cov_dstall = 1;
    end

    // seg_last: in order, once every channel has issued all its packets of the oldest request
    exp_seg = 0;
    hidx = pop_cnt % REC;
    if (adm_cnt > pop_cnt) begin
      exp_seg = 1;
      for (c = 0; c < N_CH; c++) if (fired[c] < b_cum[hidx*N_CH + c]) exp_seg = 0;
    end
    chk(seg_last === exp_seg, $sformatf("c%0d seg_last %b exp %b (oldest request %0d)", cyc, seg_last, exp_seg, pop_cnt));
    if (exp_seg) begin
      chk(seg_rob_idx === ROB_IDX_W'(b_rob[hidx]), $sformatf("c%0d seg_rob_idx %0d exp %0d", cyc, seg_rob_idx, b_rob[hidx]));
      for (c = 0; c < N_CH; c++)
        if (b_cnt[hidx*N_CH + c] > 0)
          chk(seg_head_ptr[c] === SLOT_W'(b_head[hidx*N_CH + c]), $sformatf("c%0d seg_head_ptr[%0d] %0d exp %0d", cyc, c, seg_head_ptr[c], b_head[hidx*N_CH + c]));
    end

    // coverage of the situations exercised (pre-edge view)
    if (aresetn) begin
      if (s1_occ == 1 && !rob_free && ar_valid) cov_robstall = 1;
      if (s1_occ == 1 && rob_free && live == QUEUE_DEPTH) cov_qfull = 1;
      if (accept && prev_accept) cov_b2b = 1;
      if (exp_idle && !prev_idle) cov_idle_rise = 1;
    end

    @(posedge aclk); #1;
    if (!aresetn) model_reset();
    else begin
      for (c = 0; c < N_CH; c++)
        if (fire_v[c]) begin pq_h[c] = pq_h[c] + 1; fired[c] = fired[c] + 1; pk_ch[c] = pk_ch[c] + 1; tot_pkts++; end
      if (exp_seg) begin pop_cnt = pop_cnt + 1; tot_pop++; if (pop_cnt >= 2*QUEUE_DEPTH) cov_wrap = 1; end
      if (exp_adm) begin
        if (!s1_val) begin cov_reject = 1; if (live > 0) cov_zero_behind = 1; end
        else begin
          npop = $countones(mask_e);
          if (npop == 1)      cov_one_ch = 1;
          if (npop == N_CH)   cov_all_ch = 1;
          if (s1_nl == MAX_PKTS)        cov_maxpk = 1;
          if (s1_ll == PG_LINES - 1)    cov_pageedge = 1;
          if (s1_addr[PKT_OFF_W-1:0] != 0) cov_unaligned = 1;
        end
        book_push();
        rob_seq = rob_seq + 1; adm_cnt = adm_cnt + 1; s1_occ = 0;
      end
      if (accept) begin
        stim_chk(legal(ar_addr, int'(ar_len), int'(ar_size)), $sformatf("request addr %h len %0d size %0d crosses a page or exceeds MAX_PKTS", ar_addr, ar_len, ar_size));
        s1_record();
        s1_occ = 1; pending = 0; tot_acc++;
      end
      prev_accept = accept; prev_idle = exp_idle;
    end
  endtask

  // ---------------- drive helpers ----------------
  task gen_rand_req();
    int sz, len;
    logic [AXI_AW-1:0] a;
    bit v;
    v = ($urandom % 8) != 0;
    a = AXI_AW'({$urandom, $urandom}) & ADDR_MASK;
    if (!v) a = a | BAD_BIT;                        // address outside the mapped range: the validator rejects it
    sz  = (($urandom % 2) == 0) ? SIZE_MAX : ($urandom % (SIZE_MAX + 1));
    len = $urandom % MAX_BEATS;
    if (($urandom % 3) == 0) len = $urandom % 8;
    if (($urandom % 4) == 0) a[PKT_OFF_W-1:0] = '0;
    if (($urandom % 8) == 0) a[PAGE_LSB-1:0] = '0;
    for (int t = 0; t < 64 && !legal(a, len, sz); t++) len = len / 2;
    cur_id = AXI_IDW'($urandom); cur_addr = a; cur_len = len; cur_size = sz; cur_val = v;
  endtask

  task drive();
    if (!pending) begin
      if (scr_i < scr_n) begin
        cur_id = AXI_IDW'($urandom); cur_addr = scr_a[scr_i]; cur_len = scr_l[scr_i]; cur_size = scr_s[scr_i]; cur_val = scr_v[scr_i];
        scr_i = scr_i + 1; pending = 1; tot_gen++;
      end else if (rand_left > 0 && ($urandom % 100) < p_issue) begin
        gen_rand_req(); rand_left = rand_left - 1; pending = 1; tot_gen++;
      end
    end
    ar_valid = pending;
    if (pending) begin
      ar_id = cur_id; ar_addr = cur_addr; ar_len = LEN_W'(cur_len); ar_size = SIZE_W'(cur_size); ar_validated = cur_val;
    end else begin                                  // payload is undefined while ar_valid is low
      ar_id = AXI_IDW'($urandom); ar_addr = AXI_AW'({$urandom, $urandom}); ar_len = LEN_W'($urandom);
      ar_size = SIZE_W'($urandom); ar_validated = ($urandom % 2) == 1;
    end
    rob_free = (($urandom % 100) < p_rob);
    rob_idx  = rob_free ? ROB_IDX_W'(rob_seq % ROB_DEPTH) : ROB_IDX_W'($urandom);
    for (int c = 0; c < N_CH; c++) begin
      pkt_ready[c] = (($urandom % 100) < p_rdy[c]);
      dbuf_rdy[c]  = (($urandom % 100) < p_dbf[c]);
    end
  endtask

  function automatic bit quiet();
    return !pending && scr_i >= scr_n && rand_left == 0 && s1_occ == 0 && adm_cnt == pop_cnt;
  endfunction

  task run_cycles(input int n);
    repeat (n) begin drive(); tick(); end
  endtask

  task run_drain(input int cap);
    int g;
    g = 0;
    while (!quiet() && g < cap) begin drive(); tick(); g++; end
    chk(quiet(), $sformatf("drain timeout: pending=%b s1=%0d live=%0d script %0d/%0d rand_left %0d", pending, s1_occ, adm_cnt - pop_cnt, scr_i, scr_n, rand_left));
  endtask

  task set_knobs(input int pi, input int pr, input int rdy, input int dbf);
    p_issue = pi; p_rob = pr;
    for (int c = 0; c < N_CH; c++) begin p_rdy[c] = rdy; p_dbf[c] = dbf; end
  endtask

  task scr_clear();
    scr_n = 0; scr_i = 0;
  endtask
  task scr_add(input logic [AXI_AW-1:0] a, input int len, input int sz, input bit v);
    if (scr_n < 64 && legal(a, len, sz)) begin
      scr_a[scr_n] = a; scr_l[scr_n] = len; scr_s[scr_n] = sz; scr_v[scr_n] = v; scr_n = scr_n + 1;
    end
  endtask
  task dir_add(input logic [AXI_AW-1:0] a, input int len, input int sz, input bit v);
    if (dir_n < 64 && legal(a, len, sz)) begin
      dir_a[dir_n] = a; dir_l[dir_n] = len; dir_s[dir_n] = sz; dir_v[dir_n] = v; dir_n = dir_n + 1;
    end
  endtask

  // directed requests (page PB; offsets inside the page). Illegal ones for a small MAX_PKTS are dropped.
  task build_dir();
    logic [AXI_AW-1:0] pb, bad;
    pb  = AXI_AW'(48'h0_0123_4000) & ADDR_MASK;
    bad = pb | BAD_BIT;
    dir_n = 0;
    dir_add(pb + 'h080, 19, 4, 1);                  // RTL worked example A: lines 2..6
    dir_add(pb + 'h000,  0, 4, 1);                  // one line
    dir_add(pb + 'h040,  3, 4, 1);                  // exactly one line
    dir_add(pb + 'h030,  3, 4, 1);                  // 16 B beats crossing a line: 2 lines
    dir_add(pb + 'h03F,  0, 0, 1);                  // single byte at the end of a line
    dir_add(pb + 'h03F,  1, 0, 1);                  // 2nd byte is in the next line
    dir_add(pb + 'h07F,  1, 0, 1);
    dir_add(pb + 'h00C,  7, 3, 1);                  // unaligned start, 8 B beats: burst ends in line 1
    dir_add(pb + 'h01F,  5, 2, 1);
    dir_add(pb + 'h123,  0, 0, 1);
    dir_add(pb + 'h124,  1, 1, 1);
    dir_add(pb + 'hFC0,  3, 4, 1);                  // last line of the page
    dir_add(pb + 'hF80,  7, 4, 1);                  // lines 62..63
    dir_add(pb + 'h000, MAX_PKTS*4 - 1, 4, 1);      // MAX_PKTS lines from line 0
    dir_add(pb + 'h040, MAX_PKTS*4 - 1, 4, 1);      // MAX_PKTS lines from an odd line (dropped at 64: crosses)
    dir_add(pb + 'h000, 255, 4, 1);                 // whole page (only if MAX_PKTS = 64)
    dir_add(bad + 'h100,  3, 4, 0);                 // rejected: out of range, no packets
    dir_add(bad + 'h080, 19, 4, 0);                 // rejected multi-line
    dir_add(bad + 'h000,  0, 0, 0);
  endtask

  // ---------------- resets ----------------
  task do_reset();
    ar_valid = 0; aresetn = 1; #1; aresetn = 0; model_reset(); tick(); tick(); aresetn = 1; tick();
  endtask

  task midrun_reset();
    if (s1_occ == 1 || adm_cnt != pop_cnt || pending) cov_reset_busy = 1;
    ar_valid = 0;
    #2 aresetn = 0; model_reset(); #1;
    chk(ar_ready === 1'b1 && admit_valid === 1'b0 && pkt_valid === '0 && seg_last === 1'b0 && idle === 1'b1,
        "async reset: S1 and queue cleared, outputs idle");
    tick(); aresetn = 1; tick();
  endtask

  // ---------------- one configuration phase ----------------
  task run_phase(input int flavor, input int ncl, input int ncyc);
    int acc0, adm0, pop0, k, pk0_0, pk0_1, ph_pkts;
    logic [AXI_AW-1:0] pb, bad;
    set_map(flavor, ncl);
    do_reset();
    pb  = AXI_AW'(48'h0_0456_7000) & ADDR_MASK;
    bad = pb | BAD_BIT;
    pk0_0 = pk_ch[0]; pk0_1 = pk_ch[N_CH-1]; ph_pkts = tot_pkts;
    if (ncl == 0) cov_cfg0 = 1;
    if (flavor >= 2) cov_coarse = 1;
    scr_clear(); rand_left = 0; set_knobs(100, 100, 100, 100);

    // D1 reset / idle
    run_cycles(4);
    chk(idle === 1'b1 && ar_ready === 1'b1 && admit_valid === 1'b0 && pkt_valid === '0 && seg_last === 1'b0, "D1 idle after reset");

    // D2 directed requests, one at a time with everything ready, then the same list back to back
    build_dir();
    for (int d = 0; d < dir_n; d++) begin
      scr_clear(); scr_add(dir_a[d], dir_l[d], dir_s[d], dir_v[d]);
      run_drain(2000);
    end
    scr_clear();
    for (int d = 0; d < dir_n; d++) scr_add(dir_a[d], dir_l[d], dir_s[d], dir_v[d]);
    run_drain(20000);

    // D3 ROB unavailable: S1 takes one request, nothing is admitted, the second is not accepted
    // (adm_cnt / pop_cnt / tot_acc are cumulative since the last reset: each step compares against a baseline)
    scr_clear(); acc0 = tot_acc; adm0 = adm_cnt;
    for (k = 0; k < 4; k++) scr_add(pb + k*'h100, 3, 4, 1);
    set_knobs(100, 0, 100, 100);
    run_cycles(12);
    chk(tot_acc - acc0 == 1 && adm_cnt == adm0 && s1_occ == 1, "D3 rob_free low: exactly one request held in S1, none admitted");
    set_knobs(100, 50, 100, 100); run_cycles(30);                // rob_free toggling
    set_knobs(100, 100, 100, 100); run_drain(2000);

    // D4 queue full: no pops (command FIFOs full), requests pile up to QUEUE_DEPTH, then S1 and the AR master stall
    scr_clear(); adm0 = adm_cnt; pop0 = pop_cnt;
    for (k = 0; k < QUEUE_DEPTH + 3; k++) scr_add(pb + ((k * 64) % 4096), 0, 4, 1);
    set_knobs(100, 100, 0, 100);
    run_cycles(QUEUE_DEPTH * 3 + 20);
    chk(adm_cnt - adm0 == QUEUE_DEPTH && adm_cnt - pop_cnt == QUEUE_DEPTH && pop_cnt == pop0 && s1_occ == 1,
        "D4 queue holds exactly QUEUE_DEPTH requests, none popped, S1 stalled behind it");
    set_knobs(100, 100, 100, 100); run_drain(4000);

    // D5 rejected / zero-packet requests behind an older request that is still owed packets
    scr_clear(); adm0 = adm_cnt; pop0 = pop_cnt;
    scr_add(pb + 'h000, 3, 4, 1);
    scr_add(bad + 'h100, 3, 4, 0);
    scr_add(bad + 'h200, 0, 4, 0);
    scr_add(pb + 'h300, 3, 4, 1);
    set_knobs(100, 100, 0, 100);
    run_cycles(20);
    // all four are admitted (as many as the queue holds; the rest wait in S1) but none pops: the
    // rejected ones sit behind the oldest request, which still owes packets
    chk(adm_cnt - adm0 == ((QUEUE_DEPTH < 4) ? QUEUE_DEPTH : 4) && pop_cnt == pop0,
        "D5 no seg_last while the oldest request still owes packets");
    set_knobs(100, 100, 100, 100); run_drain(2000);

    // D6 back-to-back requests, all ready
    scr_clear();
    for (k = 0; k < 16; k++) scr_add(pb + ((k * 64) % 4096), 0, 4, 1);
    run_drain(2000);

    // D7 channel independence: the last channel's command FIFO is stuck, the others run ahead
    set_knobs(100, 100, 100, 100); p_rdy[N_CH-1] = 0;
    scr_clear(); rand_left = QUEUE_DEPTH + 6;
    run_cycles(QUEUE_DEPTH * 8 + 40);
    set_knobs(100, 100, 100, 100); run_drain(8000);

    // D8 per-channel RD-SRAM stall
    p_dbf[0] = 0; rand_left = 10;
    run_cycles(40);
    set_knobs(100, 100, 100, 100); run_drain(8000);

    // D9 reset with requests in every stage (pending AR, S1, queue entries, a channel stuck mid-request)
    set_knobs(100, 100, 0, 100);
    rand_left = QUEUE_DEPTH + 4;
    run_cycles(QUEUE_DEPTH * 3 + 20);
    midrun_reset();
    set_knobs(100, 100, 100, 100); rand_left = 6;
    run_drain(4000);                                // the model expects the RD-SRAM rings back at slot 0

    // D10 constrained random, in phases of 150 cycles
    for (int c = 0; c < ncyc; c++) begin
      if (c % 150 == 0) begin
        case ((c / 150) % 5)
          0: set_knobs(60, 70, 70, 70);
          1: set_knobs(100, 100, 25, 90);              // command FIFOs mostly full
          2: set_knobs(100, 100, 90, 20);              // RD-SRAM mostly full
          3: set_knobs(40, 20, 90, 90);                // ROB mostly full
          default: set_knobs(100, 100, 100, 100);
        endcase
        for (int cc = 0; cc < N_CH; cc++) if (($urandom % 4) == 0) p_rdy[cc] = 5;   // one channel often stuck
      end
      if (rand_left < 4) rand_left = 4;
      drive(); tick();
    end
    rand_left = 0; set_knobs(100, 100, 100, 100);
    run_drain(60000);
    run_cycles(3);
    chk(idle === 1'b1, "D10 idle once everything is consumed");

    // phase-level expectations that do not come from the model
    chk(tot_gen == tot_acc + lost_pend && tot_acc == tot_pop + lost_acc,
        $sformatf("request conservation: generated %0d accepted %0d popped %0d (lost %0d/%0d)", tot_gen, tot_acc, tot_pop, lost_pend, lost_acc));
    if (ncl == 0) begin
      chk(pk_ch[0] > pk0_0 && pk_ch[N_CH-1] == pk0_1, "one active channel: only channel 0 issues packets");
    end else begin
      chk(tot_pkts - ph_pkts > 20, "phase issued packets (stimulus vacuity guard)");
    end
  endtask

  initial begin : main
    int seed_v;
    n_pass = 0; n_fail = 0; n_pkts_seen = 0; n_cov = 0; done = 0;
    seed_v = SEED; void'($urandom(seed_v));
    for (int c = 0; c < N_CH; c++) begin pk_ch[c] = 0; p_rdy[c] = 100; p_dbf[c] = 100; end
    chk(N_SLOTS >= MAX_PKTS && QUEUE_DEPTH <= ROB_DEPTH && (N_SLOTS & (N_SLOTS-1)) == 0 && (MAX_PKTS & (MAX_PKTS-1)) == 0 &&
        (QUEUE_DEPTH & (QUEUE_DEPTH-1)) == 0, "TB configuration violates the splitter contract");
    for (int f = 0; f < NFLAV; f++) run_phase(f, CH_W, NRAND / (NFLAV + 1));
    run_phase(0, 0, NRAND / (NFLAV + 1));
    n_pkts_seen = tot_pkts;
    chk(cov_b2b && cov_robstall && cov_qfull && cov_ahead && cov_dstall && cov_rdystall && cov_zero_behind &&
        cov_one_ch && cov_all_ch && cov_wrap && cov_maxpk && cov_unaligned && cov_pageedge && cov_idle_rise &&
        cov_reset_busy && cov_cfg0 && cov_coarse && cov_reject,
        $sformatf("coverage b2b=%b robstall=%b qfull=%b ahead=%b dstall=%b rdystall=%b zerobehind=%b onech=%b allch=%b wrap=%b maxpk=%b unaligned=%b pageedge=%b idlerise=%b rstbusy=%b cfg0=%b coarse=%b reject=%b",
                  cov_b2b, cov_robstall, cov_qfull, cov_ahead, cov_dstall, cov_rdystall, cov_zero_behind, cov_one_ch,
                  cov_all_ch, cov_wrap, cov_maxpk, cov_unaligned, cov_pageedge, cov_idle_rise, cov_reset_busy,
                  cov_cfg0, cov_coarse, cov_reject));
    n_cov = cov_b2b + cov_robstall + cov_qfull + cov_ahead + cov_dstall + cov_rdystall + cov_zero_behind + cov_one_ch +
            cov_all_ch + cov_wrap + cov_maxpk + cov_unaligned + cov_pageedge + cov_idle_rise + cov_reset_busy +
            cov_cfg0 + cov_coarse + cov_reject;
    done = 1;
  end

  initial begin : wd
    #(10 * (NRAND * 60 + 6000000));
    if (!done) begin n_fail++; $display("[%0t] FAIL r_burst_spltr(Q=%0d) watchdog: test did not finish", $time, QUEUE_DEPTH); done = 1; end
  end
endmodule

// -------------------------------------------------------------------------------------------
// Read-path flow composite: the real splitter + the real rmc_cif_rrob wired as rmc_cif.sv does
// (rmc_cif.sv:210-300): the splitter owns the AR handshake, rob_free / rob_idx come from the rrob,
// admit_* feed the rrob admit, seg_* feed SEG_*. The TB plays the memory controller (it completes
// every fired packet by its rob tag after a random delay, two completion streams, one completion
// per cycle) and the R-response sink. Checks the in-order response descriptor, the rrob's
// ready/valid behaviour, and the slot / tag bookkeeping across the two blocks.
// -------------------------------------------------------------------------------------------
module flow_chk #(
  parameter int ROB_DEPTH = 8,
  parameter int MAX_PKTS  = 16,
  parameter int AXI_IDW   = 8,
  parameter int N_SLOTS   = 32,
  parameter int NREQ      = 500,
  parameter int FLAV      = 0,
  parameter int SEED      = 7
)(
  output int  n_pass,
  output int  n_fail,
  output int  n_resp,
  output bit  done
);
  import rmc_cfg_pkg::*;
  localparam int AXI_AW     = SYS_ADDR_W;
  localparam int ROB_IDX_W  = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1;
  localparam int CNT_W      = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1;
  localparam int SLOT_W     = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1;
  localparam int DATA_BYTES = AXI_DW / 8;
  localparam int SIZE_MAX   = (DATA_BYTES > 1) ? $clog2(DATA_BYTES) : 0;
  localparam int MAX_BEATS  = MAX_REQ_BYTES / DATA_BYTES;
  localparam int LEN_W      = (MAX_BEATS  > 1) ? $clog2(MAX_BEATS)  : 1;
  localparam int SIZE_W     = (SIZE_MAX   > 0) ? $clog2(SIZE_MAX+1) : 1;
  localparam int NCH_LOG_W  = $clog2(CH_W + 1);
  localparam int LN_W       = (PG_LINES   > 1) ? $clog2(PG_LINES)   : 1;
  localparam int NB         = ROB_DEPTH + 4;                       // request book window
  localparam int CQN        = ROB_DEPTH * MAX_PKTS + 16;           // completion FIFO per channel
  localparam logic [AXI_AW-1:0] ADDR_MASK = AXI_AW'((48'd1 << MAP_ADDR_W) - 48'd1);
  localparam logic [AXI_AW-1:0] BAD_BIT   = AXI_AW'(48'd1) << (AXI_AW - 1);

  logic aclk = 1'b0, aresetn = 1'b1;
  always #5 aclk = ~aclk;

  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map;
  logic [NCH_LOG_W-1:0] cfg_ncl;
  logic                 ar_valid = 0, ar_ready;
  logic [AXI_IDW-1:0]   ar_id = '0;
  logic [AXI_AW-1:0]    ar_addr = '0;
  logic [LEN_W-1:0]     ar_len = '0;
  logic [SIZE_W-1:0]    ar_size = '0;
  logic                 ar_validated = 0;
  logic                 rob_free;
  logic [ROB_IDX_W-1:0] rob_idx;
  logic                 admit_valid;
  logic [AXI_IDW-1:0]   admit_id;
  logic                 admit_validated;
  logic [CNT_W-1:0]     admit_npkts;
  logic [PAGE_LSB-1:0]  admit_addr_lo;
  logic [LEN_W-1:0]     admit_len;
  logic [SIZE_W-1:0]    admit_size;
  logic [CH_W-1:0]      admit_base_ch;
  logic [N_CH-1:0]      admit_ch_mask;
  logic [N_CH-1:0]      pkt_valid, pkt_ready = '0, dbuf_rdy = '0;
  logic [N_CH-1:0][ROB_IDX_W-1:0] pkt_rob_idx;
  logic [N_CH-1:0][DADDR_W-1:0]   pkt_daddr;
  logic [N_CH-1:0][SLOT_W-1:0]    pkt_dbuf_adr;
  logic                 seg_last;
  logic [ROB_IDX_W-1:0] seg_rob_idx;
  logic [N_CH-1:0][SLOT_W-1:0]    seg_head_ptr;
  logic                 idle;
  logic                 cmpl_valid = 0, cmpl_err = 0;
  logic [ROB_IDX_W-1:0] cmpl_rob_index = '0;
  logic                 RESP_VALID, RESP_READY = 0;
  logic [AXI_IDW-1:0]   RESP_ID;
  logic [1:0]           RESP_RESP;
  logic [N_CH-1:0][SLOT_W-1:0]    RESP_HEAD_PTR;
  logic [CH_W-1:0]      RESP_BASE_CH;
  logic [PAGE_LSB-1:0]  RESP_ADDR_LO;
  logic [7:0]           RESP_LEN;
  logic [2:0]           RESP_SIZE;

  rmc_cif_r_burst_spltr #(.ROB_DEPTH(ROB_DEPTH), .MAX_PKTS(MAX_PKTS), .AXI_AW(AXI_AW),
                          .AXI_IDW(AXI_IDW), .N_SLOTS(N_SLOTS)) u_spl (
    .aclk, .aresetn, .addr_map, .cfg_n_ch_log2(cfg_ncl),
    .ar_valid, .ar_ready, .ar_id, .ar_addr, .ar_len, .ar_size, .ar_validated,
    .rob_free, .rob_idx,
    .admit_valid, .admit_id, .admit_validated, .admit_npkts, .admit_addr_lo, .admit_len,
    .admit_size, .admit_base_ch, .admit_ch_mask,
    .pkt_valid, .pkt_ready, .pkt_dbuf_adr_ready(dbuf_rdy), .pkt_rob_idx, .pkt_daddr, .pkt_dbuf_adr,
    .seg_last, .seg_rob_idx, .seg_head_ptr, .idle);
  rmc_cif_rrob #(.ROB_DEPTH(ROB_DEPTH), .MAX_PKTS(MAX_PKTS), .AXI_IDW(AXI_IDW), .N_SLOTS(N_SLOTS)) u_rrob (
    .aclk, .aresetn, .cmpl_valid, .cmpl_rob_index, .cmpl_err,
    .AW_REQUEST({admit_id, admit_validated}), .ADMIT_NPKTS(admit_npkts), .ADMIT_ADDR_LO(admit_addr_lo),
    .ADMIT_LEN(8'(admit_len)), .ADMIT_SIZE(3'(admit_size)), .ADMIT_BASE_CH(admit_base_ch),
    .A_ID_VALID(admit_valid), .A_ID_READY(rob_free), .ROB_IDX(rob_idx),
    .SEG_LAST(seg_last), .SEG_ROB_IDX(seg_rob_idx), .SEG_HEAD_PTR(seg_head_ptr),
    .RESP_VALID, .RESP_READY, .RESP_ID, .RESP_RESP, .RESP_HEAD_PTR, .RESP_BASE_CH,
    .RESP_ADDR_LO, .RESP_LEN, .RESP_SIZE);

  // ---- channel map (as in rbs_chk; this composite runs one placement) ----
  int ch_src [CH_W];
  int cur_ncl = CH_W;
  function automatic int flav_pos(input int f);
    if (f == 0) return CH_POS; else if (f == 1) return 7; else if (f == 2) return 9;
    else if (f == 3) return 11; else if (f == 4) return 12; else return 15;
  endfunction
  function automatic logic [AXI_AW-1:0] line_addr(input logic [AXI_AW-1:0] a, input int i);
    return {a[AXI_AW-1:PAGE_LSB], LN_W'(i), {PKT_OFF_W{1'b0}}};
  endfunction
  function automatic int ref_ch(input logic [AXI_AW-1:0] a);
    int c;
    c = 0;
    for (int k = 0; k < CH_W; k++)
      if (k < cur_ncl && a[ch_src[k]]) c = c | (1 << k);
    return c;
  endfunction
  function automatic logic [AXI_AW-1:0] last_byte(input logic [AXI_AW-1:0] a, input int len, input int sz);
    logic [AXI_AW-1:0] fa;
    fa = (a >> sz) << sz;
    return fa + AXI_AW'((len + 1) << sz) - AXI_AW'(1);
  endfunction
  function automatic bit legal(input logic [AXI_AW-1:0] a, input int len, input int sz);
    logic [AXI_AW-1:0] lb;
    lb = last_byte(a, len, sz);
    return ((a >> PAGE_LSB) == (lb >> PAGE_LSB)) && (int'((lb >> PKT_OFF_W) - (a >> PKT_OFF_W)) < MAX_PKTS);
  endfunction

  // ---- request book, program order (index = sequence % NB) ----
  int  acc_seq = 0, adm_seq = 0, seg_seq = 0, ret_seq = 0, gen = 0, cyc = 0;
  logic [AXI_IDW-1:0]  b_id   [NB];
  bit                  b_val  [NB];
  int                  b_len  [NB], b_size [NB], b_bch [NB], b_n [NB], b_cmpl [NB];
  bit                  b_derr [NB];
  logic [AXI_AW-1:0]   b_addr [NB];
  int                  b_cnt  [NB*N_CH], b_head [NB*N_CH], b_iss [NB*N_CH];
  int                  chan_total [N_CH];
  // modelled MC: per-channel FIFO of packets in flight
  int                  cq_idx [N_CH*CQN], cq_due [N_CH*CQN];
  bit                  cq_err [N_CH*CQN];
  int                  cq_h [N_CH], cq_t [N_CH];
  int                  cm_pick = -1, cm_s = -1;
  // AR master
  bit                  pending = 0;
  logic [AXI_IDW-1:0]  cur_id;
  logic [AXI_AW-1:0]   cur_addr;
  int                  cur_len, cur_size;
  bit                  cur_val;
  bit                  stall_prev = 0;
  localparam int RCAT_W = AXI_IDW + 2 + N_CH*SLOT_W + CH_W + PAGE_LSB + 8 + 3;
  wire [RCAT_W-1:0]    rcat = {RESP_ID, RESP_RESP, RESP_HEAD_PTR, RESP_BASE_CH, RESP_ADDR_LO, RESP_LEN, RESP_SIZE};
  logic [RCAT_W-1:0]   rsaved;

  task automatic chk(input bit c, input string msg);
    if (c) n_pass++;
    else begin n_fail++; $display("[%0t] FAIL flow(R=%0d,P=%0d,S=%0d,map=%0d) %s", $time, ROB_DEPTH, MAX_PKTS, N_SLOTS, FLAV, msg); end
  endtask

  function automatic int find_live(input int idx);                   // admitted, not retired
    for (int k = 0; k < adm_seq - ret_seq; k++)
      if (((ret_seq + k) % ROB_DEPTH) == idx) return ret_seq + k;
    return -1;
  endfunction

  task gen_req();
    int sz, len;
    logic [AXI_AW-1:0] a;
    bit v;
    v = ($urandom % 8) != 0;
    a = AXI_AW'({$urandom, $urandom}) & ADDR_MASK;
    if (!v) a = a | BAD_BIT;
    sz  = (($urandom % 2) == 0) ? SIZE_MAX : ($urandom % (SIZE_MAX + 1));
    len = $urandom % MAX_BEATS;
    if (($urandom % 3) == 0) len = $urandom % 8;
    if (($urandom % 4) == 0) a[PKT_OFF_W-1:0] = '0;
    for (int t = 0; t < 64 && !legal(a, len, sz); t++) len = len / 2;
    cur_id = AXI_IDW'($urandom); cur_addr = a; cur_len = len; cur_size = sz; cur_val = v;
  endtask

  // inputs for this cycle
  task drive();
    int c0, c;
    if (!pending && gen < NREQ && ($urandom % 100) < 70) begin gen_req(); pending = 1; gen = gen + 1; end
    ar_valid = pending;
    if (pending) begin
      ar_id = cur_id; ar_addr = cur_addr; ar_len = LEN_W'(cur_len); ar_size = SIZE_W'(cur_size); ar_validated = cur_val;
    end else begin
      ar_id = AXI_IDW'($urandom); ar_addr = AXI_AW'({$urandom, $urandom}); ar_len = LEN_W'($urandom);
      ar_size = SIZE_W'($urandom); ar_validated = ($urandom % 2) == 1;
    end
    for (c = 0; c < N_CH; c++) begin
      pkt_ready[c] = ($urandom % 100) < 70;
      dbuf_rdy[c]  = ($urandom % 100) < 85;
    end
    RESP_READY = ($urandom % 100) < 65;
    // memory controller: complete one due packet this cycle, either stream, by its rob tag
    cmpl_valid = 0; cmpl_err = 0; cm_pick = -1;
    c0 = $urandom % N_CH;
    for (int j = 0; j < N_CH && cm_pick < 0; j++) begin
      c = (c0 + j) % N_CH;
      if (cq_h[c] != cq_t[c] && cq_due[c*CQN + (cq_h[c] % CQN)] <= cyc) cm_pick = c;
    end
    if (cm_pick >= 0) begin
      cmpl_valid = 1;
      cmpl_rob_index = ROB_IDX_W'(cq_idx[cm_pick*CQN + (cq_h[cm_pick] % CQN)]);
      cmpl_err = cq_err[cm_pick*CQN + (cq_h[cm_pick] % CQN)];
    end
  endtask

  task tick();
    bit exp_rv, accept, retire;
    int c, s, bi, nxt;
    logic [AXI_AW-1:0] lb;
    #1;
    cyc++;
    // rrob offered slot / ready
    chk(rob_free === ((adm_seq - ret_seq) != ROB_DEPTH), $sformatf("c%0d rob_free %b with %0d live", cyc, rob_free, adm_seq - ret_seq));
    chk(rob_idx === ROB_IDX_W'(adm_seq % ROB_DEPTH), $sformatf("c%0d rob_idx %0d exp %0d", cyc, rob_idx, adm_seq % ROB_DEPTH));
    // admit: the oldest accepted-not-admitted request
    if (admit_valid === 1'b1) begin
      bi = adm_seq % NB;
      chk(adm_seq < acc_seq, $sformatf("c%0d admit with no accepted request", cyc));
      chk(admit_id === b_id[bi] && admit_validated === b_val[bi], $sformatf("c%0d admit id/validated", cyc));
      chk(admit_npkts === CNT_W'(b_n[bi]), $sformatf("c%0d admit_npkts %0d exp %0d", cyc, admit_npkts, b_n[bi]));
      chk(admit_addr_lo === b_addr[bi][PAGE_LSB-1:0] && admit_len === LEN_W'(b_len[bi]) && admit_size === SIZE_W'(b_size[bi]) &&
          admit_base_ch === CH_W'(b_bch[bi]), $sformatf("c%0d admit trim meta", cyc));
      for (c = 0; c < N_CH; c++) chk(admit_ch_mask[c] === (b_cnt[bi*N_CH + c] != 0), $sformatf("c%0d admit_ch_mask[%0d]", cyc, c));
    end
    // packets: owed, tagged with the right slot
    for (c = 0; c < N_CH; c++)
      if (pkt_valid[c] === 1'b1 && pkt_ready[c] === 1'b1) begin
        s = find_live(int'(pkt_rob_idx[c]));
        if (s < 0) chk(0, $sformatf("c%0d ch%0d packet tagged rob %0d matches no admitted request", cyc, c, pkt_rob_idx[c]));
        else chk(b_iss[(s % NB)*N_CH + c] < b_cnt[(s % NB)*N_CH + c], $sformatf("c%0d ch%0d packet beyond the lines owed on this channel (req %0d)", cyc, c, s));
      end
    // seg_last: in order, everything issued, rrob gets the per-channel heads
    if (seg_last === 1'b1) begin
      bi = seg_seq % NB;
      chk(seg_seq < adm_seq, $sformatf("c%0d seg_last with no admitted request", cyc));
      chk(seg_rob_idx === ROB_IDX_W'(seg_seq % ROB_DEPTH), $sformatf("c%0d seg_rob_idx %0d exp %0d", cyc, seg_rob_idx, seg_seq % ROB_DEPTH));
      for (c = 0; c < N_CH; c++) begin
        chk(b_iss[bi*N_CH + c] == b_cnt[bi*N_CH + c], $sformatf("c%0d seg_last before channel %0d issued all its packets", cyc, c));
        if (b_cnt[bi*N_CH + c] > 0) chk(seg_head_ptr[c] === SLOT_W'(b_head[bi*N_CH + c]), $sformatf("c%0d seg_head_ptr[%0d]", cyc, c));
      end
    end
    // response: valid exactly when the oldest request is sealed and fully completed
    bi = ret_seq % NB;
    exp_rv = (ret_seq < seg_seq) && (b_cmpl[bi] == b_n[bi]);
    chk(RESP_VALID === exp_rv, $sformatf("c%0d RESP_VALID %b exp %b", cyc, RESP_VALID, exp_rv));
    if (exp_rv) begin
      chk(RESP_ID === b_id[bi], $sformatf("c%0d RESP_ID %0h exp %0h", cyc, RESP_ID, b_id[bi]));
      chk(RESP_RESP === ((!b_val[bi] || b_derr[bi]) ? 2'b10 : 2'b00), $sformatf("c%0d RESP_RESP %b (val=%b derr=%b)", cyc, RESP_RESP, b_val[bi], b_derr[bi]));
      chk(RESP_LEN === 8'(b_len[bi]) && RESP_SIZE === 3'(b_size[bi]) && RESP_ADDR_LO === b_addr[bi][PAGE_LSB-1:0] &&
          RESP_BASE_CH === CH_W'(b_bch[bi]), $sformatf("c%0d RESP meta (len/size/addr_lo/base_ch)", cyc));
      for (c = 0; c < N_CH; c++)
        if (b_cnt[bi*N_CH + c] > 0) chk(RESP_HEAD_PTR[c] === SLOT_W'(b_head[bi*N_CH + c]), $sformatf("c%0d RESP_HEAD_PTR[%0d] %0d exp %0d", cyc, c, RESP_HEAD_PTR[c], b_head[bi*N_CH + c]));
    end
    if (stall_prev) chk(RESP_VALID === 1'b1 && rcat === rsaved, $sformatf("c%0d response changed while RESP_READY was low", cyc));
    accept = ar_valid && (ar_ready === 1'b1);
    retire = exp_rv && RESP_READY;

    // MC: packets fired this edge enter the completion streams
    for (c = 0; c < N_CH; c++)
      if (pkt_valid[c] === 1'b1 && pkt_ready[c] === 1'b1) begin
        s = find_live(int'(pkt_rob_idx[c]));
        if (s >= 0) b_iss[(s % NB)*N_CH + c] = b_iss[(s % NB)*N_CH + c] + 1;
        nxt = c*CQN + (cq_t[c] % CQN);
        cq_idx[nxt] = int'(pkt_rob_idx[c]); cq_due[nxt] = cyc + 1 + ($urandom % 6); cq_err[nxt] = ($urandom % 16) == 0;
        cq_t[c] = cq_t[c] + 1;
      end
    stall_prev = exp_rv && !RESP_READY;
    rsaved = rcat;
    // the completion presented this cycle
    if (cm_pick >= 0) begin
      s = find_live(int'(cmpl_rob_index));
      chk(s >= 0, $sformatf("c%0d completion for a slot with no admitted request", cyc));
      if (s >= 0) begin
        b_cmpl[s % NB] = b_cmpl[s % NB] + 1;
        if (cmpl_err) b_derr[s % NB] = 1;
      end
      cq_h[cm_pick] = cq_h[cm_pick] + 1;
    end
    if (seg_last === 1'b1) seg_seq = seg_seq + 1;
    if (retire) begin ret_seq = ret_seq + 1; n_resp = n_resp + 1; end
    if (admit_valid === 1'b1) adm_seq = adm_seq + 1;
    if (accept) begin
      bi = acc_seq % NB;
      chk(legal(ar_addr, int'(ar_len), int'(ar_size)), "TB STIMULUS ILLEGAL: request crosses a page or exceeds MAX_PKTS");
      lb = last_byte(ar_addr, int'(ar_len), int'(ar_size));
      b_id[bi] = ar_id; b_val[bi] = ar_validated; b_len[bi] = int'(ar_len); b_size[bi] = int'(ar_size);
      b_addr[bi] = ar_addr; b_cmpl[bi] = 0; b_derr[bi] = 0;
      b_bch[bi] = ref_ch(line_addr(ar_addr, int'(ar_addr[PAGE_LSB-1:PKT_OFF_W])));
      for (c = 0; c < N_CH; c++) begin b_cnt[bi*N_CH + c] = 0; b_iss[bi*N_CH + c] = 0; end
      if (ar_validated)
        for (int i = int'(ar_addr[PAGE_LSB-1:PKT_OFF_W]); i <= int'(lb[PAGE_LSB-1:PKT_OFF_W]); i++)
          b_cnt[bi*N_CH + ref_ch(line_addr(ar_addr, i))] = b_cnt[bi*N_CH + ref_ch(line_addr(ar_addr, i))] + 1;
      b_n[bi] = 0;
      for (c = 0; c < N_CH; c++) begin
        b_head[bi*N_CH + c] = chan_total[c] % N_SLOTS;
        chan_total[c] = chan_total[c] + b_cnt[bi*N_CH + c];
        b_n[bi] = b_n[bi] + b_cnt[bi*N_CH + c];
      end
      acc_seq = acc_seq + 1; pending = 0;
    end
    @(posedge aclk); #1;
  endtask

  initial begin : main
    int seed_v, g;
    logic [DEC_W*MAP_SEL_W-1:0] mf;
    n_pass = 0; n_fail = 0; n_resp = 0; done = 0;
    seed_v = SEED; void'($urandom(seed_v));
    for (int c = 0; c < N_CH; c++) begin cq_h[c] = 0; cq_t[c] = 0; chan_total[c] = 0; end
    mf = default_addr_map();
    for (int k = 0; k < CH_W; k++) begin
      ch_src[k] = flav_pos(FLAV) + k;
      mf[(PKT_OFF_W + k)*MAP_SEL_W +: MAP_SEL_W] = MAP_SEL_W'(ch_src[k]);
    end
    addr_map = mf; cfg_ncl = NCH_LOG_W'(CH_W);
    aresetn = 1; #1; aresetn = 0; #12; aresetn = 1; #3;
    g = 0;
    while (n_resp < NREQ && g < NREQ * 400) begin drive(); tick(); g++; end
    chk(n_resp == NREQ, $sformatf("flow timeout: %0d of %0d responses (deadlock?)", n_resp, NREQ));
    for (int j = 0; j < 8; j++) begin drive(); tick(); end
    // every generated request was accepted once, admitted once, sealed once and answered once
    chk(gen == NREQ && acc_seq == NREQ && adm_seq == NREQ && seg_seq == NREQ && ret_seq == NREQ,
        $sformatf("accounting: generated %0d accepted %0d admitted %0d sealed %0d retired %0d (expected %0d each)", gen, acc_seq, adm_seq, seg_seq, ret_seq, NREQ));
    chk(idle === 1'b1 && !pending, "flow idle after the last response");
    for (int c = 0; c < N_CH; c++) chk(cq_h[c] == cq_t[c], $sformatf("channel %0d: completions still outstanding", c));
    done = 1;
  end

  initial begin : wd
    #(10 * (NREQ * 500 + 200000));
    if (!done) begin n_fail++; $display("[%0t] FAIL flow watchdog: test did not finish (%0d of %0d responses)", $time, n_resp, NREQ); done = 1; end
  end
endmodule

module tb_rmc_cif_r_burst_spltr;
  int p0, f0, k0, c0, p1, f1, k1, c1, p2, f2, k2, c2, p3, f3, k3, c3, p4, f4, k4, c4;
  int fp0, ff0, fr0, fp1, ff1, fr1, fp2, ff2, fr2;
  bit d0, d1, d2, d3, d4, e0, e1, e2;
  //       ROB  Q  MAX SLOTS IDW SEED NRAND         (N_SLOTS >= MAX_PKTS, powers of two, Q <= ROB)
  rbs_chk #( 4,  4, 16,  16,  4,  1, 3500) u0 (p0, f0, k0, c0, d0);
  rbs_chk #( 8,  8, 16,  32,  8,  2, 3500) u1 (p1, f1, k1, c1, d1);
  rbs_chk #(32, 32, 64,  64,  8,  3, 3500) u2 (p2, f2, k2, c2, d2);   // RTL defaults, whole-page requests
  rbs_chk #( 8,  2,  4,   4,  2,  4, 3000) u3 (p3, f3, k3, c3, d3);   // queue much shallower than the ROB, tiny ring
  rbs_chk #(32,  8, 64,  64,  8,  5, 3500) u4 (p4, f4, k4, c4, d4);
  //         ROB MAX IDW SLOTS NREQ FLAV SEED
  flow_chk #( 8, 16,  8,  32,  500,  0,  11) fl0 (fp0, ff0, fr0, e0);
  flow_chk #(32, 64,  8,  64,  500,  2,  12) fl1 (fp1, ff1, fr1, e1);
  flow_chk #( 4,  4,  4,   4,  400,  4,  13) fl2 (fp2, ff2, fr2, e2);
  initial begin
    wait (d0 && d1 && d2 && d3 && d4 && e0 && e1 && e2);
    #20;
    $display("r_burst_spltr unit: %0d/%0d  %0d/%0d  %0d/%0d  %0d/%0d  %0d/%0d (pass/fail); packets %0d %0d %0d %0d %0d; coverage markers %0d %0d %0d %0d %0d of 18",
             p0, f0, p1, f1, p2, f2, p3, f3, p4, f4, k0, k1, k2, k3, k4, c0, c1, c2, c3, c4);
    $display("read flow (splitter + rrob): %0d/%0d (%0d resp)  %0d/%0d (%0d resp)  %0d/%0d (%0d resp)",
             fp0, ff0, fr0, fp1, ff1, fr1, fp2, ff2, fr2);
    if (f0 + f1 + f2 + f3 + f4 + ff0 + ff1 + ff2 == 0) $display("TB RESULT: PASS");
    else begin $display("TB RESULT: FAIL"); $fatal(1, "r_burst_spltr unit test failures"); end
    $finish;
  end
endmodule
