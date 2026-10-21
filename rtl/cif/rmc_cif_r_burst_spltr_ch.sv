// rmc_cif_r_burst_spltr_ch.sv
// READ burst splitter - one channel engine. Instanced N_CH times by rmc_cif_r_burst_spltr,
// one per DRAM channel (ch_id).
//
// Walks the shared queue at its own pace with its own rd_ptr. For every entry it either
// skips it (nothing for this channel) or issues exactly e_ch_pkts packets: the lines of
// that request that map to this channel, in ascending address order. It owns this channel's
// RD-SRAM allocation pointer, so one request's slots are a contiguous ring run.
//
// ch_done is NOT held here - it lives in the queue (the top owns the flops). This engine
// READS it as e_ch_done (skip vs issue) and asks the top to SET it via the set_done pulse
// on the last packet. The top does queue[rd_ptr].ch_done[ch_id] <= 1; next reader (head-pop
// or a future pass) sees it. So the loop e_ch_done <- set_done runs through the queue flop.
//
// PER ENTRY (entry at rd_ptr)
//   entry_valid = 0   : queue caught up, wait
//   e_ch_done   = 1   : SKIP, rd_ptr++ this cycle, never gated by pkt_ready/dbuf (head-pop
//                       safety relies on skip advancing at 1/cycle unconditionally)
//   e_ch_done   = 0   : ISSUE e_ch_pkts packets (>= 1), then set_done + rd_ptr++
//
// LINE SELECT (which lines are mine)
//   L0 = e_base[5:0] (line index in the 4KB page); page base = e_base[LINE_W-1:6].
//   For each page line i in 0..63: byte address = {page base, i, 6'b0}; its channel comes
//   from rmc_cfg_pkg::line_channel (the exact decode precompute uses - same function, so the
//   count and the mask can never disagree). mine[i] = (i >= L0) & (channel(i) == ch_id).
//   Lines inside the request are the lowest lines >= L0, so the first e_ch_pkts set bits of
//   mine are exactly this channel's lines of this request; L_last is not needed.
//   The selected line's byte address goes through rmc_cif_addrmap for pkt_daddr.
//
// ISSUE / HANDSHAKE
//   owes      = entry_valid & ~e_ch_done
//   pkt_valid = owes & pkt_dbuf_adr_ready      (valid never depends on pkt_ready)
//   fire      = pkt_valid & pkt_ready
//   on fire   : dbuf_ptr++ (ring), clear the issued mine bit, m++
//   first fire (m==0)            : head_ptr_we, head_ptr = this slot (dbuf_ptr)
//   last  fire (m==e_ch_pkts-1)  : set_done, rd_ptr++ and m<=0 same cycle (no bubble)
//   e_ch_pkts==1                 : first and last are the same fire
//   dbuf/FIFO not ready mid-request -> pause, resume on the same entry (mine held)

import rmc_cfg_pkg::*;

module rmc_cif_r_burst_spltr_ch #(
  parameter int ROB_DEPTH   = 32,          // ROB entries (rob_idx width)            [root knob]
  parameter int MAX_PKTS    = 64,          // max lines per request                  [root knob]
  parameter int AXI_AW      = SYS_ADDR_W,  // AXI byte address width                 [root knob]
  parameter int N_SLOTS     = 64,          // RD-SRAM slots per channel (>= MAX_PKTS) [root knob]
  parameter int QUEUE_DEPTH = ROB_DEPTH,   // shared queue entries, power of 2       [knob]

  // derived (localparam: never passed in)
  localparam int LINE_LSB   = PKT_OFF_W,                                // log2(64B line) = 6
  localparam int PAGE_LSB   = rmc_cfg_pkg::PAGE_LSB,                    // log2(4KB page), value from pkg
  localparam int PG_LINES   = rmc_cfg_pkg::PG_LINES,                    // lines per page (64), value from pkg
  localparam int ROB_IDX_W  = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1, // rob_idx width
  localparam int CNT_W      = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1, // packet count width
  localparam int SLOT_W     = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1, // RD-SRAM slot width
  localparam int LINE_W     = AXI_AW - LINE_LSB,                        // line number width
  localparam int QIDX_W     = (QUEUE_DEPTH > 1) ? $clog2(QUEUE_DEPTH) : 1, // queue index width
  localparam int QPTR_W     = QIDX_W + 1,                               // queue ptr = index + wrap
  localparam int NCH_LOG_W  = $clog2(CH_W + 1),                         // cfg_n_ch_log2 width
  localparam int LN_W       = (PG_LINES > 1) ? $clog2(PG_LINES) : 1     // line-in-page index (6)
)(
  input  logic                           aclk,              // clock
  input  logic                           aresetn,           // async reset, active low

  input  logic [CH_W-1:0]                ch_id,             // channel this engine serves

  // config (boot-time CSR, same values as the top)
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,         // CH bit positions + line -> daddr
  input  logic [NCH_LOG_W-1:0]           cfg_n_ch_log2,     // active channels = 2^this

  // shared queue, entry at this engine's rd_ptr
  output logic [QPTR_W-1:0]              rd_ptr,            // this engine's ring read pointer
  input  logic                           entry_valid,       // rd_ptr != alloc_ptr
  input  logic [LINE_W-1:0]              e_base,            // first line number of the request
  input  logic [CNT_W-1:0]               e_ch_pkts,         // packets for this channel (>=1 if owed)
  input  logic [ROB_IDX_W-1:0]           e_rob_idx,         // request's ROB slot
  input  logic                           e_ch_done,         // entry's ch_done[ch_id] (queue read)
  output logic                           set_done,          // set ch_done[ch_id] at rd_ptr (last fire)
  output logic                           head_ptr_we,       // write head_ptr[ch_id] at rd_ptr (first fire)
  output logic [SLOT_W-1:0]              head_ptr,          // first RD-SRAM slot of this request

  // packet out -> this channel's command FIFO
  output logic                           pkt_valid,         // packet offered
  input  logic                           pkt_ready,         // command FIFO accepts
  input  logic                           pkt_dbuf_adr_ready,// RD-SRAM has a free slot
  output logic [ROB_IDX_W-1:0]           pkt_rob_idx,       // completion tag
  output logic [DADDR_W-1:0]             pkt_daddr,         // DRAM coords of the line
  output logic [SLOT_W-1:0]              pkt_dbuf_adr,      // RD-SRAM slot (alloc pointer)

  output logic                           idle               // not holding an unfinished entry
);

  // STATE
  logic                  active;     // mid-entry: mine_reg holds the remaining lines
  logic [CNT_W-1:0]      m;          // packets already issued for the current entry
  logic [PG_LINES-1:0]   mine_reg;   // remaining lines for this channel (cleared as they fire)
  logic [SLOT_W-1:0]     dbuf_ptr;   // this channel's RD-SRAM ring alloc pointer
  logic [QPTR_W-1:0]     rd_ptr_q;

  assign rd_ptr = rd_ptr_q;

  // Lowest set bit index of a 64-line mask (scan high->low so the lowest index wins).
  function automatic logic [LN_W-1:0] ffs(input logic [PG_LINES-1:0] v);
    ffs = '0;
    for (int j = PG_LINES-1; j >= 0; j--) if (v[j]) ffs = LN_W'(j);
  endfunction

  // Byte address of page line i for this request: page base from e_base, i in [11:6], 0 low.
  function automatic logic [AXI_AW-1:0] line_byte(input logic [LN_W-1:0] i);
    line_byte = {e_base[LINE_W-1:LN_W], i, {LINE_LSB{1'b0}}};
  endfunction

  // mine[] for the entry currently at rd_ptr (combinational). Used directly on the first
  // packet (before active) and then latched + cleared in mine_reg.
  logic [5:0]          L0;
  logic [PG_LINES-1:0] mine_comb, mine_cur;
  assign L0 = e_base[LN_W-1:0];
  always_comb
    for (int i = 0; i < PG_LINES; i++)
      mine_comb[i] = (LN_W'(i) >= L0)
                   & (rmc_cfg_pkg::line_channel(line_byte(LN_W'(i)), addr_map, cfg_n_ch_log2) == ch_id);
  assign mine_cur = active ? mine_reg : mine_comb;

  logic [LN_W-1:0] sel_i;
  assign sel_i = ffs(mine_cur);

  // issue control
  logic            owes, fire, first, last, skip;
  logic [CNT_W-1:0] m_eff;
  assign owes      = entry_valid & ~e_ch_done;
  assign first     = ~active;                       // m == 0
  assign m_eff     = active ? m : '0;
  assign pkt_valid = owes & pkt_dbuf_adr_ready;
  assign fire      = pkt_valid & pkt_ready;
  assign last      = owes & (m_eff == e_ch_pkts - CNT_W'(1));
  assign skip      = entry_valid & e_ch_done & ~active;

  // selected line -> DRAM coords (same decode table as line_channel, so routing and
  // addressing stay consistent)
  logic [CH_W-1:0]      daddr_ch_unused;
  logic [PKT_OFF_W-1:0] daddr_off_unused;
  rmc_cif_addrmap #(.AXI_AW(AXI_AW), .DADDR_W(DADDR_W)) u_map (
    .sys_addr ({e_base[LINE_W-1:LN_W], sel_i, {LINE_LSB{1'b0}}}),
    .addr_map (addr_map),
    .daddr    (pkt_daddr),
    .ch       (daddr_ch_unused),
    .offset   (daddr_off_unused)
  );

  // outputs
  assign head_ptr_we = fire & first;
  assign head_ptr    = dbuf_ptr;
  assign set_done    = fire & last;
  assign pkt_rob_idx = e_rob_idx;
  assign pkt_dbuf_adr = dbuf_ptr;
  assign idle        = ~active & ~entry_valid;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      rd_ptr_q <= '0;
      active   <= 1'b0;
      m        <= '0;
      dbuf_ptr <= '0;
      mine_reg <= '0;
    end else begin
      if (fire)
        dbuf_ptr <= (dbuf_ptr == SLOT_W'(N_SLOTS-1)) ? '0 : dbuf_ptr + 1'b1;

      if (skip) begin
        rd_ptr_q <= rd_ptr_q + 1'b1;         // nothing for us: step over at 1/cycle
      end else if (fire) begin
        if (last) begin
          active   <= 1'b0;                  // request done; load next entry next cycle
          m        <= '0;
          rd_ptr_q <= rd_ptr_q + 1'b1;
        end else begin
          active   <= 1'b1;                  // more packets for this request
          m        <= m + 1'b1;
          mine_reg <= mine_cur & ~(PG_LINES'(1) << sel_i);
        end
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge aclk)
    if (aresetn) begin
      if (owes & (e_ch_pkts == '0))
        $error("r_burst_spltr_ch: owed an entry with ch_pkts == 0 (precompute should preset done)");
      if (fire & (mine_cur == '0))
        $error("r_burst_spltr_ch: mine[] empty but still owing packets (count/map mismatch)");
    end
`endif

endmodule : rmc_cif_r_burst_spltr_ch

// THEORY OF OPERATION   (one channel engine; read with rmc_cif_r_burst_spltr.sv)
//
// ONE LINE
//   Walk the shared queue; for each entry, either SKIP it (no lines for me) or
//   ISSUE exactly e_ch_pkts packets - this channel's lines of that request, in
//   ascending address order - into my own contiguous RD-SRAM slot run.
//
// THE THREE PER-ENTRY OUTCOMES (entry = queue slot at rd_ptr)
//   entry_valid==0 : queue caught up to alloc_ptr. Wait, do nothing.
//   e_ch_done==1   : SKIP. rd_ptr++ this cycle, UNCONDITIONALLY (never gated by
//                    pkt_ready/dbuf). Head-pop safety relies on skip advancing at
//                    exactly 1/cycle so pop can never overtake a live rd_ptr.
//   e_ch_done==0   : ISSUE e_ch_pkts packets (>=1). On the last one, set_done +
//                    rd_ptr++. If a packet can't fire (FIFO or RD-SRAM not ready)
//                    the engine PAUSES on the same entry (mine_reg holds the rest).
//
// LINE SELECT  (which of the 64 page lines are mine)
//   mine_comb[i] = (i >= L0) & (line_channel(line_byte(i)) == ch_id)
//     - i >= L0 drops lines before the request start (L0 = e_base[5:0]).
//     - line_channel is the SAME decode the top used to count, so "how many are
//       mine" (e_ch_pkts) and "which are mine" (mine[]) can never disagree.
//     - L_last is NOT needed: the request's lines are the lowest lines >= L0, so
//       the first e_ch_pkts set bits of mine are exactly this request's lines for
//       this channel. We stop after e_ch_pkts fires (m == e_ch_pkts-1 = last).
//   ffs(mine_cur) returns the lowest set bit -> sel_i, the next line to issue.
//   mine_reg = mine_cur with sel_i cleared, so the next ffs yields the next line.
//
// ISSUE / HANDSHAKE  (per fire)
//   owes      = entry_valid & ~e_ch_done
//   pkt_valid = owes & pkt_dbuf_adr_ready        (valid NEVER waits on pkt_ready)
//   fire      = pkt_valid & pkt_ready
//   first fire (m==0)           : head_ptr_we, head_ptr = dbuf_ptr (run base)
//   last  fire (m==e_ch_pkts-1) : set_done, rd_ptr++ , m<=0  (no idle bubble)
//   every fire                  : dbuf_ptr++ (ring over N_SLOTS), clear mine bit
//   e_ch_pkts==1                : first and last are the same fire
//
// CYCLE-BY-CYCLE  (Example A, ch0 from the top file: e_ch_pkts=3, lines 2,4,6)
//   assume pkt_ready=1, pkt_dbuf_adr_ready=1 throughout, dbuf_ptr starts 0.
//
//   cyc | active m  mine_cur(set bits)  sel_i fire first last | dbuf head_we done
//   ----+------------------------------------------------------+-----------------
//    T0 |   0    0   {2,4,6,...}           2    1    1    0     |  0   head[0]=0
//    T1 |   1    1   {4,6,...}             4    1    0    0     |  1
//    T2 |   1    2   {6,...}               6    1    0    1     |  2         set_done
//    T3 |  (loads next entry at the new rd_ptr; active=0, m=0)
//
//   At T0 mine_comb is used directly (active=0). At T1/T2 mine_reg (latched &
//   cleared) feeds ffs. set_done at T2 rides to queue[rd_ptr].done[ch0]; head[0]=0
//   was captured at T0. A pause (e.g. pkt_ready=0 at T1) simply repeats T1 with the
//   same sel_i until it fires - mine_reg and m are unchanged while paused.
//
// WHERE ch_done LIVES  (the loop that is easy to misread)
//   ch_done is NOT a register in this engine. It is queue[rd_ptr].done[ch_id] in
//   the top. This engine READS it (e_ch_done) and WRITES it via the set_done pulse;
//   the top does the flop update. So "set it, then later read it done" closes
//   through the queue flop, not through a combinational loop here.
//
// PIPELINING NOTES  (the engine's own critical path - separate from precompute)
//   Per-cycle combinational chain while issuing:
//       e_base -> 64x line_channel -> mine_comb        (64 parallel map decodes)
//              -> ffs (64->6 priority encode) -> sel_i
//              -> line_byte(sel_i) -> rmc_cif_addrmap  -> pkt_daddr
//   mine_comb is recomputed EVERY cycle today even though, within one entry, it
//   only loses one bit per fire. Two independent ways to shorten it:
//     1. Compute mine[] ONCE per entry (on load) and keep only mine_reg; drop
//        mine_comb from the steady-state path. First packet then costs one extra
//        cycle unless you precompute mine[] for the next entry during the current
//        one (the engine already knows rd_ptr+1's fields are stable once enqueued).
//     2. Register sel_i / the addrmap output: make pkt_daddr a registered output
//        one cycle behind the ffs, so line_channel+ffs and the addrmap decode sit
//        in different stages. pkt_valid/dbuf handshake must then pipeline with it.
//   Note the skip path MUST stay single-cycle and unconditional (head-pop safety);
//   pipeline the ISSUE datapath, not the rd_ptr advance rate for skips.
