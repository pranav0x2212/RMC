// rmc_cif_r_burst_spltr.sv
// READ burst splitter - top. Takes one validated AXI AR per cycle, splits it into 64B line
// packets routed to the DRAM channel each line maps to. Three parts:
//   1. precompute   : on admit, works out how many packets land on every channel
//   2. shared queue : one ring entry per in-flight request, walked by N_CH engines
//   3. head pop     : when every channel has finished the oldest entry, pulse seg_last
// The N_CH per-channel engines (rmc_cif_r_burst_spltr_ch) are instanced here.
//
// TERMS
//   line  : 64B DRAM atom (one BL16 on a 32b subchannel). Byte bits [5:0] are the in-line
//           offset, fixed; every bit above is mappable.
//   page  : 4KB. An AXI INCR burst never crosses it, so only line bits [11:6] change inside
//           one request -> at most 64 lines, indexed i = 0..63 inside the page.
//   L0    : line index of the first line = ar_addr[11:6];  L_last = line index of the last.
//   base  : first line number = ar_addr[AXI_AW-1:6].
//   CH    : channel bits taken from addr_map; each can sit at any address bit >= 6 and need
//           not be adjacent. rmc_cfg_pkg::line_channel is the single decode used here AND in
//           the engines, so per-channel counts and per-line routing can never disagree.
//
// 1. PRECOMPUTE (admit cycle, combinational from the s0 AR register)
//    Bounds (12-bit math, 4KB rule): a_al = ar_addr[11:0] with low ar_size bits cleared;
//    end = a_al + (ar_len+1)*2^ar_size - 1; L0 = ar_addr[11:6]; L_last = end[11:6].
//    n_pkts = L_last - L0 + 1. An INCR burst covers a contiguous run of lines L0..L_last, so
//    the count of lines is just last - first + 1 (inclusive), 1..64.
//    Per channel: for each page line i, in_range[i] = L0<=i<=L_last and channel(i) via
//    line_channel; ch_pkts[c] = popcount of lines with in_range & channel==c. sum == n_pkts.
//    Not validated -> all ch_pkts = 0, n_pkts reported 0 (retire SLVERR, nothing to DRAM).
//    Admit = s0 valid & rob_free & queue not full. Same cycle: drive admit_* to the ROB and
//    enqueue, atomically.
//
// 2. SHARED QUEUE (ring, QUEUE_DEPTH = ROB_DEPTH, power of 2)
//    Per entry: base, ch_pkts[N_CH], rob_idx, ch_done[N_CH], head_ptr[N_CH].
//    Enqueue at alloc_ptr: write fields, ch_done[c] = (ch_pkts[c]==0) preset. Engine c reads
//    the entry at its rd_ptr[c]; it writes head_ptr[c] on its first packet and set_done[c] on
//    its last. ch_done is per-(entry,channel) state the queue owns: engine c drives ONLY bit
//    [c], so two channels finishing one request the same cycle set different bits - no
//    conflict, no arbitration. e_ch_done is the engine's read of that bit; set_done is its
//    write-enable - the loop runs through this flop, nothing is inout. entry_valid[c] =
//    rd_ptr[c] != alloc_ptr keeps an engine off the slot being enqueued, and queue-full blocks
//    admit, so enqueue and engine writes never hit the same slot.
//
// 3. HEAD POP -> seg_last
//    Oldest entry pops when queue not empty and &ch_done[pop_ptr]. The &ch_done it reads is
//    the stored bits (set by the engines' set_done or the enqueue preset); that is how a
//    channel's set_done reaches the pop. That cycle: seg_last, seg_rob_idx = entry rob_idx,
//    seg_head_ptr = entry head_ptr[N_CH]. pop_ptr++. Pop is in order, so seg_last is in
//    program order. Pop moves <=1/cycle and skip advances an engine <=1/cycle, and &ch_done
//    means every engine already passed pop_ptr, so pop never overtakes an engine's rd_ptr.
//
// RULES
//   - 4KB crossing rejected upstream (request_validator). Parked.
//   - N_SLOTS >= MAX_PKTS (64): one request can land every line on one channel; RD-SRAM slots
//     free only at retire, so smaller N_SLOTS can self-deadlock.
//   - N_SLOTS and QUEUE_DEPTH powers of 2. addr_map / cfg_n_ch_log2 static after boot.

import rmc_cfg_pkg::*;

module rmc_cif_r_burst_spltr #(
  parameter int ROB_DEPTH   = 32,          // ROB entries (rob_idx width)            [root knob]
  parameter int MAX_PKTS    = 64,          // max lines per request (4KB / 64B)      [root knob]
  parameter int AXI_AW      = SYS_ADDR_W,  // AXI byte address width                 [root knob]
  parameter int AXI_IDW     = 8,           // AXI id width                           [root knob]
  parameter int N_SLOTS     = 64,          // RD-SRAM slots per channel (>= MAX_PKTS) [root knob]
  parameter int QUEUE_DEPTH = ROB_DEPTH,   // shared queue entries, power of 2       [knob]

  // derived (localparam: never passed in)
  localparam int LINE_LSB   = PKT_OFF_W,                                // log2(64B line) = 6
  localparam int PAGE_LSB   = rmc_cfg_pkg::PAGE_LSB,                    // log2(4KB page), value from pkg
  localparam int PG_LINES   = rmc_cfg_pkg::PG_LINES,                    // lines per page (64), value from pkg
  localparam int ROB_IDX_W  = (ROB_DEPTH   > 1) ? $clog2(ROB_DEPTH)   : 1, // rob_idx width
  localparam int CNT_W      = (MAX_PKTS    > 1) ? $clog2(MAX_PKTS+1)  : 1, // packet count 0..MAX_PKTS
  localparam int SLOT_W     = (N_SLOTS     > 1) ? $clog2(N_SLOTS)     : 1, // RD-SRAM slot width
  localparam int LINE_W     = AXI_AW - LINE_LSB,                        // line number width
  localparam int QIDX_W     = (QUEUE_DEPTH > 1) ? $clog2(QUEUE_DEPTH) : 1, // queue index width
  localparam int QPTR_W     = QIDX_W + 1,                               // queue ptr = index + wrap
  localparam int DATA_BYTES = AXI_DW / 8,                               // AXI beat bytes
  localparam int SIZE_MAX   = (DATA_BYTES > 1) ? $clog2(DATA_BYTES) : 0, // max ar_size
  localparam int MAX_BEATS  = MAX_REQ_BYTES / DATA_BYTES,               // max beats per request
  localparam int LEN_W      = (MAX_BEATS  > 1) ? $clog2(MAX_BEATS)  : 1, // ar_len width
  localparam int SIZE_W     = (SIZE_MAX   > 0) ? $clog2(SIZE_MAX+1) : 1, // ar_size width
  localparam int NCH_LOG_W  = $clog2(CH_W + 1),                         // cfg_n_ch_log2 width
  localparam int LN_W       = (PG_LINES > 1) ? $clog2(PG_LINES) : 1     // line-in-page index (6)
)(
  input  logic                           aclk,              // clock
  input  logic                           aresetn,           // async reset, active low

  // config (boot-time CSR, static after init)
  input  logic [DEC_W-1:0][MAP_SEL_W-1:0] addr_map,         // per-dest source bit select
  input  logic [NCH_LOG_W-1:0]           cfg_n_ch_log2,     // active channels = 2^this (<= N_CH)

  // AR in (from request_validator)
  input  logic                           ar_valid,          // request offered
  output logic                           ar_ready,          // accepted (ROB + queue have room)
  input  logic [AXI_IDW-1:0]             ar_id,             // AXI id
  input  logic [AXI_AW-1:0]              ar_addr,           // start byte address, may be unaligned
  input  logic [LEN_W-1:0]               ar_len,            // beats - 1
  input  logic [SIZE_W-1:0]              ar_size,           // log2(bytes per beat)
  input  logic                           ar_validated,      // 0 -> SLVERR, nothing sent to DRAM

  // ROB admit (writes ROB meta SRAM #1 at rob_idx)
  input  logic                           rob_free,          // ROB has a free slot
  input  logic [ROB_IDX_W-1:0]           rob_idx,           // slot the ROB offers (its alloc_ptr)
  output logic                           admit_valid,       // admit fires this cycle (ROB allocs)
  output logic [AXI_IDW-1:0]             admit_id,          // -> RESP_ID
  output logic                           admit_validated,   // 0 -> SLVERR
  output logic [CNT_W-1:0]               admit_npkts,       // total lines; ROB completion target
  output logic [PAGE_LSB-1:0]            admit_addr_lo,     // ar_addr[11:0]: [5:0] trim, [11:6] L0
  output logic [LEN_W-1:0]               admit_len,         // ar_len  -> resp beat slicing
  output logic [SIZE_W-1:0]              admit_size,        // ar_size -> resp beat slicing
  output logic [CH_W-1:0]                admit_base_ch,     // channel of line 0 (resp CH walk)
  output logic [N_CH-1:0]                admit_ch_mask,     // channels with ch_pkts > 0

  // packets out, one stream per channel (from the engines)
  output logic [N_CH-1:0]                pkt_valid,         // packet offered on channel c
  input  logic [N_CH-1:0]                pkt_ready,         // channel c command FIFO accepts
  input  logic [N_CH-1:0]                pkt_dbuf_adr_ready,// channel c RD-SRAM has a free slot
  output logic [N_CH-1:0][ROB_IDX_W-1:0] pkt_rob_idx,       // completion tag
  output logic [N_CH-1:0][DADDR_W-1:0]   pkt_daddr,         // DRAM coords of the line
  output logic [N_CH-1:0][SLOT_W-1:0]    pkt_dbuf_adr,      // RD-SRAM slot the line returns to

  // split done -> ROB (writes ROB meta SRAM #2 at seg_rob_idx)
  output logic                           seg_last,          // oldest entry fully split (pulse)
  output logic [ROB_IDX_W-1:0]           seg_rob_idx,       // its ROB slot
  output logic [N_CH-1:0][SLOT_W-1:0]    seg_head_ptr,      // per-channel first RD-SRAM slot

  output logic                           idle               // queue empty, all engines idle
);

  // One-register / two-segment precompute pipeline feeding the admit+enqueue
  // commit. Sized for an AXI-fabric clock (~1.2 GHz): the whole precompute is
  // ~24 logic levels, so one cut leaves ~12 per segment, comfortably inside the
  // budget. Elastic valid/ready chain; a descriptor flows AR -> S1 -> commit and
  // admit fires at the commit (S1 register output).
  //   stage 1  AR capture + bounds (L0,L_last) || 64x line_channel   (~12 levels)
  //   stage 2  n_pkts + in_range + popcount + base_ch + ch_mask
  //            + admit + enqueue commit                              (~12-14 levels)
  // Bounds and the decode compute in stage 1's input cone (straight off the AR
  // inputs), so the AR-accept register already holds the precompute. Latency: +1
  // cycle vs the old single-cycle admit. Throughput: accepts a new AR every cycle
  // while it drains; a full ROB/queue stalls the chain from the commit back.
  // If STA shows stage 2 heavy, nudge base_ch or n_pkts into stage 1, or add a
  // second register - do not pre-split before the tool says so.

  // queue ring pointers (declared early; the commit needs q_full)
  logic [QPTR_W-1:0]  alloc_ptr, pop_ptr;
  wire  [QIDX_W-1:0]  alloc_idx = alloc_ptr[QIDX_W-1:0];
  wire  [QIDX_W-1:0]  pop_idx   = pop_ptr[QIDX_W-1:0];
  wire                q_empty   = (alloc_ptr == pop_ptr);
  wire                q_full    = (alloc_idx == pop_idx) & (alloc_ptr[QIDX_W] != pop_ptr[QIDX_W]);

  // pipeline valid + elastic ready chain. S1 is ready (can accept a new AR) when
  // it is empty OR it is draining into the commit this cycle. A full ROB or queue
  // freezes admit, which freezes ar_ready - the stage stalls, no data lost.
  logic s1_valid;
  logic admit;
  assign admit    = s1_valid & rob_free & ~q_full; // commit fires: ROB alloc + enqueue
  assign ar_ready = ~s1_valid | admit;             // S1 empty, or draining this cycle

  // stage 1 comb: bounds (cheap adders) || 64x line_channel (map muxes), both
  // straight off the AR inputs. The two are independent (line_channel reads
  // address bits only), so they run in parallel in this segment.
  logic [PAGE_LSB-1:0] size_mask, a_al;
  logic [15:0]         n_bytes, last_byte_adr;
  logic [LN_W-1:0]     L0, L_last;

  assign size_mask     = PAGE_LSB'((32'd1 << ar_size) - 32'd1);  // low size bits
  assign a_al          = ar_addr[PAGE_LSB-1:0] & ~size_mask;     // size-aligned start
  assign n_bytes       = (16'(ar_len) + 16'd1) << ar_size;       // beats * bytes/beat
  assign last_byte_adr = 16'(a_al) + n_bytes - 16'd1;            // last byte touched
  assign L0            = ar_addr[PAGE_LSB-1:LINE_LSB];           // first line idx [11:6]
  assign L_last        = last_byte_adr[PAGE_LSB-1:LINE_LSB];     // last line idx [11:6]
  // n_pkts computed in stage 2 (keeps the final sub off the bounds chain)

  // byte address of page line i for the incoming AR
  function automatic logic [AXI_AW-1:0] ar_line_byte(input logic [LN_W-1:0] i);
    ar_line_byte = {ar_addr[AXI_AW-1:PAGE_LSB], i, {LINE_LSB{1'b0}}};
  endfunction

  // 64 parallel map decodes: channel each page line WOULD route to (bounds not
  // applied yet - whole page decoded; stage 2 masks to the in-range lines).
  logic [PG_LINES-1:0][CH_W-1:0] line_ch_v;
  always_comb
    for (int i = 0; i < PG_LINES; i++)
      line_ch_v[i] = rmc_cfg_pkg::line_channel(ar_line_byte(LN_W'(i)), addr_map, cfg_n_ch_log2);

  // stage 1 registers (the AR-accept register; also holds the precompute)
  logic [AXI_IDW-1:0]            s1_id;
  logic [AXI_AW-1:0]             s1_addr;
  logic [LEN_W-1:0]              s1_len;
  logic [SIZE_W-1:0]             s1_size;
  logic                          s1_validated;
  logic [LN_W-1:0]               s1_L0, s1_L_last;
  logic [15:0]                   s1_last_byte;
  logic [PG_LINES-1:0][CH_W-1:0] s1_line_ch_v;

  // advance valid when S1 can take (ar_ready); payload latched only on a real beat
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn)      s1_valid <= 1'b0;
    else if (ar_ready) s1_valid <= ar_valid;     // ar_valid=0 -> bubble into S1
  end
  always_ff @(posedge aclk)
    if (ar_ready & ar_valid) begin               // hold stale data while bubbled
      s1_id        <= ar_id;
      s1_addr      <= ar_addr;
      s1_len       <= ar_len;
      s1_size      <= ar_size;
      s1_validated <= ar_validated;
      s1_L0        <= L0;
      s1_L_last    <= L_last;
      s1_last_byte <= last_byte_adr;
      s1_line_ch_v <= line_ch_v;
    end

  // stage 2 (commit) comb from S1: n_pkts, in_range, full popcount, base_ch,
  // ch_pkts_eff, ch_mask, admit and the enqueue write. rob_free / rob_idx are
  // sampled here (the cycle the descriptor commits). admit itself does not depend
  // on the counts, only the enqueued data does.
  logic [PG_LINES-1:0]        in_range;
  logic [N_CH-1:0][CNT_W-1:0] ch_pkts;
  logic [CNT_W-1:0]           n_pkts;

  assign n_pkts = CNT_W'(s1_L_last) - CNT_W'(s1_L0) + CNT_W'(1); // lines in the run

  // which page lines fall inside this request's run [L0..L_last]
  always_comb
    for (int i = 0; i < PG_LINES; i++)
      in_range[i] = (LN_W'(i) >= s1_L0) & (LN_W'(i) <= s1_L_last);

  // popcount: tally the in-range lines per channel
  always_comb begin
    for (int c = 0; c < N_CH; c++) ch_pkts[c] = '0;
    for (int i = 0; i < PG_LINES; i++)
      if (in_range[i]) ch_pkts[s1_line_ch_v[i]] = ch_pkts[s1_line_ch_v[i]] + CNT_W'(1);
  end

  // invalid request (SLVERR): force counts to 0 so nothing is issued to DRAM
  logic [N_CH-1:0][CNT_W-1:0] commit_ch_pkts_eff;
  always_comb
    for (int c = 0; c < N_CH; c++)
      commit_ch_pkts_eff[c] = s1_validated ? ch_pkts[c] : '0;

  wire [CH_W-1:0]   commit_base_ch   = s1_line_ch_v[s1_L0];        // channel of L0
  wire [LINE_W-1:0] commit_base_line = s1_addr[AXI_AW-1:LINE_LSB]; // line# of L0
  logic [N_CH-1:0]  commit_ch_mask;
  always_comb                                       // channels this request uses
    for (int c = 0; c < N_CH; c++) commit_ch_mask[c] = (commit_ch_pkts_eff[c] != '0);

  // admit outputs (ROB meta SRAM #1)
  assign admit_valid     = admit;
  assign admit_id        = s1_id;
  assign admit_validated = s1_validated;
  assign admit_npkts     = s1_validated ? n_pkts : '0;
  assign admit_addr_lo   = s1_addr[PAGE_LSB-1:0];
  assign admit_len       = s1_len;
  assign admit_size      = s1_size;
  assign admit_base_ch   = commit_base_ch;
  assign admit_ch_mask   = commit_ch_mask;

  // SHARED QUEUE storage
  logic [LINE_W-1:0]           q_base    [QUEUE_DEPTH];
  logic [N_CH-1:0][CNT_W-1:0]  q_ch_pkts [QUEUE_DEPTH];
  logic [ROB_IDX_W-1:0]        q_rob     [QUEUE_DEPTH];
  logic [N_CH-1:0]             q_done    [QUEUE_DEPTH];
  logic [N_CH-1:0][SLOT_W-1:0] q_head    [QUEUE_DEPTH];

  // ENGINES: one per channel, each walking the queue with its own rd_ptr
  logic [N_CH-1:0][QPTR_W-1:0] eng_rd;
  logic [N_CH-1:0][QIDX_W-1:0] eng_ri;
  logic [N_CH-1:0]             eng_set_done, eng_head_we, eng_idle;
  logic [N_CH-1:0][SLOT_W-1:0] eng_head;

  generate
    for (genvar c = 0; c < N_CH; c++) begin : g_eng
      assign eng_ri[c] = eng_rd[c][QIDX_W-1:0];   // slot index = low bits of the engine's pointer
      rmc_cif_r_burst_spltr_ch #(
        .ROB_DEPTH (ROB_DEPTH), .MAX_PKTS (MAX_PKTS), .AXI_AW (AXI_AW),
        .N_SLOTS (N_SLOTS), .QUEUE_DEPTH (QUEUE_DEPTH)
      ) u_ch (
        .aclk, .aresetn,
        .ch_id              (CH_W'(c)),
        .addr_map           (addr_map),
        .cfg_n_ch_log2      (cfg_n_ch_log2),
        .rd_ptr             (eng_rd[c]),
        .entry_valid        (eng_rd[c] != alloc_ptr),
        .e_base             (q_base[eng_ri[c]]),
        .e_ch_pkts          (q_ch_pkts[eng_ri[c]][c]),
        .e_rob_idx          (q_rob[eng_ri[c]]),
        .e_ch_done          (q_done[eng_ri[c]][c]),
        .set_done           (eng_set_done[c]),
        .head_ptr_we        (eng_head_we[c]),
        .head_ptr           (eng_head[c]),
        .pkt_valid          (pkt_valid[c]),
        .pkt_ready          (pkt_ready[c]),
        .pkt_dbuf_adr_ready (pkt_dbuf_adr_ready[c]),
        .pkt_rob_idx        (pkt_rob_idx[c]),
        .pkt_daddr          (pkt_daddr[c]),
        .pkt_dbuf_adr       (pkt_dbuf_adr[c]),
        .idle               (eng_idle[c])
      );
    end
  endgenerate

  // HEAD POP: oldest entry retires when every channel is done with it
  wire all_done = &q_done[pop_idx];
  wire pop      = ~q_empty & all_done;
  assign seg_last     = pop;
  assign seg_rob_idx  = q_rob[pop_idx];
  assign seg_head_ptr = q_head[pop_idx];

  assign idle = (&eng_idle) & q_empty & ~s1_valid;

  // QUEUE updates: enqueue (admit), per-channel head/done writes (engines), pop.
  // admit and engine writes never target the same slot (entry_valid + q_full gates); the
  // engine writes are guarded against the alloc slot anyway so the intent is explicit.
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      alloc_ptr <= '0;
      pop_ptr   <= '0;
    end else begin
      if (admit) begin
        q_base[alloc_idx] <= commit_base_line;
        q_rob[alloc_idx]  <= rob_idx;
        for (int c = 0; c < N_CH; c++) begin
          q_ch_pkts[alloc_idx][c] <= commit_ch_pkts_eff[c];
          q_done[alloc_idx][c]    <= (commit_ch_pkts_eff[c] == '0); // preset done where nothing to do
        end
        alloc_ptr <= alloc_ptr + 1'b1;
      end

      for (int c = 0; c < N_CH; c++) begin
        if (eng_head_we[c] & ~(admit & (eng_ri[c] == alloc_idx)))
          q_head[eng_ri[c]][c] <= eng_head[c];
        if (eng_set_done[c] & ~(admit & (eng_ri[c] == alloc_idx)))
          q_done[eng_ri[c]][c] <= 1'b1;
      end

      if (pop) pop_ptr <= pop_ptr + 1'b1;
    end
  end

`ifndef SYNTHESIS
  // sum of per-channel counts must equal the total packet count on every admit
  logic [CNT_W+CH_W-1:0] cnt_sum;
  always_comb begin
    cnt_sum = '0;
    for (int c = 0; c < N_CH; c++) cnt_sum = cnt_sum + (CNT_W+CH_W)'(commit_ch_pkts_eff[c]);
  end
  always_ff @(posedge aclk)
    if (aresetn & admit) begin
      if (s1_last_byte[15:PAGE_LSB] != '0)
        $error("r_burst_spltr: AR crosses 4KB (addr=%h len=%0d size=%0d)", s1_addr, s1_len, s1_size);
      if (s1_validated & (cnt_sum != (CNT_W+CH_W)'(n_pkts)))
        $error("r_burst_spltr: sum(ch_pkts)=%0d != n_pkts=%0d", cnt_sum, n_pkts);
    end
`endif

endmodule : rmc_cif_r_burst_spltr

// THEORY OF OPERATION   (read together with rmc_cif_r_burst_spltr_ch.sv)
//
// ONE LINE
//   One AXI read request -> a run of 64B line packets, each routed to the DRAM
//   channel its address maps to, issued per channel in ascending address order,
//   with a single "all channels finished" pulse (seg_last) back to the ROB.
//
// WHY THIS SHAPE
//   A request is a contiguous run of lines L0..L_last inside one 4KB page
//   (L0 = ell-zero = the FIRST line = ar_addr[11:6]; L_last = the last line;
//   not the number ten, and the count L_last-L0+1 varies per request). The
//   address map scatters those lines across channels in an arbitrary pattern
//   (channel = line_channel(addr)). We do not know, cheaply, "how many lines go
//   to channel c" without looking at every line - so PRECOMPUTE counts them once
//   (popcount), the QUEUE parks that per-channel count, and each ENGINE drains
//   its own count at its own rate. Decoupling the engines is what lets a slow
//   channel back-pressure without stalling the others.
//
// DATA / CONTROL FLOW
//
//   AR (validated) --ar_valid/ar_ready-->  [ bounds || 64x line_channel ]
//                                             |
//            +=== S1 (register) ================================================+
//            |  bounds:   a_al, last_byte_adr  ->  L0, L_last                   |
//            |         ||  per line: 64x line_channel(i) -> line_ch_v[i]        |
//            +=== stage 2 (commit) =============================================+
//            |  n_pkts = L_last - L0 + 1 ;  in_range[i] = L0<=i<=L_last         |
//            |  popcount: ch_pkts[c] = #lines{in_range & channel==c}           |
//            |  base_ch = line_ch_v[L0] ;  ch_pkts_eff = validated ? ch_pkts:0 |
//            |  ch_mask ;  admit = s1_valid & rob_free & ~q_full               |
//            +----------------+---------------------------------+---------------+
//                             |                                 |
//                             v                                 v
//              admit_* -> ROB meta#1                 ENQUEUE queue[alloc_idx]
//              {id, validated, npkts,                {base, ch_pkts[N_CH], rob_idx,
//               addr_lo, len, size,                   done[c]=(ch_pkts[c]==0),
//               base_ch, ch_mask}                     head[N_CH]}   ; alloc_ptr++
//                                                          |
//                                                          v
//                     +-------------------- SHARED QUEUE (ring) -------------------+
//                     |  per entry: base, ch_pkts[N_CH], rob_idx, done[N_CH],      |
//                     |             head[N_CH]        (QUEUE_DEPTH = ROB_DEPTH)     |
//                     +----+---------------------------------------------+--------+
//                      rd_ptr[0]                                   rd_ptr[N_CH-1]
//                          |                                             |
//                     ENGINE ch0   ...                            ENGINE ch(N-1)
//                    skip / issue / pause                       skip / issue / pause
//                          |  pkt_* stream -> cmd FIFO                 |  pkt_* stream
//                          |  head_we[c], set_done[c]                 |
//                          +----------------->  queue[rd_ptr].{head,done}[c]  <----+
//                                                          |
//                                                          v
//                                  HEAD POP: ~q_empty & &done[pop_idx]
//                                                          |  pop_ptr++
//                                                          v
//                           seg_last + seg_rob_idx + seg_head_ptr -> ROB meta#2
//
//   Two writers touch queue[].done[c]: the enqueue preset (whole word, new slot)
//   and engine c (bit c only, an already-live slot). entry_valid (rd_ptr!=alloc)
//   plus q_full keep those off the same slot, so there is no arbitration.
//
// WORKED EXAMPLE A  -  default map, fine interleave (channel = addr[6] = line LSB)
//   Config: N_CH=2, CH_W=1, cfg_n_ch_log2=1. AXI_DW=128b => 16 B/beat, full size.
//   Request: ar_addr[11:0]=0x080 (aligned), ar_size=4 (16 B), ar_len=19 (20 beats).
//
//   bounds:  a_al=0x080;  n_bytes=20*16=320;  last_byte_adr=0x080+320-1=0x1BF
//            L0     = 0x080[11:6] = 2
//            L_last = 0x1BF[11:6] = 6
//            n_pkts = 6-2+1 = 5          lines touched: 2,3,4,5,6
//
//   channel(i)=addr[6]=i[0]:   2->0  3->1  4->0  5->1  6->0
//   ch_pkts = { ch0:3 (lines 2,4,6), ch1:2 (lines 3,5) }   sum=5 = n_pkts  OK
//   base_ch = channel(2) = 0    ch_mask = 2'b11
//
//   admit_* : npkts=5, addr_lo=0x080, len=19, size=4, base_ch=0, mask=11
//   enqueue : base=<line#>, ch_pkts={3,2}, done={0,0}, head=?
//
//   ENGINE ch0 (e_ch_pkts=3): mine = even lines >= 2 = {2,4,6,...}. ffs picks
//     the lowest each fire; mine_reg clears the issued bit:
//        fire0: line 2  dbuf_ptr 0 -> head[0]=0 (first), m=0
//        fire1: line 4  dbuf_ptr 1,               m=1
//        fire2: line 6  dbuf_ptr 2, set_done[0]   m=2 == e_ch_pkts-1 (last)
//   ENGINE ch1 (e_ch_pkts=2): mine = odd lines >= 2 = {3,5,...}
//        fire0: line 3  dbuf_ptr 0 -> head[1]=0,  m=0
//        fire1: line 5  dbuf_ptr 1, set_done[1]   m=1 (last)
//   (each engine owns its channel's RD-SRAM, so both head_ptr start at 0.)
//
//   HEAD POP: once done={1,1}, seg_last, seg_head_ptr={0,0}, pop_ptr++.
//
//   Resp rebuild (rmc_cif_r_resp) walks 2..6 and pulls slots:
//        line 2 ch0 slot head[0]+0 = 0
//        line 3 ch1 slot head[1]+0 = 0
//        line 4 ch0 slot head[0]+1 = 1
//        line 5 ch1 slot head[1]+1 = 1
//        line 6 ch0 slot head[0]+2 = 2     -> address order restored.
//
// WORKED EXAMPLE B  -  scattered map, coarse interleave (channel = addr[9])
//   Same request. channel(i) = addr[9] = bit 3 of the line index. Lines 2..6 all
//   have line-index bit 3 = 0, so EVERY line maps to channel 0:
//       ch_pkts = { ch0:5, ch1:0 }     base_ch=0    ch_mask=2'b01
//   ch1 is preset done at enqueue (ch_pkts[1]==0) and its engine SKIPs the entry
//   at 1/cycle. ch0's engine issues all 5 lines into contiguous slots 0..4.
//   Nothing special-cased: the same line_channel + popcount + ffs handle both the
//   fine (A) and coarse (B) interleave. That generality is the whole point of the
//   popcount/window design over a fixed arithmetic stride.
//
// PIPELINING NOTES  (as built: a 1-register, 2-segment precompute pipeline)
//   The old single-cycle admit was ALL combinational from the AR register:
//       addr -> bounds -> 64x line_channel -> popcount tree -> ch_pkts
//            -> base_ch, ch_mask, admit_*  AND  the enqueue write
//   That is ~24 logic levels (bounds chain ~12, in_range+popcount ~10, commit
//   ~2). On an AXI-fabric clock (~1.2 GHz, ~20-30 levels of budget per stage) it
//   sat right at the edge, so one cut is enough: stage 1 takes the bounds and the
//   decode, stage 2 (the commit) takes the popcount and the handshake.
//       stage 1  bounds  ||  64x line_channel          (independent, ~12 levels)
//       stage 2  n_pkts + in_range + popcount + base_ch + ch_mask + admit
//                + enqueue                               (~12-14 levels)
//   line_channel reads address bits only, so it is independent of the bounds math
//   and shares stage 1 with it. Both compute in stage 1's input cone (off the AR
//   inputs), so the AR-accept register already holds the precompute. n_pkts' final
//   subtract is done in stage 2 to keep it off the bounds chain.
//   Control / timing facts:
//     - whole descriptor flows with a valid bit; admit never fires before the
//       counts exist (the popcount is in the commit segment).
//     - elastic valid/ready: S1 is ready when empty or draining; a full ROB/queue
//       freezes admit and ar_ready. Accepts a new AR every cycle while draining.
//     - rob_free / rob_idx are sampled at the commit (the cycle the descriptor
//       commits and the ROB alloc lines up).
//     - the sum-check and 4KB-cross asserts sit at the commit, on s1_* / n_pkts.
//   If STA ever flags stage 2, first nudge base_ch or n_pkts into stage 1, then
//   add a second register, then (last) split the popcount adder tree into group
//   subtotals. Do not pre-split before the tool asks - at 1.2 GHz this is plenty.
//   The engines (see ch file) have their own, separate critical path - the
//   precompute pipeline does not touch them.
