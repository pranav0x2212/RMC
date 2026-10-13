# RMC

Reconfigurable Memory Controller for DDR1-5.

## Layout

| Path | Contents |
|------|----------|
| `rtl/cif/` | CIF block |
| `rtl/mc/` | MC core (scheduler, DFI fabric, data path) |
| `rtl/utils/` | generic parametric blocks (FIFO, SRAM, regfile, lookahead) |
| `rtl/csr_config/` | CSR / mode registers, APB target |
| `rtl/rtl_pkg_config/` | compile-time packages (DDR-gen select, structural params) |
| `tb/phy_model/` | PHY behavioral model (local, not committed) |
| `tb/mem_model/` | DRAM memory model (local, not committed) |
| `tb/config_tb/` | test knobs, sim config, register-init vectors |
| `tb/tests/` | testcases |
| `sim/` | run scripts (podman iverilog / verilator) |
| `scripts/` | helper / automation scripts |
| `filelists/` | `.f` compile lists |
| `docs/` | documentation |

## Config interface

Register block accessed over APB (target in `rtl/csr_config/`). Compile-time
structure (DDR generation, counts, widths) lives in `rtl/rtl_pkg_config/`.

## Address map (STAGE 24)

64 B packet (BL16 x 32b). Interleave (LSB->MSB): `offset | ch | rank | BG_lo(rot) |
col_lo(=P) | BG_hi | bank | col_hi | row`. Consecutive packets rotate BGs (diff-BG
CAS = tCCD_S, writes safe), then `col_lo` revisits a bank's next column = row-hit.
`P = packets/bank` (max 4 at a 4 KB burst; 2 KB -> 2, 1 KB/64 B -> 1). `P` sets the
per-bank TCAM / cas_fifo depth.

## Matched-bandwidth AXI width

`AXI_DW = N_DDR_CHANNELS * DDR_CHANNEL_WIDTH` (`rmc_cfg_pkg::AXI_DW`), so the AXI
ingress rate equals the DDR egress rate. This keeps the WD/RD SRAM and the ROB
inflow/outflow balanced (inflow == outflow, no buffer blow-up). Derived, not a free
per-instance parameter.

Caveat: bandwidth = width x frequency, so the width-only form is exact only when
`aclk == DDR data rate`. If the clocks differ, scale by the ratio:
`AXI_DW = N_CH * DDR_CHANNEL_W * (f_ddr / f_aclk)` — otherwise AXI bottlenecks and
the buffers drift. (A `CLK_RATIO` param is a TODO.)

## Simulation

Run via rootless podman (no native iverilog):

```
podman run --rm -v "$PWD":/w:Z -w /w docker.io/hdlc/iverilog:latest \
  sh -c 'iverilog -g2012 -o sim.out -f filelists/tb.f && vvp sim.out'
```

### CIF DFT observation + testbench

`rtl/cif/rmc_cif_dft.sv` is a passive tap block: CIF signals in, one packed `dft_obs`
bus out (also an output port of `rmc_cif`). Fields are packed LSB-first in the order
of the `O_<field>` localparams; total width is `rmc_cif_dft_pkg::dft_obs_w()`
(3196 bits at defaults). Taps cover AXI ingress, address map, validator, read/write
ROB (`rmc_cif_rrob`/`rmc_cif_wrob`), the read segmenter queue and per-channel walkers
(`rmc_cif_r_burst_spltr`/`rmc_cif_r_burst_spltr_ch`), the write segmenter stub (`rmc_cif_wseg`),
the MC request/completion boundary, WD/RD SRAM client ports, clock and reset.
`order_q`, `inflow_cnt` and `ent_valid` do not exist in the RTL and are not tapped.

```
sim/run_cif.sh          # verilator lint + iverilog sim of tb/tests/tb_rmc_cif.sv
sim/run_cif.sh lint     # lint only  (verilator --lint-only -Wall, warnings non-fatal)
sim/run_cif.sh sim      # sim only   (iverilog -g2012; exit != 0 on failure)
```
`sim/run_cif.sh all` is the same as running it with no argument. The sim step fails
unless the TB prints `TB RESULT: PASS`; lint warnings are reported but not fatal.

File lists: `filelists/cif_rtl.f` (RTL, package first), `filelists/cif_tb.f` (RTL + TB).
`tb_rmc_cif` reports IMPL (implemented logic, bus width/layout, observed integration
state), SEGF (pins forced from the TB; see below), TIED (tied-off outputs only), DFT (bus
slice == source net; not a functional check) and BLOCKED (RTL incomplete: read packets
to MC, RD-SRAM allocator, R response, completion direction routing and error tagging,
write segmenter, request builder, W/B paths, SRAM, programmable address map). `tb/tests/dft_layout_chk.sv` checks the bus against a plain
concatenation of the inputs at several parameter sets.

Nets in `rmc_cif.sv` marked `// undriven` (`ar_pkt_ready`, `ar_dbuf_base`, `w_pkt_ready`)
are the unfinished downstream interfaces; they float exactly as the unconnected pins did.
Unforced, an AR is admitted and queued but its packet is never consumed, so it never
seals or retires (checked as an OBSERVED state in T10).

Scope and limits:
- Only T10/T11 force pins. T10 drives real AR requests through the top (admit, rseg queue,
  walkers, rrob) and forces only the unfinished downstream stubs (`ar_pkt_ready`,
  `ar_dbuf_base`, rrob `RESP_READY`); T11 forces the write ROB's `SEG_*`/`RESP_READY`.
  Everything else drives the CIF top-level ports. A passing TB shows the taps match their
  sources and the forced sequences behave; it does not show end-to-end CIF correctness.
- Not integrated upstream: read packets to MC (req FIFOs/arbiter), RD-SRAM allocator,
  R response path, completion direction routing, write segmenter (stub, tied 0), W/B paths.
- Synthesis has not been run (no usable frontend in this environment); lint is Verilator only.
- `fv/` still references the removed `rmc_cif_rob`; the formal flow is stale and was not touched.

Keep all counts parametric (`N_RANKS`, bank/BG counts) — never hardcode.
