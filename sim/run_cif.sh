#!/usr/bin/env bash
# CIF flow: Verilator lint + Icarus sim of tb_rmc_cif. Usage: run_cif.sh [all|lint|sim]
# Exits nonzero on lint errors, compile errors or TB failure.
set -u
cd "$(dirname "$0")/.."
MODE="${1:-all}"       # all (default) | lint | sim
OUT=sim/cif_tb.out      # compiled simulation (gitignored)
LOG=sim/cif_tb.log      # run log, parsed for the TB result (gitignored)
rc=0                    # sticky exit status; lint/sim failures set it, compile error exits at once

if [ "$MODE" = all ] || [ "$MODE" = lint ]; then
  echo "== verilator lint (rmc_cif) =="
  # Existing RTL is a stub with pre-existing warnings: they are reported, not fatal.
  # Real errors (syntax, width, elaboration) still return nonzero.
  verilator --lint-only -Wall -Wno-fatal --top-module rmc_cif -f filelists/cif_rtl.f || rc=1
fi

if [ "$MODE" = all ] || [ "$MODE" = sim ]; then
  # Compile with the TB as top; a compile/elaboration error stops here.
  echo "== iverilog compile =="
  iverilog -g2012 -s tb_rmc_cif -o "$OUT" -f filelists/cif_tb.f || exit 1
  echo "== vvp run =="
  # -n: finish on $finish/$fatal without an interactive prompt.
  vvp -n "$OUT" | tee "$LOG"
  vrc=${PIPESTATUS[0]}   # vvp's own status (the TB $fatal's on failure)
  # Fail unless the TB printed PASS: a run that dies early prints neither line.
  if [ "$vrc" -ne 0 ] || ! grep -q "^TB RESULT: PASS" "$LOG" || grep -q "^TB RESULT: FAIL" "$LOG"; then
    echo "SIMULATION FAILED"
    rc=1
  fi
fi
exit $rc
