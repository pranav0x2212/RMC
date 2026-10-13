#!/usr/bin/env bash
# Standalone read-path unit testbenches (iverilog). Usage: run_unit.sh [all|rrob|rseg_ch|rseg]
# Exits nonzero on a compile error, a failing TB, or a missing "TB RESULT: PASS".
set -u
cd "$(dirname "$0")/.."
WHICH="${1:-all}"
case "$WHICH" in
  all|rrob|rseg_ch|rseg) ;;
  *) echo "Usage: $0 [all|rrob|rseg_ch|rseg]" >&2; exit 2 ;;
esac
rc=0

run_one() {   # name, top module, filelist
  local name=$1 top=$2 fl=$3
  echo "== $name: iverilog compile =="
  iverilog -g2012 -s "$top" -o "sim/unit_$name.out" -f "$fl" || { rc=1; return; }
  echo "== $name: vvp run =="
  vvp -n "sim/unit_$name.out" | tee "sim/unit_$name.log"
  local vrc=${PIPESTATUS[0]}
  if [ "$vrc" -ne 0 ] || ! grep -q "^TB RESULT: PASS" "sim/unit_$name.log"; then
    echo "$name: FAILED"; rc=1
  else
    echo "$name: PASSED"
  fi
}

case "$WHICH" in all|rrob)    run_one rrob    tb_rmc_cif_rrob           filelists/rrob_tb.f ;; esac
case "$WHICH" in all|rseg_ch) run_one rseg_ch tb_rmc_cif_r_burst_spltr_ch filelists/rseg_ch_tb.f ;; esac
case "$WHICH" in all|rseg)    run_one rseg    tb_rmc_cif_r_burst_spltr  filelists/rseg_tb.f ;; esac
exit $rc
