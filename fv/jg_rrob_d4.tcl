# JasperGold run for rmc_cif_rrob, config d4 (mirrors task d4_prove/d4_cover of
# fv/cif/rmc_cif_rrob/rmc_cif_rrob.sby). The .sby is SymbiYosys-only; this is the
# JasperGold equivalent of its [script] + [files] sections.
#
# NOT validated: written without access to JasperGold. First run on the remote machine
# is the syntax check (see the notes at the bottom).
#
# Run from fv/ (or anywhere; paths are resolved relative to this file):
#   /home/installs/JASPER1909/bin/jg -no_gui -proj work/jg_rrob_d4 jg_rrob_d4.tcl
#
# Properties are the unchanged ones in rmc_cif_rrob_fv.sv (no FV_LIVE, no FV_JOIN),
# same as the d4 prove/cover tasks: the neighbour assumptions m_admit_npkts, m_seg_last
# and m_cmpl stay assumes.

# fv/ directory = where this script lives
if {[catch {set fv_dir [file dirname [file normalize [info script]]]}] || $fv_dir eq ""} {
  set fv_dir [pwd]
}
set rtl_dir [file normalize $fv_dir/../rtl]

clear -all

# compile order as in the .sby: package, RTL, checker, bind
analyze -sv12 -define FORMAL \
  $rtl_dir/rtl_pkg_config/rmc_cfg_pkg.sv \
  $rtl_dir/cif/rmc_cif_rrob.sv \
  $fv_dir/cif/rmc_cif_rrob/rmc_cif_rrob_fv.sv \
  $fv_dir/cif/rmc_cif_rrob/rmc_cif_rrob_bind.sv

# d4: ROB_DEPTH=4 MAX_PKTS=4 AXI_IDW=4 N_SLOTS=4
elaborate -top rmc_cif_rrob \
  -parameter ROB_DEPTH 4 \
  -parameter MAX_PKTS  4 \
  -parameter AXI_IDW   4 \
  -parameter N_SLOTS   4

# single clock; async active-low reset (the .sby used async2sync for the same effect)
clock aclk
reset -expression {!aresetn}

# optional, not required: a bound so a hard property does not run forever
catch {set_prove_time_limit 1h}

# asserts are proven, covers are checked for reachability
prove -all

# summary of every property status
report -summary

puts "jg_rrob_d4: done"

# ---------------------------------------------------------------------------
# Checks for the first remote run (JASPER1909 syntax not verified here):
#   1. analyze/elaborate accept the options above. If not, in the jg console:
#        help analyze     help elaborate     help reset     help clock
#        help prove       help report        help set_prove_time_limit
#   2. elaborate reports ROB_DEPTH=4, MAX_PKTS=4, AXI_IDW=4, N_SLOTS=4 for the top and
#      for the bound instance rmc_cif_rrob.u_fv (check the log / design hierarchy).
#   3. The bind took effect: properties like rmc_cif_rrob.u_fv.a_ready are listed.
#   4. Run once in the GUI or with `get_property_list` before relying on the summary.
# ---------------------------------------------------------------------------
