# JasperGold run for rmc_cif_r_burst_spltr, config d4 (mirrors task d4_prove/d4_cover of
# fv/cif/rmc_cif_r_burst_spltr/rmc_cif_r_burst_spltr.sby). The .sby is SymbiYosys-only; this
# is the JasperGold equivalent of its [script] + [files] sections.
#
# NOT validated: written from a source-only audit, without access to JasperGold. Syntax is
# reused from fv/jg_rrob_d4.tcl (known to work on JASPER1909). First run on the remote
# machine is the syntax / elaboration check (see the notes at the bottom).
#
# Run from fv/ (or anywhere; paths are resolved relative to this file):
#   /home/installs/JASPER1909/bin/jg -no_gui -proj work/jg_rspltr_d4 jg_rspltr_d4.tcl
#
# Properties are the unchanged ones in rmc_cif_r_burst_spltr_fv.sv (safety only; pkt_ready
# and pkt_dbuf_adr_ready are free, no fairness assumption, no liveness).

# fv/ directory = where this script lives. jg may not set [info script] (empty), then fall
# back to the working directory: the repo root (has fv/) or fv/ itself.
set script_path ""
catch {set script_path [info script]}
if {$script_path ne ""} {
  set fv_dir [file dirname [file normalize $script_path]]
} elseif {[file isdirectory [pwd]/fv]} {
  set fv_dir [file normalize [pwd]/fv]
} else {
  set fv_dir [pwd]
}
set rtl_dir [file normalize $fv_dir/../rtl]

clear -all

# compile order as in the .sby: package, addrmap, engine, top, checker, bind.
# The .sby passes -D FORMAL, but no splitter source tests FORMAL, so it is omitted:
# JASPER1909 analyze has no -define switch (ESW087).
analyze -sv12 \
  $rtl_dir/rtl_pkg_config/rmc_cfg_pkg.sv \
  $rtl_dir/cif/rmc_cif_addrmap.sv \
  $rtl_dir/cif/rmc_cif_r_burst_spltr_ch.sv \
  $rtl_dir/cif/rmc_cif_r_burst_spltr.sv \
  $fv_dir/cif/rmc_cif_r_burst_spltr/rmc_cif_r_burst_spltr_fv.sv \
  $fv_dir/cif/rmc_cif_r_burst_spltr/rmc_cif_r_burst_spltr_bind.sv

# d4: ROB_DEPTH=4 QUEUE_DEPTH=4 MAX_PKTS=16 N_SLOTS=16 AXI_IDW=4
# AXI_AW is left at its default (SYS_ADDR_W = 48), as in the .sby.
elaborate -top rmc_cif_r_burst_spltr \
  -parameter ROB_DEPTH   4 \
  -parameter QUEUE_DEPTH 4 \
  -parameter MAX_PKTS    16 \
  -parameter N_SLOTS     16 \
  -parameter AXI_IDW     4

# single clock; async active-low reset (the .sby used async2sync for the same effect)
clock aclk
reset -expression {!aresetn}

# optional, not required: a bound so a hard property does not run forever
catch {set_prove_time_limit 1h}

# asserts are proven, covers are checked for reachability
prove -all

# summary of every property status
report -summary

puts "jg_rspltr_d4: done"

# ---------------------------------------------------------------------------
# Known risks for the first remote run (none worked around here, on purpose):
#   1. Upward / generate-scoped hierarchical references. The checker reaches into the DUT
#      by module name (rmc_cif_r_burst_spltr.pkt_valid, .eng_rd, .q_base, ...) and through
#      the generate scope (rmc_cif_r_burst_spltr.g_eng[c].u_ch.sel_i / active / dbuf_ptr /
#      m / mine_reg). Unknown whether JASPER1909 resolves these from a bound instance.
#   2. Function-result part-selects. exp_dec(...)[PKT_OFF_W +: CH_W] (w_chan and a_base_ch)
#      selects bits of a function call result, which standard SystemVerilog does not
#      allow; slang accepts it, analyze may not.
#   3. fv_e. `always_ff @(posedge aclk) fv_e <= fv_e;` has no reset and no initial value
#      and must act as a free, constant, solver-chosen queue slot. Unconfirmed whether JG
#      keeps it free or fixes / folds it, which changes what the a_pkt_*, a_h_* and
#      a_w_* properties prove.
#   4. Other unconfirmed behaviour: -parameter QUEUE_DEPTH override (its RTL default is
#      ROB_DEPTH), `reset -expression` after the first cycle (same open item as RROB),
#      and $error in the RTL `ifndef SYNTHESIS blocks.
#
# Checks for the first run:
#   1. analyze/elaborate accept the options above; if not, in the jg console:
#        help analyze     help elaborate     help reset     help clock
#        help prove       help report        help set_prove_time_limit
#   2. elaborate reports ROB_DEPTH=4, QUEUE_DEPTH=4, MAX_PKTS=16, N_SLOTS=16, AXI_IDW=4
#      for the top and for the bound instance rmc_cif_r_burst_spltr.u_fv.
#   3. The bind took effect: properties like rmc_cif_r_burst_spltr.u_fv.a_alloc are listed.
#   4. Expected counts from the source reading (not a run): about 276 assertions and 10
#      covers. A different count points at a configuration mismatch.
# ---------------------------------------------------------------------------
