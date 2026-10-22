# Overnight JasperGold run for rmc_cif_r_burst_spltr, config d4. Same design, checker,
# assumptions and `prove -all` as fv/jg_rspltr_d4.tcl (copied command-for-command, not
# sourced: the original runs prove -all itself). Differences are listed at the bottom.
#
# Launch (fresh session; new project dir and log per run, nothing existing is overwritten):
#   cd <repo>/fv
#   stamp=$(date +%Y%m%d_%H%M%S); mkdir -p work/logs
#   nohup /home/installs/JASPER1909/bin/jg -no_gui -proj work/jg_rspltr_overnight_$stamp \
#     jg_rspltr_overnight.tcl < /dev/null > work/logs/rspltr_overnight_$stamp.log 2>&1 &
#
# A finished log is NOT a pass: read the "report -summary" table and the banner at the end.

proc ts {} { if {[catch {exec date "+%Y-%m-%d %H:%M:%S"} t]} {return "?"}; return $t }
puts "jg_rspltr_overnight: start [ts]"

# fv/ directory = where this script lives (same resolution as the original)
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

analyze -sv12 \
  $rtl_dir/rtl_pkg_config/rmc_cfg_pkg.sv \
  $rtl_dir/cif/rmc_cif_addrmap.sv \
  $rtl_dir/cif/rmc_cif_r_burst_spltr_ch.sv \
  $rtl_dir/cif/rmc_cif_r_burst_spltr.sv \
  $fv_dir/cif/rmc_cif_r_burst_spltr/rmc_cif_r_burst_spltr_fv.sv \
  $fv_dir/cif/rmc_cif_r_burst_spltr/rmc_cif_r_burst_spltr_bind.sv

# d4: ROB_DEPTH=4 QUEUE_DEPTH=4 MAX_PKTS=16 N_SLOTS=16 AXI_IDW=4
elaborate -top rmc_cif_r_burst_spltr \
  -parameter ROB_DEPTH   4 \
  -parameter QUEUE_DEPTH 4 \
  -parameter MAX_PKTS    16 \
  -parameter N_SLOTS     16 \
  -parameter AXI_IDW     4

clock aclk
reset -expression {!aresetn}

# Record the installed tool's own description of the commands used below in the log, so
# the exact semantics (limit units, per-property vs total, prove options) are on file.
foreach h {{help set_prove_time_limit} {help prove} {help report}} {
  puts "---- $h"
  catch {eval $h} hmsg
  puts $hmsg
}

# Time limit: the original sets `1h` here (this is the only limit in the original script).
# Same command and same `h` suffix, 8h. Unlike the original the result is not swallowed:
# if it fails the log says so, because the tool would then run with its own default limit.
set lim_rc [catch {set_prove_time_limit 8h} lim_msg]
if {$lim_rc} {
  puts "WARNING: set_prove_time_limit 8h FAILED: $lim_msg"
  puts "WARNING: the run uses the tool's DEFAULT limit, not 8h. Check the help text above."
} else {
  puts "set_prove_time_limit 8h accepted: $lim_msg"
}

puts "jg_rspltr_overnight: prove -all start [ts]"
set prove_rc [catch {prove -all} prove_msg]
puts "jg_rspltr_overnight: prove -all end   [ts]"
if {$prove_rc} { puts "ERROR: prove -all raised: $prove_msg" }

report -summary

puts "jg_rspltr_overnight: finished [ts]  prove_rc=$prove_rc  time_limit_rc=$lim_rc"
puts "jg_rspltr_overnight: this is NOT a pass indicator. Only the summary table says what"
puts "jg_rspltr_overnight: was proven; undetermined / cex / error counts are in that table."

# ---------------------------------------------------------------------------
# Changes vs fv/jg_rspltr_d4.tcl: 1h -> 8h (error now reported); prove -all wrapped in catch
# (same call, error logged); `help` output and timestamps added; the closing "done" line and
# the notes are replaced by the banner above.
# Not done, because no command is confirmed by the existing scripts: per-property status
# counts and timeout detection in Tcl. The only status command in the repo is
# `report -summary`; `get_property_list` appears only in a comment (jg_rrob_d4.tcl:70).
# Whether 8h is a total or per-property limit, and what the tool does at the limit, is not
# stated anywhere in the repo; the `help set_prove_time_limit` output in the log is the source.
# ---------------------------------------------------------------------------
