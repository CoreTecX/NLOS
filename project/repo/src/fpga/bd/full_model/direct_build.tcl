# In-process Vivado flow avoids Windows Script Host process-launch restrictions.
set_param general.maxThreads 2
set store D:/vivado/2025.2/Vivado/data/XilinxTclStore
foreach d [glob -directory $store/support -type d *] {lappend auto_path $d}
foreach provider [glob -directory $store/tclapp -type d *] {foreach d [glob -nocomplain -directory $provider -type d *] {lappend auto_path $d}}
source $store/support/appinit/appinit.tcl
open_project build/vivado_final/full_board.xpr
set_property generate_synth_checkpoint false [get_files full_board.bd]
generate_target all [get_files full_board.bd]
update_compile_order -fileset sources_1
synth_design -top full_board_wrapper -part xczu9eg-ffvb1156-2-i -flatten_hierarchy rebuilt
if {[llength [get_cells -hier -filter {IS_BLACKBOX == 1}]]} {error "Unresolved synthesis black boxes"}
write_checkpoint -force bin/full_board_synth.dcp
report_utilization -file reports/synthesis_utilization.rpt
opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force bin/full_board_routed.dcp
report_utilization -file reports/implementation_utilization.rpt
report_timing_summary -delay_type min_max -report_unconstrained -file reports/implementation_timing.rpt
report_drc -file reports/implementation_drc.rpt
check_timing -verbose -file reports/check_timing.rpt
foreach kind {max min} {
    set paths [get_timing_paths -delay_type $kind -max_paths 1 -slack_lesser_than 100000]
    if {![llength $paths] || [get_property SLACK $paths]<0} {error "Timing failed: $kind"}
}
set errors [get_drc_violations -filter {SEVERITY == Error}]
if {[llength $errors]} {error "DRC errors: $errors"}
write_bitstream -force bin/full_board_fan_safe.bit
# In-process implementation writes the bitstream separately. The project run
# database has no completed impl_1, so export the structural XSA without -include_bit.
write_hw_platform -fixed -force -file bin/full_board.xsa
puts FULL_BOARD_BUILD_PASS
