set ps_mhz 100
if {[info exists ::env(PS_CLOCK_MHZ)]} {set ps_mhz $::env(PS_CLOCK_MHZ)}
set expected_period [expr {1000.0/$ps_mhz}]
# Independent final-checkpoint audit, including clock and fan binding.
set_param general.maxThreads 2
set store D:/vivado/2025.2/Vivado/data/XilinxTclStore
foreach d [glob -directory $store/support -type d *] {lappend auto_path $d}
foreach provider [glob -directory $store/tclapp -type d *] {foreach d [glob -nocomplain -directory $provider -type d *] {lappend auto_path $d}}
source $store/support/appinit/appinit.tcl
open_checkpoint bin/full_board_routed.dcp
set fp [open reports/routed_audit.json w]
set setup [get_timing_paths -delay_type max -max_paths 1 -slack_lesser_than 100000]
set hold [get_timing_paths -delay_type min -max_paths 1 -slack_lesser_than 100000]
if {![llength $setup] || ![llength $hold]} {error "No timing paths available"}
set wns [get_property SLACK $setup]
set whs [get_property SLACK $hold]
set bb [llength [get_cells -quiet -hier -filter {IS_BLACKBOX == 1}]]
set fan [get_ports fan]
set fan_pin [get_property PACKAGE_PIN $fan]
set fan_std [get_property IOSTANDARD $fan]
set clocks [get_clocks -quiet]
set foundclock 0
foreach clk $clocks {if {abs([get_property PERIOD $clk]-$expected_period)<0.001} {set foundclock 1}}
set obuf [get_cells -quiet -of_objects [get_pins -of_objects [get_nets -of_objects $fan]] -filter {REF_NAME == OBUF}]
if {[llength $obuf]!=1} {error "Fan OBUF missing"}
set inputnet [get_nets -segments -of_objects [get_pins -of_objects $obuf -filter {REF_PIN_NAME == I}]]
set drivers [get_cells -quiet -of_objects [get_pins -leaf -of_objects $inputnet -filter {DIRECTION == OUT}]]
set fan_low 0
foreach cell $drivers {if {[get_property REF_NAME $cell] eq "GND"} {set fan_low 1}}
report_drc -file reports/routed_audit_drc.rpt
set drc_errors [llength [get_drc_violations -quiet -filter {SEVERITY == Error}]]
report_route_status -file reports/route_status.rpt
report_clocks -file reports/clocks.rpt
puts $fp "\{\"wns_ns\":$wns,\"whs_ns\":$whs,\"black_boxes\":$bb,\"expected_clock_found\":$foundclock,\"fan_pin\":\"$fan_pin\",\"fan_iostandard\":\"$fan_std\",\"fan_constant_low\":$fan_low,\"drc_errors\":$drc_errors\}"
close $fp
if {$wns<0 || $whs<0 || $bb!=0 || !$foundclock || !$fan_low || $fan_pin ne "J10" || $fan_std ne "LVCMOS33" || $drc_errors!=0} {error "Routed audit failed"}
report_clock_interaction -file reports/clock_interaction.rpt
report_cdc -file reports/cdc.rpt
puts ROUTED_AUDIT_PASS
