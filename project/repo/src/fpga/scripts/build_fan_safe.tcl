set store D:/vivado/2025.2/Vivado/data/XilinxTclStore
foreach d [glob -directory $store/support -type d *] {lappend auto_path $d}
foreach provider [glob -directory $store/tclapp -type d *] {foreach d [glob -nocomplain -directory $provider -type d *] {lappend auto_path $d}}
source $store/support/appinit/appinit.tcl
set out D:/Temp/ps_board_20261005/fan_safe
file mkdir $out
open_checkpoint D:/psight/preboard_20261003/build/pl_direct/penumbrasight_wrapper_routed.dcp
# AXU9EGB: J10 -> NPN Q2 -> N-MOS Q1. Low turns Q2 off,
# allowing R29 to pull Q1 gate high: full-speed cooling, independent of PS.
create_port -direction OUT fan
create_cell -reference OBUF fan_obuf
create_cell -reference GND fan_ground
create_net fan_enable_low
create_net fan_output
connect_net -net fan_enable_low -objects [get_pins {fan_ground/G fan_obuf/I}]
connect_net -net fan_output -objects [get_pins fan_obuf/O]
connect_net -net fan_output -objects [get_ports fan]
set_property PACKAGE_PIN J10 [get_ports fan]
set_property IOSTANDARD LVCMOS33 [get_ports fan]
set_property DRIVE 8 [get_ports fan]
set site [get_sites -of_objects [get_package_pins J10]]
if {[llength $site] != 1} {error "Cannot resolve J10 I/O site: $site"}
place_cell fan_obuf $site
route_design -preserve
report_drc -file $out/drc.rpt
report_timing_summary -file $out/timing.rpt
foreach kind {max min} {
    set paths [get_timing_paths -delay_type $kind -max_paths 1]
    if {![llength $paths] || [get_property SLACK $paths] < 0} {error "Timing failed: $kind"}
}
set errors [get_drc_violations -filter {SEVERITY == Error}]
if {[llength $errors]} {error "DRC errors: $errors"}
write_checkpoint -force $out/penumbrasight_fan_safe.dcp
write_verilog -force $out/penumbrasight_fan_safe_netlist.v
write_bitstream -force $out/penumbrasight_fan_safe.bit
puts FAN_SAFE_BUILD_PASS
