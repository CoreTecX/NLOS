set ps_mhz 100
if {[info exists ::env(PS_CLOCK_MHZ)]} {set ps_mhz $::env(PS_CLOCK_MHZ)}
if {$ps_mhz ni {100 125 150 200}} {error "Unscreened PL clock frequency"}
set_param general.maxThreads 2
set store D:/vivado/2025.2/Vivado/data/XilinxTclStore
foreach d [glob -directory $store/support -type d *] {lappend auto_path $d}
foreach provider [glob -directory $store/tclapp -type d *] {foreach d [glob -nocomplain -directory $provider -type d *] {lappend auto_path $d}}
source $store/support/appinit/appinit.tcl
set root [pwd]
create_project full_board build/vivado_final -part xczu9eg-ffvb1156-2-i -force
set_property ip_repo_paths [list $root/full_gemv_hls/solution1/impl/ip] [current_project]
update_ip_catalog
create_bd_design full_board
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e ps
set ps_cell [get_bd_cells ps]
source scripts/ps_preset_axu9eg_alinx.tcl
set_property -dict [list CONFIG.PSU__USE__M_AXI_GP0 {1} CONFIG.PSU__USE__M_AXI_GP1 {0} CONFIG.PSU__USE__M_AXI_GP2 {0} CONFIG.PSU__MAXIGP0__DATA_WIDTH {32} CONFIG.PSU__FPGA_PL0_ENABLE {1} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__SRCSEL {IOPLL} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ $ps_mhz] $ps_cell
create_bd_cell -type ip -vlnv psight:hls:full_gemv:2.0 core
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect control
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] [get_bd_cells control]
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect memory
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1}] [get_bd_cells memory]
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset reset
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant zero
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {1}] [get_bd_cells zero]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant one
set_property CONFIG.CONST_VAL {1} [get_bd_cells one]
create_bd_port -dir O fan
connect_bd_net [get_bd_pins zero/dout] [get_bd_ports fan] [get_bd_pins reset/mb_debug_sys_rst]
connect_bd_net [get_bd_pins one/dout] [get_bd_pins reset/dcm_locked]
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins reset/ext_reset_in] [get_bd_pins reset/aux_reset_in]
connect_bd_net [get_bd_pins ps/pl_clk0] [get_bd_pins reset/slowest_sync_clk] [get_bd_pins core/ap_clk] [get_bd_pins control/aclk] [get_bd_pins memory/aclk]
foreach p [get_bd_pins -quiet ps/*aclk] {connect_bd_net [get_bd_pins ps/pl_clk0] $p}
connect_bd_net [get_bd_pins reset/peripheral_aresetn] [get_bd_pins core/ap_rst_n] [get_bd_pins control/aresetn] [get_bd_pins memory/aresetn]
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins control/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins control/M00_AXI] [get_bd_intf_pins core/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins core/m_axi_mat] [get_bd_intf_pins memory/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins core/m_axi_vec] [get_bd_intf_pins memory/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins memory/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP0_FPD]
if {[llength [get_bd_pins -quiet ps/pl_ps_irq0]]} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant irqzero
    set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {8}] [get_bd_cells irqzero]
    connect_bd_net [get_bd_pins irqzero/dout] [get_bd_pins ps/pl_ps_irq0]
}
assign_bd_address -offset 0xA1000000 -range 0x10000 -target_address_space [get_bd_addr_spaces ps/Data] [get_bd_addr_segs core/s_axi_control/Reg]
foreach sp [get_bd_addr_spaces core/Data*] {
    foreach seg [get_bd_addr_segs -quiet ps/*] {if {[string match *HP0_DDR_* $seg]} {assign_bd_address -target_address_space $sp $seg}}
}
foreach p [get_bd_intf_pins core/*] {if {[llength [get_bd_intf_nets -of_objects $p]]==0} {error "Unconnected kernel interface $p"}}
validate_bd_design
foreach pin {ext_reset_in aux_reset_in} {
    if {[get_bd_nets -of_objects [get_bd_pins reset/$pin]] ne [get_bd_nets -of_objects [get_bd_pins ps/pl_resetn0]]} {error "Reset input wiring mismatch: $pin"}
}
if {[get_property CONFIG.C_EXT_RESET_HIGH [get_bd_cells reset]] != 0 || [get_property CONFIG.C_AUX_RESET_HIGH [get_bd_cells reset]] != 0} {error "PS reset must drive active-low reset inputs"}
set actual [get_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__ACT_FREQMHZ $ps_cell]
set src [get_property CONFIG.PSU__CRL_APB__PL0_REF_CTRL__SRCSEL $ps_cell]
if {abs($actual-$ps_mhz)>0.01 || $src ne "IOPLL"} {error "Requested and physical clock mismatch"}
set fp [open reports/hardware_clock.json w]
puts $fp "\{\"requested_mhz\":$ps_mhz,\"actual_mhz\":$actual,\"source\":\"$src\",\"hp0_width\":128,\"single_clock_domain\":true\}"
close $fp
save_bd_design
generate_target all [get_files full_board.bd]
make_wrapper -files [get_files full_board.bd] -top
add_files -norecurse build/vivado_final/full_board.gen/sources_1/bd/full_board/hdl/full_board_wrapper.v
add_files -fileset constrs_1 scripts/full_board.xdc
set_property top full_board_wrapper [current_fileset]
update_compile_order -fileset sources_1
close_project
source scripts/direct_build.tcl
