# Bounded P5/P6 candidates; default remains the 100 MHz fallback.
set period 10
set axi_burst 64
set axi_outstanding 8
set axi_width 128
foreach {key var} {PS_HLS_PERIOD period PS_AXI_BURST axi_burst PS_AXI_OUTSTANDING axi_outstanding PS_AXI_WIDTH axi_width} {
    if {[info exists ::env($key)]} {set $var $::env($key)}
}
if {$period <= 0 || $axi_burst ni {32 64 128} || $axi_outstanding ni {4 8 16} || $axi_width ni {128 256}} {error "Invalid P5/P6 HLS parameters"}
set cflags "-std=c++14 -ffp-contract=off -DPS_HLS_CSIM -DPS_AXI_BURST=$axi_burst -DPS_AXI_OUTSTANDING=$axi_outstanding"
open_project -reset full_gemv_hls
set_top full_gemv
add_files src/full_gemv.cpp -cflags $cflags
add_files -tb src/reference_gemv.cpp -cflags "-std=c++14 -ffp-contract=off"
add_files -tb tests/gemv_tb.cpp -cflags "-std=c++14 -ffp-contract=off"
open_solution -reset solution1 -flow_target vivado
set_part {xczu9eg-ffvb1156-2-i}
create_clock -period $period -name default
config_interface -m_axi_addr64 -m_axi_max_widen_bitwidth $axi_width
# Unsafe math is disabled by default; do not enable reassociation or FMA.
csim_design
csynth_design
export_design -format ip_catalog -vendor psight -library hls -version 2.0
# The standard release runs scripts/run_rtl_lite.ps1 on this identical RTL.
# Optional vendor UVM cosimulation is substantially more memory/time intensive.
if {[info exists ::env(PS_RUN_UVM_COSIM)] && $::env(PS_RUN_UVM_COSIM) eq "1"} {
    cosim_design -rtl verilog -tool xsim -trace_level none
}
exit
