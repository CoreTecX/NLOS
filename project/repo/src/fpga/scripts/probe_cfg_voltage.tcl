# ============================================================================
# 探针：CFGBVS / CONFIG_VOLTAGE 在目标器件上是否受支持
#
# 用途（换器件、换板卡时可直接复用）：
#   这两条属性是 7 系列工程的常见套路，很容易被照抄到 UltraScale+ 上。
#   本脚本用最小代价问清三件事：
#     1. 属性是否存在于该器件的 design 对象上；
#     2. 直接 set_property 是成功、只警告、还是抛错（决定能不能写进 Tcl 脚本）；
#     3. 走 read_xdc 是否会让约束读入失败（决定它会不会卡住 impl_1）。
#   第 3 条最关键：XDC 是被 read_xdc 读入的，若它返回错误，实现会在读约束时中止。
#
# 为什么不用真实设计：一个单 LUT 的最小网表就足以拿到 current_design，
#   成本约 1 分钟、内存约 1 GB。**不要拿真实工程来试这种事**。
#
# 运行（一次只跑一个 Vivado 进程；本机可用内存偏紧，见 build_pl.tcl 的 JOBS 说明）：
#   vivado -mode batch -source probe_cfg_voltage.tcl -nojournal
#
# 2026-09-29 在 xczu9eg-ffvb1156-2-i 上的实测结论：
#   两个属性都不存在（Vivado 12-5460 attribute not supported）；
#   read_xdc 只报警告、rc=0，不会中止实现；但 Tcl 里直接 set_property 返回 rc=1。
#   => 写进 XDC 无害但完全空转，故本项目**不写**。详见 sim/results/cfg_voltage_probe.md。
# ============================================================================

# 目标器件可通过 argv 覆盖：vivado -mode batch -source <本脚本> -tclargs <part>
set PART xczu9eg-ffvb1156-2-i
if {[llength $argv] > 0} { set PART [lindex $argv 0] }

set OUT_DIR [file normalize [file join [file dirname [info script]] probe_cfg_rpt]]
file mkdir $OUT_DIR

puts "PROBE_BEGIN part=$PART"

create_project -in_memory -part $PART
puts "PROBE_PROJECT_PART = [get_property PART [current_project]]"

# ---- 最小网表，只为让 current_design 存在（不涉及本项目任何源码）----
set VFILE [file join $OUT_DIR ps_cfg_probe.v]
set fh [open $VFILE w]
puts $fh "module ps_cfg_probe (input a, output b);"
puts $fh "  assign b = a;"
puts $fh "endmodule"
close $fh
add_files $VFILE
synth_design -top ps_cfg_probe -part $PART -mode out_of_context -quiet
puts "PROBE_SYNTH_OK"

# ---- 1 属性是否存在于该器件 ----
set all_props [list_property [current_design]]
foreach p {CFGBVS CONFIG_VOLTAGE} {
    if {[lsearch -exact $all_props $p] >= 0} {
        puts "PROBE_PROPERTY_LISTED $p = YES"
    } else {
        puts "PROBE_PROPERTY_LISTED $p = NO"
    }
}

# ---- 2 直接 set_property：rc=0 可写，rc=1 抛错 ----
# 注意：取值必须显式配对。写成 `foreach pair {A 1 B 2}` 会把每个元素当成一个 pair，
# lassign 拿到空值，于是报出 `Cannot set property ''` 这种**脚本自身造成的假错误**。
foreach p {CONFIG_VOLTAGE CFGBVS} {
    if {$p eq "CONFIG_VOLTAGE"} { set v 1.8 } else { set v GND }
    set rc [catch {set_property $p $v [current_design]} err]
    puts "PROBE_DIRECT $p $v rc=$rc"
    puts "PROBE_DIRECT_MSG $err"
}

# ---- 3 走 read_xdc（与真实 XDC 同样的用法）----
set XFILE [file join $OUT_DIR tiny_cfg.xdc]
set fh [open $XFILE w]
puts $fh "set_property CONFIG_VOLTAGE 1.8 \[current_design\]"
puts $fh "set_property CFGBVS GND \[current_design\]"
close $fh
set rc [catch {read_xdc $XFILE} err]
puts "PROBE_READ_XDC rc=$rc"
puts "PROBE_READ_XDC_MSG $err"

# ---- 4 对照组：一个在该器件上合法的位流属性，证明探针有判别力 ----
set rc2 [catch {set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]} err2]
puts "PROBE_CONTROL COMPRESS rc=$rc2 readback=[get_property BITSTREAM.GENERAL.COMPRESS [current_design]]"

puts "PROBE_DONE"
close_project
