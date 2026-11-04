# ============================================================================
# 探针：把块设计的全部告警原文抓下来并留档
#
# 用途：BD 41-3281（AXI 接口参数未传播核对）、BD 41-237（AWUSER/ARUSER）、
#       BD 41-2384（ARLOCK）这几类告警在 `validate_bd_design` 时只在控制台出现一次，
#       之后命中缓存就不再重放。要判读它们，必须有**原文**而不是印象。
#
# 做法：打开既有 bd_project，`validate_bd_design -force` 强制重跑，
#       同时导出地址映射与每个从接口 / m_axi 的连接状态。
#       **不跑 OOC 综合、不跑实现** —— 那两个每个子运行要 3~4 GB，
#       本机可用内存只有 2.6 GB，会 OOM（见 src/fpga/scripts/build_pl.tcl 的 JOBS 说明）。
#
# 运行：
#   vivado -mode batch -source probe_bd_warnings.tcl -nojournal
# 告警原文在 stdout 与 .log 里；结构快照写在 build/reports/bd_probe/。
# ============================================================================

# 仓库根：默认按脚本位置推（src/fpga/scripts 上三级）。
# 允许用 PS_REPO_ROOT 覆盖 —— 因为 Vivado 的构建产物（build/bd_project）通常只存在于
# ASCII 构建区，而脚本本体在源码区（中文路径）。Vitis/Vivado 读 Tcl 的路径敏感度不同，
# 显式给根目录比猜位置可靠。
set REPO_ROOT [file normalize [file join [file dirname [info script]] .. .. ..]]
if {[info exists ::env(PS_REPO_ROOT)] && $::env(PS_REPO_ROOT) ne ""} {
    set REPO_ROOT [file normalize $::env(PS_REPO_ROOT)]
}
puts "PROBE_REPO_ROOT $REPO_ROOT"
set BD_PROJ   [file normalize [file join $REPO_ROOT build bd_project bd_project.xpr]]
set OUT_DIR   [file normalize [file join $REPO_ROOT build reports bd_probe]]
file mkdir $OUT_DIR

if {![file exists $BD_PROJ]} {
    puts "ERROR: 找不到块设计工程 $BD_PROJ"
    exit 1
}
puts "PROBE_BD_OPEN $BD_PROJ"
open_project $BD_PROJ

set bd_files [get_files -quiet *.bd]
if {[llength $bd_files] == 0} { puts "ERROR: 工程里没有 .bd"; exit 1 }
puts "PROBE_BD_FILES $bd_files"
open_bd_design [lindex $bd_files 0]

# ---------------------------------------------------------------- 结构快照
set snap [open [file join $OUT_DIR bd_structure.txt] w]

puts $snap "=== BD cells ==="
foreach c [get_bd_cells] { puts $snap $c }

puts $snap ""
puts $snap "=== 每个 cell 的接口引脚 -> 连接到的网络 ==="
set n_dangling 0
foreach c [get_bd_cells] {
    foreach p [get_bd_intf_pins -quiet $c/*] {
        set mode "?"
        catch { set mode [get_property MODE $p] }
        set netnames {}
        foreach n [get_bd_intf_nets -quiet -of_objects $p] { lappend netnames $n }
        if {[llength $netnames] == 0} { incr n_dangling }
        puts $snap [format "IF %-52s MODE=%-7s nets=%s" $p $mode [expr {[llength $netnames] ? $netnames : {<悬空>}}]]
    }
}
puts $snap ""
puts $snap "DANGLING_INTERFACE_COUNT $n_dangling"

puts $snap ""
puts $snap "=== 标量引脚里未连接的（只列输入类，输出悬空属正常）==="
foreach c [get_bd_cells] {
    foreach p [get_bd_pins -quiet $c/*] {
        set dir "?"
        catch { set dir [get_property DIR $p] }
        if {$dir ne "I"} { continue }
        if {[get_property CONFIG.POLARITY $p -quiet] ne ""} { continue }
        if {[llength [get_bd_nets -quiet -of_objects $p]] == 0} {
            set d "?"
            catch { set d [get_property DIR $p] }
            if {$d eq "I"} { puts $snap "PIN_UNCONNECTED $p" }
        }
    }
}

puts $snap ""
puts $snap "=== 地址段（offset / range）==="
foreach s [get_bd_addr_segs -quiet] {
    set off "?"; set rng "?"
    catch { set off [get_property OFFSET $s] }
    catch { set rng [get_property RANGE $s] }
    puts $snap [format "SEG %-72s offset=%-12s range=%s" $s $off $rng]
}
close $snap

# ---------------------------------------------------------------- 地址映射报告
catch { report_bd_addr -file [file join $OUT_DIR bd_addr.rpt] }

# ---------------------------------------------------------------- 强制重跑校验
# -force 很关键：不强制时若设计已校验过，工具不重放告警，
# 于是「没看到告警」会被误读成「没有告警」。
puts "PROBE_BD_VALIDATE_BEGIN"
set rc [catch {validate_bd_design -force} err]
puts "PROBE_BD_VALIDATE rc=$rc"
if {$rc} { puts "PROBE_BD_VALIDATE_MSG $err" }
puts "PROBE_BD_VALIDATE_END"

# ---------------------------------------------------------------- 校验后的错误/严重告警计数
foreach sev {ERROR CRITICAL WARNING} {
    set n "?"
    catch { set n [get_msg_config -severity $sev -count] }
    puts "PROBE_BD_MSGCOUNT $sev = $n"
}

puts "PROBE_BD_DONE"
close_project
