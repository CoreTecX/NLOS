# ============================================================================
# PenumbraSight PL 仅综合 · src/fpga/scripts/synth_pl_only.tcl
#
# 用途：从块设计一路跑到 **synth_design 结束** 就停 —— 不跑实现、不写位流。
#       用于「改完 RTL/BD 之后先看资源与 WNS」这一步，比 build_pl.tcl 便宜得多。
#
# 用法（在**纯 ASCII 构建区**的仓库根执行；先跑过 penumbrasight_bd.tcl）
# ----------------------------------------------------------------------------
#     vivado -mode batch -nojournal -nolog -source src/fpga/scripts/synth_pl_only.tcl
#
# 与 build_pl.tcl 的关系
# ----------------------------------------------------------------------------
#   build_pl.tcl   = 块设计 wrapper -> 约束 -> 综合 -> **实现 -> 位流** + 五份报告
#   本脚本         = 同上，但**到综合为止**（后三步不做）
#   两者共用的前置（加 wrapper、挂 XDC、设 top、数 CRITICAL WARNING）写法一致。
#
# ⚠ 本脚本**无条件** reset_run synth_1。理由：块设计一改（哪怕只加一根线），
#   旧的 synth_1 结果就作废，而它的 PROGRESS 仍是 100% —— build_pl.tcl 里那条
#   「PROGRESS 不是 100% 才 reset」在这里会**漏掉**这种情况，于是 launch 会直接
#   沿用旧结果、报出旧资源数。宁可贵一点，也不要读到过期数字。
#
# 产出
# ----------------------------------------------------------------------------
#   build/pl/report_utilization_synth.rpt   综合后资源（整体）
#   build/pl/report_utilization_synth_hier.rpt  综合后资源（层级，看各单元）
#   build/pl/report_timing_synth.rpt        综合后时序（WNS/WHS）
# ============================================================================

set REPO_ROOT [file normalize [pwd]]
set BD_PROJ   [file normalize [file join $REPO_ROOT build bd_project bd_project.xpr]]
set XDC       [file normalize [file join $REPO_ROOT src fpga constraints axu9eg.xdc]]
set OUT_DIR   [file normalize [file join $REPO_ROOT build pl]]
set BD_NAME   "penumbrasight"
set WRAPPER   "penumbrasight_wrapper"
# 并发按内存定（本机 15.5 GB）：-jobs 8 会 OOM，见 build_pl.tcl 的说明。
set JOBS [expr {[info exists ::env(PS_PL_JOBS)] ? $::env(PS_PL_JOBS) : 4}]

proc ps_count_msgs {logfile pattern} {
    if {![file exists $logfile]} { return "?" }
    set fh [open $logfile r]
    set n 0
    while {[gets $fh line] >= 0} {
        if {[string first $pattern $line] >= 0} { incr n }
    }
    close $fh
    return $n
}

proc ps_worst_slack {dtype} {
    set tp [get_timing_paths -delay_type $dtype -max_paths 1 -nworst 1 -slack_lesser_than 1000]
    if {[llength $tp] == 0} { return "NA" }
    return [get_property SLACK $tp]
}

puts "==== PenumbraSight PL 仅综合（到 synth_design，不跑实现/位流）===="
if {![file exists $BD_PROJ]} {
    puts "ERROR: 找不到 $BD_PROJ —— 先跑 src/fpga/bd/penumbrasight_bd.tcl"
    exit 1
}
if {![file exists $XDC]} {
    puts "ERROR: 找不到约束文件 $XDC"
    exit 1
}
file mkdir $OUT_DIR

open_project $BD_PROJ
puts "  器件      = [get_property PART [current_project]]"

if {[llength [get_files -quiet ${BD_NAME}.bd]] == 0} {
    puts "ERROR: 工程里没有 ${BD_NAME}.bd"
    exit 1
}
if {[llength [get_files -quiet ${WRAPPER}.v]] == 0} {
    set wf [make_wrapper -files [get_files ${BD_NAME}.bd] -top]
    add_files -norecurse $wf
    puts "  已加入 wrapper: $wf"
} else {
    puts "  wrapper 已在工程里"
}
if {[llength [get_files -quiet [file tail $XDC]]] == 0} {
    add_files -fileset constrs_1 -norecurse $XDC
    puts "  已挂约束: [file tail $XDC]"
} else {
    puts "  约束已在工程里"
}
set_property top $WRAPPER [current_fileset]
update_compile_order -fileset sources_1

# 无条件重置：块设计改过之后旧结果一定作废（见文首）
if {[llength [get_runs -quiet synth_1]]} {
    puts "  reset_run synth_1（无条件）"
    reset_run synth_1
}

puts "\n---- 综合（launch_runs synth_1 -jobs $JOBS）----"
launch_runs synth_1 -jobs $JOBS
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {
    puts "ERROR: 综合未完成，状态 [get_property STATUS [get_runs synth_1]]"
    exit 1
}
open_run synth_1 -name synth_1
report_utilization    -file [file join $OUT_DIR report_utilization_synth.rpt]
report_utilization    -hierarchical -file [file join $OUT_DIR report_utilization_synth_hier.rpt]
report_timing_summary -file [file join $OUT_DIR report_timing_synth.rpt]

set synth_log [file join $REPO_ROOT build bd_project bd_project.runs synth_1 runme.log]
set crit [ps_count_msgs $synth_log {CRITICAL WARNING}]
set errs [ps_count_msgs $synth_log {ERROR:}]

puts "\n==== 仅综合完成 ===="
puts "  synth_1 runme.log 里 CRITICAL WARNING 行数 = $crit"
puts "  synth_1 runme.log 里 ERROR: 行数            = $errs"
puts "  WNS = [ps_worst_slack max] ns"
puts "  WHS = [ps_worst_slack min] ns"
puts "  报告 = [file join $OUT_DIR report_utilization_synth.rpt]"
puts "\n---- 本脚本没做、也不该声称做完的 ----"
puts "  1) 实现（place/route）、时序签核、DRC、位流 —— 这些由 src/fpga/scripts/build_pl.tcl 跑"
puts "  2) 综合阶段的 WHS 为负是这套 flow 的既有行为（见 bd_full_path_20260930.md §4.2），"
puts "     修 hold 要等实现阶段；**不要把综合阶段的 WHS 当成时序未收敛**"
