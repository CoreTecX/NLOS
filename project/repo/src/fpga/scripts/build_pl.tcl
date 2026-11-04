# ============================================================================
# PenumbraSight PL 实现 · src/fpga/scripts/build_pl.tcl
#
# 用途：从块设计一路跑到 **bitstream** ——
#       块设计 wrapper -> 挂约束 -> 综合 -> 实现 -> 写位流 -> 出四份报告。
#       这是「上板前」最后一段能自动跑的东西。
#
# 用法（在**纯 ASCII 构建区**的仓库根执行；先跑过 penumbrasight_bd.tcl）
# ----------------------------------------------------------------------------
#     vivado -mode batch -nojournal -nolog -source src/fpga/scripts/build_pl.tcl
#
# 前置
# ----------------------------------------------------------------------------
#   1. `build/bd_project/bd_project.xpr` 已存在（由 penumbrasight_bd.tcl 建，
#      且其中块设计已 validate 通过）。
#   2. 覆盖 xczu9eg-ffvb1156-2-i 的授权在位（ENTERPRISE，2026-11-27 到期）。
#
# 产出
# ----------------------------------------------------------------------------
#   build/pl/report_utilization_synth.rpt   综合后资源
#   build/pl/report_timing_synth.rpt        综合后时序
#   build/pl/report_utilization_impl.rpt    实现后资源（含布线）
#   build/pl/report_timing_impl.rpt         实现后时序（WNS/TNS/WHS）
#   build/pl/report_drc.rpt                 实现后 DRC
#   build/bd_project/bd_project.runs/impl_1/*.bit   位流
#
# 交付边界
# ----------------------------------------------------------------------------
#   本脚本跑「默认策略」。**不声称时序一定收敛** —— 若 WNS < 0，
#   要么改策略要么回 HLS 改那条链路（X19 报告 §5 有瓶颈链路的定位）。
#   CFGBVS / CONFIG_VOLTAGE **未设**（axu9eg.xdc 里写明须查核心板原理图再填），
#   故本脚本跑出的位流**可能无法用于 QSPI 启动**，JTAG 加载不受影响。
# ============================================================================

set REPO_ROOT [file normalize [pwd]]
set BD_PROJ   [file normalize [file join $REPO_ROOT build bd_project bd_project.xpr]]
set XDC       [file normalize [file join $REPO_ROOT src fpga constraints axu9eg.xdc]]
set OUT_DIR   [file normalize [file join $REPO_ROOT build pl]]
set BD_NAME   "penumbrasight"
set WRAPPER   "penumbrasight_wrapper"
# 并发数按**内存**定，不是按核数。本机 15.5 GB / 18 逻辑核，2026-09-29 实测：
#   `-jobs 8` -> 8 个子运行同时起，各要 1.7~1.9 GB，当场 OOM
#   （失败子运行的 runme.log 末行是 `out of memory allocating 8388640 bytes`）
# 可用环境变量覆盖，便于不改文件就调： PS_PL_JOBS=1 vivado -mode batch ...
set JOBS [expr {[info exists ::env(PS_PL_JOBS)] ? $::env(PS_PL_JOBS) : 1}]

# 从 runme.log 数消息，不用 get_msg_config ——
# `get_msg_config -count` 在批次模式下语义不稳（它读的是消息配置，不是本次运行'),
# 而**数 log 里出现过几行**这件事本身是确定的。
# 教训来自 2026-09-28 RTL 侧：只数 error 会漏掉 multi-driven 那类 CRITICAL WARNING。
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

puts "==== PenumbraSight PL 实现 ===="
puts "  REPO_ROOT = $REPO_ROOT"
puts "  工程      = $BD_PROJ"
puts "  约束      = $XDC"

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

# ---------------------------------------------------------------- 1 块设计 wrapper
if {[llength [get_files -quiet ${BD_NAME}.bd]] == 0} {
    puts "ERROR: 工程里没有 ${BD_NAME}.bd"
    exit 1
}
# make_wrapper 只生成文件，**不会自动加进工程** —— 未加进来时 top 设不上。
if {[llength [get_files -quiet ${WRAPPER}.v]] == 0} {
    set wf [make_wrapper -files [get_files ${BD_NAME}.bd] -top]
    add_files -norecurse $wf
    puts "  已加入 wrapper: $wf"
} else {
    puts "  wrapper 已在工程里"
}

# ---------------------------------------------------------------- 2 约束
if {[llength [get_files -quiet [file tail $XDC]]] == 0} {
    add_files -fileset constrs_1 -norecurse $XDC
    puts "  已挂约束: [file tail $XDC]"
} else {
    puts "  约束已在工程里"
}

set_property top $WRAPPER [current_fileset]
update_compile_order -fileset sources_1
puts "  顶层      = [get_property top [current_fileset]]"

# ---------------------------------------------------------------- 2.5 运行准备
# 无条件 reset synth_1 / impl_1（2026-10-02 更正）。
#
# 原实现是「PROGRESS != 100% 才 reset」，它有一条静默的漏网：块设计（.bd）
# 或约束（xdc）改了、而 run 恰好停在 100% 时，脚本会**沿用过期结果** ——
# launch 照跑、报告照出，读到的是旧资源与旧时序，全程没有任何提示。
# synth_pl_only.tcl 那份从一开始就是无条件 reset（见其文首），两份脚本
# 行为不一致本身也是隐患。
#
# 为什么不按「输入时间戳判定」：run 的依赖图（.bd / xdc / IP / wrapper）会随
# 工具版本变化，用 mtime 判据只要漏一项就回到「静默复用」这一类缺陷。无条件
# reset 的判据是「本次运行一定用本次输入」，不需要维护依赖清单。
#
# 代价（构建时长）：每次调用都从综合重跑到位流。run_bitstream.ps1 文首记的
# 单趟耗时是 2~2.5 h，即无改动时的一次调用从「秒级返回」变成「整趟重跑」。
# 这是**用时间换正确性**的取舍：错读旧时序/旧资源比多等一小时更贵。
# 只想便宜地看综合结果时用 synth_pl_only.tcl（只到综合，同样无条件 reset）。
#
# 失败或被中断过的 run 本来也必须先 reset（否则 launch 被直接拒：
# ERROR: [Common 17-69] Run 'synth_1' needs to be reset before launching.），
# 无条件 reset 同时覆盖这一条。
foreach r {synth_1 impl_1} {
    if {[llength [get_runs -quiet $r]] == 0} { continue }
    puts "  reset_run $r（无条件；旧实现只在 PROGRESS!=100% 时重置，会复用过期结果）"
    reset_run $r
}

# ---------------------------------------------------------------- 3 综合
puts "\n---- 综合 ----"
launch_runs synth_1 -jobs $JOBS
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {
    puts "ERROR: 综合未完成，状态 [get_property STATUS [get_runs synth_1]]"
    exit 1
}
open_run synth_1 -name synth_1
report_utilization       -file [file join $OUT_DIR report_utilization_synth.rpt]
report_timing_summary    -file [file join $OUT_DIR report_timing_synth.rpt]
# 把关键告警记下来：这一课在 RTL 侧已经吃过一次，不能再只数 error
puts "  综合 CRITICAL WARNING 行数 = [ps_count_msgs [file join $REPO_ROOT build bd_project bd_project.runs synth_1 runme.log] {CRITICAL WARNING}]"
puts "  综合 ERROR 行数            = [ps_count_msgs [file join $REPO_ROOT build bd_project bd_project.runs synth_1 runme.log] {ERROR:}]"

# ---------------------------------------------------------------- 4 实现 + 位流
puts "\n---- 实现（到 write_bitstream）----"
launch_runs impl_1 -to_step write_bitstream -jobs $JOBS
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    puts "ERROR: 实现未完成，状态 [get_property STATUS [get_runs impl_1]]"
    exit 1
}
open_run impl_1
report_utilization       -file [file join $OUT_DIR report_utilization_impl.rpt]
report_timing_summary    -file [file join $OUT_DIR report_timing_impl.rpt]
report_drc               -file [file join $OUT_DIR report_drc.rpt]
set impl_crit [ps_count_msgs [file join $REPO_ROOT build bd_project bd_project.runs impl_1 runme.log] {CRITICAL WARNING}]
puts "  实现 CRITICAL WARNING 行数 = $impl_crit"

# ---------------------------------------------------------------- 5 回执
# 取时序余量。**这三行的原始写法踩过一次坑**：`get_timing_paths` 默认只返回
# 违例路径（等价于自带 `-slack_lesser_than 0`），2026-09-30 首次跑通实现时
# 时序全部满足、一条违例都没有，于是它返回空列表，紧接着 `get_property SLACK {}`
# 报 `[Common 17-55] 'get_property' expects at least one object`，脚本以退出码 1
# 结束 —— 而位流其实已经正常生成（17 MB，植入成功）。**退出码 1 在这里不代表
# 失败，只代表「余量取不到」**。所以必须对空列表容错。
proc ps_worst_slack {dtype} {
    # 用一个足够大的阈值把「最差路径」拿回来，不管它的 slack 是正是负
    set tp [get_timing_paths -delay_type $dtype -max_paths 1 -nworst 1 -slack_lesser_than 1000]
    if {[llength $tp] == 0} { return "NA" }
    return [get_property SLACK $tp]
}
set wns [ps_worst_slack max]
set whs [ps_worst_slack min]
set tp_viol [get_timing_paths -delay_type max -max_paths 100000 -nworst 100000 -slack_lesser_than 0]
set tns [expr {[llength $tp_viol] == 0 ? 0.000 : [get_property SLACK $tp_viol]}]
puts "\n==== 实现完成 ===="
puts "  WNS = $wns ns"
puts "  WHS = $whs ns"
puts "  位流 = [glob -nocomplain [file join $REPO_ROOT build bd_project bd_project.runs impl_1 *.bit]]"
if {$wns >= 0.0} {
    puts "  时序结论 = 收敛（WNS >= 0）"
} else {
    puts "  时序结论 = **未收敛**，见 report_timing_impl.rpt；改策略或回 HLS 改瓶颈链路"
}
puts "\n---- 本脚本没做、也不该声称做完的 ----"
puts "  1) CFGBVS / CONFIG_VOLTAGE 未设 -> 位流可用于 JTAG，**QSPI 启动需先补**"
puts "  2) 板端 PS 软件（寄存器偏移按定点层各 IP 的 xps_*_fix_hw.h）"
puts "  3) 上板验证本身（7.1~7.7）"
