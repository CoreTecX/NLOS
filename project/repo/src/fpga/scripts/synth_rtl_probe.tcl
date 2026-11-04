# ============================================================================
# synth_rtl_probe.tcl —— RTL 五模块的**资源与时序**探针（Vivado 非工程模式）
#
# 为什么要有这个脚本
# ----------------------------------------------------------------------------
# src/fpga/rtl/readme.md 第 6 节 E1/E2 写着「资源估算」「时序」两项待验证，
# 而本机此前没有 Vivado，五模块**从未综合过**（该 readme 第 5 行的验证状态声明
# 就是为这件事写的）。装上 Vivado 2025.2 后，这是唯一能把 E1/E2 与
# X21 附加项 9「M1 有资源与时序」一起补齐的做法。
#
# M1 是哪两个模块
# ----------------------------------------------------------------------------
# 方案 §6.1 的预处理链 = raw_preproc（解交织 + 双绿 + 10x10 块和 + 1/4/8 帧累积）
#   + vertical_diff（竖直相邻块差分 -> 9120x3 测量）。
# A2_资源预算.md 第二节的「M1 小计」对应的就是这两个。另外三个模块
# （fast_alert / confidence_engine / compositor）也一并综合 —— 它们同属 PL 侧、
# 同吃一个 pl_clk0，资源要一起算才不会漏。
#
# 用法
# ----------------------------------------------------------------------------
#     vivado -mode batch -nojournal -nolog -source src/fpga/scripts/synth_rtl_probe.tcl
#   （-tclargs 可选：首项为数字时作周期覆盖，其余项为模块子集；不传则用
#    src/fpga/scripts/clock_plan.tcl 的默认周期 10.0 ns = 100 MHz）
# 周期默认来自 clock_plan.tcl（单一来源）；**本脚本不再写 10.0 字面量**。
# 注意：raw_preproc 的 E2 原文写「目标 150 MHz」，那是 100 MHz 定案之前的假设，
#   本脚本按实际选定的 100 MHz 约束，并在报告里把两个门槛都给出结论。
#
# 约束
# ----------------------------------------------------------------------------
# 五模块都是 `clk` + `rst_n` 两线同步设计（逐个核对过端口表），且全部在 PL 内
# 由一个时钟驱动 —— 与 axu9eg.xdc 文首的「PL 无用户 I/O、无跨时钟域」判断一致。
# 所以这里只需要一条 create_clock，不需要 set_clock_groups / false_path。
# 用 `-mode out_of_context`：这些模块在系统里是被例化的，不是顶层，
# OOC 综合才能拿到干净的、不含 I/O buffer 的资源数字。
#
# 产物
# ----------------------------------------------------------------------------
#   build/synth/<模块>_util.rpt     完整利用率报告
#   build/synth/<模块>_timing.rpt   完整时序报告
#   build/synth/synth_rtl_summary.md  合计表（**要入库**，见 .gitignore 的放行）
# ============================================================================

set PART       "xczu9eg-ffvb1156-2-i"
set REPO_ROOT [file normalize [pwd]]

# 目标时钟周期：单一来源 src/fpga/scripts/clock_plan.tcl（2026-10-02 起）——
# 本脚本不再写 10.0 这个字面量。build_hls.tcl / penumbrasight_bd.tcl 同源。
source [file join $REPO_ROOT src fpga scripts clock_plan.tcl]
set CLK_PERIOD $PS_PL_CLK_PERIOD_NS

set RTL_DIR   [file join $REPO_ROOT src fpga rtl]
set OUT_DIR   [file join $REPO_ROOT build synth]
file mkdir $OUT_DIR

# M1 的两个模块放最前（X21 附加项 9 要的就是它俩），其余三个模块随后。
set MODULES [list raw_preproc vertical_diff fast_alert confidence_engine compositor]

# 可选 -tclargs：
#   · 首项若为数字 -> 作为 CLK_PERIOD 覆盖（扫周期用）；否则首项起即模块子集。
#   · 其余项一律是**模块子集**，方便只重综合改动过的那一两个。
#     vivado ... -source synth_rtl_probe.tcl                       （默认周期 + 全模块）
#     vivado ... -source synth_rtl_probe.tcl -tclargs 6.667         （扫周期）
#     vivado ... -source synth_rtl_probe.tcl -tclargs raw_preproc fast_alert（只做子集）
set _rest $argv
if {[llength $_rest] >= 1 && [string is double -strict [lindex $_rest 0]]} {
    set CLK_PERIOD [lindex $_rest 0]
    set _rest [lrange $_rest 1 end]
}
if {[llength $_rest] >= 1} {
    set MODULES $_rest
}

puts "==== PenumbraSight RTL 综合探针 ===="
puts "  REPO_ROOT = $REPO_ROOT"
puts "  PART      = $PART"
puts "  CLK       = $CLK_PERIOD ns"
puts "  模块      = $MODULES"
puts ""

# ---------------------------------------------------------------- 工具过程
# 从利用率报告里按行名取 Used 列。**不用 regexp** —— 报告里的竖线与括号
# 很容易把正则和 Tcl 的括号平衡同时弄坏，split 更稳。
proc ps_grab {txt key} {
    foreach line [split $txt "\n"] {
        if {[string first $key $line] >= 0} {
            set parts [split $line "|"]
            if {[llength $parts] >= 4} {
                return [string trim [lindex $parts 2]]
            }
        }
    }
    return "?"
}

proc ps_readfile {path} {
    set fh [open $path r]
    set txt [read $fh]
    close $fh
    return $txt
}

set SUMMARY_ROWS {}
set ALL_OK 1

foreach m $MODULES {
    puts "----------------------------------------------------------------"
    puts "==== 综合 $m ===="

    set src [file join $RTL_DIR ${m}.sv]
    if {![file exists $src]} {
        puts "ERROR: 找不到 $src"
        lappend SUMMARY_ROWS [list $m "缺源文件" "?" "?" "?" "?" "?" "?"]
        set ALL_OK 0
        continue
    }

    # 每个模块包在 catch 里：一个模块失败不应让后面四个拿不到数字。
    set rc [catch {
        # read_verilog 的 -sv 只是把意图写明（.sv 后缀本来就会被按 SV 处理）。
        # 注意：本探针最初报的 61 个
        #   `Synth 8-6735: net type must be explicitly specified for '<port>'
        #    when default_nettype is none`
        # **不是**探针脚本的问题，是 RTL 的真实缺陷 —— 五个模块在文首写了
        # `` `default_nettype none ``，而 Vivado 在它下面**拒绝** `input logic`，
        # 只接受 `input wire logic`（`output logic` 不受影响）。
        # 2026-09-28 已用最小复现判定并修好五个模块（75 处），详见
        # src/fpga/rtl/readme.md 与 sim/results/x19_hls_resources_and_timing.md。
        read_verilog -sv $src
        synth_design -top $m -part $PART -mode out_of_context
        create_clock -period $CLK_PERIOD -name pl_clk [get_ports clk]
        report_utilization -file [file join $OUT_DIR ${m}_util.rpt]
        report_timing_summary -delay_type max -file [file join $OUT_DIR ${m}_timing.rpt]
    } err]

    if {$rc != 0} {
        puts "ERROR: $m 综合失败：$err"
        lappend SUMMARY_ROWS [list $m "综合失败" "?" "?" "?" "?" "?" "?"]
        set ALL_OK 0
        continue
    }

    set util [ps_readfile [file join $OUT_DIR ${m}_util.rpt]]
    set lut  [ps_grab $util "CLB LUTs"]
    set ff   [ps_grab $util "CLB Registers"]
    set bram [ps_grab $util "Block RAM Tile"]
    set dsp  [ps_grab $util "DSPs"]
    set lram [ps_grab $util "LUT as Memory"]

    # 时序：直接问 Vivado 要最坏路径的 slack，比解析报告稳。
    set wns "?"
    set levels "?"
    set rc2 [catch {
        set tp [get_timing_paths -delay_type max -max_paths 1 -nworst 1]
        if {[llength $tp] > 0} {
            set wns    [get_property SLACK $tp]
            set levels [get_property LOGIC_LEVELS $tp]
        }
    } err2]
    if {$rc2 != 0} {
        puts "WARN: $m 时序查询失败：$err2"
    }

    # 由 WNS 反推可达频率：slack 是在目标周期上的余量，
    # 所以实际能跑到 1000 / (周期 - WNS) MHz。
    set fmax "?"
    if {$wns ne "?"} {
        set eff [expr {$CLK_PERIOD - $wns}]
        if {$eff > 0.0} {
            set fmax [format "%.0f" [expr {1000.0 / $eff}]]
        } else {
            set fmax ">9999"
        }
    }

    puts "  LUT=$lut  FF=$ff  BRAM=$bram  DSP=$dsp  LUTRAM=$lram  WNS=$wns ns  Fmax≈$fmax MHz"
    lappend SUMMARY_ROWS [list $m $lut $ff $bram $dsp $lram $wns $fmax]
}

# ---------------------------------------------------------------- 合计表
set md [file join $OUT_DIR "synth_rtl_summary.md"]
set fh [open $md w]
# ★ 这个文件的内容**必须全是 ASCII**。第一版写的是中文，结果落盘后全是乱码
#   （读出来是 "RTL 浜旀ā鍧楃患鍚堢粨鏋?"）—— Tcl 的 `open name w` 用系统码页
#   （本机 GBK），把 UTF-8 的中文写进去就互相错位。与 Vitis 控制台那条是同一类问题：
#   这类工具的输出通道只保证 ASCII。中文说明一律留在本脚本的注释里。
puts $fh "# RTL five modules - synthesis result (Vivado, non-project mode)"
puts $fh ""
puts $fh "- part   : $PART"
puts $fh "- clock  : pl_clk0, period $CLK_PERIOD ns (same source as build_hls.tcl / axu9eg.xdc)"
puts $fh "- mode   : synth_design -mode out_of_context (all five are instantiated sub-modules)"
puts $fh "- command: vivado -mode batch -nojournal -nolog -source src/fpga/scripts/synth_rtl_probe.tcl -tclargs $CLK_PERIOD"
puts $fh "- note   : OOC synthesis estimate, NOT a post-implementation signoff value."
puts $fh ""
puts $fh "| Module | CLB LUTs | CLB Registers | Block RAM Tile | DSPs | LUT as Memory | WNS (ns) | Fmax (MHz) |"
puts $fh "|---|---|---|---|---|---|---|---|"
foreach row $SUMMARY_ROWS {
    puts $fh "| $row |"
}
puts $fh ""
puts $fh "WNS >= 0 means the module meets the target period. Fmax is back-derived as 1000 / ($CLK_PERIOD - WNS); order-of-magnitude only."
close $fh

puts ""
puts "==== 全部完成。合计表：$md ===="
if {$ALL_OK != 1} {
    puts "WARN: 有模块未成功，见上面 ERROR 行。"
    exit 1
}
