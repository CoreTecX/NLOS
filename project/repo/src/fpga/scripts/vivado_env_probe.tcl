# ============================================================================
# vivado_env_probe.tcl · 「装完即验」探针（A4 清单的执行部分）
# ----------------------------------------------------------------------------
# 回答两个问题：
#   1) 这份 Vivado 能不能对真正要用的器件（xczu9eg-ffvb1156-2-i）干活？
#   2) 在当前**未配置 license** 的状态下，Vivado 放行还是拒绝？
#
# 做法：对目标器件跑一次**极小**综合。Vivado 的器件授权检查在综合一开始就执行，
#       所以几十秒就能逼出判定，不需要真跑完整流程、不需要板子。
#
# 用法（普通 PowerShell，**不要**在受限沙箱里跑）：
#   & 'D:\vivado\2025.2\Vivado\bin\vivado.bat' -mode batch -source vivado_env_probe.tcl
#
# 读法：
#   PART_LOOKUP = OK          -> 器件在本安装内（已确认）
#   SYNTH = OK                -> 该器件无需 license 即可综合（免费档覆盖）
#   SYNTH = FAIL 且消息含 license -> 确认必须拿到覆盖 ZU9EG 的授权
#   其它 FAIL 消息            -> 属环境问题，把原文整段贴出来再判断
#
# 已知坑：Vivado 启动时会写 %APPDATA%\Xilinx\Vivado\<ver>\XilinxTclStore\。
#   若该路径不可写，会以 `[Common 17-69] ... cannot open file` 失败 ——
#   那是**环境**问题，不是授权问题，别误读成"license 不通过"。
#   注意 Vivado 走 Windows API 取用户目录，改 %APPDATA% 环境变量**无效**。
# ============================================================================

puts "===== PROBE START ====="
puts "VIVADO_VERSION = [version -short]"

set part "xczu9eg-ffvb1156-2-i"

# --- 1) 器件是否存在于本安装 ---
if {[catch {set pl [get_parts $part]} e]} {
    puts "PART_LOOKUP = FAIL : $e"
} else {
    puts "PART_LOOKUP = OK   : $pl"
}

# --- 2) license 搜索路径（看有没有已配置的 license）---
foreach ev {XILINXD_license_FILE LM_license_FILE} {
    if {[info exists ::env($ev)]} {
        puts "ENV $ev = $::env($ev)"
    } else {
        puts "ENV $ev = (unset)"
    }
}

# --- 3) 工作目录：优先系统 TEMP，避免污染仓库 ---
if {[info exists ::env(TEMP)] && $::env(TEMP) ne ""} {
    set wd [file join $::env(TEMP) psight_vivado_probe]
} else {
    set wd [file join [file dirname [file normalize [info script]]] _probe_out]
}
file mkdir $wd
cd $wd
puts "WORKDIR = [pwd]"

set fh [open [file join $wd top.v] w]
puts $fh "module top (input a, output b);\n    assign b = a;\nendmodule\n"
close $fh

# --- 4) 决定性判据：对目标器件跑一次极小综合 ---
if {[catch {create_project -in_memory -part $part} e]} {
    puts "CREATE_PROJECT = FAIL : $e"
} else {
    puts "CREATE_PROJECT = OK"
    add_files -norecurse [file join $wd top.v]
    if {[catch {synth_design -top top -part $part -mode out_of_context} e]} {
        puts "SYNTH = FAIL"
        puts "-----SYNTH_MSG_BEGIN-----"
        puts $e
        puts "-----SYNTH_MSG_END-----"
    } else {
        puts "SYNTH = OK   -> 该器件在无 license 状态下即可综合"
    }
}
puts "===== PROBE END ====="
