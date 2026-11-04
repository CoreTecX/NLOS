# ============================================================================
# PenumbraSight · 裸机最小系统 XSCT 自动化脚本
# src/software/baremetal/xsct_baremetal_run.tcl
#
# 做四件事：connect -> 初始化 PS（psu_init 或等价）-> 下载位流 -> 下载 ELF 并 con，
# 同时把串口输出重定向到文件（供脚本抓 KEY=VALUE 行）。
#
# 用法（在 Vitis / XSCT 命令行里）：
#   xsct xsct_baremetal_run.tcl
# 或在 XSCT 交互里：
#   source xsct_baremetal_run.tcl
#
# 必须改的变量：
#   XSA_PATH / BIT_PATH / ELF_PATH / SERIAL_PORT
# 说明：
#   * XSA 由并行任务导出，本脚本只把它当**可配置路径**用（不写死）。
#   * PS 串口走板载 USB-串口（PS UART0），XSCT **不能**直接抓（readjtaguart 只对
#     JTAG-UART 有效）。故本脚本调 uart_capture.py（pyserial）后台抓串口到文件。
#     若你的设计里有 JTAG-UART，可把 CAPTURE_CMD 换成 readjtaguart 流程（见文末注释）。
#
# 未上板声明：本脚本未在真板上执行过；各命令的可用性依赖 Xilinx 工具版本与 XSA 内容。
# ============================================================================

# ----------------------------------------------------------------- 配置（改这里）
set XSA_PATH        "CHANGE_ME.xsa"      ;# 并行任务导出的 XSA（本脚本不解析它，仅供参考/记录）
set BIT_PATH        "CHANGE_ME.bit"      ;# 位流文件（由 XSA 出 write_bitstream 得到）
set ELF_PATH        "../bin/baremetal_minimal.elf"  ;# 本目录程序的 ELF
set PSU_INIT_TCL    ""                   ;# 可选：Vitis 生成的 psu_init.tcl；空则尝试平台内建
set PSU_FILTER      {name =~ "*PSU*"}
set APU_FILTER      {name =~ "*Cortex-A53 #0*"}
set RUN_WAIT_MS     25000                ;# con 之后等待（毫秒），要覆盖全部四步
set PSU_SETTLE_MS   2000

# 串口抓取
set SERIAL_PORT     "COM3"               ;# 板载 USB-串口（PS UART0）
set SERIAL_BAUD     115200
set UART_LOG        "./uart_baremetal.log"
set CAPTURE_SECONDS 40
# 若留空，则用同目录的 uart_capture.py；也可换成自己的抓取命令（Tcl list 形式）
set CAPTURE_CMD     ""

# ----------------------------------------------------------------- 脚本目录
if {[catch {set SCRIPT_DIR [file dirname [file normalize [info script]]]}]} {
    set SCRIPT_DIR "."
}

# ----------------------------------------------------------------- 小工具
proc section {t} { puts "\n==== $t ====" }
proc need_file {p what} {
    if {![file exists $p]} {
        error "$what 不存在：$p\n  请修改脚本顶部的变量后重跑。"
    }
}

section "0. 前置检查"
puts "XSA_PATH   = $XSA_PATH"
puts "BIT_PATH   = $BIT_PATH"
puts "ELF_PATH   = [file normalize $ELF_PATH]"
puts "UART_LOG   = [file normalize $UART_LOG]"
need_file $BIT_PATH "位流 BIT_PATH"
need_file [file normalize $ELF_PATH] "ELF_PATH"

# ----------------------------------------------------------------- 1. 连接
section "1. connect"
connect
puts "connect 完成"

# ----------------------------------------------------------------- 2. 复位 PS
section "2. 复位系统（rst -system）"
targets -set -filter $PSU_FILTER
rst -system
after $PSU_SETTLE_MS

# ----------------------------------------------------------------- 3. 初始化 PS
section "3. PS 初始化（psu_init 或等价）"
if {$PSU_INIT_TCL ne ""} {
    need_file $PSU_INIT_TCL "PSU_INIT_TCL"
    source $PSU_INIT_TCL
}
if {[info procs psu_init] ne ""} {
    psu_init
    after 1000
    if {[info procs psu_ps_pl_isolation_removal] ne ""} { psu_ps_pl_isolation_removal }
    if {[info procs psu_ps_pl_reset_config] ne ""}      { psu_ps_pl_reset_config }
    puts "psu_init 完成（含 PL isolation removal / reset config）"
} else {
    puts "WARN: 找不到 psu_init 过程。请把 Vitis 生成的 psu_init.tcl 路径填进 PSU_INIT_TCL，"
    puts "     或在已由 FSBL 初始化过 PS 的前提下运行本脚本（否则 DDR/时钟未配置，下载会失败）。"
}

# ----------------------------------------------------------------- 4. 下载位流
section "4. 下载 PL 位流"
if {[catch { fpga -file [file normalize $BIT_PATH] } msg]} {
    error "位流下载失败：$msg\n  确认 BIT_PATH 是 .bit（不是 XSA）、且 JTAG 已连上。"
}
puts "位流下载完成"

# ----------------------------------------------------------------- 5. 下载 ELF
section "5. 下载 ELF 并运行"
targets -set -filter $APU_FILTER
rst -processor
dow [file normalize $ELF_PATH]
puts "ELF 下载完成"

# ----------------------------------------------------------------- 6. 启动串口抓取
section "6. 启动串口抓取 -> $UART_LOG"
if {$CAPTURE_CMD eq ""} {
    set CAPTURE_CMD [list python [file join $SCRIPT_DIR uart_capture.py] \
                         --port $SERIAL_PORT --baud $SERIAL_BAUD \
                         --out $UART_LOG --seconds $CAPTURE_SECONDS]
}
set cap_pid ""
if {[catch { set cap_pid [exec {*}$CAPTURE_CMD &] } cmsg]} {
    puts "WARN: 串口抓取启动失败（$cmsg）。"
    puts "      请手动在另一终端抓 $SERIAL_PORT，或把 CAPTURE_CMD 指向可用命令。"
} else {
    puts "抓取进程 pid = $cap_pid（$CAPTURE_SECONDS 秒后自停；也能看到 DONE=1 提前退出）"
}
after 500

# ----------------------------------------------------------------- 7. 运行
section "7. con（运行）"
con
after $RUN_WAIT_MS
catch { stop }

section "8. 完成"
puts "串口日志： [file normalize $UART_LOG]"
puts "用下面这行抓判据："
puts "  Select-String -Path '$UART_LOG' -Pattern '^STEP=|^RESULT=|^SUMMARY_|^DONE='"

# ----------------------------------------------------------------- 其它可选
# 若设计里有 JTAG-UART（MicroBlaze MDM / axi_uartlite+jtag），XSCT 原生可抓：
#   targets -set -filter {name =~ "*jtag_uart*"}
#   readjtaguart -file $UART_LOG -timeout $CAPTURE_SECONDS
# 本设计用的是 PS UART0（经 USB-串口），故走 uart_capture.py。
