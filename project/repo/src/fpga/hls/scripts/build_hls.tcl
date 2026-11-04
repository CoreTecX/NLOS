# ============================================================================
# PenumbraSight HLS 构建脚本 · src/fpga/hls/scripts/build_hls.tcl
#
# 用途：把三个 HLS 核（projection_engine / lr_operator / pdhg_core）在
#       Vitis HLS 里各建一个工程，跑 C 仿真（csim）与综合（csynth），导出 IP。
#
# 用法（在**仓库根**下执行）：
#     vitis-run --mode hls --tcl src/fpga/hls/scripts/build_hls.tcl
#
# 只跑某一个核（可选参数，名字取 projection / lr / pdhg / all）：
#     vitis-run --mode hls --tcl src/fpga/hls/scripts/build_hls.tcl -tclargs pdhg
#
# 说明
# ---------------------------------------------------------------------------
# 1) 目标器件：ALINX AXU9EGB 的 **xczu9eg-ffvb1156-2-i**（XCZU9EG-2FFVB1156I，工业级 -2）。
#    出处：ALINX《AXU9EGB开发板用户手册》。与 penumbrasight_bd.tcl、constraints/axu9eg.xdc
#    同源 —— **换板时这三处的 PART 要一起改**。
# 2) 时钟：先按 5 ns（200 MHz）收敛标定；方案 §5.3 的算力目标是 100 ms 量级，
#    不追求更高主频。若 csynth 报时序不收敛，先加大 CLK_PERIOD 再谈改架构
#    （方案 §6.5 也给了对偶投影的降级路径）。
# 3) **顶层必须是具体函数，不能是模板**。三个核的内部实现都是
#    template <int LANES>，所以每个 .cpp 末尾给了一个非模板包装：
#        projection_engine_top（LANES = PS_PROJ_LANES_B，B 档 16）
#        lr_forward_top     （LANES = PS_LR_LANES_B_FP，B 档 16）
#        lr_adjoint_top     （与 lr_forward_top 共用同一套 MAC 原语）
#        pdhg_core_top      （LANES = PS_PDHG_LANES_B_FP，B 档 16）
#    改并行度只改这些包装里用的常量，不动核本体。
# 4) 三个核都只依赖 src/software/include/psm_format.h 与彼此；不含任何
#    第三方库，也不需要 Vivado 之外的东西。
# 5) 本脚本只**建工程**，不写 build/ 之外的任何文件；生成的工程目录
#    （*.hls_prj / solution*/）已被 .gitignore 排除。
# 6) 主机侧（不需要 Vivado）的验证请直接编译 tb.cpp：
#        g++ -std=c++17 -Wall -Wextra <core>.cpp tb.cpp -o tb.exe && ./tb.exe
#    三个 tb 都自带自检并打印真实数值（见 src/fpga/hls/readme.md）。
# ============================================================================

set PART        "xczu9eg-ffvb1156-2-i"
set SOLUTION    "solution1"
set DO_CSIM     1
set DO_CSYNTH   1

# ---------------------------------------------------------------- 时钟周期（单一来源）
# 周期由 src/fpga/scripts/clock_plan.tcl **单一定义**（2026-10-02 起）——
# 本脚本不再写 10.0 这个字面量。块设计（penumbrasight_bd.tcl）也 source 同一
# 文件，xdc 侧只留说明；**不再是「三处字面量 + 注释约定一起改」**。
#
# 该文件里的默认值是 10.0 ns = 100 MHz，依据（X19 实测，2026-09-28）：
#   ps_lr_forward_fix / ps_lr_adjoint_fix   Slack  0.00 @5.0 ns  -> 200 MHz 就收敛
#   ps_projection_fix                       Slack -1.90 @5.0 ns  -> 6.667 ns 收
#   ps_pdhg_fix                             Slack -1.39 @5.0 ns  -> 6.667 ns 仍 -0.51，10.0 ns 才收
# 即**四个核同时收在 100 MHz**，故默认值定为 10.0（原文写的 5.0 是「先按 200 标定」的临时值）。
# 按本文件第 19 行的既定原则「先加大 CLK_PERIOD 再谈改架构」，周期必须能扫；
# 扫周期用环境变量覆盖（PS_PL_CLK_PERIOD_NS，旧名 PS_HLS_CLK_PERIOD 仍可用），别改脚本：
#     set PS_PL_CLK_PERIOD_NS=6.667 && vitis-run --mode hls --tcl --input_file ...
source [file join [pwd] src fpga scripts clock_plan.tcl]
set CLK_PERIOD  $PS_PL_CLK_PERIOD_NS

# ---------------------------------------------------------------- 位宽放宽
# 这是**纯配置旋钮**：不改签名、不改 AXI 寄存器映射、不改一行算法。
#
# 为什么需要它：X19 §3.2 实测到 `csynth.rpt` 里满屏
#     `214-353 Could not widen since type i32 size is greater than or equal to
#              the max_widen_bitwidth threshold of 0`
# —— 阈值 0 表示**根本没有开位宽放宽优化**。后果是三个核全部退化成
# 「一拍一个乘累加」：投影核的瓶颈循环 II 被钉在 48，53.2 M 个字按
# 400 MB/s 走完要 532.4 ms（与实测周期数逐位吻合）。
#
# 放宽到 256 bit 后，HLS 在**可证明连续**的访问上会合并成宽读，等效带宽成倍提高。
# 它救不了步长不连续的访问（投影核的 meas 是三通道块优先布局，三个流交错），
# 那一条要改数据布局，属另一项工作。
#
# 用环境变量覆盖，便于做「开/关」对照（WIDEN=0 即回到旧行为）：
#     set PS_HLS_WIDEN=0 && vitis-run --mode hls --tcl --input_file ...
set WIDEN 256
if {[info exists ::env(PS_HLS_WIDEN)] && $::env(PS_HLS_WIDEN) ne ""} {
    set WIDEN $::env(PS_HLS_WIDEN)
}

# ---------------------------------------------------------------- 顶层模式
# shell（默认）：综合**主机验证壳**（lr_forward_top 等）。用双精度、无需 AXI，
#                主机侧 tb 与 make test-hls 就是验这一层。
# axi          ：综合 **A6 的 AXI 顶层**（ps_lr_forward_axi 等）——**块设计要的就是这一层**。
#                见 src/fpga/hls/ps_axi_top.h 与 docs/hls_interface_spec.md。
# fixed        ：综合**定点层**（*_fixed.cpp + ps_fixed_top.cpp）——**真正要上板的那一层**。
#                双精度层在 ZU9EG 的 DSP48E2 上做不了（27x18 乘法器），定点层才是目标。
#
# 为什么要有这个开关：A6 交付的 AXI 接口层此前**没有被任何脚本构建过**，
# 于是块设计里其实没有可引用的 IP。默认仍是 shell 以保持
# 「既有测试验的就是被综合的那一层」这条性质不被破坏；
# 生成给块设计用的 IP 时用 PS_HLS_TOP_MODE=axi / fixed 显式切换。
#
# 用法（★ 2026-09-28 更正：vitis-run 2025.2 **不接受 `-tclargs`**，且会把 token
# `tcl` 注入脚本的 $argv，所以核选择与模式一律走环境变量，见下方「参数来源」）：
#     set PS_HLS_WHICH=projection&& set PS_HLS_TOP_MODE=fixed&& vitis-run --mode hls --tcl --input_file .../build_hls.tcl
#     set PS_HLS_WHICH=all&&        set PS_HLS_TOP_MODE=axi&&   vitis-run --mode hls --tcl --input_file .../build_hls.tcl
set TOP_MODE    "shell"

# 仓库根 = 当前工作目录（脚本要求从仓库根执行）
set REPO_ROOT   [file normalize [pwd]]
set HLS_ROOT    [file normalize [file join $REPO_ROOT src fpga hls]]
set INC_HW      [file normalize [file join $REPO_ROOT src hardware]]
set INC_PSM     [file normalize [file join $REPO_ROOT src software include]]
set CFLAGS      "-I$INC_HW -I$INC_PSM"

# ---------------------------------------------------------------- 参数来源（环境变量优先）
# WHICH 取 projection / lr / pdhg / all；TOP_MODE 取 shell / axi / fixed
#
# ⚠ 2026-09-28 在 Vitis 2025.2 上实测到两条，**旧版「命令行优先」逻辑因此失效**：
#   1) vitis-run **不接受 `-tclargs`**（`unrecognised option '-tclargs'`）；
#   2) `--tcl <脚本>` 与 `--tcl --input_file <脚本>` 两种形式下，vitis-run 都会把
#      token `tcl` 注入脚本的 `$argv`（实测 $argv[1] == "tcl"）。于是旧逻辑把
#      TOP_MODE 解析成 'tcl'，只打印一句模式无效就退出，看起来像脚本写错了。
#   故改为：**环境变量优先**，命令行参数**只在取值合法时**才作补充，
#   并把实际看到的 $argv 打印出来（下面那行 puts），便于日后诊断。
#   用法（见仓库根 makefile 的 hls-* 与 hls-fixed 目标）：
#       PS_HLS_WHICH=projection|lr|pdhg|all
#       PS_HLS_TOP_MODE=shell|axi|fixed
set WHICH   "all"
set TOP_MODE "shell"
if {[info exists ::env(PS_HLS_WHICH)] && $::env(PS_HLS_WHICH) ne ""} {
    set WHICH $::env(PS_HLS_WHICH)
} elseif {[llength $argv] >= 1 && [lindex $argv 0] in {projection lr pdhg all}} {
    set WHICH [lindex $argv 0]
}
if {[info exists ::env(PS_HLS_TOP_MODE)] && $::env(PS_HLS_TOP_MODE) ne ""} {
    set TOP_MODE $::env(PS_HLS_TOP_MODE)
} elseif {[llength $argv] >= 2 && [lindex $argv 1] in {shell axi fixed cal}} {
    set TOP_MODE [lindex $argv 1]
}
if {$TOP_MODE ni {shell axi fixed cal}} {
    # 出错信息走**纯 ASCII**：Vitis 的 Tcl 输出经控制台按 GBK 解码时中文会变乱码
    # （实测 '椤跺眰妯″紡'），出错时反而看不见原因。下面再补一行中文原文。
    puts "ERROR: invalid TOP_MODE '$TOP_MODE' (want shell | axi | fixed | cal)"
    puts "ERROR: 顶层模式无效"
    exit 1
}
# AXI 模式要额外编入 A6 的接口层
set AXI_SRC [file normalize [file join $REPO_ROOT src fpga hls ps_axi_top.cpp]]
if {$TOP_MODE eq "axi" && ![file exists $AXI_SRC]} {
    puts "ERROR: 模式 axi 需要 $AXI_SRC，但它不存在"
    exit 1
}
# fixed 模式要额外编入定点层的 csynth 顶层（ps_fixed_top.cpp）
set FIX_SRC [file normalize [file join $REPO_ROOT src fpga hls ps_fixed_top.cpp]]
if {$TOP_MODE eq "fixed" && ![file exists $FIX_SRC]} {
    puts "ERROR: 模式 fixed 需要 $FIX_SRC，但它不存在"
    exit 1
}

puts "==== PenumbraSight HLS build ===="
puts "  REPO_ROOT = $REPO_ROOT"
puts "  HLS_ROOT  = $HLS_ROOT"
puts "  PART      = $PART  (clock $CLK_PERIOD ns)"
puts "  WHICH     = $WHICH"
if {![file exists [file join $INC_PSM psm_format.h]]} {
    puts "ERROR: 找不到 $INC_PSM/psm_format.h —— 请在仓库根下执行本脚本"
    exit 1
}

# ---------------------------------------------------------------- 通用流程
proc ps_build_core {name top tb_files src_files cflags} {
    global PART CLK_PERIOD SOLUTION DO_CSIM DO_CSYNTH WIDEN
    puts "\n==== \[$name\] 工程 $name，top = $top ===="

    open_project -reset $name

    foreach f $src_files { add_files $f -cflags $cflags }
    foreach f $tb_files  { add_files -tb $f -cflags $cflags }

    set_top $top
    open_solution -reset $SOLUTION
    set_part $PART
    create_clock -period $CLK_PERIOD -name default

    # --- 位宽放宽（config_interface 只能在 open_solution 之后调用）---
    # WIDEN 的来源与理由见本文件「位宽放宽」一节。WIDEN=0 时不调用，
    # 回到工具默认行为，便于做开/关对照而不必改脚本。
    if {$WIDEN > 0} {
        puts "---- \[$name\] config_interface -m_axi_max_widen_bitwidth $WIDEN ----"
        if {[catch { config_interface -m_axi_max_widen_bitwidth $WIDEN } err]} {
            puts "WARN: \[$name\] config_interface 失败（继续综合）：$err"
        }
    } else {
        puts "---- \[$name\] WIDEN=0：不调用 config_interface，用工具默认值 ----"
    }

    if {$DO_CSIM} {
        puts "---- \[$name\] csynth 之外的 C 仿真 ----"
        if {[catch { csim_design -clean } err]} {
            puts "ERROR: \[$name\] csim_design 失败：$err"
            close_project
            exit 1
        }
    }
    if {$DO_CSYNTH} {
        if {[catch { csynth_design } err]} {
            puts "ERROR: \[$name\] csynth_design 失败：$err"
            close_project
            exit 1
        }
        if {[catch { export_design -format ip_catalog } err]} {
            puts "WARN: \[$name\] export_design 失败（综合报告仍然可用）：$err"
        }
    }
    close_solution
    close_project
    puts "==== \[$name\] 完成 ===="
}

set DO_PROJ 1
set DO_LR   1
set DO_PDHG 1
if {$WHICH eq "projection"} { set DO_LR 0; set DO_PDHG 0 }
if {$WHICH eq "lr"}         { set DO_PROJ 0; set DO_PDHG 0 }
if {$WHICH eq "pdhg"}       { set DO_PROJ 0; set DO_LR 0 }

# ---------------------------------------------------------------- 模式选择
# 顶层名与源文件列表按 TOP_MODE 切换。三种模式：
#
#   shell（默认）：综合**主机验证壳**（projection_engine_top / lr_forward_top /
#                  pdhg_core_top），即双精度层。csim 打开 —— 既有 tb.cpp 验的
#                  就是这一层。
#   axi          ：综合 **A6 的 AXI 接口层**（ps_*_axi）—— 块设计要的就是这一层。
#                  额外编入 ps_axi_top.cpp；**关掉 csim**，理由见下。
#   fixed        ：综合**定点层** —— **真正要上板的那一层**。
#                  ps_axi_top.h 文首自己写明：XCZU9EG 的 DSP48E2 是 27x18 乘法器，
#                  双精度浮点做不了，所以 shell/axi 两个模式的产物都上不了板。
#                  本模式改用 <核>_fixed.cpp + ps_fixed_top.cpp，建 4 个工程：
#                    proj_fix（ps_projection_fix）/ lr_fwd_fix（ps_lr_forward_fix）/
#                    lr_adj_fix（ps_lr_adjoint_fix）/ pdhg_fix（ps_pdhg_fix）
#                  低秩的前向与伴随**分开建两个工程** —— 一个工程只能有一个 top，
#                  而两者都是部署必须的。
#                  并行度由宏 PS_FIX_LANES 指定（ps_fixed_top.hpp，默认 16；
#                  三个 _fixed.cpp 都显式实例化了 8/16/32，可用 -DPS_FIX_LANES=8 覆盖）。
#   cal          ：**标定档**，只为拿「PL 耗时」。与 fixed 完全同一套源与顶层，
#                  差别只有两处：额外定义 -DPS_FIX_CAL_B（把 B 档尺寸钉成编译期
#                  常量，见 ps_fixed_top.cpp 的「标定档」注释），工程目录加 _cal 后缀
#                  （不覆盖上板用的那一套 IP）。产物只进报告，不进块设计。
#                  为什么需要它：生产档的 K/M/N/n_iter 是运行期 s_axilite 寄存器参数，
#                  HLS 无法静态定界循环，csynth 报的 Latency 是 2^28 次 maxbound 外推
#                  的噪声（X19 报告 §五）。钉成常量后同一份代码就能报出真实周期数。
#
# 为什么 axi 与 fixed 都要关掉 csim
# ----------------------------------------------------------------------------
# csim 只有在「tb 验的正好是被综合的那个函数」时才有意义。axi 顶层下既有 tb.cpp
# 验的是 *_top（不是 ps_*_axi）；fixed 顶层下定点层的测试是 test_*.cpp 与
# test_fixed_top.cpp（自带 main、不是 HLS 的 tb 形式）。两种情况跑了都会给人
# 「验过了」的错觉，宁可明说未验 —— 定点层的功能由主机侧测试负责（见 readme 第三节）。
proc ps_top {shell_top axi_top fix_top} {
    global TOP_MODE
    if {$TOP_MODE eq "axi"}   { return $axi_top }
    if {$TOP_MODE eq "fixed"} { return $fix_top }
    return $shell_top
}
proc ps_srcs {base} {
    global TOP_MODE AXI_SRC
    if {$TOP_MODE eq "axi"} { return [concat $base [list $AXI_SRC]] }
    return $base
}
if {$TOP_MODE eq "axi"} {
    set DO_CSIM 0
    puts "  顶层模式    = axi（块设计用；csim 已关闭，理由见脚本内注释）"
} elseif {$TOP_MODE eq "fixed" || $TOP_MODE eq "cal"} {
    set DO_CSIM 0
    puts "  顶层模式    = $TOP_MODE（定点层，真正上板的那一层；csim 已关闭，理由见脚本内注释）"
    puts "  并行度      = PS_FIX_LANES（ps_fixed_top.hpp，默认 16）"
    if {$TOP_MODE eq "cal"} {
        puts "  ★ 标定档    = -DPS_FIX_CAL_B（B 档尺寸钉成编译期常量），工程目录带 _cal 后缀"
    }
} else {
    puts "  顶层模式    = shell（主机验证壳；既有 tb 验的就是这一层）"
}

# ================================================================ fixed 模式：4 个定点顶层
# 源文件一律 = <核>_fixed.cpp + ps_fixed_top.cpp。后者是模板核的调用方，
# 不共编会把模板实例链不进来（三个 _fixed.cpp 各自显式实例化了 LANES = 8/16/32）。
# tb 一律为空 —— 理由见上面「为什么 axi 与 fixed 都要关掉 csim」。
if {$TOP_MODE eq "fixed" || $TOP_MODE eq "cal"} {
    # cal = fixed + -DPS_FIX_CAL_B + _cal 后缀（见文首 cal 模式说明）。
    if {$TOP_MODE eq "cal"} {
        set FIX_SUF    "_cal"
        set FIX_CFLAGS "$CFLAGS -DPS_FIX_CAL_B"
        # 标定分级：二分定位 pdhg 预综合在 Performance (step 3) 失败的那个常量组。
        # 含义见 ps_fixed_top.cpp 的 PS_FIX_CAL_LEVEL 注释；不设时默认 3（与既有行为一致）。
        #   set PS_HLS_CAL_LEVEL=1 && vitis-run --mode hls --tcl --input_file ...
        if {[info exists ::env(PS_HLS_CAL_LEVEL)] && $::env(PS_HLS_CAL_LEVEL) ne ""} {
            set FIX_CFLAGS "$FIX_CFLAGS -DPS_FIX_CAL_LEVEL=$::env(PS_HLS_CAL_LEVEL)"
            puts "  ★ 标定分级  = PS_FIX_CAL_LEVEL=$::env(PS_HLS_CAL_LEVEL)"
        }
    } else {
        set FIX_SUF    ""
        set FIX_CFLAGS $CFLAGS
    }
    set FIX_LR_SRCS [list \
        [file join $HLS_ROOT lr_operator lr_operator_fixed.cpp] \
        $FIX_SRC]

    if {$DO_PROJ} {
        ps_build_core \
            "proj_fix${FIX_SUF}" \
            "ps_projection_fix" \
            {} \
            [list [file join $HLS_ROOT projection_engine projection_engine_fixed.cpp] \
                  $FIX_SRC] \
            $FIX_CFLAGS
    }

    if {$DO_LR} {
        # 前向与伴随共用同一套源与 MAC 原语，但**必须各建一个工程**：
        # 工程只能有一个 top。
        ps_build_core "lr_fwd_fix${FIX_SUF}" "ps_lr_forward_fix" {} $FIX_LR_SRCS $FIX_CFLAGS
        ps_build_core "lr_adj_fix${FIX_SUF}" "ps_lr_adjoint_fix" {} $FIX_LR_SRCS $FIX_CFLAGS
    }

    if {$DO_PDHG} {
        # pdhg_core 依赖 lr_operator 的 Bf / Bᵀr，故两个 _fixed.cpp 一起编入。
        ps_build_core \
            "pdhg_fix${FIX_SUF}" \
            "ps_pdhg_fix" \
            {} \
            [list [file join $HLS_ROOT pdhg_core pdhg_core_fixed.cpp] \
                  [file join $HLS_ROOT lr_operator lr_operator_fixed.cpp] \
                  $FIX_SRC] \
            $FIX_CFLAGS
    }
} else {
# ================================================================ shell / axi 模式
# ---------------------------------------------------------------- 1 定位引擎
if {$DO_PROJ} {
    ps_build_core \
        "projection_engine" \
        [ps_top "projection_engine_top" "ps_projection_axi" ""] \
        [list [file join $HLS_ROOT projection_engine tb.cpp]] \
        [ps_srcs [list [file join $HLS_ROOT projection_engine projection_engine.cpp]]] \
        $CFLAGS
}

# ---------------------------------------------------------------- 2 低秩算子
if {$DO_LR} {
    ps_build_core \
        "lr_operator" \
        [ps_top "lr_forward_top" "ps_lr_forward_axi" ""] \
        [list [file join $HLS_ROOT lr_operator tb.cpp]] \
        [ps_srcs [list [file join $HLS_ROOT lr_operator lr_operator.cpp]]] \
        $CFLAGS
}

# ---------------------------------------------------------------- 3 PDHG 重建核
# pdhg_core 依赖 lr_operator 的 Bf / Bᵀr，故两个 .cpp 一起编入同一个工程。
if {$DO_PDHG} {
    ps_build_core \
        "pdhg_core" \
        [ps_top "pdhg_core_top" "ps_pdhg_axi" ""] \
        [list [file join $HLS_ROOT pdhg_core tb.cpp]] \
        [ps_srcs [list [file join $HLS_ROOT pdhg_core pdhg_core.cpp] \
                      [file join $HLS_ROOT lr_operator lr_operator.cpp]]] \
        $CFLAGS
}
}

puts "\n==== 全部完成（模式 $TOP_MODE）===="
if {$TOP_MODE eq "fixed"} {
    puts "     工程目录        ：proj_fix/  lr_fwd_fix/  lr_adj_fix/  pdhg_fix/"
} else {
    puts "     工程目录        ：projection_engine/  lr_operator/  pdhg_core/"
}
puts "                      （各自 $SOLUTION/ 下有 csynth 报告；export_design 失败时报告仍在）"
