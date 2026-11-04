# ============================================================================
# PenumbraSight 顶层构建入口
#
# 这里只做**编排**，不放具体规则：
#   - C/C++ 主机侧测试  → src/software/makefile
#   - HLS 三个核        → src/fpga/hls/scripts/build_hls.tcl（需要 Vitis）
#   - RTL 五个模块      → src/fpga/rtl/tb/*.sv（需要 iverilog 或 verilator）
#   - 模型生成          → src/model/penumbrasight_model/build_model.py
#
# 设计原则
# ---------------------------------------------------------------------------
# 1) `make test` 必须能在**只有 g++ 与 Python 的机器上全绿**，不碰 Vivado。
#    凡是要 Vivado / iverilog 的目标都单独命名，并给出明确的"环境缺失"提示
#    而不是静默失败。
# 2) **Windows 优先**。本仓库当前只在 Windows + GNU make（shell 为 cmd.exe）
#    上验证过，而 sh 式的 `for ... do ... done` 在 cmd 下会报
#    `core was unexpected at this time`。所以这里：
#      · 不使用 shell 循环，改用每个核一条独立规则；
#      · 不使用 `which` / `mkdir -p` / `rm -rf`，改用下面这组按 OS 切换的变量；
#      · 用 `&&` 串行是安全的（cmd 与 sh 都支持）。
#    改动本文件后请至少跑一次 `make test-hls` 确认没退化。
#
# 用法：make help
# ============================================================================

REPO_ROOT := $(CURDIR)
MODEL_DIR := src/model
SW_DIR    := src/software
RTL_DIR   := src/fpga/rtl
HLS_DIR   := src/fpga/hls
HW_DIR    := src/hardware
HWDATA_DIR := $(HW_DIR)/hwdata
CONF_DIR  := configs
# 全量模型（243 候选约 300 MB）**不入 Git**，落在 sim/models/ 而不是 configs/ ——
# configs/ 只留 YAML。见 sim/models/readme.md §一 与 docs/port_completeness.md §二。
SIM_MODELS_DIR := sim/models
BUILD_DIR := build
# U 落盘布局门禁（2026-09-29 新增）。「U 少一次转置」在形状 / payload CRC32 /
# u_lsb_error 三条既有校验下**全都通过**，只有专查「U 每列是否填满 + Gram≈单位阵」
# 才拦得住，故把它挂进回归与模型生成收尾。
PSM_LAYOUT_TOOL := build/scripts/verify_psm_u_layout.py
PSM_LAYOUT_FIXTURES := tests/vectors/model_b_r8_c3.psm tests/vectors/model_paper_r8_c3.psm

PYTHON ?= python
CXX    ?= g++

# ------------------------------------------------------------------ 平台适配
ifeq ($(OS),Windows_NT)
EXESUF   := .exe
# $(shell ...) 在 cmd 下的写法；`if not exist` 是 cmd 内建语法
PS_MKDIR := $(shell if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)")
PS_WHERE  = $(shell where $(1) 2>NUL)
PS_RMDIR := cmd /c rmdir /s /q
# **不要用 `cd src/model && python`** —— cmd 会把 `cd src/model` 里的 `/m` 当成开关，
# 直接报错退出（实测 exit code 2）。这里改成往环境里注入 PYTHONPATH，
# 让模块导入生效，从而完全不需要 cd。
PYENV    := set PYTHONPATH=src\model&&
# HLS 综合的核选择。Vitis 2025.2 的 vitis-run **不再接受 -tclargs**
# （实测 `unrecognised option '-tclargs'`），故改为经环境变量传给
# src/fpga/hls/scripts/build_hls.tcl（该脚本已加此入口，见其内注释）。
# ★ 必须用递归展开（`=`，不是 `:=`）：`$(1)` 只在 `call` 时才该求值。
#   写成 `:=` 会在**定义处**就把 `$(1)` 展成空，得到 `set PS_HLS_WHICH=&&`。
HLSENV   = set PS_HLS_WHICH=$(1)&&
else
EXESUF   :=
PS_MKDIR := $(shell mkdir -p "$(BUILD_DIR)")
PS_WHERE  = $(shell which $(1) 2>/dev/null)
PS_RMDIR := rm -rf
PYENV    := PYTHONPATH=src/model
HLSENV   = PS_HLS_WHICH=$(1)
endif

# 三个 HLS 核；含 pdhg 需要一起编 lr_operator（它依赖 Bf / Bᵀr）
HLS_CORES := projection_engine lr_operator pdhg_core
HLS_CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas \
                -Isrc/hardware -I$(SW_DIR)/include -I$(HLS_DIR)
LR_SRC := $(HLS_DIR)/lr_operator/lr_operator.cpp

.PHONY: help all test test-axi test-hls test-fixed test-fixed-lr test-fixed-exp test-fixed-pdhg \
        test-fixed-projection test-fixed-top test-hls-projection test-hls-lr test-hls-pdhg \
        test-preprocess test-x6x7 vectors-x6x7 \
        test-software test-board-fix test-system test-model test-model-unit verify-psm-u test-x14 test-x21 test-x4-align test-sd-frame model model-dry model-paper manifest \
        test-thresholds test-meas-bridge test-board-top test-camera-24h test-uio-entry \
        hls hls-projection hls-lr hls-pdhg hls-fixed hls-cal \
        synth-rtl rtl-sim rtl-sim-one rtl-xsim rtl-bitexact rtl-bitexact-iv rtl-full rtl-full-iv check-env check-tcl check-eng \
        archive clean distclean

# ------------------------------------------------------------------ 默认目标
help:
	@echo   CURRENT FULL MODEL: make m8 / m8-test / m8-verify / m8-workspace-check / m8-release-check
	@echo PenumbraSight 顶层入口
	@echo.
	@echo   交付物
	@echo     make model            生成 B 档全量 model.psm + manifest（243 候选，落 sim/models）
	@echo     make model-dry        只报网格规模，不真算
	@echo     make model-paper      生成论文档对照模型（315 候选 9x7x5，落 sim/models）
	@echo.
	@echo   测试（不需要 Vivado / 不需要板卡）
	@echo     make test             跑下面全套
	@echo     make test-hls         编译并运行 3 个 HLS 核的 tb.cpp（g++）
	@echo     make test-preprocess  跑 M1 预处理链 vs S0-3 测试向量（L1-1）
	@echo     make test-x6x7        跑 12 bit 双打包口径（X6/X7，含位宽核算）
	@echo     make test-software    跑 ARM 侧五进程的 host-test
	@echo     make test-board-fix  跑定点四核调用层主机自检（影子窗口，无板卡）
	@echo     make test-board-top  跑板端顶层应用主机自检（影子窗口：账本/清单/版本/最小用例/四核/帧缓冲）
	@echo     make test-uio-entry  跑 UIO 入口与连续物理内存分配单测（注入式后端；系统调用等板卡）
	@echo     make test-camera-24h 相机 24h 稳定性判据逻辑自检（接口+桩；真机取流项 PENDING）
	@echo     make test-meas-bridge 跑「24 位打包拍 → meas_norm → 定点核入参」接缝（影子窗口）
	@echo     make test-sd-frame   跑 SD 卡帧 .psf 读取层自检（入库 fixture；真实帧见 build/scripts/sd_frame_selftest.py）
	@echo     make test-system     跑全系统端到端验收 P1~P5（定位口径待定；硬件项 PENDING）
	@echo     make test-model       跑 PC 侧模型包的格式自检
	@echo     make test-model-unit  跑 src/model/tests 的 6 个模块单测（无需 pytest）
	@echo     make verify-psm-u     校验 tests/vectors 两个 fixture 的 U 落盘布局
	@echo     make test-x14         核对「修正后的 A」构造不变式（X14，判据 1e-12）
	@echo     make test-x21         核对交付定义判据表 ↔ 实现一致（X21）
	@echo     make test-x4-align    遮挡栅格化 ↔ MATLAB poly2mask 对齐（X4 收尾，判据 1e-6）
	@echo     make test-thresholds  RTL 阈值标定的算术自检（整数 E / 百分位 / Q0.16，无需数据）
	@echo     make check-tcl        Tcl/XDC 括号平衡（无 Vivado 时唯一的 tcl 检查）
	@echo     make check-eng        工程化一致性（时钟单一来源 / depth / 中断 / reset）
	@echo.
	@echo   向量
	@echo     make vectors-x6x7     重建 X6/X7 golden 向量（双打包口径，需 numpy）
	@echo.
	@echo   需要额外环境
	@echo     make hls              三个 HLS 核建工程+csim+csynth（需 Vitis HLS）
	@echo     make rtl-sim          五个 RTL 模块跑 testbench（需 iverilog）
	@echo     make rtl-xsim         五个 RTL 模块经 Vivado xsim 跑门禁（需 xsim；不传 --timescale）
	@echo     make rtl-bitexact     raw_preproc 与 Python 参考链 bit-exact 对拍（需 xsim，小尺寸）
	@echo     make rtl-bitexact-iv  同上，但用 iverilog（无 Vivado 时的交叉复核）
	@echo     make rtl-full        raw_preproc 全尺寸（1920x1920, 4 帧）xsim 跑通 + 全量逐值（需 xsim）
	@echo     make rtl-full-iv     同上，但用 iverilog
	@echo.
	@echo     make check-env        体检本机有哪些工具可用
	@echo     make clean            删 build/ 与各子目录中间产物

all: test

# ------------------------------------------------------------------ 环境体检
check-env:
	@echo ==== 环境体检 ====
	@echo python    : $(PYTHON)
	@echo g++       : $(CXX)
	@echo vitis-run : $(if $(call PS_WHERE,vitis-run),有,缺失 HLS 综合需要)
	@echo iverilog  : $(if $(call PS_WHERE,iverilog),有,缺失 RTL 仿真需要)
	@echo verilator : $(if $(call PS_WHERE,verilator),有,缺失 RTL 仿真备选)

# ------------------------------------------------------------------ Tcl / XDC 静态检查
# `src/fpga/scripts/readme.md` 写明：本机没有 Vivado，对块设计与约束脚本**只能**做
# 「花括号/方括号平衡」这一项检查。此前那句话不可复现（仓库里没有脚本），这里补上它
# 并挂进 make test —— 否则 2026-09-28 的器件迁移（KV260 -> XCZU9EG）若改坏括号
# 不会被任何本地检查发现。
# 覆盖：src/fpga/constraints/axu9eg.xdc、bd/penumbrasight_bd.tcl、
#       hls/scripts/build_hls.tcl。
check-tcl:
	@echo ==== Tcl / XDC 括号平衡检查 ====
	$(PYTHON) src/fpga/scripts/check_tcl_balance.py

# 工程化一致性（2026-10-02 新增）：把时钟单一来源、ps_axi_top 的 depth、
# 块设计中断通路、build_pl 的无条件 reset、死代码清理这几项做成文本层门禁。
# 纯静态、不碰 Vivado/Vitis，故可进 make test（见 src/fpga/scripts/check_eng_consistency.py）。
check-eng:
	@echo ==== 工程化一致性检查（时钟单一来源 / depth / 中断 / reset）====
	$(PYTHON) src/fpga/scripts/check_eng_consistency.py

# ------------------------------------------------------------------ 测试
test: test-frame-e2e check-env check-tcl check-eng test-axi test-hls test-fixed test-preprocess test-x6x7 test-software test-board-fix test-meas-bridge test-board-top test-camera-24h test-uio-entry test-system test-model test-model-unit verify-psm-u test-x14 test-x21 test-sd-frame test-thresholds
	@echo.
	@echo ==== 主机侧全套测试结束 ====

# ------------------------------------------------------------------ 定点层
# 这三个测试此前**从未接过线**：`test-hls` 编的是 tb.cpp（双精度层），
# 而定点层自己的 test_*.cpp 一直是手工编译运行的。补进来是因为定点层
# 才是最终上板的那一层（AXU9EGB 的 DSP48E2 做不了双精度浮点）。
#
# 其中 test_lr_operator 的 L7 段落是**增益表示的判据**：它量出 Q16.16 下
# 每候选自适应 scale 会取整到 0（真实 B 档模型实测 6/6 全为 0，差判据
# 2500 倍）。改增益表示之后这个测试必须仍然全绿。
FIXED_INC := -Isrc/software/include -I$(HLS_DIR)

test-fixed: test-fixed-lr test-fixed-exp test-fixed-pdhg test-fixed-projection test-fixed-top

# 定点层 csynth 顶层的**透明度**判据（2026-09-28 新增）。
# ps_fixed_top.cpp 只做「拼装结构体 -> 转调模板核 -> 逐字段回写」，数学一行都不重写，
# 所以顶层与「直接调用模板核」必须**逐值相同**（本测试 121 项断言）。接口层的 bug
# 只应该表现为「连错线」，不应该表现为「结果不对」—— 这条就是把它钉住。
# 它同时是 fixed 模式的回归保障：`make hls ... fixed` 综合的就是这几个顶层。
test-fixed-top: $(BUILD_DIR)
	@echo ---- 定点层 csynth 顶层（ps_fixed_top 透明度）----
	$(CXX) $(HLS_CXXFLAGS) $(HLS_DIR)/test_fixed_top.cpp $(HLS_DIR)/ps_fixed_top.cpp \
		$(HLS_DIR)/projection_engine/projection_engine_fixed.cpp \
		$(HLS_DIR)/lr_operator/lr_operator_fixed.cpp \
		$(HLS_DIR)/pdhg_core/pdhg_core_fixed.cpp \
		-o $(BUILD_DIR)/tb_fixed_top$(EXESUF)
	$(BUILD_DIR)/tb_fixed_top$(EXESUF)
	@echo ---- 定点层四个测试结束 ----

test-fixed-exp: $(BUILD_DIR)
	@echo ---- [定点] 增益指数路径比对（新路径的唯一执行证据）----
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas $(FIXED_INC) \
	    $(HLS_DIR)/lr_operator/lr_operator_fixed.cpp \
	    $(HLS_DIR)/lr_operator/test_lr_exp_gain.cpp \
	    -o $(BUILD_DIR)/test_lr_exp_gain$(EXESUF)
	$(BUILD_DIR)/test_lr_exp_gain$(EXESUF)

test-fixed-lr: $(BUILD_DIR)
	@echo ---- [定点] lr_operator：Bf / Bᵀ + 增益表示判据（L7）----
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas $(FIXED_INC) \
	    $(HLS_DIR)/lr_operator/lr_operator_fixed.cpp \
	    $(HLS_DIR)/lr_operator/test_lr_operator.cpp \
	    -o $(BUILD_DIR)/test_lr_operator$(EXESUF)
	$(BUILD_DIR)/test_lr_operator$(EXESUF)

test-fixed-pdhg: $(BUILD_DIR)
	@echo ---- [定点] pdhg_core：迭代/投影/饱和 ----
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas $(FIXED_INC) \
	    $(HLS_DIR)/pdhg_core/pdhg_core_fixed.cpp \
	    $(HLS_DIR)/lr_operator/lr_operator_fixed.cpp \
	    $(HLS_DIR)/pdhg_core/test_pdhg.cpp \
	    -o $(BUILD_DIR)/test_pdhg$(EXESUF)
	$(BUILD_DIR)/test_pdhg$(EXESUF)

test-fixed-projection: $(BUILD_DIR)
	@echo ---- [定点] projection_engine：打分/排序/位宽核算 ----
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas $(FIXED_INC) \
	    $(HLS_DIR)/projection_engine/projection_engine_fixed.cpp \
	    $(HLS_DIR)/projection_engine/test_projection.cpp \
	    -o $(BUILD_DIR)/test_projection$(EXESUF)
	$(BUILD_DIR)/test_projection$(EXESUF)

# A6 的接口层：把 ps_axi_top.cpp 用两个口径各编译一遍。
# **注意这只验语法与类型，真正的综合、时序、位宽可行性都要等 Vitis**
# （见 docs/hls_interface_spec.md §七 的验证状态表）。
test-axi: $(BUILD_DIR)
	@echo ==== [0/3] HLS 顶层 AXI 接口层（A6）编译检查 ====
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas -c $(HLS_DIR)/ps_axi_top.cpp -o $(BUILD_DIR)/ps_axi_host.o -I$(SW_DIR)/include
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas -D__SYNTHESIS__ -c $(HLS_DIR)/ps_axi_top.cpp -o $(BUILD_DIR)/ps_axi_syn.o -I$(SW_DIR)/include
	@echo ---- 主机口径与综合口径均编译通过 ----

# 每个核一条独立规则：cmd 下不能用 shell 循环。
# 每个 tb 自带自检，全通过返回 0、失败返回非 0，所以 make 会正确报错。
test-hls: test-hls-projection test-hls-lr test-hls-pdhg
	@echo ---- 三个 HLS 核全部通过 ----

test-hls-projection: $(BUILD_DIR)
	@echo ---- projection_engine ----
	$(CXX) $(HLS_CXXFLAGS) $(HLS_DIR)/projection_engine/projection_engine.cpp $(LR_SRC) $(HLS_DIR)/projection_engine/tb.cpp -o $(BUILD_DIR)/tb_projection_engine$(EXESUF)
	$(BUILD_DIR)/tb_projection_engine$(EXESUF)

test-hls-lr: $(BUILD_DIR)
	@echo ---- lr_operator ----
	$(CXX) $(HLS_CXXFLAGS) $(HLS_DIR)/lr_operator/lr_operator.cpp $(HLS_DIR)/lr_operator/tb.cpp -o $(BUILD_DIR)/tb_lr_operator$(EXESUF)
	$(BUILD_DIR)/tb_lr_operator$(EXESUF)

test-hls-pdhg: $(BUILD_DIR)
	@echo ---- pdhg_core ----
	$(CXX) $(HLS_CXXFLAGS) $(HLS_DIR)/pdhg_core/pdhg_core.cpp $(LR_SRC) $(HLS_DIR)/pdhg_core/tb.cpp -o $(BUILD_DIR)/tb_pdhg_core$(EXESUF)
	$(BUILD_DIR)/tb_pdhg_core$(EXESUF)

# ------------------------------------------------------------------ M1 预处理链
# 下面两项此前**从未接过线**：test_preprocess.cpp（L1-1，S0-3 向量的主机比对）
# 与 test_x6_x7_preprocess.cpp（X6/X7，12 bit 双打包口径），都要手工 g++ 编译
# 才跑得起来。接进 `make test` 的理由：M1 是整条链的入口，它的打包口径
# （packed / fullscale）直接决定下游 MAC 与 BRAM 宽度 —— 差 4 bit 不是精度问题，
# 是「差分通路够不够宽」的问题（见 sim/results/x6_x7_preprocess_bitdepth.md）。
#
# 向量目录用**命令行参数**传给测试程序，不用 -D 宏：Windows 下 cmd 会把
# `-DHWDATA_DIR="..."` 的两侧引号吃掉，宏展开成不带引号的裸路径（本项目踩过，
# 见 sim/results/l1_1_preprocess_hls.md 第四节）。
test-preprocess: $(BUILD_DIR)
	@echo ---- [M1] L1-1 预处理链 vs S0-3 测试向量（逐值）----
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas -I$(HW_DIR) $(HW_DIR)/test_preprocess.cpp -o $(BUILD_DIR)/test_preprocess$(EXESUF)
	$(BUILD_DIR)/test_preprocess$(EXESUF) $(HWDATA_DIR)

test-x6x7: $(BUILD_DIR)
	@echo ---- [X6/X7] 12 bit RGGB 解马赛克 + 分块求和，双打包口径 ----
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas -I$(HW_DIR) $(HW_DIR)/test_x6_x7_preprocess.cpp -o $(BUILD_DIR)/test_x6_x7_preprocess$(EXESUF)
	$(BUILD_DIR)/test_x6_x7_preprocess$(EXESUF) $(HWDATA_DIR)

# 向量是入库的裸二进制；若要重建（例如改了块大小或帧数）跑这一条。
# 它只依赖 numpy，不碰 Vivado。
vectors-x6x7:
	$(PYENV) $(PYTHON) $(MODEL_DIR)/scripts/make_x6_x7_vectors.py

test-software:
	@echo ==== ARM 侧五进程 host-test ====
	$(MAKE) -C $(SW_DIR) host-test

# RTL 预处理链输出到定点核输入之间的那级转换（共享指数量化 + 通道优先重排）。
# 与 quantize.py 逐位对拍；板端落位尚待实测，见 src/software/meas_norm/meas_norm.h。
test-meas-norm:
	@echo ==== meas_norm 逐位对拍 quantize.py ====
	$(MAKE) -C $(SW_DIR) test-meas-norm

# 把 meas_norm 接进板端流程之后的接缝自检（2026-09-30）：
# 「RTL 24 位打包拍 -> meas 缓冲 -> 帧量程折算 -> 投影核 job/寄存器」。
# 影子窗口 + 主机缓冲，无板卡；验的是接线与数值口径，不是 AXI 时序。
test-meas-bridge:
	@echo ==== 24 位打包拍 → meas_norm → 定点投影核入参 接缝 ====
	$(MAKE) -C $(SW_DIR) test-meas-bridge

# 定点四核调用层（board/fix_core 的调用方）主机自检：影子窗口，无板卡。
# 上板时序、AXI 实际搬运不在本目标范围（见 src/software/fpga_control/fix_core_driver.h）。
test-board-fix:
	@echo ==== 定点四核调用层主机自检（影子窗口）====
	$(MAKE) -C $(SW_DIR) test-fix-core

# 板端顶层应用（SD .psf -> DDR -> PL 预处理链 -> 四核 -> 结果回读 -> 帧缓冲 -> DP）的
# 主机自检。影子窗口 + 宿主 DDR，无板卡：验账本/布局/寄存器清单（与驱动逐条比对）/
# 版本读回三态/最小用例/四核串联/帧缓冲格式/.psf 装载。真实时序与 DP 输出等板卡。
test-board-top:
	@echo ==== 板端顶层应用主机自检（影子窗口）====
	$(MAKE) -C $(SW_DIR) test-board-top

# UIO 入口与连续物理内存分配（2026-10-01 新增）：ps_fix_window_open_uio 从留桩改成
# 标准 UIO 后端后的主机单测。设备访问全走注入后端，覆盖正常映射/设备缺失/权限不足/
# 映射长度不对/物理地址获取失败。**真 open/mmap/ioctl、设备树、内核配置项一律板端
# 实测**，本目标只证明逻辑分支，见 sim/results/uio_entry_20261001.md。
test-uio-entry:
	@echo ==== UIO 入口与连续物理内存分配单测（注入式后端）====
	$(MAKE) -C $(SW_DIR) test-uio-entry

# 相机 24h 稳定性与参数漂移（方案 §12 Task 10）。相机当前未联网、也不允许接入
# （先脱离相机做 SD 卡数据链验证），故设备访问做成接口 + 桩；这里跑的是**主机侧
# 判据逻辑自检**（合成时间轴 + 注入丢帧/断流/漂移场景，逐条断言判决）。真机 24h
# 取流、真机参数回读、温度对板端时序的影响一律 PENDING，不代跑。见
# tests/hardware/test_camera_24h.py 与 tests/hardware/readme.md。
test-camera-24h:
	@echo ==== 相机 24h 稳定性判据逻辑自检（接口+桩）====
	$(PYTHON) tests/hardware/test_camera_24h.py --self-check

# SD 卡帧（.psf）读取层：跑的是**入库 fixture**（384 字节，小几何），
# 因此不需要仓库外的论文数据集，可以进 make test。
# 真实帧的端到端比对（转换 -> 读回 -> 逐值一致 -> 块和与期望值比对）是
# build/scripts/sd_frame_selftest.py，它需要 .mat，故**不**进 make test：
#     python build/scripts/sd_frame_selftest.py --mat "<...>\image_test_colbar.mat"
test-sd-frame:
	@echo ==== SD 卡帧（.psf）读取层自检 ====
	$(MAKE) -C $(SW_DIR) test-sd-frame

# 全系统端到端验收（P1~P5）。无板卡/相机时硬件相关项判 PENDING、退出码仍为 0；
# 定位口径由 --loc-policy 显式给出（默认 undecided），见 tests/system/readme.md。
test-system:
	@echo ==== tests/system 端到端验收（P1~P5，定位口径待定）====
	$(PYENV) $(PYTHON) tests/system/test_end_to_end.py

test-model:
	@echo ==== PC 侧模型包格式自检 ====
	$(PYENV) $(PYTHON) -c "import penumbrasight_model.psm as m; assert m.HEADER_BYTES == 64, m.HEADER_BYTES; print('  psm.py 导入正常，头部字节数 =', m.HEADER_BYTES)"

# ------------------------------------------------------------------ 模型包 U 布局门禁（2026-09-29）
# 「U 少一次转置」这类错在形状 / payload CRC32 / u_lsb_error 三条既有校验下**全都通过**：
# 落盘后形状仍是 (rank_max, M)、CRC 算的是写出去的那份自洽字节、u_lsb_error 在转置前算。
# 只有专查「U 每列是否填满 + 归一化 Gram≈单位阵」才拦得住，故单列一条门禁。
# tests/vectors/ 的两个 fixture 入库，恒可校验；退出码非零即失败（可直接当门禁）。
verify-psm-u:
	@echo ==== U 落盘布局校验（rank-major / 每列非零 / Gram≈单位阵）====
	$(PYTHON) $(PSM_LAYOUT_TOOL) $(PSM_LAYOUT_FIXTURES)

# ------------------------------------------------------------------ 模型层单测（src/model/tests）
# 这 6 个文件都用 unittest 风格的自带 runner，可直接 `python 文件.py` 跑并返回退出码，
# **不依赖 pytest**（本机无 pytest）。此前只有 test_preprocess.py 且未接过线，故一起挂进 make test。
# 2026-09-30：补上 test_types.py（方案 §12 Task 1；此前是 port_completeness §11.6
# 记的最后一处文件级缺件）。
test-model-unit:
	@echo ==== 模型层单测（types 已单列 test_types）====
	$(PYTHON) src/model/tests/test_types.py
	$(PYTHON) src/model/tests/test_preprocess.py
	$(PYTHON) src/model/tests/test_forward.py
	$(PYTHON) src/model/tests/test_psm.py
	$(PYTHON) src/model/tests/test_localization.py
	$(PYTHON) src/model/tests/test_reconstruction.py

# ------------------------------------------------------------------ 阈值标定（P2 台账）
# RTL 的 8 个阈值输入（fast_alert 3 个 + confidence_engine 5 个）在上板前需要标定，
# 但仓库内**没有标定集**（见 src/fpga/rtl/readme.md D1~D3 与本次报告 §任务3）。
# 这里挂进 make test 的只是**算术自检**：整数 floor 的 E、最近秩百分位、Q0.16/Q0.8
# 编码 —— 与是否有真数据无关，恒可跑，且把「用浮点百分位直接写进 cfg_e_thresh」
# 这类口径错误钉住。真标定用 `python src/model/scripts/calibrate_thresholds.py ...`。
test-thresholds:
	@echo ==== 阈值标定算术自检（整数 E / 最近秩百分位 / Q0.16 编码）====
	$(PYTHON) src/model/scripts/calibrate_thresholds.py --self-test

# ------------------------------------------------------------------ X14 / X21（台账 P2）
# X14 挂到 make test 的只有**不变式**这一条：`比值法构造的 no-mirror A` 必须与
# 「整列直接重算」逐列一致（判据 1e-12）。它是 X14 报告里那个「修正后的 A」的
# 可信度前提。重建本身（86 s、需 132 MB 的 sim/vectors/paper_A_va1.npy）不进 make test，
# 见 sim/results/x14_recon_corrected.md；缺向量时脚本 SKIP（返回 0）。
# 注意本目标**只读** sim/vectors，不写任何东西。
test-x14:
	@echo ==== X14 「修正后的 A」构造不变式 ====
	$(PYTHON) src/model/scripts/x14_recon_corrected_a.py --identity-only

# X21 的判据是「判据表与实现一致」，所以它本身必须可机判：核对
#   交付定义文档（工作区 01_分析报告/07_单人移植任务表/）的判据表
#   ↔ 本脚本里的实现侧契约 CRITERIA
#   ↔ build/scripts/acceptance_first_version.py 生成的 sim/results/x13_acceptance.md
# 定义文档在仓库之外；裸 clone 时脚本报 SKIP 并返回 0，不让 make test 变红。
test-x21:
	@echo ==== X21 交付定义判据表 ↔ 实现 一致性 ====
	$(PYTHON) build/scripts/x21_criteria_consistency.py

# ------------------------------------------------------------------ X4 收尾（遮挡栅格化对齐）
# 判据：新口径（判定点 = 整数像素中心，even-odd —— 即 MATLAB `poly2mask` 语义）与
# MATLAB 亲手导出的 `simA` 相对 Frobenius 误差 ≤ 1e-6。
# **实测 2.146e-16（va0）/ 3.338e-16（va1）**，非零元集合与参照完全相同；
# 旧口径（墙采样点 + 闭区间）在同一驱动上给出 4.568e-2 / 5.153e-2，作 A/B 对照。
# 见 sim/results/x4_poly2mask_alignment.md。
#
# 参照量（MATLAB 的 ref_out/raw_chair_va*__simA.f64）在仓库之外，用环境变量给路径：
#     PowerShell:  $env:PS_DATA_ROOT='<...>\Ordinary-Camera-master\Data'
#                  $env:PS_REF_DIR='<...>\Ordinary-Camera-master\ref_out'
# 未给路径或缺文件时脚本打印 SKIP 并返回 0（与 test-x14 / test-x21 同一约定），
# 所以本目标在裸 clone 上也不会让构建变红。
# **本目标只读**：不往 sim/results 或任何地方写文件（报告是手写的）。
# 不进 `make test` 的原因：全量要 ~60 s，且强依赖仓库外的论文数据集。
test-x4-align:
	@echo ==== X4 收尾 遮挡栅格化 ↔ MATLAB poly2mask 对齐（判据 1e-6）====
	$(PYTHON) src/model/scripts/x4_poly2mask_alignment.py \
	    --data-root "$(PS_DATA_ROOT)" --raw-dir "$(PS_REF_DIR)"

# ------------------------------------------------------------------ 模型生成
# 输出落在 sim/models/（不是 configs/）：全量模型 ~300 MB，按方案 §10 不入 Git；
# configs/ 只留 YAML。manifest 是入库物，与模型同目录。
# 注：build_model.py 自己也算几何哈希，所以生成前必须先有
#     configs/geometry_b.yaml（B 档）/ configs/geometry_paper.yaml（论文档），
#     缺哪个文件它会**按当前档位**点名报出来（缺文件会退化为哈希 0）。
model-dry:
	$(PYENV) $(PYTHON) -m penumbrasight_model.build_model --geometry b --dry-run --out "$(REPO_ROOT)/$(SIM_MODELS_DIR)/model_b.psm"

model:
	@echo ==== 生成 B 档全量模型（243 候选）====
	$(PYENV) $(PYTHON) -u -m penumbrasight_model.build_model --geometry b --out "$(REPO_ROOT)/$(SIM_MODELS_DIR)/model_b.psm" --manifest "$(REPO_ROOT)/$(SIM_MODELS_DIR)/model_b.manifest.json"
	@echo ---- 收尾：U 落盘布局校验（生成器内置，这里再显式跑一次便于一眼看结果）----
	$(PYTHON) $(PSM_LAYOUT_TOOL) "$(REPO_ROOT)/$(SIM_MODELS_DIR)/model_b.psm"
	@echo ==== 完成，见 $(SIM_MODELS_DIR)/model_b.manifest.json ====

model-paper:
	@echo ==== 生成论文档对照模型（315 候选 9x7x5 / 墙网格 63x63）====
	$(PYENV) $(PYTHON) -u -m penumbrasight_model.build_model --geometry paper --out "$(REPO_ROOT)/$(SIM_MODELS_DIR)/model_paper.psm" --manifest "$(REPO_ROOT)/$(SIM_MODELS_DIR)/model_paper.manifest.json"
	@echo ---- 收尾：U 落盘布局校验（生成器内置，这里再显式跑一次便于一眼看结果）----
	$(PYTHON) $(PSM_LAYOUT_TOOL) "$(REPO_ROOT)/$(SIM_MODELS_DIR)/model_paper.psm"
	@echo ==== 完成，见 $(SIM_MODELS_DIR)/model_paper.manifest.json ====

manifest:
	@echo ---- $(CONF_DIR) ----
	-@dir /b $(CONF_DIR)\*.manifest.json 2>NUL
	@echo ---- $(SIM_MODELS_DIR) ----
	-@dir /b $(SIM_MODELS_DIR)\*.manifest.json 2>NUL

# ------------------------------------------------------------------ 需要 Vivado
# 方案 §三：中文用户名与中文路径必须在装 Vivado 之前解决，装完再改要重装。
# 见 docs/port_completeness.md §九。
# HLS 综合。⚠ 2026-09-28 在 Vitis 2025.2 上实测到三条硬约束：
#   1) **必须在纯 ASCII 路径下执行**。Vitis 2025.2 打不开中文路径下的 Tcl 脚本，
#      报 `Sourcing Tcl script '<path>' ... can't be opened.`。本仓库默认位于
#      「26秋嵌入式/PenumbraSight工作区」下，**直接跑必然失败**；
#      请从 ASCII 构建区执行（见 docs/environment_ascii_build.md）。
#   2) vitis-run **不接受 `-tclargs`**（`unrecognised option`），核选择改走
#      PS_HLS_WHICH 环境变量（见 HLSENV 与 build_hls.tcl 内注释）。
#   3) `--tcl <file>` 的参数语义与旧版不同（会把 `tcl` 当脚本参数传进 $argv），
#      故改用文档化的 `--input_file <file>`。
hls:
	@$(if $(call PS_WHERE,vitis-run),,$(error 找不到 vitis-run：HLS 综合需要 Vitis HLS。主机侧替代验证请跑 make test-hls))
	$(call HLSENV,all)vitis-run --mode hls --tcl --input_file $(HLS_DIR)/scripts/build_hls.tcl

hls-projection:
	@$(if $(call PS_WHERE,vitis-run),,$(error 找不到 vitis-run))
	$(call HLSENV,projection)vitis-run --mode hls --tcl --input_file $(HLS_DIR)/scripts/build_hls.tcl

hls-lr:
	@$(if $(call PS_WHERE,vitis-run),,$(error 找不到 vitis-run))
	$(call HLSENV,lr)vitis-run --mode hls --tcl --input_file $(HLS_DIR)/scripts/build_hls.tcl

hls-pdhg:
	@$(if $(call PS_WHERE,vitis-run),,$(error 找不到 vitis-run))
	$(call HLSENV,pdhg)vitis-run --mode hls --tcl --input_file $(HLS_DIR)/scripts/build_hls.tcl

# 定点层（**真正要上板的那一层**）的 4 个工程：proj_fix / lr_fwd_fix / lr_adj_fix / pdhg_fix。
# 默认模式是 shell，所以这里必须显式带上 PS_HLS_TOP_MODE=fixed。
hls-fixed:
	@$(if $(call PS_WHERE,vitis-run),,$(error 找不到 vitis-run))
	set PS_HLS_WHICH=all&& set PS_HLS_TOP_MODE=fixed&& vitis-run --mode hls --tcl --input_file $(HLS_DIR)/scripts/build_hls.tcl
	@echo ---- 定点层结束：proj_fix / lr_fwd_fix / lr_adj_fix / pdhg_fix ----

# PL 耗时标定档（2026-09-28 新增，X19）。与 hls-fixed **同一套源与顶层**，只多
# -DPS_FIX_CAL_B（把 B 档尺寸钉成编译期常量，见 ps_fixed_top.cpp 的「标定档」注释）
# 与 _cal 工程后缀（不覆盖上板用的那四个 IP）。
# 为什么需要它：生产档的 K/M/N/n_iter 是**运行期 s_axilite 寄存器参数**，HLS 无法
# 静态定界循环，csynth 报的 Latency 是 2^28 次 maxbound 外推的噪声（X19 报告 §五）。
# 钉成常量后同一份代码就能报出真实周期数：
#     PL 耗时 = 报告里的 Latency(cycles) × 10 ns      （100 MHz）
# 产物只进报告，不进块设计。
hls-cal:
	@$(if $(call PS_WHERE,vitis-run),,$(error 找不到 vitis-run))
	set PS_HLS_WHICH=all&& set PS_HLS_TOP_MODE=cal&& vitis-run --mode hls --tcl --input_file $(HLS_DIR)/scripts/build_hls.tcl
	@echo ---- 标定档结束：proj_fix_cal / lr_fwd_fix_cal / lr_adj_fix_cal / pdhg_fix_cal ----

# ------------------------------------------------------------------ Vivado 综合（需要 Vivado）
# RTL 五模块的资源与时序。这是 src/fpga/rtl/readme.md 第 6 节 E1/E2 的空缺，
# 也是 X21 附加项 9「M1 有资源与时序」此前唯一的空白。
# 器件与周期与 build_hls.tcl / penumbrasight_bd.tcl / axu9eg.xdc 同源：
# xczu9eg-ffvb1156-2-i / 10.0 ns。周期由 src/fpga/scripts/clock_plan.tcl 单一定义
# （2026-10-02 起），故这里**不再把 10.0 作为 -tclargs 传进去**（传了反而掩盖
# 「单一来源」，见 clock_plan.tcl 文首）。扫周期用 PS_PL_CLK_PERIOD_NS 覆盖。
# 走 `-mode batch` 非工程模式（不建 xpr），报告落在 build/synth/ 且**要入库**
# （.gitignore 已放行 build/synth/*.rpt 与 *.md）。
synth-rtl: $(BUILD_DIR)
	@$(if $(call PS_WHERE,vivado),,$(error 找不到 vivado：RTL 综合需要 Vivado，加进 PATH 后重试))
	vivado -mode batch -nojournal -nolog -source src/fpga/scripts/synth_rtl_probe.tcl
	@echo ---- RTL 综合结束：报告见 build/synth/ ----

# ------------------------------------------------------------------ 需要 iverilog
# 五个模块共约 100 KB RTL，**从未执行过**（见 docs/port_completeness.md §九）。
# 首轮大概率有实质 bug，别把这一步的产出当已验收。
rtl-sim-one: $(BUILD_DIR)
	@$(if $(M),,$(error 用法：make rtl-sim-one M=vertical_diff))
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog：RTL 仿真需要 Icarus Verilog))
	iverilog -g2012 -o $(BUILD_DIR)/rtl_$(M).vvp $(RTL_DIR)/$(M).sv $(RTL_DIR)/tb/$(M)_tb.sv
	vvp $(BUILD_DIR)/rtl_$(M).vvp

# 五个模块一起跑 —— 每个一条独立规则，避免 shell 循环
# 2026-09-30 追加两个模块（G2 帧末 tlast 整形 / G1+G4 寄存器叶）：它们是**纯 Verilog
# `.v`**（要被块设计当 module reference 引用，Vivado 不接受 SV 顶层，见
# src/fpga/rtl/pl_wrap/bd_raw_preproc.v 的文件头），故源文件后缀与上面五个不同。
# 2026-09-30 收口 G5 时再追加 sat_ratio_div.v（饱和比例除法器，同上为纯 Verilog）。
# 2026-10-01 追加 compositor_tready（G3 反压专项，小而快；见其 TB 文件头）。
rtl-sim: rtl-sim-raw_preproc rtl-sim-vertical_diff rtl-sim-compositor rtl-sim-confidence_engine rtl-sim-fast_alert \
         rtl-sim-axis_frame_sync rtl-sim-pl_ctrl_regs rtl-sim-sat_ratio_div rtl-sim-compositor_tready
	@echo ---- 九个 RTL 测试结束 ----

rtl-sim-raw_preproc: $(BUILD_DIR)
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog))
	iverilog -g2012 -o $(BUILD_DIR)/rtl_raw_preproc.vvp $(RTL_DIR)/raw_preproc.sv $(RTL_DIR)/tb/raw_preproc_tb.sv
	vvp $(BUILD_DIR)/rtl_raw_preproc.vvp

rtl-sim-vertical_diff: $(BUILD_DIR)
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog))
	iverilog -g2012 -o $(BUILD_DIR)/rtl_vertical_diff.vvp $(RTL_DIR)/vertical_diff.sv $(RTL_DIR)/tb/vertical_diff_tb.sv
	vvp $(BUILD_DIR)/rtl_vertical_diff.vvp

rtl-sim-compositor: $(BUILD_DIR)
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog))
	iverilog -g2012 -o $(BUILD_DIR)/rtl_compositor.vvp $(RTL_DIR)/compositor.sv $(RTL_DIR)/tb/compositor_tb.sv
	vvp $(BUILD_DIR)/rtl_compositor.vvp

rtl-sim-confidence_engine: $(BUILD_DIR)
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog))
	iverilog -g2012 -o $(BUILD_DIR)/rtl_confidence_engine.vvp $(RTL_DIR)/confidence_engine.sv $(RTL_DIR)/tb/confidence_engine_tb.sv
	vvp $(BUILD_DIR)/rtl_confidence_engine.vvp

rtl-sim-fast_alert: $(BUILD_DIR)
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog))
	iverilog -g2012 -o $(BUILD_DIR)/rtl_fast_alert.vvp $(RTL_DIR)/fast_alert.sv $(RTL_DIR)/tb/fast_alert_tb.sv
	vvp $(BUILD_DIR)/rtl_fast_alert.vvp

# 2026-09-30 新增（G2 / G1+G4，源文件是 .v）
rtl-sim-axis_frame_sync: $(BUILD_DIR)
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog))
	iverilog -g2012 -o $(BUILD_DIR)/rtl_axis_frame_sync.vvp $(RTL_DIR)/axis_frame_sync.v $(RTL_DIR)/tb/axis_frame_sync_tb.sv
	vvp $(BUILD_DIR)/rtl_axis_frame_sync.vvp

# pl_ctrl_regs 内部实例化 sat_ratio_div（G5 收口），故必须把后者一起编进来。
rtl-sim-pl_ctrl_regs: $(BUILD_DIR)
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog))
	iverilog -g2012 -o $(BUILD_DIR)/rtl_pl_ctrl_regs.vvp $(RTL_DIR)/pl_ctrl_regs.v $(RTL_DIR)/sat_ratio_div.v $(RTL_DIR)/tb/pl_ctrl_regs_tb.sv
	vvp $(BUILD_DIR)/rtl_pl_ctrl_regs.vvp

rtl-sim-sat_ratio_div: $(BUILD_DIR)
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog))
	iverilog -g2012 -o $(BUILD_DIR)/rtl_sat_ratio_div.vvp $(RTL_DIR)/sat_ratio_div.v $(RTL_DIR)/tb/sat_ratio_div_tb.sv
	vvp $(BUILD_DIR)/rtl_sat_ratio_div.vvp

# G3 反压专项（2026-10-01）：tready 拉低时整条光栅逐拍冻结、拉高后不丢拍。
# 只在 iverilog 侧挂牌：TB 名与核名不同（compositor_tready_tb 验 compositor），
# xsim 门禁驱动按「<module>.sv + <module>_tb.sv」找文件，故不另立 xsim 门禁；
# xsim 侧由 compositor 的金门禁（tready 恒 1）覆盖该端口的 elaborate。
rtl-sim-compositor_tready: $(BUILD_DIR)
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog))
	iverilog -g2012 -o $(BUILD_DIR)/rtl_compositor_tready.vvp $(RTL_DIR)/compositor.sv $(RTL_DIR)/tb/compositor_tready_tb.sv
	vvp $(BUILD_DIR)/rtl_compositor_tready.vvp

# ------------------------------------------------------------------ 需要 Vivado xsim
# 四个 test_*.py（src/fpga/sim/）用 xsim 驱动 RTL 五模块，严格串行。
# 与 rtl-sim（iverilog）并列，属 Vivado 口径；四个脚本**不传 --timescale**，
# 这本身就是「RTL 已补 timescale、xsim 不再需要命令行补默认值」的验证。
# 每条 recipe 一行，任一非零即中止 —— 可直接当门禁。
# 需 Vivado，故**不进** make test（make test 必须能在只有 g++/Python 的机器上全绿）。
rtl-xsim:
	@$(if $(call PS_WHERE,xelab),,$(error 找不到 xelab：xsim 门禁需要 Vivado，确保 D:\vivado\2025.2\Vivado\bin 在 PATH))
	$(PYTHON) src/fpga/sim/test_raw_preproc.py
	$(PYTHON) src/fpga/sim/test_vertical_diff.py
	$(PYTHON) src/fpga/sim/test_fast_alert.py
	$(PYTHON) src/fpga/sim/test_confidence.py
	$(PYTHON) src/fpga/sim/test_axis_frame_sync.py
	$(PYTHON) src/fpga/sim/test_pl_ctrl_regs.py
	$(PYTHON) src/fpga/sim/test_sat_ratio_div.py

# ------------------------------------------------------------------ RTL <-> Python bit-exact（需要仿真器）
# RTL readme 的 B4：五个 RTL 模块此前只与各自 TB 的期望值比对（TB 自证），
# 没有与 src/model 的 Python 参考链逐向量对拍。这里给出**最小可行**的一条：
# 用纯文本中间格式（输入 = Python 写、RTL 读；输出 = RTL 写、Python 读）把同一
# 输入分别喂给 RTL 仿真与 Python，逐值严格相等。当前只覆盖 raw_preproc，
# 规模是小尺寸（WALL_N=4/BLOCK=2 -> 16x16），**不是**全尺寸；局限见
# sim/results/preboard_test_gap_20260930.md §任务2。
# --tool iverilog 时只要 iverilog，不需要 Vivado（用于无 license 的机器交叉复核）。
rtl-bitexact:
	@echo "==== raw_preproc RTL <-> Python 参考链 bit-exact（小尺寸）===="
	$(PYTHON) src/fpga/sim/test_raw_preproc_bitexact.py --tool xsim

rtl-bitexact-iv:
	@echo "==== raw_preproc RTL <-> Python 参考链 bit-exact（iverilog 交叉复核）===="
	$(PYTHON) src/fpga/sim/test_raw_preproc_bitexact.py --tool iverilog

# ------------------------------------------------------------------ raw_preproc 全尺寸（需要仿真器）
# 补 `xsim_first_run_20260929.md` §8 / `pl_t2_rebuild_and_seam_20260929.md` §七 第 2 条
# 的「全尺寸（1920x1920）xsim 未跑」。B 档几何：BLOCK=10 / WALL_N=96 -> RAW_SIDE=1920，
# 4 帧共 14,745,600 个像素，输出 96x96x3 = 27,648 拍**全量逐值**比对。
# 本机实测（2026-09-30）：xsim 墙钟 57 s、峰值 RSS 215 MB、仿真 148.5 ms（1.48e8 ns）
# —— 可行，无需分块/抽帧替代方案。数字与命令见
# sim/results/preboard_test_gap_20260930.md §任务6。
rtl-full:
	@$(if $(call PS_WHERE,xelab),,$(error 找不到 xelab：全尺寸 xsim 需要 Vivado))
	@$(PS_MKDIR)
	cd $(BUILD_DIR) && xvlog -sv ../src/fpga/rtl/raw_preproc.sv ../src/fpga/rtl/tb/raw_preproc_full_tb.sv
	cd $(BUILD_DIR) && xelab raw_preproc_full_tb -s sim_full
	cd $(BUILD_DIR) && xsim sim_full -R

rtl-full-iv:
	@$(if $(call PS_WHERE,iverilog),,$(error 找不到 iverilog))
	@$(PS_MKDIR)
	iverilog -g2012 -o $(BUILD_DIR)/rtl_raw_preproc_full.vvp $(RTL_DIR)/raw_preproc.sv $(RTL_DIR)/tb/raw_preproc_full_tb.sv
	vvp $(BUILD_DIR)/rtl_raw_preproc_full.vvp

# ------------------------------------------------------------------ 目录与清理
$(BUILD_DIR):
	@$(PS_MKDIR)

archive:
	@echo 已归档的并行实现见 _archive/ 目录；说明见其中的 readme.md

clean:
	-$(MAKE) -C $(SW_DIR) clean
	$(PYENV) $(PYTHON) -c "import glob,os,shutil;bd='$(BUILD_DIR)';keep={'scripts','reports'};rm=[os.remove(p) for p in glob.glob(os.path.join(bd,'*')) if os.path.isfile(p)];dd=[shutil.rmtree(d) for d in glob.glob(os.path.join(bd,'*')) if os.path.isdir(d) and os.path.basename(d) not in keep];print('[clean] build/ 生成物已清（scripts/ 与 reports/ 保留）')"

# distclean 除了中间产物，连生成的模型一起清掉。
# 只删**模型二进制**（`.psm`，不入 Git）：manifest 是入库的交付物，保留。
# configs\ 那两行是历史遗留路径（X29 起模型改落 sim/models\）；保留它们只是
# 让旧工作区里遗留的文件也能被清掉，不做别的事。
# 注意：tests/vectors/*.psm 是入库的 fixture，**不在这里删**。
distclean: clean
	-del /q $(SIM_MODELS_DIR)\model_b.psm $(SIM_MODELS_DIR)\model_paper.psm 2>NUL
	-del /q $(CONF_DIR)\model_b.psm $(CONF_DIR)\model_paper.psm 2>NUL
	@echo 已删除生成的模型（$(SIM_MODELS_DIR)\ / $(CONF_DIR)\）；manifest 与 fixture（tests/vectors/）保留

.PHONY: test-frame-e2e
test-frame-e2e:
	$(PYTHON) build/scripts/preboard_frame_e2e.py

# Current complete FP64 model. Sources are assembled from the module manifest.
# These targets never program or access a board. Old targets remain for regression.
M8_ARGS ?=
.PHONY: m8 m8-host m8-arm m8-test m8-verify m8-prepare-eda m8-release-check m8-workspace-check
m8:
	$(PYTHON) -B build/scripts/full_model.py build --target all $(M8_ARGS)
m8-host:
	$(PYTHON) -B build/scripts/full_model.py build --target host $(M8_ARGS)
m8-arm:
	$(PYTHON) -B build/scripts/full_model.py build --target arm $(M8_ARGS)
m8-test:
	$(PYTHON) -B build/scripts/full_model.py test $(M8_ARGS)
m8-verify:
	$(PYTHON) -B build/scripts/full_model.py verify $(M8_ARGS)
m8-prepare-eda:
	$(PYTHON) -B build/scripts/full_model.py prepare-eda $(M8_ARGS)
m8-release-check:
	$(PYTHON) -B build/scripts/full_model.py release-check $(M8_ARGS)

m8-workspace-check:
	$(PYTHON) -B build/scripts/full_model.py workspace-check $(M8_ARGS)
