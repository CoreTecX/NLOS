# ============================================================================
# PenumbraSight —— PS 侧（ARM/Linux 用户态）顶层 Makefile
#
# 方案 §7 的五条进程与其模块：
#   camera_service  取流（Spinnaker/GenTL；主机 file / synthetic）
#   model_loader    model.psm 校验 + 装入 DDR
#   fpga_control    §8.2 AXI-Lite 寄存器读写、提交 buffer、等中断
#   telemetry       帧率/曝光/温度/功耗/候选分数/残差/迭代 -> CSV
#   ui_server       固定位置|自定位、灰度|RGB、1/4/8 帧、性能页
#
# 同一份源码两处使用
# ---------------------------------------------------------------------------
#   主机自检（本 Makefile 的默认路径）：
#       gcc/g++ 8.1 (MinGW) + Python3(含 numpy)
#       make host-test     # 编译全部模块 + 跑自检 + 跑两个既有契约测试
#   板端（AXU9EGB Linux ARM64）：
#       把 CC/CXX 换成 aarch64-linux-gnu-gcc/g++（或板上原生 gcc），
#       其余不变；桩件后端（fpga_control 的影子寄存器、camera_service 的
#       file/synthetic）在板上照样能编译，但 fpga_control 打开 /dev/uioN
#       会返回 Unavailable —— 那是留给板端 TODO 的接口，不是「能跑」的意思。
#
# 为什么模块 .c 用 $(CC) 而自检 .cpp 用 $(CXX)
# ---------------------------------------------------------------------------
#   五个模块是**纯 C**（对外 API 都是 extern "C" + POD），板端/主机都能用 C 编译，
#   这样交叉编译到 aarch64 时依赖最少。host_selftest.cpp 及两个既有测试是 C++
#   （它们要复用仓库既有的 ps_contract.hpp / ps_platform.hpp 做交叉比对）。
#   两条链的告警口径一致：-Wall -Wextra，目标零警告。
#
# 常用目标
# ---------------------------------------------------------------------------
#   make            编译全部模块 + 自检 + 两个契约测试（不跑）
#   make modules    只编译模块（bin/libps_software.a）
#   make host-test  编译并**运行**自检与两个契约测试（CI 入口）
#   make selftest   只编译自检
#   make test-x12 / make test-psm-format   单独跑某个契约测试
#   make clean      清理 bin/ 与 ps_tmp/ 下的生成物
#   make help       目标一览
# ============================================================================

# ---------------------------------------------------------------- 工具链

# 用 `=`（而不是 `?=`）显式赋值：某些环境会把 CC 预置成 `cc`（本机就是，
# 而 Windows 上没有 `cc` 这个名字），`?=` 会让它生效并导致 "CreateProcess failed"。
# 需要换工具链时在命令行覆盖：make CC=aarch64-linux-gnu-gcc CXX=aarch64-linux-gnu-g++
CC       = gcc
CXX      = g++
AR       = ar
PYTHON   = python

# 主机的 ANSI stdio：MinGW 的 printf 在 -std=cXX 严格模式下会退化成 msvcrt 的
# 老实现，%llu / %zu 可能打印错乱。打开后走 MinGW 自己的实现（板上 glibc 无此项）。
ifeq ($(OS),Windows_NT)
CC_ANSI_STDIO   := -D__USE_MINGW_ANSI_STDIO=1
EXESUF          := .exe
else
CC_ANSI_STDIO   :=
EXESUF          :=
endif

# ---------------------------------------------------------------- 目录/标志

BIN_DIR  := bin
TMP_DIR  := ps_tmp

OPT      = -O2
# -Wno-unknown-pragmas：仓库的 HLS 源码（../hardware/*.hpp）里有 `#pragma HLS ...`，
# gcc/g++ 不认识这类 pragma，-Wall 会对每一条都报 unknown-pragmas。
# 那些 pragma 是给 Vitis HLS 的，不是错误，故只关掉这一条，其余告警一条不少。
WARN     = -Wall -Wextra -Wno-unknown-pragmas

# 每个模块一个 -I：模块头文件与其 .c 放在同一目录（板端交叉编译时目录结构不变），
# 自检与测试因此可以 #include "camera_service.h" 这样直接写模块名。
INCLUDES = -Iinclude -I. -Imodel_loader -Ifpga_control -Icamera_service \
           -Itelemetry -Iui -Imeas_norm -Isd_frame -Iboard_app \
           -I../hardware -I../../board

# 同样用 `=`：若环境里已存在 CFLAGS，`?=` 会让本工程的 -std/-Wall -Wextra 全部失效。
# 需要追加/覆盖就在命令行给：make CFLAGS="..." CXXFLAGS="..."
CFLAGS   = -std=c11   $(WARN) $(OPT) $(INCLUDES) $(CC_ANSI_STDIO)
CXXFLAGS = -std=c++17 $(WARN) $(OPT) $(INCLUDES) $(CC_ANSI_STDIO)

# ---------------------------------------------------------------- 模块

# 五个模块目录里的全部 .c（新增模块只要放进这些目录就自动进构建）。
# 追加 board/fix_core.c：定点四核的寄存器层（零点调用方），由 fpga_control 的
# fix_core_driver.c 调用。放在这里是为了 host_selftest 链接 MODULE_OBJ 时其符号
# 可达（否则 fix_core_driver.o 会是未解析引用）。
# 追加 board_app/*.c：板端顶层应用核心（账本/布局/清单/版本/最小用例/四核/帧缓冲）。
# 注：board_app/board_main.c 的 main 被 PS_BOARD_APP_MAIN 包住，主机构建下它只出
# 类型声明，不会与 host_selftest 的 main 冲突（见该文件头）。
MODULE_SRC := $(wildcard model_loader/*.c) \
              $(wildcard fpga_control/*.c) \
              $(wildcard meas_norm/*.c) \
              $(wildcard sd_frame/*.c) \
              $(wildcard camera_service/*.c) \
              $(wildcard telemetry/*.c) \
              $(wildcard ui/*.c) \
              $(wildcard board_app/*.c) \
              ../../board/fix_core.c
MODULE_OBJ := $(addprefix $(BIN_DIR)/,$(notdir $(MODULE_SRC:.c=.o)))
MODULE_LIB := $(BIN_DIR)/libps_software.a

VPATH := model_loader:fpga_control:meas_norm:sd_frame:camera_service:telemetry:ui:board_app:../../board

SELFTEST_BIN := $(BIN_DIR)/host_selftest$(EXESUF)
TPF_BIN      := $(BIN_DIR)/test_psm_format$(EXESUF)
T12_BIN      := $(BIN_DIR)/test_x12_contract$(EXESUF)
FIXTEST_BIN  := $(BIN_DIR)/test_fix_core$(EXESUF)
MEASVEC_BIN  := $(BIN_DIR)/test_meas_norm$(EXESUF)
MEASBRIDGE_BIN := $(BIN_DIR)/test_meas_bridge$(EXESUF)
SDTEST_BIN   := $(BIN_DIR)/test_sd_frame$(EXESUF)
BOARDTOP_BIN := $(BIN_DIR)/test_board_top$(EXESUF)
UIOENTRY_BIN := $(BIN_DIR)/test_uio_entry$(EXESUF)

# SD 卡帧（.psf）的入库 fixture：小几何（side=8 -> plane 4, block 2, wall 2），
# 384 字节 + 期望 JSON。有它，test-sd-frame 就不需要仓库外的论文数据集，
# 因此可以进 make test。真实帧（7.4 MB）不入库。
# 文件名里的 _f000 是帧序号（一帧一文件，见 docs/sd_frame_format.md §7）。
SDFIX        := ../../tests/vectors/sd_frame/tiny8x8_f000.psf
SDFIX_BLOCK  := 2

HOST_MODEL   := $(TMP_DIR)/host_test_model.psm
HOST_EXPECT  := $(TMP_DIR)/host_test_model.expected.txt

# 预处理链输出 -> 定点核输入 转换层的对拍向量（由 Python 权威口径生成）。
MEASVEC_DIR  := $(TMP_DIR)/meas_norm
MEASVEC_GEN  := tools/gen_meas_norm_vectors.py
MEASVEC_MAN  := $(MEASVEC_DIR)/manifest.txt

# 三个 C++ 测试/自检共用的头文件依赖。
# 此前这三条规则**只依赖各自的 .cpp**，改了头文件不会触发重编 —— 实测踩到：
# `../hardware/preprocess.hpp` 里 PS_BS_BITS 已是 24，但 `make test-software`
# 拿旧 exe 打印 23（位宽判据被静默漂移掩盖）。显式挂成依赖，改头必重编。
SHARED_HDRS := ../hardware/preprocess.hpp ../hardware/geometry.hpp ../hardware/gram.hpp \
               ps_contract.hpp ps_sources.hpp ps_platform.hpp ps_ipc.hpp \
               include/psm_format.h include/ps_regs.h include/ps_cache.h \
               ../../board/fix_core.h ../../board/ps_fix_regs.h \
               fpga_control/fix_core_driver.h meas_norm/meas_norm.h \
               fpga_control/uio_backend.h board_app/board_pmem.h \
               sd_frame/sd_frame.h \
               board_app/board_top.h board_app/board_regseq.h board_app/board_fb.h

# ---------------------------------------------------------------- 目标

.PHONY: all modules selftest tests host-test run-selftest test-x12 \
        test-psm-format test-fix-core test-meas-norm test-meas-bridge sd-frame-bin test-sd-frame \
        test-board-top test-uio-entry board-app clean help

all: modules selftest tests

modules: $(MODULE_LIB)

selftest: $(SELFTEST_BIN)

tests: $(TPF_BIN) $(T12_BIN) $(FIXTEST_BIN) $(MEASBRIDGE_BIN) $(SDTEST_BIN) $(BOARDTOP_BIN) $(UIOENTRY_BIN)

$(BIN_DIR) $(TMP_DIR):
	$(PYTHON) -c "import os,sys;os.makedirs(sys.argv[1],exist_ok=True)" $@

# 模块对象：VPATH 找到 .c，统一用 C 编译（见文件头说明）
$(BIN_DIR)/%.o: %.c | $(BIN_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(MODULE_LIB): $(MODULE_OBJ) | $(BIN_DIR)
	$(AR) rcs $@ $(MODULE_OBJ)

$(SELFTEST_BIN): host_selftest.cpp $(MODULE_OBJ) $(SHARED_HDRS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) host_selftest.cpp $(MODULE_OBJ) -o $@

$(TPF_BIN): test_psm_format.cpp $(SHARED_HDRS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

$(T12_BIN): test_x12_contract.cpp $(SHARED_HDRS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@

# 定点四核调用层的主机自检。**只用** fix_core_driver.c + board/fix_core.c 两个 .c，
# 不链接其余模块：这条链与双精度层（ps_regs.h）刻意隔离，见 fix_core_driver.h。
# 2026-09-30：fix_core_driver.c 新增对 meas_norm 的调用（帧量程接缝），故这里必须
# 再链接 meas_norm/meas_norm.c —— 否则 ps_meas_norm_from_rtl24 未解析（实测报错）。
# 2026-10-01：fix_core_driver.c 的 UIO 入口改为调 uio_backend，故须再链接
# fpga_control/uio_backend.c（否则 ps_uio_open_map / ps_uio_env_default 未解析）。
$(FIXTEST_BIN): test_fix_core.c fpga_control/fix_core_driver.c \
                fpga_control/uio_backend.c \
                meas_norm/meas_norm.c ../../board/fix_core.c $(SHARED_HDRS) | $(BIN_DIR)
	$(CC) $(CFLAGS) test_fix_core.c fpga_control/fix_core_driver.c \
	    fpga_control/uio_backend.c \
	    meas_norm/meas_norm.c ../../board/fix_core.c -o $@

# 「预处理链输出 -> 定点核输入」转换层的主机对拍。只链接本层与测试：这一级不碰
# 寄存器、不依赖 fix_core，故与双精度层/寄存器层都隔离（见 meas_norm.h 的依赖纪律）。
$(MEASVEC_BIN): test_meas_norm.c meas_norm/meas_norm.c meas_norm/meas_norm.h | $(BIN_DIR)
	$(CC) $(CFLAGS) test_meas_norm.c meas_norm/meas_norm.c -o $@

# 接缝自检：24 位打包拍 -> meas_norm -> 投影核入参。要同时链接三层实现 ——
# fix_core_driver.c（编排入口 ps_fix_prepare_meas/attach）、meas_norm.c（信号调理）、
# board/fix_core.c（影子窗口寄存器层）。这正是「meas_norm 接进板端流程」那根线的
# 主机侧证据（见 sim/results/preboard_test_gap_20260930.md §任务5）。
$(MEASBRIDGE_BIN): test_meas_bridge.c fpga_control/fix_core_driver.c \
                   fpga_control/uio_backend.c \
                   meas_norm/meas_norm.c ../../board/fix_core.c $(SHARED_HDRS) | $(BIN_DIR)
	$(CC) $(CFLAGS) test_meas_bridge.c fpga_control/fix_core_driver.c \
	    fpga_control/uio_backend.c meas_norm/meas_norm.c ../../board/fix_core.c -o $@

# SD 卡帧读取层的主机自检。链接两处实现：sd_frame.c（核心 + 文件入口）与
# model_loader/psm_crc32.c（CRC 的**唯一**实现，与 Python zlib.crc32 同参数）。
# 只链接这两个，不链接其余模块：这一级只做「文件 -> 缓冲区」，与寄存器层、
# 模型层、取流层都无关系（见 sd_frame.h 的依赖纪律）。
$(SDTEST_BIN): test_sd_frame.c sd_frame/sd_frame.c model_loader/psm_crc32.c $(SHARED_HDRS) | $(BIN_DIR)
	$(CC) $(CFLAGS) test_sd_frame.c sd_frame/sd_frame.c model_loader/psm_crc32.c -o $@

# 板端顶层应用核心的主机自检。要同时链接四层实现：
#   board_app/{board_top,board_regseq,board_fb}.c（顶层/清单/帧缓冲）
#   board_app/ps_cache.c（PL 访问 DDR 的 cache 成对守卫）
#   fpga_control/fix_core_driver.c + board/fix_core.c（核调用层与影子寄存器）
#   sd_frame/sd_frame.c + model_loader/psm_crc32.c（.psf 读取与 CRC）
#   meas_norm/meas_norm.c（fix_core_driver 的帧量程接缝）
# board_app/board_main.c 不在链接列表里：它的 main 只在板端构建时出现（见其文件头）。
$(BOARDTOP_BIN): test_board_top.c board_app/board_top.c board_app/board_regseq.c \
                 board_app/board_fb.c board_app/ps_cache.c \
                 fpga_control/fix_core_driver.c \
                 fpga_control/uio_backend.c \
                 meas_norm/meas_norm.c sd_frame/sd_frame.c model_loader/psm_crc32.c \
                 ../../board/fix_core.c $(SHARED_HDRS) | $(BIN_DIR)
	$(CC) $(CFLAGS) test_board_top.c board_app/board_top.c board_app/board_regseq.c \
	    board_app/board_fb.c board_app/ps_cache.c \
	    fpga_control/fix_core_driver.c fpga_control/uio_backend.c \
	    meas_norm/meas_norm.c \
	    sd_frame/sd_frame.c model_loader/psm_crc32.c ../../board/fix_core.c -o $@

# UIO 入口与连续物理内存分配的主机单测。只链接这两个可移植实现：
#   fpga_control/uio_backend.c  （标准 UIO 的打开/覆盖校验/页对齐逻辑）
#   board_app/board_pmem.c      （udmabuf / dma-heap / devmem 的选择与错误分支）
# 设备访问用注入的假后端（见测试文件头），因此不需要真设备、不需要 POSIX 头，
# 也不链接 fix_core_driver.c（那会把 meas_norm 等一整条链拉进来）。
# 真板的系统调用实现（uio_backend_posix.c / board_pmem_posix.c）只在 Linux 构建里
# 编到内容，主机下是空 TU。
$(UIOENTRY_BIN): test_uio_entry.c fpga_control/uio_backend.c board_app/board_pmem.c \
                 fpga_control/uio_backend.h board_app/board_pmem.h | $(BIN_DIR)
	$(CC) $(CFLAGS) test_uio_entry.c fpga_control/uio_backend.c \
	    board_app/board_pmem.c -o $@

# 对拍向量由 Python 侧（quantize.shared_exponent 权威口径）生成；依赖脚本本身，
# 改了生成脚本会重新生成，不会拿旧向量去比新口径。
$(MEASVEC_MAN): $(MEASVEC_GEN) | $(TMP_DIR)
	$(PYTHON) $(MEASVEC_GEN) $(MEASVEC_DIR)

# Python 侧生成小 model.psm 与期望表（CRC/几何哈希/偏移全部由 psm.py 计算）。
# 依赖脚本本身：改了生成脚本就会重新生成，不会拿旧的期望表去比新的模型。
$(HOST_MODEL): tools/make_host_test_model.py | $(TMP_DIR)
	$(PYTHON) tools/make_host_test_model.py $@

# ---------------------------------------------------------------- 运行

# 注意：下面 echo 的文本里**不能出现 < 或 >**（Windows 上 make 用 cmd 执行 recipe，
# 它们会被当成重定向；曾经用 "<->" 写提示语，结果 make 去打开一个不存在的文件报
# "The system cannot find the file specified."）。需要箭头时用 Unicode 的 →。
host-test: $(SELFTEST_BIN) $(TPF_BIN) $(T12_BIN) $(HOST_MODEL) $(MEASVEC_BIN) $(MEASVEC_MAN) $(MEASBRIDGE_BIN) $(BOARDTOP_BIN)
	@echo [host-test] 1/6 模块自检（寄存器/模型/取流/FPGA/遥测/UI）
	$(SELFTEST_BIN) $(HOST_MODEL) $(HOST_EXPECT) $(TMP_DIR)
	@echo [host-test] 2/6 psm_format.h 与 psm.py 的偏移/长度一致性
	$(TPF_BIN)
	@echo [host-test] 3/6 X12 契约与三条数据源
	$(T12_BIN)
	@echo [host-test] 4/6 预处理链输出 → 定点核输入 转换层（对 quantize.py 逐位对拍）
	$(MEASVEC_BIN) $(MEASVEC_DIR)
	@echo [host-test] 5/6 24 位打包拍 → meas_norm → 定点核入参 接缝
	$(MEASBRIDGE_BIN) $(MEASVEC_DIR)
	@echo [host-test] 6/6 板端顶层应用（账本/布局/清单/版本/最小用例/四核/帧缓冲/.psf）
	$(BOARDTOP_BIN)
	@echo [host-test] 全部完成：退出码 0 表示全绿

run-selftest: $(SELFTEST_BIN) $(HOST_MODEL)
	$(SELFTEST_BIN) $(HOST_MODEL) $(HOST_EXPECT) $(TMP_DIR)

test-psm-format: $(TPF_BIN)
	$(TPF_BIN)

test-x12: $(T12_BIN)
	$(T12_BIN)

# 定点四核调用层自检（影子窗口，无板卡）。退出码非零即失败；上板时序不在本目标范围。
test-fix-core: $(FIXTEST_BIN)
	@echo [test-fix-core] 定点四核调用层（影子窗口；上板时序等板卡）
	$(FIXTEST_BIN)

# RTL 预处理链输出 -> 定点核输入 转换层的主机对拍：先由 Python 生成向量与期望
# （权威口径 quantize.shared_exponent），再跑 C 逐位比较。退出码非零即失败。
test-meas-norm: $(MEASVEC_BIN) $(MEASVEC_MAN)
	@echo [test-meas-norm] 共享指数归一化 + 通道优先重排 + 24→32 符号扩展
	$(MEASVEC_BIN) $(MEASVEC_DIR)

# 接缝自检（2026-09-30 新增）：把 meas_norm 接进 fix_core_driver 之后，验
# 「RTL 24 位打包拍 -> meas 缓冲 -> 帧量程折算 -> 投影核 job/寄存器」。先由 Python
# 生成向量（与 test-meas-norm 共用），再跑 C。退出码非零即失败。
test-meas-bridge: $(MEASBRIDGE_BIN) $(MEASVEC_MAN)
	@echo [test-meas-bridge] 24 位打包拍 → meas_norm → 定点投影核入参
	$(MEASBRIDGE_BIN) $(MEASVEC_DIR)

# SD 卡帧（.psf）读取层：两条读取路径（流式/内存）互相比对、块和与独立写法交叉
# 验证、四类负例的**指定**错误码。跑的是入库 fixture，不需要仓库外的论文数据集，
# 所以这一条能进 make test。真实帧的端到端比对在 build/scripts/sd_frame_selftest.py。
test-sd-frame: $(SDTEST_BIN)
	@echo [test-sd-frame] .psf 读回：fixture=$(SDFIX) block=$(SDFIX_BLOCK)
	$(SDTEST_BIN) $(SDFIX) $(SDFIX_BLOCK)

# 只编二进制，不跑。build/scripts/sd_frame_selftest.py 在找不到二进制时会调它。
sd-frame-bin: $(SDTEST_BIN)

# 板端顶层应用的主机自检：账本/布局/寄存器清单（与驱动逐条比对）/版本读回/
# 最小用例/四核串联/帧缓冲/.psf 装载。影子窗口，无板卡；退出码非零即失败。
test-board-top: $(BOARDTOP_BIN)
	@echo [test-board-top] 板端顶层应用主机自检（影子窗口；上板时序/DP 输出等板卡）
	$(BOARDTOP_BIN)

# UIO 入口与连续物理内存分配的主机单测（注入式后端，无板卡）。退出码非零即失败；
# 真设备树、真 open/mmap/ioctl 等系统调用项一律板端实测，本目标不代跑。
test-uio-entry: $(UIOENTRY_BIN)
	@echo [test-uio-entry] UIO 入口 + 连续物理内存分配（注入式后端；系统调用等板卡）
	$(UIOENTRY_BIN)

# 板端应用二进制（PetaLinux 用户态）。含 <sys/mman.h> 等 POSIX 头，**只在目标平台可编**：
#     make -C src/software CC=aarch64-linux-gnu-gcc board-app
# 主机（MinGW）下本目标显式拒绝，避免把「在 Windows 上编过」误当成「能在板上跑」。
board-app: | $(BIN_DIR)
ifeq ($(OS),Windows_NT)
	@echo [board-app] 本目标只在板端 Linux 上可用（主机缺 sys/mman.h 等）；
	@echo   请在板端或交叉编译环境执行：
	@echo     make -C src/software CC=aarch64-linux-gnu-gcc board-app
else
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=200809L -DPS_BOARD_APP_MAIN board_app/board_main.c board_app/board_top.c \
	    board_app/board_regseq.c board_app/board_fb.c board_app/ps_cache.c \
	    board_app/board_pmem.c board_app/board_pmem_posix.c \
	    board_app/frame_pipeline.c board_app/frame_runtime.c board_app/frame_display.c \
	    fpga_control/fix_core_driver.c \
	    fpga_control/uio_backend.c fpga_control/uio_backend_posix.c \
	    meas_norm/meas_norm.c sd_frame/sd_frame.c model_loader/psm_crc32.c \
	    ../../board/fix_core.c -lm -o $(BIN_DIR)/ps_board_app$(EXESUF)
	@echo [board-app] 产物 $(BIN_DIR)/ps_board_app$(EXESUF)（未在板上验证）
endif

# ---------------------------------------------------------------- 清理

clean:
	$(PYTHON) -c "import glob,os,shutil;f=glob.glob('$(BIN_DIR)/*.o')+glob.glob('$(BIN_DIR)/*.a')+glob.glob('$(BIN_DIR)/*$(EXESUF)')+glob.glob('$(TMP_DIR)/host_test_*')+glob.glob('$(TMP_DIR)/bad_*');n=len(f);[os.remove(p) for p in f];shutil.rmtree('$(MEASVEC_DIR)',ignore_errors=True);shutil.rmtree('$(TMP_DIR)/sd_frame',ignore_errors=True);print('[clean] removed',n,'files')"

help:
	@echo PenumbraSight PS 侧 Makefile —— 可用目标：
	@echo   all              编译模块 + 自检 + 契约测试（默认）
	@echo   modules          只编译模块，产物 $(MODULE_LIB)
	@echo   selftest         只编译自检，产物 $(SELFTEST_BIN)
	@echo   tests            只编译各契约测试
	@echo   host-test        编译并运行自检 + 契约测试（CI 入口）
	@echo   run-selftest     只跑自检（需要先有 $(HOST_MODEL)）
	@echo   test-psm-format  跑 psm_format.h 与 psm.py 的一致性测试
	@echo   test-x12         跑 X12 契约与三条数据源测试
	@echo   test-fix-core    跑定点四核调用层自检（影子窗口，无板卡）
	@echo   test-meas-norm   跑 RTL→定点核 转换层的逐位对拍（Python 权威口径）
	@echo   test-meas-bridge 跑「24 位打包拍 → meas_norm → 投影核入参」接缝自检
	@echo   test-sd-frame    跑 .psf 读取层自检（fixture 上；真实帧见 build/scripts/sd_frame_selftest.py）
	@echo   sd-frame-bin     只编 .psf 读取层自检二进制，不跑
	@echo   test-board-top   跑板端顶层应用自检（影子窗口：账本/清单/版本/最小用例/四核/帧缓冲）
	@echo   test-uio-entry   跑 UIO 入口与连续物理内存分配单测（注入式后端；系统调用等板卡）
	@echo   board-app        交叉编译板端应用二进制（目标平台专用；主机下显式拒绝）
	@echo   clean            清理 bin/ 与 $(TMP_DIR)/ 下的生成物
	@echo 变量：CC CXX AR PYTHON OPT（默认 -O2）CFLAGS CXXFLAGS

# Complete FP64 M8 application, separate ABI from the historical fixed-point app.
.PHONY: m8 m8-host m8-arm m8-test m8-verify m8-release-check m8-workspace-check
m8 m8-host m8-arm m8-test m8-verify m8-release-check m8-workspace-check:
	$(MAKE) -C ../.. $@ PYTHON="$(PYTHON)" M8_ARGS="$(M8_ARGS)"
