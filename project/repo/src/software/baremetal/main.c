/* ============================================================================
 * PenumbraSight · 裸机最小系统（JTAG / XSCT 下跑）· 程序入口
 * src/software/baremetal/main.c
 *
 * 目标：板子到手（ALINX AXU9EGB / ZU9EG，JTAG + 串口可用，显示屏未接）后，
 * 在 XSCT 里下载本 ELF 并运行，用一个可判定的最小用例验证「位流 + PS + 寄存器
 * 通路」。四步，每步打 KEY=VALUE 行供串口抓取比对：
 *
 *   STEP=1  读回 PL 控制页的 VERSION（判据：==1），并读 ID 做存在性探测。
 *   STEP=2  测 pl_clk0 频率（判据：100 MHz；无 PL 计数器时如实报 SKIPPED）。
 *   STEP=3  单核最小用例 ps_lr_forward_fix（K=1/M=4/N=1，判定 y 的四个值）。
 *   STEP=4  四核串联（按 board_regseq 顺序回放，打印每核起止与超时步号）。
 *
 * 依赖：只用 Xilinx 裸机 BSP 的基本功能（Xil_Out32/Xil_In32、xil_printf、
 * XTime_GetTime、xil_cache）。寄存器偏移与基址全部来自源码：
 *   board/ps_fix_regs.h    （四核，基址 0xA0000000..；行号见各引用处）
 *   board/ps_pl_regs.h     （PL 控制页，基址 0xA00A0000；行号见各引用处）
 *
 * 未上板声明
 * ---------------------------------------------------------------------------
 * **本程序尚未在任何板子上跑过。** 编译通过不等于板上行为正确。期望值由 HLS
 * 定点核源码逐级手算（推导见 baremetal_app.h 与 sim/results/baremetal_minimal_20261001.md），
 * 未做实测。所有缓存/异常处理路径均为「按 Xilinx 惯例写的、待板验证」的实现。
 *
 * 打印格式：KEY=VALUE，一行一 KEY，`\r\n` 结尾；十六进制一律 8 位大写字、前缀 0x，
 * 64 位量拆成 KEY_LO / KEY_HI 两个 32 位行打印（xil_printf 不支持 %llu）。
 * ==========================================================================*/

#include "baremetal_app.h"

#include <stdint.h>

#include "xil_printf.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xtime_l.h"

#if BARE_ABORT_GUARD
#include "xil_exception.h"
#endif

/* 寄存器与基址：只用源码宏，不写裸字面量 */
#include "ps_fix_regs.h"
#include "ps_pl_regs.h"

/* ======================================================================== */
/* 小工具：十六进制打印（xil_printf 的宽度/补零支持依版本而异，自己来最稳）   */
/* ======================================================================== */

static void hex32_to_str(uint32_t v, char *out)
{
    static const char h[] = "0123456789ABCDEF";
    int i;
    for (i = 7; i >= 0; --i) { out[i] = h[v & 0xFu]; v >>= 4; }
    out[8] = '\0';
}

static void kv_dec(const char *k, uint32_t v)
{
    xil_printf("%s=%u\r\n", k, (unsigned)v);
}

static void kv_hex(const char *k, uint32_t v)
{
    char b[9];
    hex32_to_str(v, b);
    xil_printf("%s=0x%s\r\n", k, b);
}

static void kv_i32(const char *k, int32_t v)
{
    xil_printf("%s=%d\r\n", k, (int)v);
}

/* ======================================================================== */
/* 寄存器访问：物理地址直写（baremetal 恒等映射）                            */
/* ======================================================================== */

#define BARE_W(addr, v)  Xil_Out32((UINTPTR)(addr), (uint32_t)(v))
#define BARE_R(addr)     Xil_In32((UINTPTR)(addr))

/* ======================================================================== */
/* 四核序列的步号记录（编号对齐 board_regseq.c 的清单：第 1 步 = 版本读回）    */
/* 超时打印「停在第几步」正是任务要求的那条定位信息。                        */
/* ======================================================================== */

static uint32_t    g_step = 0;          /* 当前步号（1 起） */
static const char *g_step_name = "-";   /* 当前步名 */

static void seq_write(uint32_t abs_addr, uint32_t v, const char *name)
{
    g_step++;
    g_step_name = name;
    BARE_W(abs_addr, v);
}

static void seq_base64(uint32_t ctrlr_base, uint32_t off, uint64_t a,
                       const char *name_hi, const char *name_lo)
{
    /* 先 HI 再 LO（board/ps_fix_regs.h:270-277、fix_core.c:36-43 的同一纪律） */
    g_step++; g_step_name = name_hi;
    BARE_W(ctrlr_base + off + PS_FIX_64_HI, (uint32_t)(a >> 32));
    g_step++; g_step_name = name_lo;
    BARE_W(ctrlr_base + off + PS_FIX_64_LO, (uint32_t)(a & 0xFFFFFFFFu));
}

/* 返回 0 = 等到 ap_done；-1 = 超时（并打印停在第几步）。 */
static int seq_poll(uint32_t ctrl_base, const char *core, uint32_t limit)
{
    uint32_t n = limit;
    g_step++; g_step_name = "CTRL.ap_done";
    while (n-- > 0u) {
        if ((BARE_R(ctrl_base + PS_FIX_REG_CTRL) & PS_FIX_CTRL_AP_DONE) != 0u) {
            return 0;
        }
    }
    {
        char b[9];
        hex32_to_str(ctrl_base + PS_FIX_REG_CTRL, b);
        xil_printf("CORE=%s STATUS=TIMEOUT STEP=%u NAME=%s ADDR=0x%s\r\n",
                   core, (unsigned)g_step, g_step_name, b);
    }
    return -1;
}

static int seq_start(uint32_t ctrl_base, const char *core, uint32_t limit)
{
    seq_write(ctrl_base + PS_FIX_REG_CTRL, PS_FIX_CTRL_AP_START, "CTRL.ap_start");
    return seq_poll(ctrl_base, core, limit);
}

/* ======================================================================== */
/* Data Abort 兜底：探测可能未映射的 PL 控制页时，捕获异常并跳过该条 load，    */
/* 让程序继续（把结果标成 UNMAPPED），而不是停在异常里。                     */
/* 仅 aarch64 编译；EL 假设为 EL3（Xilinx standalone for ZynqMP 默认）。      */
/* 未上板验证 —— 见报告「未上板声明」。                                     */
/* ======================================================================== */

#if BARE_ABORT_GUARD && defined(__aarch64__)
static volatile int g_dabort = 0;

static void bare_dabort_handler(void *arg)
{
    uint64_t elr;
    (void)arg;
    g_dabort = 1;
    /* 把返回地址 +4，跳过触发同步异常的那条 load，避免死循环重试 */
    __asm__ volatile("mrs %0, elr_el3" : "=r"(elr));
    elr += 4u;
    __asm__ volatile("msr elr_el3, %0" :: "r"(elr));
}

static void bare_abort_guard_install(void)
{
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_DATA_ABORT_INT,
                                 (Xil_ExceptionHandler)bare_dabort_handler, NULL);
    Xil_ExceptionEnable();
}
#else
static void bare_abort_guard_install(void) { }
#endif

/* ======================================================================== */
/* DDR 缓冲池：静态数组，4096 对齐；地址即物理地址（PL 经 HP 端口可达）       */
/* ======================================================================== */

typedef struct { uint64_t phys; void *ptr; } bare_blk_t;

static uint8_t g_arena[BARE_ARENA_BYTES] __attribute__((aligned(4096)));
static uint32_t g_arena_off = 0u;

static bare_blk_t bare_alloc(uint32_t bytes)
{
    bare_blk_t b;
    uint32_t a = (g_arena_off + 4095u) & ~4095u;
    if (bytes == 0u) { bytes = 4u; }
    b.ptr  = (void *)&g_arena[a];
    b.phys = (uint64_t)(uintptr_t)&g_arena[a];
    g_arena_off = a + ((bytes + 4095u) & ~4095u);
    return b;
}

/* ======================================================================== */
/* STEP 1：版本读回                                                          */
/* ======================================================================== */

static int g_pl_page_ok = 0;   /* 1 = 该页读得到（ID/VERSION 合法） */

static void step1_version(void)
{
    uint32_t id = 0u, ver = 0u;
    char b[9];

    xil_printf("STEP=1 KIND=PL_VERSION\r\n");
    xil_printf("PL_BASE=");
    hex32_to_str((uint32_t)PS_PL_BASE, b); xil_printf("0x%s\r\n", b);
    kv_hex("PL_ID_ADDR", (uint32_t)(PS_PL_BASE + PS_PL_ID));
    kv_hex("PL_VER_ADDR", (uint32_t)(PS_PL_BASE + PS_PL_VERSION));

    /* 存在性探测：先读 ID（board_regseq.h:53-55 的同一做法） */
    BARE_R(PS_PL_BASE + PS_PL_ID);
#if BARE_ABORT_GUARD && defined(__aarch64__)
    g_dabort = 0;
#endif
    id  = BARE_R(PS_PL_BASE + PS_PL_ID);
    ver = BARE_R(PS_PL_BASE + PS_PL_VERSION);

#if BARE_ABORT_GUARD && defined(__aarch64__)
    if (g_dabort) {
        xil_printf("PL_PROBE=ABORT\r\n");
        kv_dec("ID", 0u);
        kv_dec("VERSION", 0u);
        xil_printf("VERSION_EXPECT=%u\r\n", (unsigned)BARE_EXPECT_IF_VERSION);
        xil_printf("RESULT=UNMAPPED\r\n");
        xil_printf("REASON=PL_CONTROL_PAGE_NOT_MAPPED\r\n");
        g_step = 1u; g_step_name = "PL_CTRL.VERSION";
        return;
    }
#endif

    kv_hex("ID", id);
    kv_hex("ID_EXPECT", BARE_EXPECT_PL_ID);
    kv_dec("VERSION", ver);
    kv_dec("VERSION_EXPECT", BARE_EXPECT_IF_VERSION);

    if (id == BARE_EXPECT_PL_ID && ver == BARE_EXPECT_PL_VERSION) {
        xil_printf("RESULT=PASS\r\n");
        g_pl_page_ok = 1;
    } else if (id == BARE_EXPECT_PL_ID) {
        xil_printf("RESULT=FAIL\r\nREASON=VERSION_MISMATCH\r\n");
    } else if (id == 0u) {
        xil_printf("RESULT=UNMAPPED\r\nREASON=READ_ZERO\r\n");
    } else {
        xil_printf("RESULT=FAIL\r\nREASON=ID_MISMATCH\r\n");
    }

    /* 本步是清单第 1 条（board_regseq.h 的 include_version 步骤） */
    g_step = 1u;
    g_step_name = "PL_CTRL.VERSION";
}

/* ======================================================================== */
/* STEP 2：pl_clk0 测量                                                      */
/* ======================================================================== */

static void step2_pl_clk(void)
{
    xil_printf("STEP=2 KIND=PL_CLK\r\n");
    xil_printf("PL_CLK_TARGET_HZ=100000000\r\n");
    kv_dec("COUNTS_PER_SECOND", (uint32_t)COUNTS_PER_SECOND);

#if BARE_HAS_PL_COUNTER
    {
        XTime t0, t1;
        uint32_t c0, c1, dc;
        uint64_t elapsed_ticks, elapsed_us, f_hz, res_ppm;

        xil_printf("PL_CLK_MEAS=PL_FREE_COUNTER\r\n");
        kv_hex("PL_CLK_COUNTER_ADDR", (uint32_t)(BARE_PL_COUNTER_ADDR));
        kv_dec("PL_CLK_GATE_MS", (uint32_t)(BARE_PL_GATE_MS));

        /* 读计数 -> 记时 -> 忙等到门宽 -> 再读计数 -> 记时 */
        c0 = BARE_R(BARE_PL_COUNTER_ADDR);
        XTime_GetTime(&t0);
        do {
            XTime_GetTime(&t1);
        } while ((t1 - t0) < (XTime)(((uint64_t)COUNTS_PER_SECOND * BARE_PL_GATE_MS) / 1000u));
        c1 = BARE_R(BARE_PL_COUNTER_ADDR);

        dc = (uint32_t)(c1 - c0);            /* 32 位差，只要门宽内不回绕 */
        elapsed_ticks = (uint64_t)(t1 - t0);
        elapsed_us = (elapsed_ticks * 1000000ull) / (uint64_t)COUNTS_PER_SECOND;
        f_hz = (elapsed_us == 0ull) ? 0ull : ((uint64_t)dc * 1000000ull) / elapsed_us;
        res_ppm = (dc == 0u) ? 0ull : (1000000ull / (uint64_t)dc);

        kv_dec("PL_CLK_COUNTS", dc);
        kv_dec("PL_CLK_GATE_US", (uint32_t)elapsed_us);
        kv_hex("PL_CLK_HZ_LO", (uint32_t)(f_hz & 0xFFFFFFFFull));
        kv_hex("PL_CLK_HZ_HI", (uint32_t)(f_hz >> 32));
        kv_dec("PL_CLK_RES_PPM", (uint32_t)res_ppm);

        /* 判据：与 100 MHz 的相对偏差。0.1% = 1000 ppm。 */
        {
            int64_t diff = (int64_t)f_hz - 100000000ll;
            int64_t ppm  = (diff * 1000000ll) / 100000000ll;
            if (ppm < 0) { ppm = -ppm; }
            kv_dec("PL_CLK_DEV_PPM", (uint32_t)ppm);
            xil_printf("PL_CLK_CAN_RESOLVE_100K=%s\r\n",
                       (res_ppm < 1000ull) ? "YES" : "NO");
            xil_printf("PL_CLK_PASS=%s\r\n", (ppm <= 1000ll) ? "PASS" : "FAIL");
        }
    }
#else
    xil_printf("PL_CLK_MEAS=SKIPPED_NO_PL_COUNTER\r\n");
    xil_printf("PL_CLK_REASON=NO_FREE_RUNNING_PL_COUNTER_IN_REGISTER_MAP\r\n");
    kv_dec("PL_CLK_PASS", 0u);

#if (BARE_AXI_DIAG_READS > 0)
    /* 只在 PL 控制页确实映射时才读 SCRATCH —— 否则会再触发一次 Data Abort。
     * 用 STEP=1 的存在性结果 g_pl_page_ok 当门禁。 */
    if (!g_pl_page_ok) {
        xil_printf("AXI_DIAG_READS=0\r\n");
        xil_printf("AXI_DIAG_SKIPPED=PL_PAGE_UNMAPPED\r\n");
    } else
    {
        /* 退化诊断：连续读控制页寄存器，测每次访问的墙钟时间。
         * 这只能证明 AXI/PL 时钟域活着，**不是频率测量**，不能区分 100 与 99.9 MHz。 */
        XTime t0, t1;
        uint32_t i;
        uint64_t ticks, ns_per;
        volatile uint32_t sink = 0u;
        XTime_GetTime(&t0);
        for (i = 0u; i < BARE_AXI_DIAG_READS; ++i) {
            sink ^= BARE_R(PS_PL_BASE + PS_PL_SCRATCH);
        }
        XTime_GetTime(&t1);
        ticks = (uint64_t)(t1 - t0);
        ns_per = (ticks * 1000000000ull) /
                 ((uint64_t)COUNTS_PER_SECOND * (uint64_t)BARE_AXI_DIAG_READS);
        kv_dec("AXI_DIAG_READS", BARE_AXI_DIAG_READS);
        kv_dec("AXI_DIAG_NS_PER_ACCESS", (uint32_t)ns_per);
        xil_printf("AXI_DIAG_NOTE=NOT_A_FREQUENCY_MEASUREMENT\r\n");
    }
#else
    xil_printf("AXI_DIAG_READS=0\r\n");
#endif
#endif
}

/* ======================================================================== */
/* STEP 3：单核最小用例（ps_lr_forward_fix）                                 */
/* ======================================================================== */

static int g_min_pass = 0;

static void step3_min_core(void)
{
    const uint32_t cb = (uint32_t)PS_FIX_CORE_LR_FWD_CTRL;    /* 0xA0020000 */
    const uint32_t cr = (uint32_t)PS_FIX_CORE_LR_FWD_CTRL_R;  /* 0xA0030000 */
    bare_blk_t bU, bS, bV, bf, by, bwz, bwzs;
    int16_t  *U; uint16_t *S; int16_t *V; int32_t *f, *y, *wz, *wzs;
    XTime t0, t1;
    uint32_t ctrl_val, ret;
    int rc;
    char b[9];

    /* step3 用同一套步号记录器，只是它的编号是本步局部用途，
     * 与 board_regseq 的编号无关；step4 开头会把计数器重置到 board_regseq 的编号。 */
    g_step = 0;
    g_step_name = "-";

    xil_printf("STEP=3 KIND=SINGLE_CORE CORE=lr_fwd\r\n");
    xil_printf("CTRL_BASE=");
    hex32_to_str(cb, b); xil_printf("0x%s\r\n", b);
    xil_printf("CTRL_R_BASE=");
    hex32_to_str(cr, b); xil_printf("0x%s\r\n", b);
    kv_dec("K", BARE_MIN_K);
    kv_dec("M", BARE_MIN_M);
    kv_dec("N", BARE_MIN_N);

    bU   = bare_alloc((uint32_t)(BARE_MIN_K * BARE_MIN_M * 2u));
    bS   = bare_alloc((uint32_t)(BARE_MIN_K * 2u));
    bV   = bare_alloc((uint32_t)(BARE_MIN_K * BARE_MIN_N * 2u));
    bf   = bare_alloc((uint32_t)(BARE_MIN_N * 4u));
    by   = bare_alloc((uint32_t)(BARE_MIN_M * 4u));
    bwz  = bare_alloc((uint32_t)(BARE_MIN_K * 4u));
    bwzs = bare_alloc((uint32_t)(BARE_MIN_K * 4u));

    U   = (int16_t *)bU.ptr;
    S   = (uint16_t *)bS.ptr;
    V   = (int16_t *)bV.ptr;
    f   = (int32_t *)bf.ptr;
    y   = (int32_t *)by.ptr;
    wz  = (int32_t *)bwz.ptr;
    wzs = (int32_t *)bwzs.ptr;

    /* 合成数据（不依赖 SD 卡）：见 baremetal_app.h 的逐级推导 */
    U[0] = 1; U[1] = 2; U[2] = 3; U[3] = 4;
    S[0] = (uint16_t)BARE_MIN_S_CODE;
    V[0] = 1;
    f[0] = 1;
    y[0] = y[1] = y[2] = y[3] = 0;
    wz[0] = 0; wzs[0] = 0;

    kv_hex("BASE_U_LO",   (uint32_t)(bU.phys & 0xFFFFFFFFu));
    kv_hex("BASE_U_HI",   (uint32_t)(bU.phys >> 32));
    kv_hex("BASE_S_LO",   (uint32_t)(bS.phys & 0xFFFFFFFFu));
    kv_hex("BASE_V_LO",   (uint32_t)(bV.phys & 0xFFFFFFFFu));
    kv_hex("BASE_F_LO",   (uint32_t)(bf.phys & 0xFFFFFFFFu));
    kv_hex("BASE_Y_LO",   (uint32_t)(by.phys & 0xFFFFFFFFu));
    kv_hex("BASE_WK_Z_LO",(uint32_t)(bwz.phys & 0xFFFFFFFFu));
    kv_hex("BASE_WK_ZS_LO",(uint32_t)(bwzs.phys & 0xFFFFFFFFu));

    /* 写基址（先 HI 再 LO）------ 对齐 board/ps_fix_regs.h:158-164 的 offset |
     * stage 2 标量（行 140-148）后写。 */
    seq_base64(cr, LRF_BASE_U,     bU.phys,   "LRF_BASE_U.HI",   "LRF_BASE_U.LO");
    seq_base64(cr, LRF_BASE_S,     bS.phys,   "LRF_BASE_S.HI",   "LRF_BASE_S.LO");
    seq_base64(cr, LRF_BASE_V,     bV.phys,   "LRF_BASE_V.HI",   "LRF_BASE_V.LO");
    seq_base64(cr, LRF_BASE_F,     bf.phys,   "LRF_BASE_F.HI",   "LRF_BASE_F.LO");
    seq_base64(cr, LRF_BASE_Y,     by.phys,   "LRF_BASE_Y.HI",   "LRF_BASE_Y.LO");
    seq_base64(cr, LRF_BASE_WK_Z,  bwz.phys,  "LRF_BASE_WK_Z.HI","LRF_BASE_WK_Z.LO");
    seq_base64(cr, LRF_BASE_WK_ZS, bwzs.phys, "LRF_BASE_WK_ZS.HI","LRF_BASE_WK_ZS.LO");

    seq_write(cb + LRF_K, BARE_MIN_K, "LRF_K");
    seq_write(cb + LRF_M, BARE_MIN_M, "LRF_M");
    seq_write(cb + LRF_N, BARE_MIN_N, "LRF_N");
    seq_write(cb + LRF_U_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "LRF_U_GAIN_Q16");
    seq_write(cb + LRF_S_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "LRF_S_GAIN_Q16");
    seq_write(cb + LRF_V_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "LRF_V_GAIN_Q16");
    seq_write(cb + LRF_U_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "LRF_U_EXP");
    seq_write(cb + LRF_S_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "LRF_S_EXP");
    seq_write(cb + LRF_V_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "LRF_V_EXP");

    kv_dec("SCALAR_U_GAIN_Q16", (uint32_t)BARE_MIN_GAIN_Q16);
    kv_hex("SCALAR_U_EXP", (uint32_t)BARE_MIN_GAIN_EXP);

    /* 核写 y：先把输入刷到 DDR，并把输出行作废，再启动 */
    Xil_DCacheFlushRange((INTPTR)U, (uint32_t)(BARE_MIN_K * BARE_MIN_M * 2u));
    Xil_DCacheFlushRange((INTPTR)S, (uint32_t)(BARE_MIN_K * 2u));
    Xil_DCacheFlushRange((INTPTR)V, (uint32_t)(BARE_MIN_K * BARE_MIN_N * 2u));
    Xil_DCacheFlushRange((INTPTR)f, (uint32_t)(BARE_MIN_N * 4u));
    Xil_DCacheFlushRange((INTPTR)wz, (uint32_t)(BARE_MIN_K * 4u));
    Xil_DCacheFlushRange((INTPTR)wzs, (uint32_t)(BARE_MIN_K * 4u));
    Xil_DCacheFlushRange((INTPTR)y, (uint32_t)(BARE_MIN_M * 4u));

    XTime_GetTime(&t0);
    rc = seq_start(cb, "lr_fwd", BARE_MIN_POLL_LIMIT);
    XTime_GetTime(&t1);

    kv_dec("AP_DONE", (rc == 0) ? 1u : 0u);
    {
        uint64_t us = ((uint64_t)(t1 - t0) * 1000000ull) / (uint64_t)COUNTS_PER_SECOND;
        kv_dec("ELAPSED_US", (uint32_t)us);
        kv_dec("ELAPSED_TICKS_LO", (uint32_t)((uint64_t)(t1 - t0) & 0xFFFFFFFFu));
        kv_dec("ELAPSED_TICKS_HI", (uint32_t)((uint64_t)(t1 - t0) >> 32));
    }

    ctrl_val = BARE_R(cb + PS_FIX_REG_CTRL);
    ret      = BARE_R(cb + PS_FIX_REG_AP_RETURN);
    kv_hex("CTRL_AFTER", ctrl_val);
    kv_dec("AP_RETURN", ret);
    kv_dec("AP_RETURN_EXPECT", 0u);   /* kLrOk = 0（ps_fixed_top.cpp:219-223） */

    /* 读结果：先读数据、再读 VLD（VLD 是 COR） */
    {
        uint32_t lo = BARE_R(cb + LRF_MAC_COUNT);
        uint32_t hi = BARE_R(cb + LRF_MAC_COUNT + 4u);
        uint32_t vld = BARE_R(cb + LRF_MAC_COUNT + 8u);
        kv_hex("MAC_COUNT_LO", lo);
        kv_hex("MAC_COUNT_HI", hi);
        kv_dec("MAC_COUNT_VLD", vld & 1u);
        kv_dec("MAC_COUNT_EXPECT_LANES16", BARE_MIN_MAC_EXPECT);
    }
    {
        uint32_t sat = BARE_R(cb + LRF_SAT_EVENTS);
        uint32_t vld = BARE_R(cb + LRF_SAT_EVENTS + PS_FIX_VLD_OFF);
        uint32_t ovf = BARE_R(cb + LRF_OUT_OVERFLOW);
        uint32_t gc  = BARE_R(cb + LRF_GAIN_CLAMPED);
        kv_dec("SAT_EVENTS", sat);
        kv_dec("SAT_EVENTS_VLD", vld & 1u);
        kv_dec("OUT_OVERFLOW", ovf);
        kv_dec("GAIN_CLAMPED", gc);
    }

    /* 读回 DDR 里的 y（先作废本行再读） */
    Xil_DCacheInvalidateRange((INTPTR)y, (uint32_t)(BARE_MIN_M * 4u));
    kv_i32("Y0", y[0]);
    kv_i32("Y1", y[1]);
    kv_i32("Y2", y[2]);
    kv_i32("Y3", y[3]);
    kv_i32("Y0_EXPECT", BARE_MIN_Y0_EXPECT);
    kv_i32("Y1_EXPECT", BARE_MIN_Y1_EXPECT);
    kv_i32("Y2_EXPECT", BARE_MIN_Y2_EXPECT);
    kv_i32("Y3_EXPECT", BARE_MIN_Y3_EXPECT);

    if (rc == 0 && ret == 0u &&
        y[0] == BARE_MIN_Y0_EXPECT && y[1] == BARE_MIN_Y1_EXPECT &&
        y[2] == BARE_MIN_Y2_EXPECT && y[3] == BARE_MIN_Y3_EXPECT) {
        xil_printf("RESULT=PASS\r\n");
        g_min_pass = 1;
    } else {
        xil_printf("RESULT=FAIL\r\n");
        xil_printf("REASON=%s%s%s\r\n",
                   (rc != 0) ? "TIMEOUT " : "",
                   (ret != 0u) ? "AP_RETURN_NZ " : "",
                   ((y[0] != BARE_MIN_Y0_EXPECT || y[1] != BARE_MIN_Y1_EXPECT ||
                     y[2] != BARE_MIN_Y2_EXPECT || y[3] != BARE_MIN_Y3_EXPECT)) ? "Y_MISMATCH" : "");
    }
    (void)ctrl_val;
}

/* ======================================================================== */
/* STEP 4：四核串联（顺序与偏移完全照 board_regseq.c，尺寸取小值）            */
/* ======================================================================== */

static int g_seq_ok_cores = 0;

static void run_proj_small(void)
{
    const uint32_t cb = (uint32_t)PS_FIX_CORE_PROJ_CTRL;    /* 0xA0000000 */
    const uint32_t cr = (uint32_t)PS_FIX_CORE_PROJ_CTRL_R;  /* 0xA0010000 */
    bare_blk_t bmeas, buc, bgain, bso, bsch;
    int32_t *meas, *gain; int16_t *uc; uint64_t *so, *sch;

    xil_printf("CORE_START=proj\r\n");

    bmeas = bare_alloc(4u);
    buc   = bare_alloc(2u);
    bgain = bare_alloc(4u);
    bso   = bare_alloc(8u);
    bsch  = bare_alloc(8u * 3u);
    meas = (int32_t *)bmeas.ptr; uc = (int16_t *)buc.ptr;
    gain = (int32_t *)bgain.ptr; so = (uint64_t *)bso.ptr; sch = (uint64_t *)bsch.ptr;
    meas[0] = 1; uc[0] = 1; gain[0] = BARE_MIN_GAIN_Q16; so[0] = 0; sch[0]=sch[1]=sch[2]=0;

    /* board_regseq.c:92-104 的顺序：5 基址 -> 7 标量 */
    seq_base64(cr, PROJ_BASE_MEAS,     bmeas.phys, "PROJ_BASE_MEAS.HI",     "PROJ_BASE_MEAS.LO");
    seq_base64(cr, PROJ_BASE_U_CODES,  buc.phys,   "PROJ_BASE_U_CODES.HI",  "PROJ_BASE_U_CODES.LO");
    seq_base64(cr, PROJ_BASE_GAIN_Q16, bgain.phys, "PROJ_BASE_GAIN_Q16.HI", "PROJ_BASE_GAIN_Q16.LO");
    seq_base64(cr, PROJ_BASE_SCORE_OUT,bso.phys,   "PROJ_BASE_SCORE_OUT.HI","PROJ_BASE_SCORE_OUT.LO");
    seq_base64(cr, PROJ_BASE_SCORE_CH, bsch.phys,  "PROJ_BASE_SCORE_CH.HI", "PROJ_BASE_SCORE_CH.LO");
    seq_write(cb + PROJ_P, 1, "PROJ_P");
    seq_write(cb + PROJ_K, 1, "PROJ_K");
    seq_write(cb + PROJ_M, 1, "PROJ_M");
    seq_write(cb + PROJ_NX, 1, "PROJ_NX");
    seq_write(cb + PROJ_NY, 1, "PROJ_NY");
    seq_write(cb + PROJ_NZ, 1, "PROJ_NZ");
    seq_write(cb + PROJ_MARGIN_THRESH_Q16, 0, "PROJ_MARGIN_THRESH_Q16");

    Xil_DCacheFlushRange((INTPTR)meas, 4u);
    Xil_DCacheFlushRange((INTPTR)uc, 2u);
    Xil_DCacheFlushRange((INTPTR)gain, 4u);

    if (seq_start(cb, "proj", BARE_FOURCORE_POLL_LIMIT) == 0) {
        uint32_t ret = BARE_R(cb + PS_FIX_REG_AP_RETURN);
        xil_printf("CORE_DONE=proj AP_RETURN=%u\r\n", (unsigned)ret);
        if (ret == 0u) { g_seq_ok_cores++; }
    }
}

static void run_fwd_small(void)
{
    const uint32_t cb = (uint32_t)PS_FIX_CORE_LR_FWD_CTRL;
    const uint32_t cr = (uint32_t)PS_FIX_CORE_LR_FWD_CTRL_R;
    bare_blk_t bU, bS, bV, bf, by, bwz, bwzs;
    int16_t *U, *V; uint16_t *S; int32_t *f, *y, *wz, *wzs;

    xil_printf("CORE_START=fwd\r\n");

    bU = bare_alloc((uint32_t)(BARE_MIN_K*BARE_MIN_M*2u));
    bS = bare_alloc((uint32_t)(BARE_MIN_K*2u));
    bV = bare_alloc((uint32_t)(BARE_MIN_K*BARE_MIN_N*2u));
    bf = bare_alloc((uint32_t)(BARE_MIN_N*4u));
    by = bare_alloc((uint32_t)(BARE_MIN_M*4u));
    bwz = bare_alloc((uint32_t)(BARE_MIN_K*4u));
    bwzs = bare_alloc((uint32_t)(BARE_MIN_K*4u));
    U=(int16_t*)bU.ptr; S=(uint16_t*)bS.ptr; V=(int16_t*)bV.ptr;
    f=(int32_t*)bf.ptr; y=(int32_t*)by.ptr; wz=(int32_t*)bwz.ptr; wzs=(int32_t*)bwzs.ptr;
    U[0]=1;U[1]=2;U[2]=3;U[3]=4; S[0]=(uint16_t)BARE_MIN_S_CODE; V[0]=1;
    f[0]=1; y[0]=y[1]=y[2]=y[3]=0; wz[0]=0; wzs[0]=0;

    /* board_regseq.c:113-129 的顺序：7 基址 -> 9 标量 */
    seq_base64(cr, LRF_BASE_U,     bU.phys,   "LRF_BASE_U.HI",     "LRF_BASE_U.LO");
    seq_base64(cr, LRF_BASE_S,     bS.phys,   "LRF_BASE_S.HI",     "LRF_BASE_S.LO");
    seq_base64(cr, LRF_BASE_V,     bV.phys,   "LRF_BASE_V.HI",     "LRF_BASE_V.LO");
    seq_base64(cr, LRF_BASE_F,     bf.phys,   "LRF_BASE_F.HI",     "LRF_BASE_F.LO");
    seq_base64(cr, LRF_BASE_Y,     by.phys,   "LRF_BASE_Y.HI",     "LRF_BASE_Y.LO");
    seq_base64(cr, LRF_BASE_WK_Z,  bwz.phys,  "LRF_BASE_WK_Z.HI",  "LRF_BASE_WK_Z.LO");
    seq_base64(cr, LRF_BASE_WK_ZS, bwzs.phys, "LRF_BASE_WK_ZS.HI", "LRF_BASE_WK_ZS.LO");
    seq_write(cb + LRF_K, BARE_MIN_K, "LRF_K");
    seq_write(cb + LRF_M, BARE_MIN_M, "LRF_M");
    seq_write(cb + LRF_N, BARE_MIN_N, "LRF_N");
    seq_write(cb + LRF_U_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "LRF_U_GAIN_Q16");
    seq_write(cb + LRF_S_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "LRF_S_GAIN_Q16");
    seq_write(cb + LRF_V_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "LRF_V_GAIN_Q16");
    seq_write(cb + LRF_U_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "LRF_U_EXP");
    seq_write(cb + LRF_S_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "LRF_S_EXP");
    seq_write(cb + LRF_V_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "LRF_V_EXP");

    Xil_DCacheFlushRange((INTPTR)U, (uint32_t)(BARE_MIN_K*BARE_MIN_M*2u));
    Xil_DCacheFlushRange((INTPTR)S, (uint32_t)(BARE_MIN_K*2u));
    Xil_DCacheFlushRange((INTPTR)V, (uint32_t)(BARE_MIN_K*BARE_MIN_N*2u));
    Xil_DCacheFlushRange((INTPTR)f, (uint32_t)(BARE_MIN_N*4u));
    Xil_DCacheFlushRange((INTPTR)y, (uint32_t)(BARE_MIN_M*4u));

    if (seq_start(cb, "fwd", BARE_FOURCORE_POLL_LIMIT) == 0) {
        uint32_t ret = BARE_R(cb + PS_FIX_REG_AP_RETURN);
        Xil_DCacheInvalidateRange((INTPTR)y, (uint32_t)(BARE_MIN_M*4u));
        xil_printf("CORE_DONE=fwd AP_RETURN=%u Y0=%d Y1=%d Y2=%d Y3=%d\r\n",
                   (unsigned)ret, (int)y[0], (int)y[1], (int)y[2], (int)y[3]);
        if (ret == 0u) { g_seq_ok_cores++; }
    }
}

static void run_adj_small(void)
{
    const uint32_t cb = (uint32_t)PS_FIX_CORE_LR_ADJ_CTRL;
    const uint32_t cr = (uint32_t)PS_FIX_CORE_LR_ADJ_CTRL_R;
    bare_blk_t bU, bS, bV, br, bg, bww, bwws;
    int16_t *U, *V; uint16_t *S; int32_t *r, *g, *ww, *wws;

    xil_printf("CORE_START=adj\r\n");

    bU = bare_alloc((uint32_t)(BARE_MIN_K*BARE_MIN_M*2u));
    bS = bare_alloc((uint32_t)(BARE_MIN_K*2u));
    bV = bare_alloc((uint32_t)(BARE_MIN_K*BARE_MIN_N*2u));
    br = bare_alloc((uint32_t)(BARE_MIN_M*4u));
    bg = bare_alloc((uint32_t)(BARE_MIN_N*4u));
    bww = bare_alloc((uint32_t)(BARE_MIN_K*4u));
    bwws = bare_alloc((uint32_t)(BARE_MIN_K*4u));
    U=(int16_t*)bU.ptr; S=(uint16_t*)bS.ptr; V=(int16_t*)bV.ptr;
    r=(int32_t*)br.ptr; g=(int32_t*)bg.ptr; ww=(int32_t*)bww.ptr; wws=(int32_t*)bwws.ptr;
    U[0]=1;U[1]=2;U[2]=3;U[3]=4; S[0]=(uint16_t)BARE_MIN_S_CODE; V[0]=1;
    r[0]=r[1]=r[2]=r[3]=1; g[0]=0; ww[0]=0; wws[0]=0;

    /* board_regseq.c:139-155 的顺序：7 基址 -> 9 标量（标量偏移与 fwd 相同） */
    seq_base64(cr, LRA_BASE_U,     bU.phys,   "LRA_BASE_U.HI",     "LRA_BASE_U.LO");
    seq_base64(cr, LRA_BASE_S,     bS.phys,   "LRA_BASE_S.HI",     "LRA_BASE_S.LO");
    seq_base64(cr, LRA_BASE_V,     bV.phys,   "LRA_BASE_V.HI",     "LRA_BASE_V.LO");
    seq_base64(cr, LRA_BASE_R,     br.phys,   "LRA_BASE_R.HI",     "LRA_BASE_R.LO");
    seq_base64(cr, LRA_BASE_G,     bg.phys,   "LRA_BASE_G.HI",     "LRA_BASE_G.LO");
    seq_base64(cr, LRA_BASE_WK_W,  bww.phys,  "LRA_BASE_WK_W.HI",  "LRA_BASE_WK_W.LO");
    seq_base64(cr, LRA_BASE_WK_WS, bwws.phys, "LRA_BASE_WK_WS.HI", "LRA_BASE_WK_WS.LO");
    seq_write(cb + LRA_K, BARE_MIN_K, "LRA_K");
    seq_write(cb + LRA_M, BARE_MIN_M, "LRA_M");
    seq_write(cb + LRA_N, BARE_MIN_N, "LRA_N");
    seq_write(cb + LRA_U_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "LRA_U_GAIN_Q16");
    seq_write(cb + LRA_S_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "LRA_S_GAIN_Q16");
    seq_write(cb + LRA_V_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "LRA_V_GAIN_Q16");
    seq_write(cb + LRA_U_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "LRA_U_EXP");
    seq_write(cb + LRA_S_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "LRA_S_EXP");
    seq_write(cb + LRA_V_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "LRA_V_EXP");

    Xil_DCacheFlushRange((INTPTR)U, (uint32_t)(BARE_MIN_K*BARE_MIN_M*2u));
    Xil_DCacheFlushRange((INTPTR)S, (uint32_t)(BARE_MIN_K*2u));
    Xil_DCacheFlushRange((INTPTR)V, (uint32_t)(BARE_MIN_K*BARE_MIN_N*2u));
    Xil_DCacheFlushRange((INTPTR)r, (uint32_t)(BARE_MIN_M*4u));
    Xil_DCacheFlushRange((INTPTR)g, (uint32_t)(BARE_MIN_N*4u));

    if (seq_start(cb, "adj", BARE_FOURCORE_POLL_LIMIT) == 0) {
        uint32_t ret = BARE_R(cb + PS_FIX_REG_AP_RETURN);
        Xil_DCacheInvalidateRange((INTPTR)g, (uint32_t)(BARE_MIN_N*4u));
        xil_printf("CORE_DONE=adj AP_RETURN=%u G0=%d\r\n", (unsigned)ret, (int)g[0]);
        if (ret == 0u) { g_seq_ok_cores++; }
    }
}

static void run_pdhg_small(void)
{
    const uint32_t cb = (uint32_t)PS_FIX_CORE_PDHG_CTRL;
    const uint32_t cr = (uint32_t)PS_FIX_CORE_PDHG_CTRL_R;
    bare_blk_t bU,bS,bV,bd3,bf3;
    bare_blk_t bfprev,bfbar,bq,bgrad,bdivq,by,br,btr,btmp;
    bare_blk_t bnu,bes,boc,brho,brs,bds,bmc;
    int16_t *U,*V; uint16_t *S; int32_t *d3,*f3;

    xil_printf("CORE_START=pdhg\r\n");

    bU=bare_alloc((uint32_t)(BARE_MIN_K*BARE_MIN_M*2u));
    bS=bare_alloc((uint32_t)(BARE_MIN_K*2u));
    bV=bare_alloc((uint32_t)(BARE_MIN_K*BARE_MIN_N*2u));
    bd3=bare_alloc(3u*4u);
    bf3=bare_alloc(3u*4u);
    bfprev=bare_alloc((uint32_t)(BARE_MIN_N*4u));
    bfbar =bare_alloc((uint32_t)(BARE_MIN_N*4u));
    bq    =bare_alloc((uint32_t)(2u*BARE_MIN_N*4u));
    bgrad =bare_alloc((uint32_t)(2u*BARE_MIN_N*4u));
    bdivq =bare_alloc((uint32_t)(BARE_MIN_N*4u));
    by    =bare_alloc((uint32_t)(BARE_MIN_M*4u));
    br    =bare_alloc((uint32_t)(BARE_MIN_M*4u));
    btr   =bare_alloc((uint32_t)(BARE_MIN_N*4u));
    btmp  =bare_alloc((uint32_t)(BARE_MIN_N*4u));
    bnu   =bare_alloc(3u*4u);
    bes   =bare_alloc(3u*4u);
    boc   =bare_alloc(3u*4u);
    brho  =bare_alloc(3u*4u);
    brs   =bare_alloc(3u*8u);
    bds   =bare_alloc(3u*8u);
    bmc   =bare_alloc(3u*8u);

    U=(int16_t*)bU.ptr; S=(uint16_t*)bS.ptr; V=(int16_t*)bV.ptr;
    U[0]=1; S[0]=(uint16_t)BARE_MIN_S_CODE; V[0]=1;
    d3=(int32_t*)bd3.ptr; f3=(int32_t*)bf3.ptr;
    d3[0]=d3[1]=d3[2]=0; f3[0]=f3[1]=f3[2]=0;

    /* board_regseq.c:164-205 的顺序：21 基址 -> 20 标量 */
    seq_base64(cr, PDHG_BASE_U,            bU.phys,    "PDHG_BASE_U.HI",            "PDHG_BASE_U.LO");
    seq_base64(cr, PDHG_BASE_S,            bS.phys,    "PDHG_BASE_S.HI",            "PDHG_BASE_S.LO");
    seq_base64(cr, PDHG_BASE_V,            bV.phys,    "PDHG_BASE_V.HI",            "PDHG_BASE_V.LO");
    seq_base64(cr, PDHG_BASE_D3,           bd3.phys,   "PDHG_BASE_D3.HI",           "PDHG_BASE_D3.LO");
    seq_base64(cr, PDHG_BASE_F3,           bf3.phys,   "PDHG_BASE_F3.HI",           "PDHG_BASE_F3.LO");
    seq_base64(cr, PDHG_BASE_WK_F_PREV,    bfprev.phys,"PDHG_BASE_WK_F_PREV.HI",    "PDHG_BASE_WK_F_PREV.LO");
    seq_base64(cr, PDHG_BASE_WK_FBAR,      bfbar.phys, "PDHG_BASE_WK_FBAR.HI",      "PDHG_BASE_WK_FBAR.LO");
    seq_base64(cr, PDHG_BASE_WK_Q,         bq.phys,    "PDHG_BASE_WK_Q.HI",         "PDHG_BASE_WK_Q.LO");
    seq_base64(cr, PDHG_BASE_WK_GRAD,      bgrad.phys, "PDHG_BASE_WK_GRAD.HI",      "PDHG_BASE_WK_GRAD.LO");
    seq_base64(cr, PDHG_BASE_WK_DIVQ,      bdivq.phys, "PDHG_BASE_WK_DIVQ.HI",      "PDHG_BASE_WK_DIVQ.LO");
    seq_base64(cr, PDHG_BASE_WK_Y,         by.phys,    "PDHG_BASE_WK_Y.HI",         "PDHG_BASE_WK_Y.LO");
    seq_base64(cr, PDHG_BASE_WK_R,         br.phys,    "PDHG_BASE_WK_R.HI",         "PDHG_BASE_WK_R.LO");
    seq_base64(cr, PDHG_BASE_WK_BTR,       btr.phys,   "PDHG_BASE_WK_BTR.HI",       "PDHG_BASE_WK_BTR.LO");
    seq_base64(cr, PDHG_BASE_WK_TMP,       btmp.phys,  "PDHG_BASE_WK_TMP.HI",       "PDHG_BASE_WK_TMP.LO");
    seq_base64(cr, PDHG_BASE_OUT_N_USED,        bnu.phys,  "PDHG_BASE_OUT_N_USED.HI",        "PDHG_BASE_OUT_N_USED.LO");
    seq_base64(cr, PDHG_BASE_OUT_EARLY_STOPPED, bes.phys,  "PDHG_BASE_OUT_EARLY_STOPPED.HI", "PDHG_BASE_OUT_EARLY_STOPPED.LO");
    seq_base64(cr, PDHG_BASE_OUT_OVERFLOW_CH,   boc.phys,  "PDHG_BASE_OUT_OVERFLOW_CH.HI",   "PDHG_BASE_OUT_OVERFLOW_CH.LO");
    seq_base64(cr, PDHG_BASE_OUT_RHO_Q16,       brho.phys, "PDHG_BASE_OUT_RHO_Q16.HI",       "PDHG_BASE_OUT_RHO_Q16.LO");
    seq_base64(cr, PDHG_BASE_OUT_RES_SQ,        brs.phys,  "PDHG_BASE_OUT_RES_SQ.HI",        "PDHG_BASE_OUT_RES_SQ.LO");
    seq_base64(cr, PDHG_BASE_OUT_D_SQ,          bds.phys,  "PDHG_BASE_OUT_D_SQ.HI",          "PDHG_BASE_OUT_D_SQ.LO");
    seq_base64(cr, PDHG_BASE_OUT_MAC_COUNT,     bmc.phys,  "PDHG_BASE_OUT_MAC_COUNT.HI",     "PDHG_BASE_OUT_MAC_COUNT.LO");

    seq_write(cb + PDHG_K, BARE_MIN_K, "PDHG_K");
    seq_write(cb + PDHG_M, BARE_MIN_M, "PDHG_M");
    seq_write(cb + PDHG_N, BARE_MIN_N, "PDHG_N");
    seq_write(cb + PDHG_U_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "PDHG_U_GAIN_Q16");
    seq_write(cb + PDHG_S_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "PDHG_S_GAIN_Q16");
    seq_write(cb + PDHG_V_GAIN_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "PDHG_V_GAIN_Q16");
    seq_write(cb + PDHG_U_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "PDHG_U_EXP");
    seq_write(cb + PDHG_S_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "PDHG_S_EXP");
    seq_write(cb + PDHG_V_EXP, (uint32_t)BARE_MIN_GAIN_EXP, "PDHG_V_EXP");
    seq_write(cb + PDHG_N_ROWS, 1, "PDHG_N_ROWS");
    seq_write(cb + PDHG_N_COLS, 1, "PDHG_N_COLS");
    seq_write(cb + PDHG_N_ITER, 1, "PDHG_N_ITER");
    seq_write(cb + PDHG_MIN_ITERS, 1, "PDHG_MIN_ITERS");
    seq_write(cb + PDHG_RESIDUAL_EVERY, 1, "PDHG_RESIDUAL_EVERY");
    seq_write(cb + PDHG_TV_MODE, 0, "PDHG_TV_MODE");
    seq_write(cb + PDHG_TAU_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "PDHG_TAU_Q16");
    seq_write(cb + PDHG_SIGMA_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "PDHG_SIGMA_Q16");
    seq_write(cb + PDHG_LAMBDA_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "PDHG_LAMBDA_Q16");
    seq_write(cb + PDHG_THETA_Q16, (uint32_t)BARE_MIN_GAIN_Q16, "PDHG_THETA_Q16");
    seq_write(cb + PDHG_RHO_STOP_Q8, 1, "PDHG_RHO_STOP_Q8");

    Xil_DCacheFlushRange((INTPTR)U, (uint32_t)(BARE_MIN_K*BARE_MIN_M*2u));
    Xil_DCacheFlushRange((INTPTR)S, (uint32_t)(BARE_MIN_K*2u));
    Xil_DCacheFlushRange((INTPTR)V, (uint32_t)(BARE_MIN_K*BARE_MIN_N*2u));
    Xil_DCacheFlushRange((INTPTR)d3, 3u*4u);
    Xil_DCacheFlushRange((INTPTR)f3, 3u*4u);

    if (seq_start(cb, "pdhg", BARE_FOURCORE_POLL_LIMIT) == 0) {
        uint32_t ret = BARE_R(cb + PS_FIX_REG_AP_RETURN);
        xil_printf("CORE_DONE=pdhg AP_RETURN=%u\r\n", (unsigned)ret);
        if (ret == 0u) { g_seq_ok_cores++; }
    }
}

static void step4_four_core(void)
{
    uint32_t before;

    /* 重置到 board_regseq 的编号：第 1 步 = PL_CTRL.VERSION（已在 step1 读过），
     * 故四核序列从第 2 步开始，末步应为 134。 */
    g_step = 1u;
    g_step_name = "PL_CTRL.VERSION";
    before = g_step;

    xil_printf("STEP=4 KIND=FOUR_CORE SEQ=proj,fwd,adj,pdhg\r\n");
    xil_printf("SEQ_FIRST_STEP=%u\r\n", (unsigned)(before + 1u));

    run_proj_small();
    run_fwd_small();
    run_adj_small();
    run_pdhg_small();

    kv_dec("SEQ_LAST_STEP", g_step);
    kv_dec("SEQ_TOTAL_STEPS", g_step);
    kv_dec("SEQ_EXPECT_TOTAL", BARE_REGSEQ_TOTAL_STEPS);
    kv_dec("SEQ_OK_CORES", (uint32_t)g_seq_ok_cores);
    xil_printf("SEQ_RESULT=%s\r\n", (g_seq_ok_cores == 4) ? "PASS" : "FAIL");
}

/* ======================================================================== */
/* main                                                                       */
/* ======================================================================== */

int main(void)
{
    xil_printf("\r\n");
    xil_printf("BARE_BUILD=baremetal_minimal_20261001\r\n");
    xil_printf("PLATFORM=AXU9EGB_ZU9EG\r\n");
    kv_dec("PS_FIX_LANES", (uint32_t)PS_FIX_LANES);

    bare_abort_guard_install();

    step1_version();
    step2_pl_clk();
    step3_min_core();
    step4_four_core();

    xil_printf("SUMMARY_VERSION=%s\r\n", g_pl_page_ok ? "PASS" : "NONZERO_NONPASS");
    xil_printf("SUMMARY_MIN_CORE=%s\r\n", g_min_pass ? "PASS" : "FAIL");
    xil_printf("SUMMARY_FOUR_CORE=%s\r\n", (g_seq_ok_cores == 4) ? "PASS" : "FAIL");
    xil_printf("DONE=1\r\n");

    for (;;) { /* 停在原地，等 XSCT 停止/重跑 */ }
    /* return 0; */
}
