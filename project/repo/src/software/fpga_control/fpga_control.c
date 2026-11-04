// ============================================================================
// fpga_control.c —— §8.2 AXI-Lite 寄存器的读写、作业提交与中断等待
//
// 三个后端，同一张函数指针表（见 fpga_control.h）
// ---------------------------------------------------------------------------
//   stub  影子寄存器：把 16 个字放在内存里，按同样的位定义与允许集合工作。
//         主机自检测的就是**寄存器协议**：默认值、允许集合、错误码、64 bit
//         拆装、超时复位、低可信度门限。
//   uio   /dev/uioN + mmap：**留桩**，打开即返回 Unavailable。
//   xrt   XRT：**留桩**，同上。
//
// 板端实现 UIO 的注意事项（按这个顺序做）
// ---------------------------------------------------------------------------
//   1. 寄存器窗口基址：
//        cat /sys/class/uio/uioN/name                 # 名字里应能认出本 IP
//        cat /sys/class/uio/uioN/maps/map0/addr       # 物理基址
//        cat /sys/class/uio/uioN/maps/map0/size       # 窗口大小（应 = PS_REG_BYTES）
//      fd = open("/dev/uioN", O_RDWR | O_SYNC);
//      map = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
//      映射失败的典型原因是设备树里 reg 的 size 写小了（只写 4 KB 而不是 64 KB）。
//   2. 读写：*(volatile uint32_t*)((char*)map + off)；**必须 volatile**，
//      否则 -O2 下编译器会认为「读回来的一定等于刚写进去的」而优化掉整个读。
//   3. 中断：read(fd, &count, 4) 阻塞等；**收到后必须 write(fd, &one4, 4) 重新
//      使能**，否则只会收到第一次中断（UIO 最经典的坑，且症状是「第二次作业
//      永远超时」）。超时用 poll(fd, POLLIN, timeout_ms) 包一层。
//   4. 超时与复位：等待中断超时 2 s（Task 10 Step 4 原话）。超时后先写
//      CONTROL.reset=1，再读 STATUS 确认 busy 已落，然后才允许下一次提交。
//      超时与复位次数必须进 telemetry，不能只打印一行。
//   5. buffer 地址：把 model_loader 给的物理地址写进 0x28~0x3C，且必须先确认
//      4 KB 对齐（跨 4 KB 的 AXI 突发会被拆包）。
//   6. XRT 替代路径：xrt::device(0) + xrt::ip，ip.read_register(off) /
//      ip.write_register(off, val)；中断用 xrt::run::wait(timeout)。
//      工具链版本必须与 BSP/EDF 同一发布系列。
//
// 桩件的数字是**设计目标**，不是实测值
// ---------------------------------------------------------------------------
//   影子后端按 §5.3 的设计目标推算 LATENCY_CYCLES（定位 ≤100 ms、16 次重建
//   ≤300 ms、150 MHz）。它只保证寄存器语义、量纲与位定义正确，
//   **不构成任何性能结论**（§总体约束：实测前一律标注为设计目标）。
// ============================================================================

#include "fpga_control.h"

// 需要 PS_PSM_ARRAY_ALIGN（模型地址必须 4 KB 对齐）与 B 档隐藏图尺寸
// （算重建输出缓冲字节数）。寄存器层与模型格式层的耦合只有这两处，
// 故直接复用冻结的 psm_format.h，而不是各自抄一份常量。
#include "psm_format.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <time.h>
#endif

// ---------------------------------------------------------------- 平台小件

static void ps_fpga_sleep_ms(unsigned ms) {
#if defined(_WIN32)
    Sleep((DWORD)ms);
#else
    struct timespec ts;
    ts.tv_sec = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    nanosleep(&ts, NULL);
#endif
}

// 极小的字符串拼接器：所有对外文本都从这里出，保证 NUL 结尾且不越界。
typedef struct {
    char*  buf;
    size_t cap;
    size_t used;
} ps_sb_t;

static void ps_sb_init(ps_sb_t* sb, char* buf, size_t cap) {
    sb->buf = buf;
    sb->cap = cap;
    sb->used = 0u;
    if (buf && cap > 0u) buf[0] = '\0';
}

static void ps_sb_put(ps_sb_t* sb, const char* s) {
    if (!sb->buf || sb->cap == 0u || !s) return;
    while (*s != '\0' && sb->used + 1u < sb->cap) {
        sb->buf[sb->used++] = *s++;
    }
    sb->buf[sb->used] = '\0';
}

static void ps_sb_fmt(ps_sb_t* sb, const char* fmt, ...) {
    if (!sb->buf || sb->cap == 0u || !fmt) return;
    if (sb->used + 1u >= sb->cap) return;
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(sb->buf + sb->used, sb->cap - sb->used, fmt, ap);
    va_end(ap);
    if (n > 0) {
        const size_t adv = (size_t)n;
        sb->used = (sb->used + adv < sb->cap) ? (sb->used + adv) : (sb->cap - 1u);
    }
}

// 内容指纹（决定论地模拟「定位输出」）。不是校验和，只是让桩件的结果可复现。
static uint64_t ps_fpga_fnv1a(const void* data, size_t bytes) {
    const unsigned char* p = (const unsigned char*)data;
    uint64_t h = 1469598103934665603ull;
    size_t i;
    for (i = 0u; i < bytes; ++i) {
        h ^= (uint64_t)p[i];
        h *= 1099511628211ull;
    }
    return h;
}

const char* ps_fpga_status_name(int rc) {
    switch (rc) {
        case PS_FPGA_OK:              return "OK";
        case PS_FPGA_ERR_BAD_PARAM:   return "参数非法";
        case PS_FPGA_ERR_TIMEOUT:     return "等待中断超时";
        case PS_FPGA_ERR_IO:          return "映射/读写失败";
        case PS_FPGA_ERR_UNAVAILABLE: return "真实后端未实现（实物未到）";
        default:                      return "未知状态";
    }
}

// ---------------------------------------------------------------- 桩件后端

static int ps_stub_open(void* ctx, const char* dev, char* err, size_t errcap) {
    ps_fpga_stub_ctx_t* c = (ps_fpga_stub_ctx_t*)ctx;
    (void)dev;
    (void)err;
    (void)errcap;
    if (!c) return PS_FPGA_ERR_IO;
    memset(c, 0, sizeof(*c));
    // 复位后的默认值 = §8.2 表格里写的值。桩件用它们初始化，于是
    // 「没配置就提交作业」时的行为与板上一致。
    c->regs[PS_REG_FRAME_COUNT / 4u]     = PS_REG_DEFAULT_FRAME_COUNT;
    c->regs[PS_REG_ITER_COUNT / 4u]      = PS_REG_DEFAULT_ITER_COUNT;
    c->regs[PS_REG_RANK / 4u]            = PS_REG_DEFAULT_RANK;
    c->regs[PS_REG_CANDIDATE_COUNT / 4u] = PS_REG_DEFAULT_CANDIDATE_COUNT;
    return PS_FPGA_OK;
}

static void ps_stub_close(void* ctx) {
    (void)ctx;
}

static int ps_stub_read32(void* ctx, uint32_t off, uint32_t* val) {
    ps_fpga_stub_ctx_t* c = (ps_fpga_stub_ctx_t*)ctx;
    if (!c || !val) return PS_FPGA_ERR_IO;
    if (off >= PS_REG_BYTES || (off % PS_REG_WORD_BYTES) != 0u) {
        return PS_FPGA_ERR_IO;
    }
    *val = c->regs[off / PS_REG_WORD_BYTES];
    return PS_FPGA_OK;
}

static int ps_stub_write32(void* ctx, uint32_t off, uint32_t val) {
    ps_fpga_stub_ctx_t* c = (ps_fpga_stub_ctx_t*)ctx;
    if (!c) return PS_FPGA_ERR_IO;
    if (off >= PS_REG_BYTES || (off % PS_REG_WORD_BYTES) != 0u) {
        return PS_FPGA_ERR_IO;
    }
    // 记下写序列（追加；满了丢最旧的），供「64 bit 地址对写序」这类断言使用。
    if (c->trace_n < PS_FPGA_WRITE_TRACE_MAX) {
        c->trace[c->trace_n].off = off;
        c->trace[c->trace_n].val = val;
        ++c->trace_n;
    } else {
        int i;
        for (i = 1; i < PS_FPGA_WRITE_TRACE_MAX; ++i) c->trace[i - 1] = c->trace[i];
        c->trace[PS_FPGA_WRITE_TRACE_MAX - 1].off = off;
        c->trace[PS_FPGA_WRITE_TRACE_MAX - 1].val = val;
    }
    c->regs[off / PS_REG_WORD_BYTES] = val;
    return PS_FPGA_OK;
}

// 等一次中断。桩件下 1 ms 一步轮询 irq_pending：
//   * 作业正常完成 -> 已经置了 irq_pending，立刻返回；
//   * 注入超时的作业 -> 永远不置 DONE，于是真的会超时。
// 这一点是刻意的：若不等中断而直接返回成功，「中断丢了导致超时」这条路径
// 就永远不被执行 —— 而它恰恰是上板最容易出问题的地方（UIO 忘记重新使能中断）。
static int ps_stub_wait_irq(void* ctx, int timeout_ms) {
    ps_fpga_stub_ctx_t* c = (ps_fpga_stub_ctx_t*)ctx;
    if (!c) return PS_FPGA_ERR_IO;
    int waited = 0;
    while (waited <= timeout_ms) {
        if (c->irq_pending) {
            c->irq_pending = 0;
            c->irq_count++;
            return PS_FPGA_OK;
        }
        ps_fpga_sleep_ms(1u);
        ++waited;
    }
    c->timeout_count++;
    return PS_FPGA_ERR_TIMEOUT;
}

static const ps_fpga_backend_t g_backend_stub = {
    "stub", ps_stub_open, ps_stub_close, ps_stub_read32, ps_stub_write32,
    ps_stub_wait_irq};

// ---------------------------------------------------------------- 留桩后端

static int ps_unavail_open(void* ctx, const char* dev, char* err, size_t errcap) {
    (void)ctx;
    if (err && errcap > 0u) {
        (void)snprintf(err, errcap,
                       "真实后端未实现：设备 %s 不可用（TODO 见 fpga_control.c）",
                       dev ? dev : "(null)");
    }
    return PS_FPGA_ERR_UNAVAILABLE;
}
static void ps_unavail_close(void* ctx) { (void)ctx; }
static int ps_unavail_read32(void* ctx, uint32_t off, uint32_t* val) {
    (void)ctx; (void)off; (void)val;
    return PS_FPGA_ERR_UNAVAILABLE;
}
static int ps_unavail_write32(void* ctx, uint32_t off, uint32_t val) {
    (void)ctx; (void)off; (void)val;
    return PS_FPGA_ERR_UNAVAILABLE;
}
static int ps_unavail_wait_irq(void* ctx, int timeout_ms) {
    (void)ctx; (void)timeout_ms;
    return PS_FPGA_ERR_UNAVAILABLE;
}

static const ps_fpga_backend_t g_backend_uio = {
    "uio", ps_unavail_open, ps_unavail_close, ps_unavail_read32,
    ps_unavail_write32, ps_unavail_wait_irq};

static const ps_fpga_backend_t g_backend_xrt = {
    "xrt", ps_unavail_open, ps_unavail_close, ps_unavail_read32,
    ps_unavail_write32, ps_unavail_wait_irq};

const ps_fpga_backend_t* ps_fpga_backend_stub(void) { return &g_backend_stub; }
const ps_fpga_backend_t* ps_fpga_backend_uio(void)  { return &g_backend_uio; }
const ps_fpga_backend_t* ps_fpga_backend_xrt(void)  { return &g_backend_xrt; }

// ---------------------------------------------------------------- 打开/关闭

int ps_fpga_open(ps_fpga_t* d, const char* dev) {
    if (!d) return PS_FPGA_ERR_IO;
    memset(d, 0, sizeof(*d));

    const int want_stub = (dev == NULL) || (dev[0] == '\0') ||
                          (strcmp(dev, "stub") == 0);
    if (want_stub) {
        d->backend = &g_backend_stub;
    } else if (strncmp(dev, "xrt:", 4) == 0) {
        d->backend = &g_backend_xrt;
    } else {
        d->backend = &g_backend_uio;
    }
    if (dev) {
        const size_t n = strlen(dev);
        const size_t k = (n < sizeof(d->dev) - 1u) ? n : (sizeof(d->dev) - 1u);
        memcpy(d->dev, dev, k);
        d->dev[k] = '\0';
    }

    d->ctx = &d->stub;
    char err[160];
    err[0] = '\0';
    const int rc = d->backend->open(d->ctx, d->dev, err, sizeof(err));
    if (rc != PS_FPGA_OK) {
        fprintf(stderr, "[fpga_control] %s\n", err[0] ? err : "打开失败");
        return rc;
    }
    d->opened = 1;
    return PS_FPGA_OK;
}

void ps_fpga_close(ps_fpga_t* d) {
    if (!d) return;
    if (d->backend && d->backend->close) d->backend->close(d->ctx);
    // TODO(板端)：munmap(d->mmio.map, d->mmio.map_bytes) + close(d->mmio.fd)。
    d->opened = 0;
}

// ---------------------------------------------------------------- 寄存器访问

int ps_fpga_read32(ps_fpga_t* d, uint32_t off, uint32_t* val) {
    if (!d || !d->opened || !val) return PS_FPGA_ERR_IO;
    // 越界与未对齐在 PS 侧就挡住：板上一旦真的发出这次访问，AXI 会返回
    // SLVERR 或者更糟 —— 静默写到相邻寄存器。
    if (off >= PS_REG_BYTES || (off % PS_REG_WORD_BYTES) != 0u) {
        return PS_FPGA_ERR_IO;
    }
    return d->backend->read32(d->ctx, off, val);
}

int ps_fpga_write32(ps_fpga_t* d, uint32_t off, uint32_t val) {
    if (!d || !d->opened) return PS_FPGA_ERR_IO;
    if (off >= PS_REG_BYTES || (off % PS_REG_WORD_BYTES) != 0u) {
        return PS_FPGA_ERR_IO;
    }
    return d->backend->write32(d->ctx, off, val);
}

// 64 bit 地址：**先写 HI(+4) 再写 LO(+0)**。
// 理由（与 board/fix_core.c 的 ps_fix_set_base 同一条纪律）：硬件在 START 之后
// 才开始取地址，但如果先落 LO，两次写之间会出现「新 LO + 旧 HI」这种半截地址，
// 硬件可能据此去访问一个不存在的缓冲区。先写 HI 则 LO 落笔时 HI 必已是新值。
// 注意：**读**的顺序不改（read_addr64 仍是 LO 再 HI），读不产生半截地址窗口。
int ps_fpga_write_addr64(ps_fpga_t* d, uint32_t off_lo, uint64_t addr) {
    int rc = ps_fpga_write32(d, off_lo + PS_REG_WORD_BYTES, ps_reg_addr_hi(addr));
    if (rc != PS_FPGA_OK) return rc;
    return ps_fpga_write32(d, off_lo, ps_reg_addr_lo(addr));
}

int ps_fpga_read_addr64(ps_fpga_t* d, uint32_t off_lo, uint64_t* out) {
    uint32_t lo = 0u, hi = 0u;
    int rc = ps_fpga_read32(d, off_lo, &lo);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_read32(d, off_lo + PS_REG_WORD_BYTES, &hi);
    if (rc != PS_FPGA_OK) return rc;
    if (out) *out = ps_reg_addr_join(lo, hi);
    return PS_FPGA_OK;
}

int ps_fpga_wait_irq(ps_fpga_t* d, int timeout_ms) {
    if (!d || !d->opened) return PS_FPGA_ERR_IO;
    return d->backend->wait_irq(d->ctx, timeout_ms);
}

int ps_fpga_reset(ps_fpga_t* d) {
    if (!d || !d->opened) return PS_FPGA_ERR_IO;
    // 只清状态/控制/结果，保留配置寄存器（帧数/迭代/秩/候选数）。
    int rc = ps_fpga_write32(d, PS_REG_CONTROL, 0u);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write32(d, PS_REG_STATUS, 0u);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write32(d, PS_REG_ERROR_CODE, PS_REG_ERR_NONE);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write32(d, PS_REG_SELECTED_ID, 0u);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write32(d, PS_REG_CONFIDENCE, 0u);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write32(d, PS_REG_LATENCY_CYCLES, 0u);
    if (rc != PS_FPGA_OK) return rc;
    if (d->backend == &g_backend_stub) {
        d->stub.irq_pending = 0;
        d->stub.reset_count++;
    }
    return PS_FPGA_OK;
}

// ---------------------------------------------------------------- 配置

int ps_fpga_configure(ps_fpga_t* d, const ps_reg_job_config_t* cfg) {
    if (!d || !d->opened || !cfg) return PS_FPGA_ERR_IO;
    if (!ps_reg_frame_count_valid(cfg->frame_count) ||
        !ps_reg_iter_count_valid(cfg->iter_count) ||
        !ps_reg_rank_valid(cfg->rank)) {
        (void)ps_fpga_write32(d, PS_REG_ERROR_CODE, PS_REG_ERR_ILLEGAL_PARAM);
        (void)ps_fpga_write32(d, PS_REG_STATUS, PS_STS_ERROR);
        if (d->backend == &g_backend_stub) d->stub.bad_param_count++;
        fprintf(stderr,
                "[fpga_control] 参数非法：frames=%u iters=%u rank=%u"
                "（允许集合见 ps_regs.h）\n",
                (unsigned)cfg->frame_count, (unsigned)cfg->iter_count,
                (unsigned)cfg->rank);
        return PS_FPGA_ERR_BAD_PARAM;
    }

    int rc = ps_fpga_write32(d, PS_REG_FRAME_COUNT, cfg->frame_count);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write32(d, PS_REG_ITER_COUNT, cfg->iter_count);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write32(d, PS_REG_RANK, cfg->rank);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write32(d, PS_REG_CANDIDATE_COUNT,
                         cfg->candidate_count ? cfg->candidate_count
                                              : PS_REG_DEFAULT_CANDIDATE_COUNT);
    if (rc != PS_FPGA_OK) return rc;

    ps_reg_control_bits_t b;
    b.start = 0u;
    b.reset = 0u;
    b.selfloc = cfg->selfloc ? 1u : 0u;
    b.rgb = cfg->rgb ? 1u : 0u;
    return ps_fpga_write32(d, PS_REG_CONTROL, ps_reg_pack_control(&b));
}

// ---------------------------------------------------------------- 提交作业

int ps_fpga_submit(ps_fpga_t* d, const ps_reg_job_config_t* cfg,
                   const ps_reg_buffer_addrs_t* addrs, ps_reg_job_result_t* out,
                   int timeout_ms) {
    if (!d || !d->opened || !cfg || !addrs || !out) return PS_FPGA_ERR_IO;

    // 模型必须先装载：地址为 0 时立刻置错误码返回。板上同语义 ——
    // 宁可不出图，也不要拿 DDR 里的垃圾当模型算。
    if (addrs->model == 0u) {
        (void)ps_fpga_write32(d, PS_REG_ERROR_CODE, PS_REG_ERR_MODEL_NOT_LOADED);
        (void)ps_fpga_write32(d, PS_REG_STATUS, PS_STS_ERROR);
        if (d->backend == &g_backend_stub) d->stub.bad_param_count++;
        fprintf(stderr, "[fpga_control] 模型地址为 0：先跑 model_loader\n");
        return PS_FPGA_ERR_BAD_PARAM;
    }
    // 4 KB 对齐是硬要求：跨 4 KB 边界的 AXI 突发会被拆包，§5.3 的带宽假设作废。
    if ((addrs->model % (uint64_t)PS_PSM_ARRAY_ALIGN) != 0u) {
        (void)ps_fpga_write32(d, PS_REG_ERROR_CODE, PS_REG_ERR_ILLEGAL_PARAM);
        (void)ps_fpga_write32(d, PS_REG_STATUS, PS_STS_ERROR);
        if (d->backend == &g_backend_stub) d->stub.bad_param_count++;
        fprintf(stderr, "[fpga_control] 模型地址未按 4096 对齐：0x%llX\n",
                (unsigned long long)addrs->model);
        return PS_FPGA_ERR_BAD_PARAM;
    }

    int rc = ps_fpga_configure(d, cfg);
    if (rc != PS_FPGA_OK) return rc;

    rc = ps_fpga_write_addr64(d, PS_REG_RAW_ADDR_LO, addrs->raw);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write_addr64(d, PS_REG_MODEL_ADDR_LO, addrs->model);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write_addr64(d, PS_REG_OUT_ADDR_LO, addrs->output);
    if (rc != PS_FPGA_OK) return rc;

    (void)ps_fpga_write32(d, PS_REG_ERROR_CODE, PS_REG_ERR_NONE);
    (void)ps_fpga_write32(d, PS_REG_STATUS, PS_STS_BUSY);

    uint32_t ctrl = 0u;
    rc = ps_fpga_read32(d, PS_REG_CONTROL, &ctrl);
    if (rc != PS_FPGA_OK) return rc;
    rc = ps_fpga_write32(d, PS_REG_CONTROL,
                         (ctrl & PS_CTRL_BIT_MASK) | PS_CTRL_START);
    if (rc != PS_FPGA_OK) return rc;

    if (d->backend != &g_backend_stub) {
        // 真实后端：等中断，再回读状态与结果。
        rc = ps_fpga_wait_irq(d, timeout_ms);
        if (rc != PS_FPGA_OK) return rc;
        uint32_t st = 0u, errc = 0u, sel = 0u, conf = 0u, lat = 0u;
        (void)ps_fpga_read32(d, PS_REG_STATUS, &st);
        (void)ps_fpga_read32(d, PS_REG_ERROR_CODE, &errc);
        (void)ps_fpga_read32(d, PS_REG_SELECTED_ID, &sel);
        (void)ps_fpga_read32(d, PS_REG_CONFIDENCE, &conf);
        (void)ps_fpga_read32(d, PS_REG_LATENCY_CYCLES, &lat);
        memset(out, 0, sizeof(*out));
        out->selected_id = sel;
        out->candidate_count = cfg->candidate_count;
        out->confidence_q16 = (uint16_t)(conf & 0xFFFFu);
        out->rank = (uint16_t)cfg->rank;
        out->iterations = (uint16_t)cfg->iter_count;
        out->error_code = (uint16_t)(errc & 0xFFFFu);
        out->flags = (st & PS_STS_LOW_CONF) ? (uint16_t)PS_REG_JOB_FLAG_LOW_CONF
                                           : (uint16_t)0u;
        out->latency_cycles = lat;
        out->latency_us = ps_reg_cycles_to_us(lat);
        if (st & PS_STS_ERROR) return PS_FPGA_ERR_IO;
        return PS_FPGA_OK;
    }

    // ---------------- 影子寄存器：模拟一次 PL 作业 ----------------
    ps_fpga_stub_ctx_t* c = &d->stub;
    c->irq_pending = 0;      // 新作业开始前不残留上一次中断
    c->submit_count++;

    // 设计目标（§5.3 / §1.1 B 档，150 MHz）：
    //   定位 ≤ 100 ms；16 次重建 ≤ 300 ms（按迭代数线性摊）。
    {
        const uint64_t loc_cycles = cfg->selfloc
            ? (uint64_t)PS_REG_PL_CLOCK_MHZ * 100000u     /* 100 ms */
            : 0u;
        const uint64_t per_iter = (uint64_t)PS_REG_PL_CLOCK_MHZ * 300000u / 16u;
        const uint64_t cycles = loc_cycles + per_iter * (uint64_t)cfg->iter_count;
        c->cycle += cycles;

        if ((cfg->flags & PS_JOB_CFG_FLAG_INJECT_TIMEOUT) != 0u) {
            // 注入超时：保持 BUSY、不置 DONE、不产生中断 —— 板上这时是真卡住了。
            (void)ps_fpga_wait_irq(d, timeout_ms);
            fprintf(stderr,
                    "[fpga_control] 作业超时（注入）：PL 仍 busy，需要复位\n");
            return PS_FPGA_ERR_TIMEOUT;
        }

        const uint32_t cand = cfg->candidate_count ? cfg->candidate_count
                                                   : PS_REG_DEFAULT_CANDIDATE_COUNT;
        uint32_t sel = cfg->selfloc
            ? (uint32_t)(ps_fpga_fnv1a(&cfg->job_id, sizeof(cfg->job_id))
                         % (uint64_t)cand)
            : (uint32_t)cfg->fixed_candidate_id;
        if (sel >= cand) sel = cand - 1u;   // 越界钳到边界，不静默环绕

        // margin/confidence 是**占位量纲**：真实值来自定位引擎的最佳/次佳分数
        // （§6.4 margin = (s1-s2)/(s1+eps)）。桩件令 confidence := margin，
        // 好处是量纲一致、门限比较真实执行。
        // 取值区间**刻意跨过可信度门限**（lo < 门限 <= 上界），于是 low_conf 的
        // 两种结果都会出现 —— 门限比较若不是真的在执行，自检里这一条就会挂。
        // 真实引擎里 confidence 是「输入 SNR / 候选 margin / 归一化残差 / 饱和
        // 比例」四项复合（§6.6），不是这里的占位量。
        const uint32_t mix = (uint32_t)ps_fpga_fnv1a(&sel, sizeof(sel));
        const uint32_t margin_lo = PS_REG_CONF_THRESHOLD_Q16 / 4u;            // 0.125
        const uint32_t margin_span = PS_REG_CONF_THRESHOLD_Q16 + 0x4000u;     // 0.75
        const uint16_t margin_q16 =
            (uint16_t)(margin_lo + (mix % margin_span));                      // [0.125,0.875)
        const uint16_t conf_q16 = margin_q16;
        const uint16_t flags = ps_reg_confidence_is_low(conf_q16)
            ? (uint16_t)PS_REG_JOB_FLAG_LOW_CONF : (uint16_t)0u;

        const uint64_t hidden_len = (uint64_t)PS_PSM_HIDDEN_H * PS_PSM_HIDDEN_W;
        const uint64_t out_bytes = hidden_len * 4ull * (cfg->rgb ? 3ull : 1ull);

        (void)ps_fpga_write32(d, PS_REG_SELECTED_ID, sel);
        (void)ps_fpga_write32(d, PS_REG_CONFIDENCE, conf_q16);
        (void)ps_fpga_write32(d, PS_REG_LATENCY_CYCLES, (uint32_t)cycles);
        (void)ps_fpga_write32(d, PS_REG_ERROR_CODE, PS_REG_ERR_NONE);
        (void)ps_fpga_write32(d, PS_REG_STATUS,
                              PS_STS_DONE | (flags ? PS_STS_LOW_CONF : 0u));
        c->irq_pending = 1;

        rc = ps_fpga_wait_irq(d, timeout_ms);
        if (rc != PS_FPGA_OK) return rc;

        memset(out, 0, sizeof(*out));
        out->selected_id = sel;
        out->candidate_count = cand;
        out->confidence_q16 = conf_q16;
        out->margin_q16 = margin_q16;
        out->rank = (uint16_t)cfg->rank;
        out->iterations = (uint16_t)cfg->iter_count;
        out->flags = flags;
        out->error_code = (uint16_t)PS_REG_ERR_NONE;
        out->latency_cycles = cycles;
        out->latency_us = ps_reg_cycles_to_us(cycles);
        out->output_bytes = out_bytes;
        out->model_hash = 0u;   // 由调用方（作业上下文）回填，桩件不编造哈希
    }
    return PS_FPGA_OK;
}

// ---------------------------------------------------------------- 观测

const char* ps_fpga_backend_name(const ps_fpga_t* d) {
    if (!d || !d->backend) return "none";
    return d->backend->name;
}

int ps_fpga_is_stub(const ps_fpga_t* d) {
    return (d && d->backend == &g_backend_stub) ? 1 : 0;
}

uint64_t ps_fpga_irq_count(const ps_fpga_t* d) {
    return (d && ps_fpga_is_stub(d)) ? d->stub.irq_count : 0ull;
}
uint64_t ps_fpga_timeout_count(const ps_fpga_t* d) {
    return (d && ps_fpga_is_stub(d)) ? d->stub.timeout_count : 0ull;
}
uint64_t ps_fpga_reset_count(const ps_fpga_t* d) {
    return (d && ps_fpga_is_stub(d)) ? d->stub.reset_count : 0ull;
}
uint64_t ps_fpga_submit_count(const ps_fpga_t* d) {
    return (d && ps_fpga_is_stub(d)) ? d->stub.submit_count : 0ull;
}

int ps_fpga_stub_write_trace(const ps_fpga_t* d, ps_fpga_write_ev_t* out, int cap) {
    if (!d || !ps_fpga_is_stub(d)) return 0;
    {
        int n = d->stub.trace_n;
        if (n > PS_FPGA_WRITE_TRACE_MAX) n = PS_FPGA_WRITE_TRACE_MAX;
        if (out && cap > 0) {
            int i;
            for (i = 0; i < n && i < cap; ++i) out[i] = d->stub.trace[i];
            if (n > cap) n = cap;
        }
        return n;
    }
}

void ps_fpga_stub_trace_reset(ps_fpga_t* d) {
    if (!d || !ps_fpga_is_stub(d)) return;
    d->stub.trace_n = 0;
}

// §8.2 寄存器表整表打印：上板联调第一页。
int ps_fpga_dump_regs(const ps_fpga_t* d, char* out, size_t cap) {
    if (!out || cap == 0u) return -1;
    ps_sb_t sb;
    ps_sb_init(&sb, out, cap);

    struct Row {
        uint32_t    off;
        const char* name;
        const char* note;
    };
    static const struct Row kRows[PS_REG_WORDS] = {
        {PS_REG_CONTROL,         "CONTROL",         "bit0 start/bit1 reset/bit2 selfloc/bit3 rgb"},
        {PS_REG_STATUS,          "STATUS",          "bit0 busy/bit1 done/bit2 error/bit3 low_conf"},
        {PS_REG_FRAME_COUNT,     "FRAME_COUNT",     "1/4/8"},
        {PS_REG_ITER_COUNT,      "ITER_COUNT",      "8/12/16/32"},
        {PS_REG_RANK,            "RANK",            "32/48/64/96"},
        {PS_REG_CANDIDATE_COUNT, "CANDIDATE_COUNT", "默认 243"},
        {PS_REG_SELECTED_ID,     "SELECTED_ID",     "定位输出或固定输入"},
        {PS_REG_CONFIDENCE,      "CONFIDENCE",      "Q0.16"},
        {PS_REG_LATENCY_CYCLES,  "LATENCY_CYCLES",  "PL 处理周期"},
        {PS_REG_ERROR_CODE,      "ERROR_CODE",      "0 = 无错"},
        {PS_REG_RAW_ADDR_LO,     "RAW_ADDR_LO",     "64 bit 低半"},
        {PS_REG_RAW_ADDR_HI,     "RAW_ADDR_HI",     "64 bit 高半"},
        {PS_REG_MODEL_ADDR_LO,   "MODEL_ADDR_LO",   "64 bit 低半"},
        {PS_REG_MODEL_ADDR_HI,   "MODEL_ADDR_HI",   "64 bit 高半"},
        {PS_REG_OUT_ADDR_LO,     "OUT_ADDR_LO",     "64 bit 低半"},
        {PS_REG_OUT_ADDR_HI,     "OUT_ADDR_HI",     "64 bit 高半"},
    };

    uint32_t regs[PS_REG_WORDS];
    memset(regs, 0, sizeof(regs));
    unsigned i;
    if (d) {
        for (i = 0u; i < PS_REG_WORDS; ++i) {
            (void)ps_fpga_read32((ps_fpga_t*)d, (uint32_t)(i * 4u), &regs[i]);
        }
    }

    ps_sb_put(&sb, "off    name             value      含义\n");
    for (i = 0u; i < PS_REG_WORDS; ++i) {
        const struct Row* r = &kRows[i];
        char decoded[64];
        decoded[0] = '\0';
        if (r->off == PS_REG_CONTROL) {
            (void)ps_reg_decode_control(regs[i], decoded, sizeof(decoded));
        } else if (r->off == PS_REG_STATUS) {
            (void)ps_reg_decode_status(regs[i], decoded, sizeof(decoded));
        } else if (r->off == PS_REG_ERROR_CODE) {
            (void)snprintf(decoded, sizeof(decoded), "%s",
                           ps_reg_error_name(regs[i]));
        } else if (r->off == PS_REG_CONFIDENCE) {
            (void)snprintf(decoded, sizeof(decoded), "= %.4f",
                           ps_reg_q16_to_double((uint16_t)regs[i]));
        } else {
            (void)snprintf(decoded, sizeof(decoded), "%s", r->note);
        }
        ps_sb_fmt(&sb, "0x%02X   %-16s 0x%-8X %s\n",
                  (unsigned)r->off, r->name, (unsigned)regs[i], decoded);
    }
    // 三对地址单独合成 64 bit 显示，避免有人只看 LO 就下结论。
    {
        const uint32_t los[3] = {PS_REG_RAW_ADDR_LO, PS_REG_MODEL_ADDR_LO,
                                 PS_REG_OUT_ADDR_LO};
        const char* names[3] = {"RAW  ", "MODEL", "OUT  "};
        unsigned k;
        for (k = 0u; k < 3u; ++k) {
            const uint64_t a = ps_reg_addr_join(regs[los[k] / 4u],
                                                regs[los[k] / 4u + 1u]);
            ps_sb_fmt(&sb, "%s 64 bit 地址 = 0x%llX\n", names[k],
                      (unsigned long long)a);
        }
    }
    return (int)sb.used;
}
