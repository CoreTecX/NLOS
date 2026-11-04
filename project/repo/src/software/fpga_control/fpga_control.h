// ============================================================================
// fpga_control.h —— PS 侧 PL 控制（方案 §7 第 3 条 / Task 10 Step 4）
//
// 职责：
//   * 把作业配置写进 §8.2 的 AXI-Lite 寄存器（帧数/迭代/秩/候选数/自定位/RGB）；
//   * 写三个 buffer 的 64 bit 物理地址（RAW / model / output）；
//   * 提交作业、等待中断、回读 selected_id / confidence / latency_cycles；
//   * 等待中断超时（板端 2 s）后复位，并把「超时发生过」如实报出来。
//
// 不做：不生成模型、不解释 U/S/V、不做重建（那是 PL 的活）。
//
// 寄存器读写为什么是「一组函数指针」
// ---------------------------------------------------------------------------
// 方案 §7 第 3 条给的是「通过 UIO **或** XRT」两条路。两者在 PS 侧的差别只有
// 「怎么把 32 bit 写进某个偏移、怎么等一次中断」，其余逻辑（允许集合、地址对齐、
// 超时复位、结果回读）完全一样。把差异收进一张函数指针表：
//
//     ps_fpga_backend_t { open, close, read32, write32, wait_irq }
//
// 于是：换后端不动上层；主机上没有 PL，就用同一张表实现一个「影子寄存器」后端，
// 自检测的是**寄存器协议本身**（位定义、允许集合、错误码、64 bit 拆装、超时复位），
// 而不是假的返回值。
//
// 三个后端
// ---------------------------------------------------------------------------
//   ps_fpga_backend_stub()   影子寄存器（主机自检 / 联调）
//   ps_fpga_backend_uio()    /dev/uioN + mmap（**留桩**：打开即返回 Unavailable）
//   ps_fpga_backend_xrt()    XRT（**留桩**：同上）
// 留桩后端刻意**明确失败**而不是假装成功：假装成功会让「实物未到」这件事
// 一路潜伏到演示现场。
// ============================================================================

#ifndef PENUMBRASIGHT_FPGA_CONTROL_H
#define PENUMBRASIGHT_FPGA_CONTROL_H

#include <stdint.h>

#include "ps_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- 返回码

#define PS_FPGA_OK             0
#define PS_FPGA_ERR_BAD_PARAM  1   // 参数不在允许集合 / 模型地址为 0 或未对齐
#define PS_FPGA_ERR_TIMEOUT    2   // 等待中断超时（板端 = 2 s）
#define PS_FPGA_ERR_IO         3   // 映射/读写失败
#define PS_FPGA_ERR_UNAVAILABLE 4  // 真实后端未实现（实物未到）

const char* ps_fpga_status_name(int rc);

// ---------------------------------------------------------------- 后端表

// 一张表 = 一个后端。ctx 由各后端自己解释（桩件用 ps_fpga_stub_ctx_t）。
typedef struct ps_fpga_backend {
    const char* name;
    int  (*open)(void* ctx, const char* dev, char* err, size_t errcap);
    void (*close)(void* ctx);
    int  (*read32)(void* ctx, uint32_t off, uint32_t* val);
    int  (*write32)(void* ctx, uint32_t off, uint32_t val);
    int  (*wait_irq)(void* ctx, int timeout_ms);
} ps_fpga_backend_t;

const ps_fpga_backend_t* ps_fpga_backend_stub(void);
const ps_fpga_backend_t* ps_fpga_backend_uio(void);
const ps_fpga_backend_t* ps_fpga_backend_xrt(void);

// ---------------------------------------------------------------- 影子寄存器

// 影子后端记录的一次寄存器写（偏移 + 值）。写序判据要靠它：
// 「先写 HI 再写 LO」在只存最终值的内存里不可观测，故把**写序列**也存下来。
typedef struct {
    uint32_t off;
    uint32_t val;
} ps_fpga_write_ev_t;

// 保留最近 PS_FPGA_WRITE_TRACE_MAX 次写（环形，按发生顺序）。
#define PS_FPGA_WRITE_TRACE_MAX 32

// 桩件的上下文：§8.2 的 16 个字 + 统计量。
// 统计量不是装饰：超时次数与复位次数必须能被上层（telemetry）读走，
// 否则「偶发超时」只能靠翻日志猜。
typedef struct {
    uint32_t regs[PS_REG_WORDS];
    uint64_t cycle;          // 影子时间轴（PL 周期）
    int      irq_pending;
    uint64_t irq_count;
    uint64_t submit_count;
    uint64_t timeout_count;
    uint64_t reset_count;
    uint64_t bad_param_count;
    // 写序列（见 ps_fpga_write_ev_t 的说明）。只增不改，超出容量丢最旧的。
    ps_fpga_write_ev_t trace[PS_FPGA_WRITE_TRACE_MAX];
    int      trace_n;
} ps_fpga_stub_ctx_t;

// 真实后端（UIO / XRT）的上下文。板端用得到的字段先摆出来，桩件不用。
//   UIO：fd = open("/dev/uioN")，map = mmap(/sys/class/uio/uioN/maps/map0)，
//        等中断靠 read(fd, &count, 4)。
//   XRT：xrt::device / xrt::ip 句柄（板端把 void* 换成真实类型）。
typedef struct {
    int      fd;
    void*    map;
    uint64_t map_bytes;
    void*    xrt_handle;
} ps_fpga_mmio_ctx_t;

// ---------------------------------------------------------------- 设备句柄

typedef struct {
    const ps_fpga_backend_t* backend;
    void* ctx;
    int   opened;
    char  dev[96];
    ps_fpga_stub_ctx_t stub;   // 影子寄存器上下文（内嵌，桩件不需要堆分配）
    ps_fpga_mmio_ctx_t mmio;   // 真实后端上下文（留桩时只用于打印设备名）
} ps_fpga_t;

// 打开设备。dev 语义：
//   NULL / "" / "stub"   -> 影子寄存器后端
//   "xrt:0"              -> XRT 后端（留桩）
//   其它（如 /dev/uio0） -> UIO 后端（留桩）
int ps_fpga_open(ps_fpga_t* d, const char* dev);
void ps_fpga_close(ps_fpga_t* d);

// ---------------------------------------------------------------- 寄存器访问

int ps_fpga_read32(ps_fpga_t* d, uint32_t off, uint32_t* val);
int ps_fpga_write32(ps_fpga_t* d, uint32_t off, uint32_t val);
int ps_fpga_write_addr64(ps_fpga_t* d, uint32_t off_lo, uint64_t addr);
int ps_fpga_read_addr64(ps_fpga_t* d, uint32_t off_lo, uint64_t* out);

// 等一次中断。返回 PS_FPGA_OK（收到）或 PS_FPGA_ERR_TIMEOUT。
int ps_fpga_wait_irq(ps_fpga_t* d, int timeout_ms);

// 硬件复位：只清状态/控制/结果寄存器，**保留配置寄存器**（帧数/迭代/秩/候选数）。
// 板上复位后 PS 往往不想把整套配置再写一遍；反过来，若哪天改成「全清零」，
// 下一次作业会按默认值跑而结果对不上，属于典型的静默错误。
int ps_fpga_reset(ps_fpga_t* d);

// ---------------------------------------------------------------- 作业

// 写配置寄存器。参数不在允许集合内 -> PS_FPGA_ERR_BAD_PARAM，并置
// ERROR_CODE = 参数非法、STATUS.error = 1。
int ps_fpga_configure(ps_fpga_t* d, const ps_reg_job_config_t* cfg);

// 提交一次「采集 -> 预处理 -> 定位 -> 重建」作业：写地址 -> START -> 等中断 -> 回读。
// 返回 PS_FPGA_OK / PS_FPGA_ERR_BAD_PARAM / PS_FPGA_ERR_TIMEOUT（此时 PL 仍 busy，
// 调用方必须 ps_fpga_reset 后再继续）。
int ps_fpga_submit(ps_fpga_t* d, const ps_reg_job_config_t* cfg,
                   const ps_reg_buffer_addrs_t* addrs, ps_reg_job_result_t* out,
                   int timeout_ms);

// ---------------------------------------------------------------- 观测

const char* ps_fpga_backend_name(const ps_fpga_t* d);
int ps_fpga_is_stub(const ps_fpga_t* d);
uint64_t ps_fpga_irq_count(const ps_fpga_t* d);
uint64_t ps_fpga_timeout_count(const ps_fpga_t* d);
uint64_t ps_fpga_reset_count(const ps_fpga_t* d);
uint64_t ps_fpga_submit_count(const ps_fpga_t* d);

// 影子后端的写序列（最近若干次，按发生顺序；非桩件后端返回 0）。
// 用途：断言 64 bit 地址对的写序 = [HI, LO]（见 host_selftest 的 D 段）。
// 返回写入 out 的条数（<= cap）；out 为 NULL 时只返回可读条数。
int ps_fpga_stub_write_trace(const ps_fpga_t* d, ps_fpga_write_ev_t* out, int cap);

// 清空写序列（仅桩件有效）。便于「只看这一段」的断言，不被 open 后的既有写干扰。
void ps_fpga_stub_trace_reset(ps_fpga_t* d);

// 把整张 §8.2 寄存器表渲染成人读文本（上板联调第一件事就是打印它：
// 「写进去没反应」= 值不变，「映射窗口错了」= 读回不对）。
// 返回写入的字符数（不含结尾 NUL），cap==0 时返回 -1。
int ps_fpga_dump_regs(const ps_fpga_t* d, char* out, size_t cap);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // PENUMBRASIGHT_FPGA_CONTROL_H
