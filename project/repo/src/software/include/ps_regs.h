// ============================================================================
// ps_regs.h —— AXI-Lite 寄存器契约（方案 §8.2，逐项对应）
//
// 表（偏移 / 名称 / 含义）
// ---------------------------------------------------------------------------
//   0x00 CONTROL          bit0 start, bit1 reset, bit2 selfloc, bit3 rgb
//   0x04 STATUS           bit0 busy, bit1 done, bit2 error, bit3 low_conf
//   0x08 FRAME_COUNT      1/4/8
//   0x0C ITER_COUNT       8/12/16/32
//   0x10 RANK             32/48/64/96
//   0x14 CANDIDATE_COUNT  默认 243
//   0x18 SELECTED_ID      定位输出或固定输入
//   0x1C CONFIDENCE       Q0.16
//   0x20 LATENCY_CYCLES   PL 处理周期
//   0x24 ERROR_CODE       模型/DDR/溢出错误
//   0x28~0x3C BUFFER_ADDR RAW / model / output 的 64 bit 地址对
//
// 为什么这里一个魔数都不许散落
// ---------------------------------------------------------------------------
// 本文件是**PL 侧 RTL 的接口**。偏移写错、位定义写错的症状不是崩溃，而是
// 「写进去没反应」或「状态位永远为 0」；允许集合写错（例如 rank=40）则是 PL
// 去读越界的系数区，表现为「图整体偏暗或出条纹」—— 都不像 bug。
// 因此：偏移、位、默认值、允许集合、错误码全部只在本文件定义一次，其它模块
// 一律 include 本文件并从这些名字来，禁止在别处出现 0x04 / 1u<<2 这类裸字面量。
//
// 64 bit 地址为什么要拆成 LO/HI 两个 32 bit 寄存器
// ---------------------------------------------------------------------------
// AXI-Lite 的既定数据通路是 32 bit（Vivado 默认）。把寄存器加宽到 64 bit 会引出
// 跨 64 bit 的字节序与对齐问题，而地址只有三对、又是写多读少，拆高低 32 bit
// 是代价最小的做法。三对正好占满 0x28..0x3F，故窗口 = 16 字 = 64 字节。
//
// 本文件是纯 C（同时可被 C++ include）：只有宏、静态内联函数与 POD 结构体。
// 板端（aarch64-linux-gnu-gcc）与主机（MinGW gcc/g++）编译同一份。
// ============================================================================

#ifndef PENUMBRASIGHT_PS_REGS_H
#define PENUMBRASIGHT_PS_REGS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- 偏移

#define PS_REG_CONTROL          0x00u
#define PS_REG_STATUS           0x04u
#define PS_REG_FRAME_COUNT      0x08u
#define PS_REG_ITER_COUNT       0x0Cu
#define PS_REG_RANK             0x10u
#define PS_REG_CANDIDATE_COUNT  0x14u
#define PS_REG_SELECTED_ID      0x18u
#define PS_REG_CONFIDENCE       0x1Cu
#define PS_REG_LATENCY_CYCLES   0x20u
#define PS_REG_ERROR_CODE       0x24u
#define PS_REG_RAW_ADDR_LO      0x28u
#define PS_REG_RAW_ADDR_HI      0x2Cu
#define PS_REG_MODEL_ADDR_LO    0x30u
#define PS_REG_MODEL_ADDR_HI    0x34u
#define PS_REG_OUT_ADDR_LO      0x38u
#define PS_REG_OUT_ADDR_HI      0x3Cu

// 寄存器窗口 = 16 个字 = 64 字节。RTL 侧的地址译码必须与 PS_REG_BYTES 一致。
#define PS_REG_WORDS            16u
#define PS_REG_BYTES            (PS_REG_WORDS * 4u)
#define PS_REG_WORD_BYTES       4u

// ---------------------------------------------------------------- 位定义

// CONTROL（写）：bit0 start, bit1 reset, bit2 selfloc, bit3 rgb
#define PS_CTRL_START   (1u << 0)
#define PS_CTRL_RESET   (1u << 1)
#define PS_CTRL_SELFLOC (1u << 2)
#define PS_CTRL_RGB     (1u << 3)
#define PS_CTRL_BIT_MASK \
    (PS_CTRL_START | PS_CTRL_RESET | PS_CTRL_SELFLOC | PS_CTRL_RGB)
#define PS_CTRL_BIT_COUNT 4u

// STATUS（读）：bit0 busy, bit1 done, bit2 error, bit3 low_conf
#define PS_STS_BUSY     (1u << 0)
#define PS_STS_DONE     (1u << 1)
#define PS_STS_ERROR    (1u << 2)
#define PS_STS_LOW_CONF (1u << 3)
#define PS_STS_BIT_MASK \
    (PS_STS_BUSY | PS_STS_DONE | PS_STS_ERROR | PS_STS_LOW_CONF)
#define PS_STS_BIT_COUNT 4u

// ---------------------------------------------------------------- 默认值

// bitstream 复位后的值。桩件（影子寄存器）用它们初始化，于是「没配置就提交作业」
// 时的行为与板上一致；这也让「默认值 = §8.2 表格里的值」成为可自检的一条。
#define PS_REG_DEFAULT_FRAME_COUNT     4u
#define PS_REG_DEFAULT_ITER_COUNT      16u
#define PS_REG_DEFAULT_RANK            64u
#define PS_REG_DEFAULT_CANDIDATE_COUNT 243u

// ---------------------------------------------------------------- 允许集合

// §5.1/§8.2 给定的允许集合。写成函数而不是注释，是为了能在写寄存器**之前**拦下来。
// 板上写一个非法值不会报错，只会让 PL 读越界，故必须在 PS 侧挡住。
static inline int ps_reg_frame_count_valid(uint32_t v) {
    return v == 1u || v == 4u || v == 8u;
}
static inline int ps_reg_iter_count_valid(uint32_t v) {
    return v == 8u || v == 12u || v == 16u || v == 32u;
}
static inline int ps_reg_rank_valid(uint32_t v) {
    return v == 32u || v == 48u || v == 64u || v == 96u;
}

// ---------------------------------------------------------------- 位域

// 把寄存器位「解开」成命名字段，供上层做判断与显示。
// 之所以不直接用位运算散落各处：低可信度门限比较、BUSY/DONE 互斥判断这类
// 逻辑一旦有人写成 (v & 4) 而不是 (v & PS_STS_ERROR)，读代码的人无法发现。
typedef struct {
    uint32_t start;
    uint32_t reset;
    uint32_t selfloc;
    uint32_t rgb;
} ps_reg_control_bits_t;

typedef struct {
    uint32_t busy;
    uint32_t done;
    uint32_t error;
    uint32_t low_conf;
} ps_reg_status_bits_t;

// 位掩码表 + 下标取位：自检靠它遍历「4 位全组合」，不必手写 16 个常数。
static inline uint32_t ps_reg_control_bit(unsigned i) {
    static const uint32_t kBits[PS_CTRL_BIT_COUNT] = {
        PS_CTRL_START, PS_CTRL_RESET, PS_CTRL_SELFLOC, PS_CTRL_RGB};
    return (i < PS_CTRL_BIT_COUNT) ? kBits[i] : 0u;
}
static inline const char* ps_reg_control_bit_name(unsigned i) {
    static const char* const kNames[PS_CTRL_BIT_COUNT] = {
        "start", "reset", "selfloc", "rgb"};
    return (i < PS_CTRL_BIT_COUNT) ? kNames[i] : "?";
}
static inline uint32_t ps_reg_status_bit(unsigned i) {
    static const uint32_t kBits[PS_STS_BIT_COUNT] = {
        PS_STS_BUSY, PS_STS_DONE, PS_STS_ERROR, PS_STS_LOW_CONF};
    return (i < PS_STS_BIT_COUNT) ? kBits[i] : 0u;
}
static inline const char* ps_reg_status_bit_name(unsigned i) {
    static const char* const kNames[PS_STS_BIT_COUNT] = {
        "busy", "done", "error", "low_conf"};
    return (i < PS_STS_BIT_COUNT) ? kNames[i] : "?";
}

// 位域 -> 寄存器（拼装）。入参为 0/NULL 时按全 0 处理，不产生未定义行为。
static inline uint32_t ps_reg_pack_control(const ps_reg_control_bits_t* b) {
    uint32_t v = 0u;
    if (!b) return 0u;
    if (b->start)   v |= PS_CTRL_START;
    if (b->reset)   v |= PS_CTRL_RESET;
    if (b->selfloc) v |= PS_CTRL_SELFLOC;
    if (b->rgb)     v |= PS_CTRL_RGB;
    return v;
}
static inline uint32_t ps_reg_pack_status(const ps_reg_status_bits_t* b) {
    uint32_t v = 0u;
    if (!b) return 0u;
    if (b->busy)     v |= PS_STS_BUSY;
    if (b->done)     v |= PS_STS_DONE;
    if (b->error)    v |= PS_STS_ERROR;
    if (b->low_conf) v |= PS_STS_LOW_CONF;
    return v;
}

// 寄存器 -> 位域（解析）。规定：只认位上是否为 1，未知位置不回写。
static inline void ps_reg_unpack_control(uint32_t v, ps_reg_control_bits_t* b) {
    if (!b) return;
    b->start   = (v & PS_CTRL_START)   ? 1u : 0u;
    b->reset   = (v & PS_CTRL_RESET)   ? 1u : 0u;
    b->selfloc = (v & PS_CTRL_SELFLOC) ? 1u : 0u;
    b->rgb     = (v & PS_CTRL_RGB)     ? 1u : 0u;
}
static inline void ps_reg_unpack_status(uint32_t v, ps_reg_status_bits_t* b) {
    if (!b) return;
    b->busy     = (v & PS_STS_BUSY)     ? 1u : 0u;
    b->done     = (v & PS_STS_DONE)     ? 1u : 0u;
    b->error    = (v & PS_STS_ERROR)    ? 1u : 0u;
    b->low_conf = (v & PS_STS_LOW_CONF) ? 1u : 0u;
}

// ---------------------------------------------------------------- 短串解码

// 把置位的那几位写成 "a b c" 形式。返回写入的字符数（不含结尾 NUL）。
// 返回 -1 表示 cap == 0（什么都没写）。一定保证 NUL 结尾。
static inline int ps_reg_decode_bits(uint32_t v, char* out, size_t cap,
                                     uint32_t (*bitof)(unsigned),
                                     const char* (*nameof)(unsigned),
                                     unsigned nbits) {
    if (!out || cap == 0u) return -1;
    size_t used = 0u;
    unsigned i;
    for (i = 0u; i < nbits; ++i) {
        if ((v & bitof(i)) == 0u) continue;
        const char* nm = nameof(i);
        size_t k = 0u;
        if (used != 0u) {
            if (used + 1u >= cap) break;
            out[used++] = ' ';
        }
        while (nm[k] != '\0') {
            if (used + 1u >= cap) break;
            out[used++] = nm[k++];
        }
    }
    out[used] = '\0';
    return (int)used;
}

static inline int ps_reg_decode_control(uint32_t v, char* out, size_t cap) {
    return ps_reg_decode_bits(v, out, cap, ps_reg_control_bit,
                              ps_reg_control_bit_name, PS_CTRL_BIT_COUNT);
}
static inline int ps_reg_decode_status(uint32_t v, char* out, size_t cap) {
    return ps_reg_decode_bits(v, out, cap, ps_reg_status_bit,
                              ps_reg_status_bit_name, PS_STS_BIT_COUNT);
}

// ---------------------------------------------------------------- Q0.16

// CONFIDENCE 是 Q0.16：满量程 1.0 对应 65536，**不是** 65535。
// 用 65535 归一化会让 1.0 永远差一点点到不了，门限比较随之出错。
// 这个换算只在这里定义一份：telemetry 要写进日志、ui 要显示成百分数，
// 两处各写一份 /65536 迟早会漂移。
#define PS_REG_Q16_ONE 65536u

// 可信度门限（§6.6）。0.5 是**占位值**：真实门限必须由调参集标定，
// 盲测期间不得修改（Task 8 Step 2 的冻结要求）。
#define PS_REG_CONF_THRESHOLD_Q16 0x8000u

static inline double ps_reg_q16_to_double(uint16_t q) {
    return (double)q / (double)PS_REG_Q16_ONE;
}

// 反向换算：按四舍五入，并**钳到 [0, 0xFFFF]**。不钳的话，浮点误差或上游
// 越界值会在这里回绕成「0.001 变成了满可信」，属于最难查的一类错。
static inline uint16_t ps_reg_double_to_q16(double x) {
    double s = x * (double)PS_REG_Q16_ONE;
    if (s <= 0.0) return 0u;
    if (s >= 65535.0) return 65535u;
    return (uint16_t)(s + 0.5);
}

// 低可信度：§8.2 的 STATUS.low_conf 与 JobResult.flags bit0 用同一门限，
// 故比较只写一份。
static inline int ps_reg_confidence_is_low(uint16_t q16) {
    return q16 < (uint16_t)PS_REG_CONF_THRESHOLD_Q16;
}

// ---------------------------------------------------------------- 64 bit 地址对

static inline uint32_t ps_reg_addr_lo(uint64_t a) {
    return (uint32_t)(a & 0xFFFFFFFFull);
}
static inline uint32_t ps_reg_addr_hi(uint64_t a) {
    return (uint32_t)((a >> 32) & 0xFFFFFFFFull);
}
static inline uint64_t ps_reg_addr_join(uint32_t lo, uint32_t hi) {
    return ((uint64_t)hi << 32) | (uint64_t)lo;
}

// ---------------------------------------------------------------- 错误码

// §8.2 的 ERROR_CODE：模型/DDR/溢出错误。0 必须是「无错误」。
#define PS_REG_ERR_NONE             0u
#define PS_REG_ERR_MODEL_NOT_LOADED 1u  // 模型地址为 0（未装载）
#define PS_REG_ERR_DDR_TIMEOUT      2u  // DDR 读写超时
#define PS_REG_ERR_MAC_OVERFLOW     3u  // MAC 累加溢出
#define PS_REG_ERR_RESIDUAL_OVERFLOW 4u // 分数/残差溢出
#define PS_REG_ERR_GEOM_MISMATCH    5u  // 几何哈希与当前配置不一致（§6.6 灰屏）
#define PS_REG_ERR_ILLEGAL_PARAM    6u  // 寄存器值不在允许集合内
#define PS_REG_ERR_WATCHDOG         7u  // 看门狗超时

// 错误码 -> 人话。日志与 ui 用同一份，避免两处翻译不一致。
static inline const char* ps_reg_error_name(uint32_t code) {
    switch (code) {
        case PS_REG_ERR_NONE:                return "无错误";
        case PS_REG_ERR_MODEL_NOT_LOADED:    return "模型未装载";
        case PS_REG_ERR_DDR_TIMEOUT:         return "DDR 超时";
        case PS_REG_ERR_MAC_OVERFLOW:        return "MAC 溢出";
        case PS_REG_ERR_RESIDUAL_OVERFLOW:   return "残差/分数溢出";
        case PS_REG_ERR_GEOM_MISMATCH:       return "几何哈希不一致";
        case PS_REG_ERR_ILLEGAL_PARAM:       return "寄存器参数非法";
        case PS_REG_ERR_WATCHDOG:            return "看门狗超时";
        default:                             return "未知错误码";
    }
}

// ---------------------------------------------------------------- 时钟与量纲

// PL 计算时钟的设计目标：150 MHz（§1.1 B 档）。周期 <-> 微秒一律走这里，
// 免得有人按 100 MHz 算出一份「看着像」的延迟。
#define PS_REG_PL_CLOCK_MHZ 150u

static inline uint64_t ps_reg_cycles_to_us(uint64_t cycles) {
    return cycles / (uint64_t)PS_REG_PL_CLOCK_MHZ;
}
static inline uint64_t ps_reg_us_to_cycles(uint64_t us) {
    return us * (uint64_t)PS_REG_PL_CLOCK_MHZ;
}

// ---------------------------------------------------------------- 作业载荷

// 作业配置：字段与 §8.2 的 FRAME_COUNT / ITER_COUNT / RANK / CANDIDATE_COUNT
// 以及 CONTROL.selfloc / .rgb 一一对应。ui 只填这个结构体，不直接碰寄存器。
#define PS_JOB_CFG_FLAG_INJECT_TIMEOUT 0x1u   // 仅桩件/自检：注入一次超时

typedef struct {
    uint32_t job_id;
    uint32_t frame_count;       // 1 / 4 / 8
    uint32_t iter_count;        // 8 / 12 / 16 / 32
    uint32_t rank;              // 32 / 48 / 64 / 96
    uint32_t candidate_count;   // 默认 243
    uint32_t selfloc;           // 1 = 自定位；0 = 用 fixed_candidate_id
    uint32_t rgb;               // 1 = RGB 三通道；0 = 单色
    int32_t  fixed_candidate_id;
    uint32_t flags;             // PS_JOB_CFG_FLAG_*
} ps_reg_job_config_t;

// 作业结果标志（与 STATUS.low_conf 呼应）
#define PS_REG_JOB_FLAG_LOW_CONF 0x0001u
#define PS_REG_JOB_FLAG_TIMEOUT  0x0002u

// 作业结果：§8.2 的 SELECTED_ID / CONFIDENCE / LATENCY_CYCLES / ERROR_CODE
// 在这里有了跨模块的名字。
typedef struct {
    uint32_t selected_id;
    uint32_t candidate_count;
    uint16_t confidence_q16;   // Q0.16
    uint16_t margin_q16;       // (s1-s2)/(s1+eps)，Q0.16
    uint16_t rank;
    uint16_t iterations;
    uint16_t flags;            // PS_REG_JOB_FLAG_*
    uint16_t error_code;       // PS_REG_ERR_*
    uint64_t latency_cycles;
    uint64_t latency_us;
    uint64_t output_bytes;
    uint32_t model_hash;       // 本次作业所用模型的 geometry_hash
} ps_reg_job_result_t;

// 三个 buffer 的 64 bit 地址（§8.2 的 0x28~0x3C 三对）。0 = 未设置。
typedef struct {
    uint64_t raw;
    uint64_t model;
    uint64_t output;
} ps_reg_buffer_addrs_t;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // PENUMBRASIGHT_PS_REGS_H
