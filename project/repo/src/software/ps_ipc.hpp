// ============================================================================
// PenumbraSight PS 侧进程间契约（header-only）
//
// 五条进程的关系（方案 §7 / Task 10）：
//
//   camera_service ──FrameReady──┐
//                                ▼
//   model_loader ──ModelLoaded──► fpga_control ──JobResult──┬─► telemetry (JSONL)
//                                   ▲                       └─► ui (状态页 / OSD)
//                                   └──JobRequest── ui
//
// 这一层只冻结三件事，别的一概不做：
//   1. **定长消息**（96 字节）与消息类型枚举；
//   2. 载荷结构体（帧元数据 / 作业配置 / 作业结果 / UI 指令）的**字段与顺序**；
//   3. 「信箱」这个最朴素的传输：追加写定长记录 + 读方持游标顺序读。
//
// 为什么载荷不用指针、不用变长
// ---------------------------------------------------------------------------
// 五条进程之间传的是「谁都能解释的一串字节」。一旦引入指针或变长编码，
// 就会立刻依赖两边的 ABI 与堆布局，而板端 PS 与主机不是同一套 libc。
// 定长 POD + memcpy 是唯一能让「同一份结构体在主机与板上逐字节相同」的做法。
// 校验方式：ps_msg_static_checks() 里的 sizeof 断言（C++ 编译期静态断言）。
//
// 关于「信箱」这个桩
// ---------------------------------------------------------------------------
// 板端最终要换成：共享内存环形缓冲（相机帧）+ eventfd/中断（通知）+ 环形日志。
// 但在主机上既没有 PL 也没有共享内存，所以这里用一个**追加写的文件**当信箱：
// 语义上是「顺序追加的定长记录 + 每读方持游标」，与环形缓冲对上层是同一种使用
// 方式（顺序读、不回头、游标自管），因此上层代码不需要为切换载体改写。
//
// 必须清楚的三个局限（板端要一并解决）：
//   * 不是并发安全的：无锁、无部分写保护。板端由单一生产者在每个槽位上做
//     原子序号提交（seqlock），本桩件不模拟；
//   * 没有背压：写方不会因为读方慢而阻塞，长时间运行会无限增长
//     （板端是固定槽位环形缓冲，覆盖最旧帧并计数）；
//   * 没有通知：读方必须轮询（板端用 eventfd / UIO 中断唤醒）。
// ============================================================================

#ifndef PENUMBRASIGHT_PS_IPC_HPP
#define PENUMBRASIGHT_PS_IPC_HPP

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "ps_platform.hpp"

namespace ps {

// ============================================================================
// 一、消息
// ============================================================================

// 消息魔数 'PSMB'（小端读作 0x424D5350）。用来在读到半条记录/错位时立刻发现，
// 而不是把垃圾当消息解释下去。
#define PS_MSG_MAGIC 0x424D5350u
#define PS_MSG_VERSION 1u

enum PsProcId {
    kProcBroadcast = 0,   // 广播：telemetry 与 ui 都要看的结果消息用它
    kProcCamera    = 1,
    kProcModel     = 2,
    kProcFpga      = 3,
    kProcTelemetry = 4,
    kProcUi        = 5
};

enum PsMsgType {
    kMsgFrameReady   = 1,   // camera -> (fpga, telemetry)：一帧 RAW 已落 DDR
    kMsgModelLoaded  = 2,   // model -> (fpga, telemetry)：模型已校验并装入 DDR
    kMsgJobRequest   = 3,   // ui -> fpga：一次采集+定位+重建作业
    kMsgJobResult    = 4,   // fpga -> (telemetry, ui)：selected_id / 可信度 / 延迟
    kMsgUiCommand    = 5,   // ui -> fpga：配置切换（帧数/迭代/秩/固定位置）
    kMsgStatusTick   = 6,   // 任意 -> telemetry：周期性状态（温度/功耗/帧率）
    kMsgLogLine      = 7,   // 任意 -> telemetry：一条人类可读日志
    kMsgError        = 8    // 任意 -> (telemetry, ui)：错误码与上下文
};

inline const char* ps_msg_type_name(uint8_t t) {
    switch (t) {
        case kMsgFrameReady:  return "FrameReady";
        case kMsgModelLoaded: return "ModelLoaded";
        case kMsgJobRequest:  return "JobRequest";
        case kMsgJobResult:   return "JobResult";
        case kMsgUiCommand:   return "UiCommand";
        case kMsgStatusTick:  return "StatusTick";
        case kMsgLogLine:     return "LogLine";
        case kMsgError:       return "Error";
        default:              return "Unknown";
    }
}

inline const char* ps_proc_name(uint8_t p) {
    switch (p) {
        case kProcBroadcast: return "all";
        case kProcCamera:    return "camera_service";
        case kProcModel:     return "model_loader";
        case kProcFpga:      return "fpga_control";
        case kProcTelemetry: return "telemetry";
        case kProcUi:        return "ui";
        default:             return "?";
    }
}

// 96 字节定长消息。载荷 52 字节，够放下下面四个结构体中最大的那个（48）。
//
// 字段顺序刻意排成「先定长头、后载荷」，且头部 20 字节内不含 64 bit 字段，
// 使任何一端手写偏移时都不容易算错。
#pragma pack(push, 1)
struct PsMsg {
    uint32_t magic;          // 'PSMB'
    uint16_t version;        // 1
    uint8_t  type;           // PsMsgType
    uint8_t  src;            // PsProcId
    uint16_t dst;            // PsProcId（kProcBroadcast = 都收）
    uint16_t payload_bytes;  // 载荷有效字节数
    uint32_t seq;            // 发送方自增序号，用于 go/join 与丢消息检测
    uint32_t flags;          // 位定义随 type 而定，默认 0
    uint64_t t_mono_us;      // 发送时刻（单调微秒）
    uint64_t arg0;           // 随 type：FrameReady=frame_id / JobResult=frame_id
    uint64_t arg1;           // 随 type：JobResult=latency_cycles / JobRequest=job_id
    uint8_t  payload[52];
};
#pragma pack(pop)

#define PS_MSG_BYTES 96u

// 载荷结构体：全部 POD + 定长，且 sizeof 在下面被静态断言钉死。
// 之所以逐个写明字节数：这些结构体会通过文件/共享内存跨进程传递，
// 多一个填充字节就意味着两侧总长不一致，而错位读到的数字**看着仍然合理**。

// 帧元数据（FrameReady 的载荷）。§4.2 要求每帧带上曝光与增益，
// §4.4 要求能用帧序号检测跳变，所以 frame_id 是必填项而不是可选项。
#pragma pack(push, 1)
struct PsFrameMeta {
    uint64_t frame_id;        // 相机侧递增；跳变 = 丢帧
    uint64_t t_capture_us;    // 曝光结束（单调微秒）
    int64_t  exposure_us;     // 实际生效曝光
    uint32_t bytes;           // 一帧 RAW 有效字节数
    uint32_t stride_bytes;    // 行跨距（ROI 可能带 padding）
    uint16_t width;
    uint16_t height;
    int16_t  gain_centi_db;   // 增益，0.01 dB 为单位（避免浮点跨 ABI）
    uint16_t bit_depth;       // 有效位深（12）
};                            // 8+8+8+4+4+2+2+2+2 = 40
#pragma pack(pop)

// 作业配置（JobRequest 的载荷）。字段与 §8.2 的寄存器一一对应：
// FRAME_COUNT / ITER_COUNT / RANK / CANDIDATE_COUNT / CONTROL.selfloc / .rgb。
#pragma pack(push, 1)
struct PsJobConfig {
    uint32_t job_id;
    uint32_t frame_count;      // 1 / 4 / 8
    uint32_t iter_count;       // 8 / 12 / 16 / 32
    uint32_t rank;             // 32 / 48 / 64 / 96
    uint32_t candidate_count;  // 默认 243
    uint8_t  selfloc;          // 1 = 自定位；0 = 用 fixed_candidate_id
    uint8_t  rgb;              // 1 = RGB 三通道；0 = 单色
    uint8_t  reserved0;
    uint8_t  reserved1;
    int32_t  fixed_candidate_id;
    uint32_t flags;            // bit0 = 注入超时（仅桩件/测试用）
};                             // 4*5 + 4 + 4 + 4 = 32
#pragma pack(pop)

// 作业结果（JobResult 的载荷）。§8.2 的 SELECTED_ID / CONFIDENCE /
// LATENCY_CYCLES / ERROR_CODE 在这里有了跨进程的名字。
#define PS_JOB_FLAG_LOW_CONF  0x0001u
#define PS_JOB_FLAG_EARLY_STOP 0x0002u
#define PS_JOB_FLAG_TIMEOUT   0x0004u

#pragma pack(push, 1)
struct PsJobResult {
    uint32_t selected_id;      // 定位输出；selfloc=0 时 = fixed_candidate_id
    uint32_t candidate_count;
    uint16_t confidence_q16;   // Q0.16
    uint16_t margin_q16;       // (s1-s2)/(s1+eps)，Q0.16
    uint16_t rank;
    uint16_t iterations;
    uint16_t flags;            // PS_JOB_FLAG_*
    uint16_t error_code;       // §8.2 ERROR_CODE；0 = 无错
    uint64_t latency_cycles;   // PL 处理周期
    uint64_t latency_us;       // 换算后的微秒（便于遥测直接写）
    uint64_t output_bytes;     // 重建结果缓冲字节数
    uint32_t model_hash;       // 本次作业所用模型的 geometry_hash
};                             // 4+4+12+24+4 = 48
#pragma pack(pop)

// UI 指令（UiCommand 的载荷）。**UI 只发配置，不参与重建**（方案 §7）。
#pragma pack(push, 1)
struct PsUiCommand {
    uint32_t mode;             // 0=固定位置 1=自定位 2=灰度 3=RGB
    uint32_t frames;           // 1/4/8
    uint32_t iters;            // 8/12/16/32
    uint32_t rank;             // 32/48/64/96
    int32_t  fixed_candidate_id;
    uint32_t flags;
    uint32_t reserved;
};                             // 28
#pragma pack(pop)

// 编译期钉死尺寸。任一处算错，编译器在这里报，而不是在板上表现为
// 「消息类型正确、字段整体偏移 4 字节」。
#if defined(__cplusplus)
static_assert(sizeof(PsMsg) == PS_MSG_BYTES, "PsMsg 必须是 96 字节");
static_assert(sizeof(PsFrameMeta) <= 52, "PsFrameMeta 必须放得进载荷");
static_assert(sizeof(PsJobConfig) <= 52, "PsJobConfig 必须放得进载荷");
static_assert(sizeof(PsJobResult) <= 52, "PsJobResult 必须放得进载荷");
static_assert(sizeof(PsUiCommand) <= 52, "PsUiCommand 必须放得进载荷");
#endif

// ============================================================================
// 一之二、Q0.16 量纲（可信度与 margin 的公共换算）
// ============================================================================
//
// 放在共享契约里而不是 fpga_control 里：telemetry 要把它写到 JSONL、ui 要把它
// 显示成百分数，三处各写一份 /65536 迟早会漂移成 /65535。
//
// **不要用 65535 归一化**：Q0.16 的满量程是 1.0（即 65536 分之一），
// 用 65535 会让 confidence 永远差一点点到不了 1.0，在门限比较上出错。
inline double ps_q16_to_double(uint16_t q) {
    return static_cast<double>(q) / 65536.0;
}

// 可信度门限（§6.6）。0.5 只是**占位值**：真实门限必须由调参集标定，
// 且盲测期间不得修改（Task 8 Step 2 的冻结要求）。
#define PS_CONFIDENCE_THRESHOLD_Q16 0x8000u

// ============================================================================
// 二、消息构造
// ============================================================================

inline PsMsg ps_msg_make(uint8_t type, uint8_t src, uint16_t dst, uint32_t seq) {
    PsMsg m;
    std::memset(&m, 0, sizeof(m));
    m.magic = PS_MSG_MAGIC;
    m.version = PS_MSG_VERSION;
    m.type = type;
    m.src = src;
    m.dst = dst;
    m.seq = seq;
    m.t_mono_us = mono_us();
    return m;
}

// 把任意 POD 载荷装进消息。载荷超过 52 字节时**静默截断是不允许的**，
// 故这里断言式地拒绝（返回 false），宁可丢一条消息也不传半条结构体。
template <typename T>
inline bool ps_msg_set_payload(PsMsg& m, const T& pod) {
    if (sizeof(T) > sizeof(m.payload)) return false;
    std::memcpy(m.payload, &pod, sizeof(T));
    m.payload_bytes = static_cast<uint16_t>(sizeof(T));
    return true;
}

template <typename T>
inline bool ps_msg_get_payload(const PsMsg& m, T* out) {
    if (!out) return false;
    if (m.payload_bytes < sizeof(T)) return false;
    std::memcpy(out, m.payload, sizeof(T));
    return true;
}

// ============================================================================
// 三、信箱（桩件传输载体）
// ============================================================================

// 板端最终要换成「共享内存 + 通知」。因此这里把所有载体相关的细节压在
// 三个函数后面：一条记录 96 字节、追加写、读方持游标。

inline std::string ps_mailbox_default_path() {
    return scratch_path("ps_penumbrasight.mb");
}

// 截断信箱（demo/自检开头调用，等价于板端的「重置环形缓冲」）。
inline int ps_mailbox_reset(const std::string& path) {
    if (!require_ascii_path(path.c_str(), "mailbox")) return -1;
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return -1;
    std::fclose(f);
    return 0;
}

// 追加一条消息。返回 0 成功，-1 失败。
inline int ps_mailbox_send(const std::string& path, const PsMsg& m) {
    if (!require_ascii_path(path.c_str(), "mailbox")) return -1;
    std::FILE* f = std::fopen(path.c_str(), "ab");
    if (!f) return -1;
    const size_t w = std::fwrite(&m, 1, sizeof(m), f);
    std::fclose(f);
    return (w == sizeof(m)) ? 0 : -1;
}

// 从 *cursor 处读一条。返回 1 = 读到、0 = 暂无新消息、-1 = 出错（含魔数不符）。
// 魔数校验是必需的：一条记录如果是半写状态（进程被杀），后续记录会整体错位，
// 此时必须立刻停下并让上层知道，而不是继续解释垃圾数据。
inline int ps_mailbox_poll(const std::string& path, uint64_t* cursor, PsMsg* out) {
    if (!cursor || !out) return -1;
    if (!require_ascii_path(path.c_str(), "mailbox")) return -1;
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return 0;   // 信箱还没建 = 暂无消息，不算错
    if (std::fseek(f, static_cast<long>(*cursor), SEEK_SET) != 0) {
        std::fclose(f);
        return -1;
    }
    PsMsg m;
    const size_t r = std::fread(&m, 1, sizeof(m), f);
    std::fclose(f);
    if (r == 0u) return 0;                  // 尾部，无新消息
    if (r != sizeof(m)) return -1;          // 半条记录
    if (m.magic != PS_MSG_MAGIC) return -1; // 错位或半写
    *out = m;
    *cursor += sizeof(m);
    return 1;
}

// 统计信箱里当前的消息条数（自检与 telemetry 的收尾报告用）。
// 文件长度不是 96 的整数倍时返回负数，暴露半写状态。
inline long long ps_mailbox_count(const std::string& path) {
    if (!require_ascii_path(path.c_str(), "mailbox")) return -1;
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return 0;
    std::fseek(f, 0, SEEK_END);
    const long long bytes = static_cast<long long>(std::ftell(f));
    std::fclose(f);
    if (bytes % static_cast<long long>(sizeof(PsMsg)) != 0) return -1;
    return bytes / static_cast<long long>(sizeof(PsMsg));
}

// 一条消息的紧凑单行摘要（日志与 ui 状态页共用，避免两处格式漂移）。
inline void ps_msg_summary(const PsMsg& m, char* out, size_t cap) {
    std::snprintf(out, cap, "%s %s->%s seq=%u bytes=%u arg0=%llu arg1=%llu",
                  ps_msg_type_name(m.type), ps_proc_name(m.src),
                  ps_proc_name(static_cast<uint8_t>(m.dst)), m.seq,
                  static_cast<unsigned>(m.payload_bytes),
                  static_cast<unsigned long long>(m.arg0),
                  static_cast<unsigned long long>(m.arg1));
}

}  // namespace ps

#endif  // PENUMBRASIGHT_PS_IPC_HPP
