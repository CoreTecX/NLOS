// ============================================================================
// X12 PS 侧数据契约（header-only，纯逻辑，不碰 IO）
//
// 目的：把「相机取流 / 文件灌入 / 仿真激励」三条路径**收敛到同一个接口**，
// 使 M1 换数据源不需要改一行。X12 的判据是「三条数据源共用同一接口」。
//
// 契约分四块：
//   一、RawStreamSpec    —— 决定 M1 输入形状与位宽的流规格
//   二、ps_validate_spec —— 把 X1 那条新约束变成可执行检查
//   三、AXI-Stream 打包   —— 三条路径必须产出**逐位相同**的拍序列
//   四、Gram 搬出预算     —— X12 判据里那条 DDR 带宽核算
//
// 与 X1 接口冻结的关系
// -------------------
// X1 第四节给 M1 定的是：
//   `void m1_preprocess(const uint16_t* raw_frames, int n_frames,
//                       const PreprocParams& p, MeasOut& out);`
// 其中**新加的一条显式约束**是「M1 的输出边长必须等于几何配置的墙网格边长」。
// 本文件把这条约束做成 `ps_validate_spec()`，让它在数据源接入的那一刻就被检查，
// 而不是等到墙网格尺寸对不上时才发现。
//
// 同时冻结一条容易混用的口径：**彩色时先解交织再分块**，所以
//   墙网格边长 = (裁剪边长 / 2) / 分块边长
// 官方档 crop=2016 / block=8 / 彩色 → 1008 / 8 = **126**，与 chair_test.mat 的
// walln_points=126 对上；B 档 crop=1920 / block=10 → 960 / 10 = **96**。
// 两个档位的 crop 不同，**不得混用**（X1 第四节 M1 的核心约束）。
// ============================================================================

#ifndef PENUMBRASIGHT_PS_CONTRACT_HPP
#define PENUMBRASIGHT_PS_CONTRACT_HPP

#include <cstddef>
#include <cstdint>

// 一拍 8 字节（64 bit AXI-Stream），装 4 个 16 bit 像素。
// 综合时如需改位宽，用 -D 覆盖；三条数据源都读这两个宏，故必然一致。
#ifndef PS_BEAT_BYTES
#define PS_BEAT_BYTES 8
#endif
#define PS_PIX_PER_BEAT (PS_BEAT_BYTES / 2)
#define PS_PIX_BYTES   2            // 像素在流里统一占 16 bit（容器），与位深无关

namespace ps {

// ============================================================================
// 一、流规格
// ============================================================================

// 三条数据源必须能描述成同一个结构——这就是「同一接口」的含义。
struct RawStreamSpec {
    int      width;          // RAW 幅宽（像素）
    int      height;         // RAW 幅高（像素）
    int      container_bits; // 容器位宽：8 / 12 / 16（**不是**有效位深）
    int      color;          // 0 = 黑白（单通道直通）；1 = 彩色（Bayer 解交织）
    int      bayer_phase;    // 0 = RGGB（左上 R）；1 = BGGR（左上 B）。color=0 时忽略
    int      crop;           // 中心裁剪边长；0 = 不裁（取 min(width,height)）
    int      block;          // 分块边长（方形）
    int      frames;         // 帧累积数
    uint64_t dark_level;     // 暗电平，先减后处理
};

// 有效位深范围：容器是 16 bit 满量程，但数据只用到 container_bits 位。
// 用 2^n − 1 而不是 2^n，与 preprocess.hpp 的饱和口径一致。
inline uint64_t ps_depth_fullscale(int container_bits) {
    if (container_bits <= 0 || container_bits >= 32) return 0;
    return ((uint64_t)1 << container_bits) - 1;
}

// 裁剪边长：crop 为 0 时取 min(width, height)。
inline int ps_crop_side(const RawStreamSpec& s) {
    if (s.crop > 0) return s.crop;
    return (s.width < s.height) ? s.width : s.height;
}

// 解交织后的平面边长。黑白直通，彩色每维减半。
inline int ps_plane_side(const RawStreamSpec& s) {
    return s.color ? (ps_crop_side(s) / 2) : ps_crop_side(s);
}

// 墙网格边长。这是 M1 输出的边长，必须等于几何配置的 wall.n。
inline int ps_wall_side(const RawStreamSpec& s) {
    if (s.block <= 0) return 0;
    return ps_plane_side(s) / s.block;
}

// ============================================================================
// 二、校验（把 X1 的新约束变成可执行检查）
// ============================================================================

enum PsSpecStatus {
    kPsOk              = 0,
    kPsBadSpec         = 1,   // 规格自相矛盾（见各自条件）
    kPsSideMismatch    = 2,   // 推出的墙网格边长与几何配置不符
    kPsContainerTooLarge = 3  // 容器位宽超过编译期 PS_IN_BITS
};

// 逐条校验。`wall_n` 传入几何配置的墙网格边长，`in_bits` 传编译期 PS_IN_BITS。
//
// 四条硬条件（都来自真实踩过的坑）：
//   1. 裁剪边长必须能被 2*block 整除（彩色）或 block 整除（黑白）。
//      不能整除意味着 M1 输出边长会静默取整，墙网格就对不上了。
//   2. 容器位宽不得超过编译期输入位宽——超了会在进 M1 之前被截断。
//   3. 彩色相机的裁剪边长必须是偶数（否则解交织无从下手）。
//   4. 推出的墙网格边长必须**恰好等于** wall_n。这是 X1 那条新约束。
inline int ps_validate_spec(const RawStreamSpec& s, int wall_n, int in_bits) {
    if (s.width <= 0 || s.height <= 0 || s.block <= 0 || s.frames <= 0) {
        return kPsBadSpec;
    }
    if (s.container_bits <= 0 || s.container_bits > 16) return kPsBadSpec;
    if (s.container_bits > in_bits) return kPsContainerTooLarge;
    if (s.color && (s.bayer_phase != 0 && s.bayer_phase != 1)) return kPsBadSpec;

    const int cs = ps_crop_side(s);
    if (cs <= 0 || cs > s.width || cs > s.height) return kPsBadSpec;
    if (s.color && (cs % 2 != 0)) return kPsBadSpec;

    const int unit = s.color ? (2 * s.block) : s.block;
    if (cs % unit != 0) return kPsBadSpec;

    if (ps_wall_side(s) != wall_n) return kPsSideMismatch;
    return kPsOk;
}

// 把 M1 的输出形状一并推出来，供上层开缓冲。返回 bh*bw，差分长度 (bh-1)*bw。
inline long long ps_output_shape(const RawStreamSpec& s, int* bh, int* bw) {
    const int n = ps_wall_side(s);
    if (bh) *bh = n;
    if (bw) *bw = n;
    return (long long)n * n;
}

// ============================================================================
// 三、AXI-Stream 打包契约
//
// 三条路径（相机 / 文件 / 仿真）**必须产出逐位相同的拍序列**，否则
// 「同一接口」就只是签名相同而语义不同。故把打包口径冻结在这里：
//
//   * 一拍 PS_BEAT_BYTES 字节，装 PS_PIX_PER_BEAT 个像素
//   * 像素按 16 bit 小端写入：pix[k] 的低字节在前
//   * 不足一拍时高字节补零（**不**回绕、**不**重复）
//   * 帧内按行优先（先行后列），与 preprocess.hpp 的展平一致
//   * 帧边界对齐到拍；每帧最后一拍置 TLAST（用 ps_beat_is_last 判断）
// ============================================================================

// 把 n（0 < n <= PS_PIX_PER_BEAT）个像素装进一拍。
inline void ps_pack_beat(const uint16_t* pix, int n, uint8_t beat[PS_BEAT_BYTES]) {
    for (int b = 0; b < PS_BEAT_BYTES; ++b) beat[b] = 0;
    for (int k = 0; k < n && k < PS_PIX_PER_BEAT; ++k) {
        beat[2 * k]     = (uint8_t)(pix[k] & 0xFF);
        beat[2 * k + 1] = (uint8_t)((pix[k] >> 8) & 0xFF);
    }
}

// 反向：从一拍取出 n 个像素。
inline void ps_unpack_beat(const uint8_t beat[PS_BEAT_BYTES], uint16_t* pix, int n) {
    for (int k = 0; k < n && k < PS_PIX_PER_BEAT; ++k) {
        pix[k] = (uint16_t)(((uint16_t)beat[2 * k] |
                            ((uint16_t)beat[2 * k + 1] << 8)));
    }
}

// 一帧的拍数。帧长按像素算，向上取整到拍。
inline long long ps_beats_per_frame(const RawStreamSpec& s) {
    const long long px = (long long)s.width * s.height;
    return (px + PS_PIX_PER_BEAT - 1) / PS_PIX_PER_BEAT;
}

// 判断某一拍是否为帧内最后一拍（TLAST）。
inline int ps_beat_is_last(const RawStreamSpec& s, long long beat_index) {
    return (beat_index + 1 == ps_beats_per_frame(s)) ? 1 : 0;
}

// 一帧的字节数（按流算，不是按原始数据算——尾部补零也算进去）。
inline long long ps_frame_stream_bytes(const RawStreamSpec& s) {
    return ps_beats_per_frame(s) * (long long)PS_BEAT_BYTES;
}

// ============================================================================
// 四、Gram 搬出预算（X12 判据里那条 DDR 核算）
//
// 定位核每评估一个候选，要把 Gram 累加器从 PL 搬到 PS 做特征分解。
// 用量：A_dᵀA_d 只存上三角 + A_dᵀy。
// ============================================================================

struct GramEgressBudget {
    long long gram_entries;      // 上三角项数 = n(n+1)/2
    int       bits_per_entry;    // 20（X16 实测）
    long long gram_bytes;        // 上三角字节数
    long long aty_bytes;         // A_dᵀy 字节数（同一位宽）
    long long bytes_per_candidate;
    long long candidates;        // 972 = 官方第 1 轮网格
    long long bytes_total;
    double    seconds_at_2GBps;  // 按 2.0 GB/s 有效带宽
    double    seconds_at_1GBps;  // 悲观：1.0 GB/s
    double    bytes_per_cand_at_32bit;  // 对照：若整块 32 bit 存
};

// 位宽向上取整到字节：20 bit → 3 B。真实实现若按 20 bit 紧凑打包则按位算，
// 这里给出**两种口径**，取悲观的那个当结论。
inline GramEgressBudget ps_gram_egress_budget(int n_scene, int bits_per_entry,
                                              long long n_candidates) {
    GramEgressBudget b;
    b.gram_entries = (long long)n_scene * (n_scene + 1) / 2;
    b.bits_per_entry = bits_per_entry;
    // 按位紧凑：总位数 / 8
    b.gram_bytes = (b.gram_entries * bits_per_entry + 7) / 8;
    b.aty_bytes  = ((long long)n_scene * bits_per_entry + 7) / 8;
    b.bytes_per_candidate = b.gram_bytes + b.aty_bytes;
    b.candidates = n_candidates;
    b.bytes_total = b.bytes_per_candidate * n_candidates;
    b.seconds_at_2GBps = (double)b.bytes_total / 2.0e9;
    b.seconds_at_1GBps = (double)b.bytes_total / 1.0e9;
    // 对照：整块 32 bit 且存满正方形（这是被 X16 否掉的那条路）
    b.bytes_per_cand_at_32bit = (long long)n_scene * n_scene * 4;
    return b;
}

}  // namespace ps

#endif  // PENUMBRASIGHT_PS_CONTRACT_HPP
