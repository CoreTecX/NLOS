// ============================================================================
// X12 三条数据源（header-only，主机侧）
//
//   kSrcCamera  相机取流   —— 同一接口，实物未到故 pull 返回「不可用」
//   kSrcFile    文件灌入   —— 裸帧文件，uint16 小端、帧主序
//   kSrcSim     仿真激励   —— 由 (seed, 索引) 的纯哈希给出，可任意位置续取
//
// 三者共用同一个 `RawSource` 结构（函数指针 + 不透明 ctx），上层只用
// `ps_pump_frames()` 驱动，**不知道也不关心**数据从哪来。X12 的判据就是这一条。
//
// pull 的返回语义（照 read(2) 的口径，避免两套码混用）
//   >= 0  实际取到的像素个数；**0 表示流已尽**
//   -1    数据源不可用（实物未到）
//   -2    出错
// 首版曾让 pull 返回状态枚举而由调用方假定「取满了」——那在流尾会虚高计数。
// 位置无关哈希让「取不满」与「取满」都能正确表达，故改用计数语义。
//
// 为什么仿真源用「位置无关哈希」而不是有状态的 LCG：
// 有状态生成器一旦被分段 pull（PL 流水上必然如此）就无法从中间续起，只能
// rewind 重跑。位置无关的哈希让 pull 可以在任意偏移处独立取值，与真实流一致。
// ============================================================================

#ifndef PENUMBRASIGHT_PS_SOURCES_HPP
#define PENUMBRASIGHT_PS_SOURCES_HPP

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>

#include "ps_contract.hpp"

namespace ps {

// ============================================================================
// 统一接口
// ============================================================================

enum RawSourceKind {
    kSrcCamera = 0,
    kSrcFile   = 1,
    kSrcSim    = 2
};

#define PS_PULL_UNAVAILABLE (-1)
#define PS_PULL_ERROR       (-2)

// 取不超过 max_pixels 个像素写入 dst；返回实际个数，0 表示流尽，负数为错误。
struct RawSource {
    RawSourceKind kind;
    void*         ctx;
    int         (*pull)(void* ctx, uint16_t* dst, int max_pixels);
    int         (*rewind)(void* ctx);
    const char*   name;
    int           spec_known;   // 1 = ctx 里带着 RawStreamSpec，可自报规格
};

// 用一条源把 n_frames 帧的像素灌满 dst（长度 = 每帧像素数 * n_frames）。
// 这是三条路径**唯一**的驱动方式——换源不改这里。
// 返回灌入的像素数；负数为错误码。
inline long long ps_pump_frames(RawSource& src, long long pixels_per_frame,
                                int n_frames, uint16_t* dst) {
    const long long want = pixels_per_frame * (long long)n_frames;
    long long got = 0;
    while (got < want) {
        const long long remain = want - got;
        const int ask = (remain > 65536) ? 65536 : (int)remain;
        const int r = src.pull(src.ctx, dst + got, ask);
        if (r < 0) return (long long)r;
        if (r == 0) break;                 // 流已尽
        got += r;
    }
    return got;
}

// ============================================================================
// 仿真激励源
// ============================================================================

// 位置无关的 splitmix64，纯函数、无状态。
inline uint64_t ps_sim_hash(uint64_t seed, long long k) {
    uint64_t x = seed ^ ((uint64_t)k * 0x9E3779B97F4A7C15ULL);
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

// 第 k 个像素的值。落在 [dark_level, 满量程] 内，故不会在 M1 里越界。
inline uint16_t ps_sim_pixel(const RawStreamSpec& s, uint64_t seed, long long k) {
    const uint64_t fs = ps_depth_fullscale(s.container_bits);
    const uint64_t lo = (s.dark_level <= fs) ? s.dark_level : 0;
    const uint64_t span = (fs >= lo) ? (fs - lo) : 0;
    const uint64_t h = ps_sim_hash(seed, k);
    uint64_t v = lo + ((span > 0) ? (h % (span + 1)) : 0);
    if (v > fs) v = fs;
    return (uint16_t)v;
}

// 一次性生成整段到缓冲。离线分析、造测试向量都走这一条，
// 与 pull 逐段取出的结果**必然一致**（都是 ps_sim_pixel 的纯函数）。
inline void ps_sim_fill(const RawStreamSpec& s, uint64_t seed, uint16_t* dst,
                        long long n_pixels) {
    for (long long k = 0; k < n_pixels; ++k) {
        dst[k] = ps_sim_pixel(s, seed, k);
    }
}

struct SimRawCtx {
    RawStreamSpec spec;
    uint64_t      seed;
    long long     total_pixels;
    long long     pos;
};

inline int ps_sim_pull(void* v, uint16_t* dst, int max_pixels) {
    SimRawCtx* c = (SimRawCtx*)v;
    if (!c || max_pixels <= 0) return PS_PULL_ERROR;
    if (c->pos >= c->total_pixels) return 0;
    long long n = c->total_pixels - c->pos;
    if (n > max_pixels) n = max_pixels;
    for (long long i = 0; i < n; ++i) {
        dst[i] = ps_sim_pixel(c->spec, c->seed, c->pos + i);
    }
    c->pos += n;
    return (int)n;
}

inline int ps_sim_rewind(void* v) {
    SimRawCtx* c = (SimRawCtx*)v;
    if (!c) return PS_PULL_ERROR;
    c->pos = 0;
    return 0;
}

inline RawSource ps_make_sim_source(SimRawCtx* c) {
    RawSource s;
    s.kind = kSrcSim; s.ctx = c;
    s.pull = ps_sim_pull; s.rewind = ps_sim_rewind;
    s.name = "sim"; s.spec_known = 1;
    if (c) c->pos = 0;
    return s;
}

// ============================================================================
// 文件灌入源
//
// 文件布局（冻结）：无头，uint16 小端，帧主序
//   [frame0: width*height 个像素][frame1: ...]...
// 与 sim 源产出的字节**完全相同**，所以两者可以互换（见测试 T2/T3）。
// ============================================================================

struct FileRawCtx {
    std::FILE* f;
    long long  total_pixels;
    long long  pos;
};

inline int ps_file_pull(void* v, uint16_t* dst, int max_pixels) {
    FileRawCtx* c = (FileRawCtx*)v;
    if (!c || !c->f || max_pixels <= 0) return PS_PULL_ERROR;
    if (c->pos >= c->total_pixels) return 0;
    long long n = c->total_pixels - c->pos;
    if (n > max_pixels) n = max_pixels;
    const std::size_t want = (std::size_t)n * sizeof(uint16_t);
    const std::size_t got = std::fread(dst, 1, want, c->f);
    if (got != want) return PS_PULL_ERROR;
    c->pos += n;
    return (int)n;
}

inline int ps_file_rewind(void* v) {
    FileRawCtx* c = (FileRawCtx*)v;
    if (!c || !c->f) return PS_PULL_ERROR;
    std::fseek(c->f, 0, SEEK_SET);
    c->pos = 0;
    return 0;
}

inline RawSource ps_make_file_source(FileRawCtx* c) {
    RawSource s;
    s.kind = kSrcFile; s.ctx = c;
    s.pull = ps_file_pull; s.rewind = ps_file_rewind;
    s.name = "file"; s.spec_known = 0;
    if (c) c->pos = 0;
    return s;
}

// 前置声明：ps_file_open 在出错分支里要关掉半开的文件，定义在下方。
inline void ps_file_close(FileRawCtx* c);

// 打开 + 依文件长度推断总像素数。frame_pixels 传每帧像素数。
// 返回 0 成功，负数为错误码。
inline int ps_file_open(FileRawCtx* c, const std::string& path,
                        long long frame_pixels) {
    std::memset(c, 0, sizeof(*c));
    c->f = std::fopen(path.c_str(), "rb");
    if (!c->f) return PS_PULL_ERROR;
    std::fseek(c->f, 0, SEEK_END);
    const long long bytes = (long long)std::ftell(c->f);
    std::fseek(c->f, 0, SEEK_SET);
    c->total_pixels = bytes / (long long)sizeof(uint16_t);
    c->pos = 0;
    if (frame_pixels > 0 && (c->total_pixels % frame_pixels) != 0) {
        ps_file_close(c);
        return PS_PULL_ERROR;   // 长度不是整数帧
    }
    return 0;
}

inline void ps_file_close(FileRawCtx* c) {
    if (c && c->f) { std::fclose(c->f); c->f = NULL; }
}

// 把像素写成一个裸帧文件。测试与离线造向量都用它。
inline int ps_write_raw_frames(const std::string& path, const uint16_t* data,
                               long long n_pixels) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return PS_PULL_ERROR;
    const std::size_t w = std::fwrite(data, sizeof(uint16_t),
                                      (std::size_t)n_pixels, f);
    std::fclose(f);
    return (w == (std::size_t)n_pixels) ? 0 : PS_PULL_ERROR;
}

// ============================================================================
// 相机取流源
//
// 实物未到，故 pull 恒返回 PS_PULL_UNAVAILABLE。**但它实现了同一接口**，
// 所以上层不用为「还没有相机」写分支——这正是 X12 要买到的性质。
// 实物到后，只需把 ps_camera_pull 换成真正的 V4L2 / MVS SDK 调用，
// 上层的 pump 与 M1 一行不改。
// ============================================================================

struct CameraRawCtx {
    RawStreamSpec spec;
    char          dev[64];
    int           opened;      // 0 = 未打开
    int           last_err;    // 最近一次失败原因
};

inline int ps_camera_open(CameraRawCtx* c, const char* dev) {
    if (!c) return PS_PULL_ERROR;
    c->opened = 0;
    c->last_err = PS_PULL_UNAVAILABLE;
    std::memset(c->dev, 0, sizeof(c->dev));
    if (dev) std::strncpy(c->dev, dev, sizeof(c->dev) - 1);
    return PS_PULL_UNAVAILABLE;   // 实物未到
}

inline int ps_camera_pull(void* v, uint16_t* dst, int max_pixels) {
    (void)dst; (void)max_pixels;
    CameraRawCtx* c = (CameraRawCtx*)v;
    if (!c) return PS_PULL_ERROR;
    c->last_err = PS_PULL_UNAVAILABLE;
    return PS_PULL_UNAVAILABLE;
}

inline int ps_camera_rewind(void* v) {
    CameraRawCtx* c = (CameraRawCtx*)v;
    if (!c) return PS_PULL_ERROR;
    return PS_PULL_UNAVAILABLE;
}

inline RawSource ps_make_camera_source(CameraRawCtx* c) {
    RawSource s;
    s.kind = kSrcCamera; s.ctx = c;
    s.pull = ps_camera_pull; s.rewind = ps_camera_rewind;
    s.name = "camera"; s.spec_known = 1;
    return s;
}

}  // namespace ps

#endif  // PENUMBRASIGHT_PS_SOURCES_HPP
