// ============================================================================
// PenumbraSight model.psm 二进制格式（方案 §8.1）
//
// 这个头文件是 src/model/penumbrasight_model/psm.py 的 C 端**逐字节镜像**。
// 两侧必须同时改；改完必须跑 tests 里的 format parity 用例（见文件末尾）。
//
// 为什么用 #pragma pack(1)
// ---------------------------------------------------------------------------
// 方案 §8.1 的头部是 64 字节且字段间无填充。默认对齐下编译器会在
// uint64_t payload_bytes 前插入填充，头部会变成 72 字节，板端按 64 读会把
// payload 的起始位置读错 8 字节 —— 而且所有字段看起来都是「合理的数字」，
// 不会崩，只会静默错位。故这里显式 pack(1) 并用 sizeof 静态断言兜住。
//
// 端序
// ---------------------------------------------------------------------------
// 全小端。ALINX AXU9EGB 的 Cortex-A53 是小端，PL 侧 AXI 也是小端，故不需要
// 字节交换；但**量化数据的符号性**要小心：U/V 是 int16、S 是 uint16，
// 直接当 uint16 搬会让负数变成大正数，见 ps_psm_candidate_t 的注释。
//
// 对齐
// ---------------------------------------------------------------------------
// 每个候选记录内四个数组**各自按 4096 字节对齐**，记录本身也 4096 对齐，
// 于是任一数组的绝对偏移都是 4096 的整数倍 —— 这是给 AXI DataMover /
// DMA 描述符用的（跨 4 KB 边界的突发会被拆包，性能掉一截）。
// ============================================================================

#ifndef PENUMBRASIGHT_PSM_FORMAT_H
#define PENUMBRASIGHT_PSM_FORMAT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- 常量

#define PS_PSM_MAGIC0        'P'
#define PS_PSM_MAGIC1        'S'
#define PS_PSM_MAGIC2        'M'
#define PS_PSM_MAGIC3        '1'
#define PS_PSM_MAGIC         "PSM1"

#define PS_PSM_VERSION       1u
#define PS_PSM_HEADER_BYTES  64u
#define PS_PSM_ARRAY_ALIGN   4096u

// B 档冻结尺寸（configs/geometry_b.yaml）
#define PS_PSM_MEAS_H        95u    // 差分后测量行数 = 96 - 1
#define PS_PSM_MEAS_W        96u    // 差分后测量列数
#define PS_PSM_HIDDEN_H      18u    // 隐藏图像行数（z）
#define PS_PSM_HIDDEN_W      24u    // 隐藏图像列数（x）
#define PS_PSM_CANDIDATES    243u   // 9 x 9 x 3

#define PS_PSM_RANK_MAX      64u    // R
#define PS_PSM_RANK_DEFAULT  64u
#define PS_PSM_LOC_RANK      8u     // K

// ---------------------------------------------------------------- 头部

#pragma pack(push, 1)
typedef struct {
    char     magic[4];          // +0   "PSM1"
    uint16_t version;           // +4   1
    uint16_t header_bytes;      // +6   64
    uint16_t meas_h;            // +8   95
    uint16_t meas_w;            // +10  96
    uint16_t hidden_h;          // +12  18
    uint16_t hidden_w;          // +14  24
    uint16_t candidates;        // +16  243
    uint16_t rank_max;          // +18  64（R=96 模型写 96）
    uint16_t rank_default;      // +20  64
    uint16_t loc_rank;          // +22  8
    uint32_t geometry_hash;     // +24  见 psm.py build_geometry_hash
    uint32_t payload_crc32;     // +28  payload 的 CRC32（zlib 多项式）
    uint64_t payload_bytes;     // +32
    uint8_t  reserved[24];      // +40  全 0（保留）
    // 保留区**不**用来放测量归一化的每帧共享指数：那是逐帧的运行时量（由实拍
    // 数据当场算出），与静态模型无关，口径与每帧取值记在 experiment manifest
    // （见 quantize.shared_exponent 与 experiments/manifests/schema.json 的
    // acquisition.measurement_normalization）。模型记录内需要存的绝对幅度是
    // 每个候选的 s_max，它已落在候选头的 +20（见下）。
} ps_psm_header_t;              // 共 64 字节
#pragma pack(pop)

// 头部字段偏移（板端 model_loader 用它做自检，不依赖编译器布局）
#define PS_PSM_OFF_MAGIC          0u
#define PS_PSM_OFF_GEOMETRY_HASH  24u
#define PS_PSM_OFF_PAYLOAD_CRC32  28u
#define PS_PSM_OFF_PAYLOAD_BYTES  32u

// ---------------------------------------------------------------- 候选记录

#pragma pack(push, 1)
typedef struct {
    int16_t x_mm;                // 候选 x，单位 mm（相对标称遮挡中心）
    int16_t wall_distance_mm;    // 候选墙向距离，单位 mm
    int16_t z_mm;                // 候选 z，单位 mm
    int16_t reserved;            // +6   全 0
    float   u_scale;             // +8   U 的 scale（Q1.15 的自适应步长）
    float   s_scale;             // +12  S 的 scale（正常是 2^-14）
    float   v_scale;             // +16  V 的 scale
    float   s_max;               // +20  该候选的**最大奇异值**（未归一化）
} ps_psm_candidate_hdr_t;        // 共 24 字节
#pragma pack(pop)

// 候选头字段偏移
#define PS_PSM_OFF_CAND_S_MAX     20u

// s_max 的意义（2026-09-29 起入 psm，此前只在 manifest 里）
// ---------------------------------------------------------------------------
// psm 里的 S 是**归一化**奇异值（最大值 1），Q2.14 存放；还原绝对幅度必须乘
// s_max。修复前 s_max 只在 manifest.json，只读 psm 的板端拿到的是 B/s_max，
// 重建得靠调用方从旁边补一个系数 —— 那是接口缺口。现在每个候选记录自带宽
// 度：S_abs[k] = S_code[k] * s_scale * s_max。
//
// 为什么放在候选头而不是 64 字节文件头：s_max 逐候选不同（实测 B 档落在
// 63~81），文件头是每文件一份，装不下 243 个候选的值。候选头 20 -> 24 字节
// 不改变任何数组偏移：U 起点仍是 align_up(sizeof(hdr), 4096) == 4096，
// 故 record_bytes 与 payload_bytes 一个字节都不变（见 ps_psm_u_offset）。

// 记录内四个数组的**相对**偏移（字节）。用函数算而不是宏，方便按 rank_max 变化。
//
// 注意：U 与 V 都是 **rank-major**（rank 为最外层）。定位只读前 K 个 rank 时，
// 那 K*M 个 int16 在文件里是**连续**的，一次 pread / 一次 DMA 就够。
// 若哪天改成 measurement-major，「只读前 K 列」立刻退化成 9120 次跨步读取，
// 方案 §5.3 的「单次读取约 35.5 MB」假设作废。
static inline uint64_t ps_psm_align_up(uint64_t n, uint64_t a) {
    return ((n + a - 1u) / a) * a;
}

static inline uint64_t ps_psm_u_offset(uint16_t rank_max, uint64_t meas_len) {
    (void)rank_max;
    (void)meas_len;
    // U 永远紧跟在候选头之后（候选头 24 B -> 补齐到 4096）
    return ps_psm_align_up(sizeof(ps_psm_candidate_hdr_t), PS_PSM_ARRAY_ALIGN);
}

static inline uint64_t ps_psm_s_offset(uint16_t rank_max, uint64_t meas_len) {
    const uint64_t u = ps_psm_u_offset(rank_max, meas_len);
    return ps_psm_align_up(u + (uint64_t)rank_max * meas_len * 2u, PS_PSM_ARRAY_ALIGN);
}

static inline uint64_t ps_psm_v_offset(uint16_t rank_max, uint64_t meas_len) {
    const uint64_t s = ps_psm_s_offset(rank_max, meas_len);
    return ps_psm_align_up(s + (uint64_t)rank_max * 2u, PS_PSM_ARRAY_ALIGN);
}

static inline uint64_t ps_psm_mask_offset(uint16_t rank_max, uint64_t meas_len,
                                          uint64_t hidden_len) {
    const uint64_t v = ps_psm_v_offset(rank_max, meas_len);
    return ps_psm_align_up(v + (uint64_t)rank_max * hidden_len * 2u,
                           PS_PSM_ARRAY_ALIGN);
}

// 单条候选记录的总字节数（含内部对齐与尾部补齐）。
// B 档 R=64 / M=9120 / N=432 时等于 1,236,992 = 302 x 4096。
static inline uint64_t ps_psm_record_bytes(uint16_t rank_max, uint64_t meas_len,
                                           uint64_t hidden_len) {
    const uint64_t m = ps_psm_mask_offset(rank_max, meas_len, hidden_len);
    return ps_psm_align_up(m + hidden_len, PS_PSM_ARRAY_ALIGN);
}

// 便捷派生量
static inline uint64_t ps_psm_meas_len(const ps_psm_header_t* h) {
    return (uint64_t)h->meas_h * (uint64_t)h->meas_w;
}
static inline uint64_t ps_psm_hidden_len(const ps_psm_header_t* h) {
    return (uint64_t)h->hidden_h * (uint64_t)h->hidden_w;
}
static inline uint64_t ps_psm_record_bytes_hdr(const ps_psm_header_t* h) {
    return ps_psm_record_bytes(h->rank_max, ps_psm_meas_len(h),
                               ps_psm_hidden_len(h));
}

// 第 p 个候选记录的**绝对**文件偏移。
static inline uint64_t ps_psm_candidate_offset(const ps_psm_header_t* h,
                                               uint32_t index) {
    return (uint64_t)PS_PSM_HEADER_BYTES + (uint64_t)index * ps_psm_record_bytes_hdr(h);
}

// ---------------------------------------------------------------- 量化数据访问
//
// U/V 是 int16（有符号）、S 是 uint16（无符号）、mask 是 uint8。
// 把 U 当 uint16 读会让每个负数变成 30000 多的大正数，点积结果完全错乱、
// 而且不越界不崩。故这里给三个显式的读取器，**不要在别处用指针硬转**。
//
// 另外：U/V 是定点码字，真实值是 code * scale。scale 在候选头里，**每个候选
// 不同**，所以先 ps_psm_candidate_hdr_t 再读数组，顺序不能反。

static inline int16_t ps_psm_read_i16(const uint8_t* base, uint64_t byte_off) {
    int16_t v;
    __builtin_memcpy(&v, base + byte_off, sizeof(v));
    return v;
}

static inline uint16_t ps_psm_read_u16(const uint8_t* base, uint64_t byte_off) {
    uint16_t v;
    __builtin_memcpy(&v, base + byte_off, sizeof(v));
    return v;
}

static inline float ps_psm_read_f32(const uint8_t* base, uint64_t byte_off) {
    float v;
    __builtin_memcpy(&v, base + byte_off, sizeof(v));
    return v;
}

// 取某个候选的最大奇异值 s_max。S 在 psm 里是归一化值，绝对幅度是
//     S_abs[k] = ps_psm_read_S_k(cand_base, h, k) * s_scale * s_max
// 板端只走 psm（不看 manifest）时用它补回绝对幅度，接口见候选头的注释。
static inline float ps_psm_read_s_max(const uint8_t* cand_base) {
    return ps_psm_read_f32(cand_base, PS_PSM_OFF_CAND_S_MAX);
}

// 取某个候选前 k 个 rank 的 U 系数第 m 个元素（返回定点码字，未乘 scale）。
// 布局 rank-major => 元素 (k_i, m) 的字节偏移 = k_i * meas_len * 2 + m * 2。
static inline int16_t ps_psm_read_U_km(const uint8_t* cand_base,
                                       const ps_psm_header_t* h,
                                       uint16_t k_i, uint64_t m) {
    const uint64_t off = ps_psm_u_offset(h->rank_max, ps_psm_meas_len(h))
                       + ((uint64_t)k_i * ps_psm_meas_len(h) + m) * 2u;
    return ps_psm_read_i16(cand_base, off);
}

static inline uint16_t ps_psm_read_S_k(const uint8_t* cand_base,
                                       const ps_psm_header_t* h, uint16_t k_i) {
    const uint64_t off = ps_psm_s_offset(h->rank_max, ps_psm_meas_len(h))
                       + (uint64_t)k_i * 2u;
    return ps_psm_read_u16(cand_base, off);
}

static inline int16_t ps_psm_read_V_kn(const uint8_t* cand_base,
                                       const ps_psm_header_t* h,
                                       uint16_t k_i, uint64_t n) {
    const uint64_t off = ps_psm_v_offset(h->rank_max, ps_psm_meas_len(h))
                       + ((uint64_t)k_i * ps_psm_hidden_len(h) + n) * 2u;
    return ps_psm_read_i16(cand_base, off);
}

static inline uint8_t ps_psm_read_mask_n(const uint8_t* cand_base,
                                         const ps_psm_header_t* h, uint64_t n) {
    const uint64_t off = ps_psm_mask_offset(h->rank_max, ps_psm_meas_len(h),
                                            ps_psm_hidden_len(h)) + n;
    return cand_base[off];
}

// ---------------------------------------------------------------- 校验

enum {
    PS_PSM_OK              = 0,
    PS_PSM_ERR_MAGIC       = 1,
    PS_PSM_ERR_VERSION     = 2,
    PS_PSM_ERR_HEADER_SIZE = 3,
    PS_PSM_ERR_SHAPE       = 4,   // 尺寸组合不合法或与期望不符
    PS_PSM_ERR_RANK        = 5,   // 秩关系不满足 0 < loc <= default <= max
    PS_PSM_ERR_GEOM_HASH   = 6,   // 模型几何哈希与当前配置不一致
    PS_PSM_ERR_CRC32       = 7,
    PS_PSM_ERR_SIZE        = 8,   // payload 长度对不上
    PS_PSM_ERR_CANDIDATES  = 9,   // 候选数与实际记录数不符
    PS_PSM_ERR_ALIGN       = 10   // 数组偏移不是 4096 的整数倍
};

// 只做**不需要读全文件**的头部校验（板端启动路径）。
// 几何哈希必须与当前配置比对：方案 §6.6 要求不一致时强制灰屏，不许出图。
//
// expect_geometry_hash 传 0 表示跳过哈希检查（仅调试用）。
static inline int ps_psm_check_header(const ps_psm_header_t* h,
                                      uint32_t expect_geometry_hash,
                                      uint32_t expect_meas_h,
                                      uint32_t expect_meas_w,
                                      uint32_t expect_hidden_h,
                                      uint32_t expect_hidden_w) {
    if (h->magic[0] != PS_PSM_MAGIC0 || h->magic[1] != PS_PSM_MAGIC1 ||
        h->magic[2] != PS_PSM_MAGIC2 || h->magic[3] != PS_PSM_MAGIC3) {
        return PS_PSM_ERR_MAGIC;
    }
    if (h->version != PS_PSM_VERSION) return PS_PSM_ERR_VERSION;
    if (h->header_bytes != PS_PSM_HEADER_BYTES) return PS_PSM_ERR_HEADER_SIZE;
    if (h->meas_h == 0 || h->meas_w == 0 ||
        h->hidden_h == 0 || h->hidden_w == 0 || h->candidates == 0) {
        return PS_PSM_ERR_SHAPE;
    }
    if (!(h->loc_rank > 0 && h->loc_rank <= h->rank_default &&
          h->rank_default <= h->rank_max)) {
        return PS_PSM_ERR_RANK;
    }
    if (expect_meas_h && h->meas_h != expect_meas_h) return PS_PSM_ERR_SHAPE;
    if (expect_meas_w && h->meas_w != expect_meas_w) return PS_PSM_ERR_SHAPE;
    if (expect_hidden_h && h->hidden_h != expect_hidden_h) return PS_PSM_ERR_SHAPE;
    if (expect_hidden_w && h->hidden_w != expect_hidden_w) return PS_PSM_ERR_SHAPE;
    if (expect_geometry_hash && h->geometry_hash != expect_geometry_hash) {
        return PS_PSM_ERR_GEOM_HASH;
    }
    // 对齐自检：四个数组偏移都必须是 4096 的整数倍
    {
        const uint64_t m = ps_psm_meas_len(h), n = ps_psm_hidden_len(h);
        const uint64_t offs[4] = {
            ps_psm_u_offset(h->rank_max, m),
            ps_psm_s_offset(h->rank_max, m),
            ps_psm_v_offset(h->rank_max, m),
            ps_psm_mask_offset(h->rank_max, m, n),
        };
        for (int i = 0; i < 4; ++i) {
            if (offs[i] % PS_PSM_ARRAY_ALIGN != 0u) return PS_PSM_ERR_ALIGN;
        }
    }
    // payload 长度自检
    {
        const uint64_t need = (uint64_t)h->candidates * ps_psm_record_bytes_hdr(h);
        if (h->payload_bytes != need) return PS_PSM_ERR_SIZE;
    }
    return PS_PSM_OK;
}

// 头部与候选头的 sizeof 必须与 psm.py 逐字节一致。编译器布局出问题时在这里炸，
// 而不是在板上表现为「读到的数字看着都合理但结果全错」。
#if defined(__cplusplus)
static_assert(sizeof(ps_psm_header_t) == PS_PSM_HEADER_BYTES,
              "ps_psm_header_t 必须正好 64 字节；检查 #pragma pack(1) 是否生效");
static_assert(sizeof(ps_psm_candidate_hdr_t) == 24,
              "ps_psm_candidate_hdr_t 必须正好 24 字节");
#else
_Static_assert(sizeof(ps_psm_header_t) == PS_PSM_HEADER_BYTES,
               "ps_psm_header_t 必须正好 64 字节");
_Static_assert(sizeof(ps_psm_candidate_hdr_t) == 24,
               "ps_psm_candidate_hdr_t 必须正好 24 字节");
#endif

// ---------------------------------------------------------------- CRC32
//
// 与 Python zlib.crc32 同参数（多项式 0xEDB88320 反射，初值 0xFFFFFFFF，
// 末取反）。这里只声明，实现放在 software/model_loader/psm_crc32.c，
// 便于板端复用同一份代码。
uint32_t ps_crc32(uint32_t crc, const uint8_t* data, size_t len);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // PENUMBRASIGHT_PSM_FORMAT_H
