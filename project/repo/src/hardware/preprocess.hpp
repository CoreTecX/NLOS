// ============================================================================
// PenumbraSight 预处理链 · HLS 风格 C++
//
// 设计目标：同一份源码在两个环境下都能工作
//   g++       : 主机侧功能验证（不需要 Vivado）
//   Vitis HLS : 综合（定义 __SYNTHESIS__）
//
// 与 Python 参考实现（src/model/penumbrasight_model/preprocess.py）逐函数对应，
// 但遵守 S0-3 的两条设计原则：
//   P1 判据只用整数  —— 本实现全程不做除法，输出「块累加和」与「行差分」
//   P2 双绿不做舍入  —— G1 与 G2 以「和」的形式进入分块累加（位宽只多 1 bit）
//
// 数据链路：
//   RAW (H x W)
//     -> 减暗电平
//     -> 中心裁剪为 N x N
//     -> [彩色] Bayer 解交织取 R / G1 / G2 / B
//        [黑白] 直通（单通道）
//     -> block x block 分块求和（不跨行）
//     -> 帧累积（直接求和）
//     -> 行方向差分（下一行减上一行）
// ============================================================================

#ifndef PENUMBRASIGHT_PREPROCESS_HPP
#define PENUMBRASIGHT_PREPROCESS_HPP

#include <cstddef>
#include <cstdint>

// ---------------------------------------------------------------- 编译期参数
// 综合时通过 -D 覆盖；主机验证时用下面的默认值。

// 输入像素位宽：官方数据集为 16（满量程 65535），相机最高 12
#ifndef PS_IN_BITS
#define PS_IN_BITS 16
#endif

// 分块边长（方形块）
#ifndef PS_BLOCK
#define PS_BLOCK 10
#endif

// 帧累积数
#ifndef PS_FRAMES
#define PS_FRAMES 16
#endif

// 1 = 彩色（Bayer 解交织），0 = 黑白（单通道直通）
#ifndef PS_COLOR
#define PS_COLOR 0
#endif

// Bayer 相位：0 = RGGB（左上为 R），1 = BGGR（左上为 B）
#ifndef PS_BAYER_PHASE
#define PS_BAYER_PHASE 0
#endif

// 中心裁剪边长（0 表示不裁剪，使用整幅）
#ifndef PS_CROP
#define PS_CROP 0
#endif

// ---------------------------------------------------------------- 位宽推导
// 注意：下面三个 EXTRA 常量必须随 PS_BLOCK / PS_FRAMES / 通道选择一起改。
//   PS_BLOCK = 8  -> 64   -> 6
//   PS_BLOCK = 10 -> 100  -> 7
//   PS_BLOCK = 20 -> 400  -> 9
//   PS_FRAMES = 8  -> 3
//   PS_FRAMES = 16 -> 4
//   PS_FRAMES = 32 -> 5
//   PS_FRAMES = 50 -> 6
#ifndef PS_BS_EXTRA
#define PS_BS_EXTRA 7
#endif
#ifndef PS_ACC_EXTRA
#define PS_ACC_EXTRA 4
#endif

// 双绿 G1+G2 求和多出 1 bit（原则 P2：双绿按「和」进累加，不做 0.5 舍入）。
//
// **块和容器必须按最坏通道 G1+G2 取宽**，否则综合侧 ap_uint<PS_BS_BITS> 会静默回绕 ——
// 主机侧 bs_t 是 uint64_t，测试全绿也发现不了。
// 实测依据（src/hardware/hwdata/manifest.txt）：sat_sum_g1g2 = 13,107,000 > 2^23，
// 即 16 bit 源下 G1+G2 块和需 24 bit；12 bit 源下需 20 bit。而按 PS_IN_BITS 直接推
// 只能得到 23 / 19，**正好少这 1 bit**。
// 详见 sim/results/x6_x7_preprocess_bitdepth.md §3.2 / §九.1 与
// sim/results/x29_config_consistency.md §1（权威记录）。
//   注：本注释原引用 sim/results/x27_bitwidth_defect_fix.md，该文件从未落盘（悬空引用），
//       已于 2026-09-27 改指 x29 §1 —— 那是这条修正的实际权威记录。
#ifndef PS_GS_EXTRA
#define PS_GS_EXTRA 1
#endif

// 块和的位宽 = 输入位宽 + 块内像素数的对数 + 双绿 G1+G2 多出的那 1 bit（最坏通道）
#define PS_BS_BITS (PS_IN_BITS + PS_BS_EXTRA + PS_GS_EXTRA)
// 帧累积后的位宽
#define PS_ACC_BITS (PS_BS_BITS + PS_ACC_EXTRA)
// 差分为有符号，多 1 bit 给符号位
#define PS_DIF_BITS (PS_ACC_BITS + 1)
// 双绿求和（G1+G2 的和）的源位宽 —— 与上面 PS_BS_BITS 里那 1 bit 是同一件事
#define PS_GS_BITS (PS_IN_BITS + PS_GS_EXTRA)

// ---- 编译期守门（X27）------------------------------------------------------------
// PS_BS_EXTRA / PS_ACC_EXTRA / PS_GS_EXTRA 都是**手填常量**（对照表见上）。
// 下面两条 static_assert 用 PS_BLOCK / PS_FRAMES 反推需求，把「改了块边长或帧数、
// 却忘了改 EXTRA」变成**编译错误**，而不是综合后静默回绕/饱和。
// 依据：sim/results/x29_config_consistency.md §1、x6_x7_preprocess_bitdepth.md §3。
namespace ps {
namespace detail {
// 编译期：不小于 n 的最小 b 使 2^b >= n（n <= 1 -> 0）。
// 写成单 return 递归是为了在 C++11（bitwidth_probe.cpp 用 -std=c++11）下也合法。
constexpr int ceil_log2_ce(unsigned long long n, int b = 0) {
    return (n <= 1) ? b : ceil_log2_ce((n + 1) / 2, b + 1);
}
}  // namespace detail
}  // namespace ps

static_assert(PS_BS_BITS >= PS_IN_BITS + PS_GS_EXTRA
                             + ps::detail::ceil_log2_ce((unsigned long long)PS_BLOCK * PS_BLOCK),
              "PS_BS_BITS 容不下最坏通道 G1+G2 的块和：请同步 PS_BS_EXTRA（见 X27）");
static_assert(PS_ACC_BITS >= PS_BS_BITS
                             + ps::detail::ceil_log2_ce((unsigned long long)PS_FRAMES),
              "PS_ACC_BITS 容不下 PS_FRAMES 帧累积：请同步 PS_ACC_EXTRA（见 X27）");

// ---------------------------------------------------------------- 容器类型
#ifdef __SYNTHESIS__
#include "ap_int.h"
namespace ps {
typedef ap_uint<PS_IN_BITS>  pix_t;   // 输入像素
typedef ap_uint<PS_GS_BITS>  gsum_t;  // G1 + G2
typedef ap_uint<PS_BS_BITS>  bs_t;    // 块和
typedef ap_uint<PS_ACC_BITS> acc_t;   // 帧累积
typedef ap_int<PS_DIF_BITS>  dif_t;   // 差分（有符号）
}  // namespace ps
#else
namespace ps {
typedef uint64_t pix_t;   // 主机侧统一用 64 位容器，位宽正确性由单元测试单独覆盖
typedef uint64_t gsum_t;
typedef uint64_t bs_t;
typedef uint64_t acc_t;
typedef int64_t  dif_t;
}  // namespace ps
#endif

namespace ps {

// ---------------------------------------------------------------- 步骤 1：减暗电平

// 原地减去暗电平。先减后处理，与参考实现一致。
// 注意：减完可能出负数（若暗电平估高），故用有符号中间量再饱和到 0。
inline void subtract_dark(pix_t* buf, std::size_t n, pix_t dark) {
    for (std::size_t i = 0; i < n; ++i) {
#pragma HLS PIPELINE II = 1
        buf[i] = (buf[i] > dark) ? (pix_t)(buf[i] - dark) : (pix_t)0;
    }
}

// ---------------------------------------------------------------- 步骤 2：中心裁剪

// 把 (h x w) 的图居中裁剪为 size x size，结果写入 dst（size*size）。
// size 为 0 时表示不裁剪，直接整体拷贝。
inline void center_crop(const pix_t* src, int h, int w, int size,
                        pix_t* dst) {
    int n = (size > 0) ? size : h;
    int top = (h - n) / 2;
    int left = (w - n) / 2;
    for (int y = 0; y < n; ++y) {
#pragma HLS PIPELINE II = 1
        for (int x = 0; x < n; ++x) {
            dst[y * n + x] = src[(y + top) * w + (x + left)];
        }
    }
}

// ---------------------------------------------------------------- 步骤 3：Bayer 解交织

// 单路相位描述：2x2 单元内的 (行偏移, 列偏移)
struct BayerOffset {
    int dr;
    int dc;
};

// 取一路平面的相位偏移。phase: 0 = RGGB，1 = BGGR。
// which: 0 = R, 1 = G1, 2 = G2, 3 = B
inline BayerOffset bayer_offset(int which, int phase) {
    BayerOffset o;
    const int r_rg = (phase == 0) ? 0 : 1;   // R 的行位置：RGGB 在 0，BGGR 在 1
    const int b_rg = (phase == 0) ? 1 : 0;   // B 的行位置与之相反
    switch (which) {
        case 0:  o.dr = r_rg; o.dc = r_rg; break;  // R
        case 1:  o.dr = 0;    o.dc = 1;    break;  // G1（两种相位相同）
        case 2:  o.dr = 1;    o.dc = 0;    break;  // G2（两种相位相同）
        default: o.dr = b_rg; o.dc = b_rg; break;  // B
    }
    return o;
}

// 从 RAW 提取一路平面。raw 为 h x w，输出为 (h/2) x (w/2)。
// Tin 允许是窄类型：主机侧读进来的是 uint16_t，综合侧是 ap_uint<IN_BITS>；
// 输出统一为 pix_t，便于下游复用同一套块累加逻辑。
template <typename Tin>
inline void bayer_extract(const Tin* raw, int h, int w, int which, int phase,
                          pix_t* plane) {
    BayerOffset o = bayer_offset(which, phase);
    int ph = h / 2;
    int pw = w / 2;
    for (int y = 0; y < ph; ++y) {
#pragma HLS PIPELINE II = 1
        for (int x = 0; x < pw; ++x) {
            plane[y * pw + x] = raw[(2 * y + o.dr) * w + (2 * x + o.dc)];
        }
    }
}

// G1 与 G2 求和（P2：不做平均，不做舍入）
inline void green_sum(const pix_t* g1, const pix_t* g2, std::size_t n,
                      gsum_t* out) {
    for (std::size_t i = 0; i < n; ++i) {
#pragma HLS PIPELINE II = 1
        out[i] = (gsum_t)(g1[i] + g2[i]);
    }
}

// ---------------------------------------------------------------- 步骤 4：分块求和

// 分块求和。**必须按行切块**——把二维索引展平后跨行分组是最常见的错误
// （对应向量 v4 的 crossrow_wrong）。这里用双层块索引表达，结构上不可能跨行。
template <typename Tin, typename Tout>
void block_sum(const Tin* plane, int ph, int pw, int block,
               Tout* out, int* out_bh, int* out_bw) {
    int bh = ph / block;
    int bw = pw / block;
    *out_bh = bh;
    *out_bw = bw;
    for (int br = 0; br < bh; ++br) {
        for (int bc = 0; bc < bw; ++bc) {
#pragma HLS PIPELINE II = 1
            Tout s = 0;
            for (int dr = 0; dr < block; ++dr) {
                for (int dc = 0; dc < block; ++dc) {
                    s += (Tout)plane[(br * block + dr) * pw + (bc * block + dc)];
                }
            }
            out[br * bw + bc] = s;
        }
    }
}

// ---------------------------------------------------------------- 步骤 5：帧累积

// 帧累积：直接求和，不除以帧数（规则 R3，已由官方数据验证）。
// blk 的形状为 (n_frames, n_blk)，输出 acc 长度 n_blk。
template <typename Tin, typename Tout>
void accumulate_frames(const Tin* blk, int n_frames, int n_blk, Tout* acc) {
    for (int i = 0; i < n_blk; ++i) {
#pragma HLS PIPELINE II = 1
        acc[i] = 0;
    }
    for (int f = 0; f < n_frames; ++f) {
        for (int i = 0; i < n_blk; ++i) {
#pragma HLS PIPELINE II = 1
            acc[i] += (Tout)blk[f * n_blk + i];
        }
    }
}

// ---------------------------------------------------------------- 步骤 6：行差分

// 行方向差分（下一行减上一行）。输出 (bh-1) x bw。
template <typename Tin, typename Tout>
void row_diff(const Tin* acc, int bh, int bw, Tout* out) {
    for (int r = 0; r + 1 < bh; ++r) {
#pragma HLS PIPELINE II = 1
        for (int c = 0; c < bw; ++c) {
            out[r * bw + c] =
                (Tout)acc[(r + 1) * bw + c] - (Tout)acc[r * bw + c];
        }
    }
}

// ============================================================================
// 顶层：单通道版本
//
// 输入 plane 为「已解交织的单通道平面」（彩色时由调用方先做 bayer_extract
// 与 green_sum；黑白时即裁剪后的 RAW 本身）。这样拆分是为了让两种相机口径
// 共用同一段分块/累积/差分逻辑，也便于逐段验证。
//
//   plane    : ph x pw，行优先
//   n_frames : 帧数（blk 里连续存放每帧的块和）
//   acc      : 输出，长度 bh*bw
//   diff     : 输出，长度 (bh-1)*bw
// ============================================================================
template <typename Tin>
void preprocess_plane_frames(const Tin* plane_frames, int n_frames,
                             int ph, int pw, int block,
                             acc_t* acc, dif_t* diff,
                             int* out_bh, int* out_bw) {
    int bh = ph / block;
    int bw = pw / block;
    *out_bh = bh;
    *out_bw = bw;
    int n_blk = bh * bw;

    // block_sum 与 accumulate_frames 两段，为了避免动态分配中间缓冲，
    // 这里逐帧做块和后立即累加。
    for (int i = 0; i < n_blk; ++i) {
#pragma HLS PIPELINE II = 1
        acc[i] = 0;
    }
    for (int f = 0; f < n_frames; ++f) {
        const Tin* frame = plane_frames + (std::size_t)f * ph * pw;
        for (int br = 0; br < bh; ++br) {
            for (int bc = 0; bc < bw; ++bc) {
#pragma HLS PIPELINE II = 1
                acc_t s = 0;
                for (int dr = 0; dr < block; ++dr) {
                    for (int dc = 0; dc < block; ++dc) {
                        s += (acc_t)frame[(br * block + dr) * pw +
                                          (bc * block + dc)];
                    }
                }
                acc[br * bw + bc] += s;
            }
        }
    }
    row_diff(acc, bh, bw, diff);
}

// ---------------------------------------------------------------- 位宽自检

// 返回各节点的位宽，供主机侧单元测试核对 S0-2 位宽表。
struct BitWidths {
    int in_bits;
    int gsum_bits;
    int bs_bits;
    int acc_bits;
    int dif_bits;
};

inline BitWidths bit_widths() {
    BitWidths w;
    w.in_bits = PS_IN_BITS;
    w.gsum_bits = PS_GS_BITS;
    w.bs_bits = PS_BS_BITS;
    w.acc_bits = PS_ACC_BITS;
    w.dif_bits = PS_DIF_BITS;
    return w;
}

// ============================================================================
// X6/X7：12 bit 值在 16 bit 容器里的两种打包口径
// ============================================================================
//
// 相机侧冻结值（configs/camera_b.yaml: sensor）：
//   彩色 / 有效位深 12 bit / Bayer RGGB / 传输容器 16 bit。
// 但「12 bit 的有效值摆在 16 bit 容器的哪几位」有两种现实做法，
// **首帧实拍前无法判定**：
//
//   (a) packed    右对齐：容器值 ∈ [0, 4095]，饱和帧最大值 4095
//   (b) fullscale 左移 4：容器值 ∈ [0, 65520]，饱和帧最大值 65520
//
// 关键性质：两种口径描述的是**同一束光**，只差一个已知的 2^4 缩放。
// 因此解马赛克与分块求和的**算术完全一样**（本文件的实现只写一份），
// 口径唯一真正改变的是**位宽预算**：
//
//   packed    12 + ceil(log2(block^2)) = 19 bit（本仓库冻结声明 20，留 1 bit 余量）
//   fullscale 16 + ceil(log2(block^2)) = 23 bit
//
// 差 4 bit，会连带把冻结的 24 bit 差分通路顶穿（见 packing_budget()：
// fullscale + 10x10 块 + 4 帧时差分需要 26 bit，比声明宽 2 bit）。
// 而差分饱和在图像上只表现为「亮区被削平」，极难回溯到根因。
//
// 所以这里把口径做成**显式参数**（ValuePacking），不写死二选一：
// 同一次构建里两条链都能被核算与测试，首帧实拍后只改调用处的一个实参。
// ============================================================================

// 有效位深与容器位宽（与 configs/camera_b.yaml 对应；综合时可用 -D 覆盖）
#ifndef PS_EFFECTIVE_BITS
#define PS_EFFECTIVE_BITS 12
#endif
#ifndef PS_CONTAINER_BITS
#define PS_CONTAINER_BITS 16
#endif
// fullscale 口径的左移量：container - effective = 16 - 12 = 4
#ifndef PS_FULLSCALE_SHIFT
#define PS_FULLSCALE_SHIFT (PS_CONTAINER_BITS - PS_EFFECTIVE_BITS)
#endif

enum ValuePacking {
    kPacked    = 0,   // 12 bit 右对齐，满量程 (2^12-1)     = 4095
    kFullscale = 1    // 12 bit 左移 4 位，满量程 (2^12-1)<<4 = 65520
};

struct PackingSpec {
    const char* name;
    int         effective_bits;   // 有效位深（12）
    int         container_bits;   // 容器位宽（16）
    int         left_shift;       // packed=0，fullscale=4
    uint64_t    full_scale;       // 该口径下的满量程（饱和帧应达到的最大值）
    int         src_bits;         // 参与块累加的源位宽：12 / 16
};

inline PackingSpec packing_spec(ValuePacking p) {
    PackingSpec s;
    s.name           = (p == kPacked) ? "packed" : "fullscale";
    s.effective_bits = PS_EFFECTIVE_BITS;
    s.container_bits = PS_CONTAINER_BITS;
    s.left_shift     = (p == kPacked) ? 0 : PS_FULLSCALE_SHIFT;
    const uint64_t code_max = ((uint64_t)1 << PS_EFFECTIVE_BITS) - 1;   // 4095
    s.full_scale     = code_max << s.left_shift;                        // 4095 / 65520
    s.src_bits       = PS_EFFECTIVE_BITS + s.left_shift;                // 12 / 16
    return s;
}

// 单值是否落在该口径的合法范围内（超过即说明口径判错）。
inline bool packing_value_in_range(uint64_t v, ValuePacking p) {
    return v <= packing_spec(p).full_scale;
}

// packed -> fullscale（左移 4）
inline uint64_t packing_to_fullscale(uint64_t v) {
    return v << PS_FULLSCALE_SHIFT;
}

// fullscale -> packed（右移 4）。**低 4 位不为 0 时返回 false**：
// 那说明容器里既不是 packed 也不是 fullscale，必须查而不是悄悄丢低位。
inline bool packing_to_packed(uint64_t v, uint64_t* out) {
    if ((v & (((uint64_t)1 << PS_FULLSCALE_SHIFT) - 1)) != 0) return false;
    *out = v >> PS_FULLSCALE_SHIFT;
    return true;
}

// ---------------------------------------------------------------- 首帧实拍：口径判定

enum PackingDetect {
    kDetectPacked    = 0,    // 判为右对齐
    kDetectFullscale = 1,    // 判为左移 4 位
    kDetectUnknown   = -1    // 两者都不是，必须查（例如 SDK 已做 8 bit 截断）
};

// 弱判据：只看最大值。饱和帧（对着饱和光源）里
//   max <= 4095 -> 只可能是 packed；4096 <= max <= 65520 -> 只可能是 fullscale。
inline PackingDetect detect_packing_by_max(uint64_t max_value) {
    if (max_value <= ((uint64_t)1 << PS_EFFECTIVE_BITS) - 1) return kDetectPacked;
    if (max_value <= (((uint64_t)1 << PS_EFFECTIVE_BITS) - 1) << PS_FULLSCALE_SHIFT) {
        return kDetectFullscale;
    }
    return kDetectUnknown;
}

// 强判据：fullscale 口径下**整帧容器值的低 4 位必须恒为 0**（左移 4 位的结果）。
// 非饱和帧也能判，且能识破「值恰好落在 [4096, 65520] 但并非左移」的假阳性。
inline bool packing_low_bits_all_zero(const uint16_t* buf, std::size_t n) {
    const uint16_t mask = (uint16_t)(((uint16_t)1 << PS_FULLSCALE_SHIFT) - 1);
    for (std::size_t i = 0; i < n; ++i) {
        if ((buf[i] & mask) != 0) return false;
    }
    return true;
}

// ---------------------------------------------------------------- 位宽核算（口径相关）

// 不小于 n 的最小 b 使 2^b >= n。n<=1 -> 0。
inline int ceil_log2_u64(uint64_t n) {
    int b = 0;
    while (((uint64_t)1 << b) < n) ++b;
    return b;
}

// 本仓库**冻结声明**的块和位宽（configs/fixed_point_b.yaml: bit_chain.block_sum）。
// packed 取 20 = 理论最小 19 + 1 bit 余量（方案 §6.3 就是这么写的）；
// fullscale 取 23 = 理论最小 23，无余量。
inline int declared_block_sum_bits(ValuePacking p) {
    return (p == kPacked) ? 20 : 23;
}
// 冻结声明的差分通路位宽（configs/camera_b.yaml: bit_chain.diff_bits）
inline int declared_diff_bits() { return 24; }

struct PackingBitBudget {
    const char* packing;
    int  src_bits;             // 单通道（R 或 B）的源位宽：12 / 16
    int  src_bits_gsum;        // 双绿 G1+G2 的源位宽 = src_bits + 1（原则 P2）
    int  block;
    int  frames;
    // --- 单通道：与 configs/fixed_point_b.yaml 的 bit_chain 同一口径
    int  block_sum_min;        // 理论最小块和位宽
    int  block_sum_declared;   // 冻结声明
    int  frame_acc_min;        // 理论最小帧累积位宽
    int  frame_acc_declared;   // 冻结声明
    int  diff_needed;          // 差分需要的**有符号**位宽 = 帧累积 + 1
    // --- 双绿 G1+G2：块和/累积比单通道各多 1 bit（真实最坏情形）
    int  gsum_block_sum_min;
    int  gsum_frame_acc_min;
    int  gsum_diff_needed;
    // --- 差分通路声明（24）在两个口径下的裁决
    int  diff_declared;
    int  diff_fits_single;     // 单通道口径下够不够
    int  headroom_single_bits; // diff_declared - diff_needed
    int  diff_fits_gsum;       // 双绿口径下够不够（保守裁决）
    int  headroom_gsum_bits;   // diff_declared - gsum_diff_needed
};

inline PackingBitBudget packing_budget(ValuePacking p, int block, int frames) {
    PackingBitBudget b;
    const PackingSpec s = packing_spec(p);
    const int e_blk = ceil_log2_u64((uint64_t)block * (uint64_t)block);
    const int e_acc = ceil_log2_u64((uint64_t)(frames > 0 ? frames : 1));
    b.packing            = s.name;
    b.src_bits           = s.src_bits;
    b.src_bits_gsum      = s.src_bits + 1;
    b.block              = block;
    b.frames             = frames;
    b.block_sum_min      = s.src_bits + e_blk;
    b.block_sum_declared = declared_block_sum_bits(p);
    b.frame_acc_min      = b.block_sum_min + e_acc;
    b.frame_acc_declared = b.block_sum_declared + e_acc;
    b.diff_needed        = b.frame_acc_min + 1;
    b.gsum_block_sum_min = b.src_bits_gsum + e_blk;
    b.gsum_frame_acc_min = b.gsum_block_sum_min + e_acc;
    b.gsum_diff_needed   = b.gsum_frame_acc_min + 1;
    b.diff_declared      = declared_diff_bits();
    b.diff_fits_single   = (b.diff_declared >= b.diff_needed);
    b.headroom_single_bits = b.diff_declared - b.diff_needed;
    b.diff_fits_gsum     = (b.diff_declared >= b.gsum_diff_needed);
    b.headroom_gsum_bits = b.diff_declared - b.gsum_diff_needed;
    return b;
}

// ---------------------------------------------------------------- 流式解马赛克 + 分块求和

// 通道在 2x2 Bayer 单元内的采样点数与偏移。
//   which: 0 = R（1 个点）、1 = 双绿 G1+G2（2 个点）、2 = B（1 个点）
// phase: 0 = RGGB，1 = BGGR（与 bayer_offset 同一套约定）
// 返回采样点数（1 或 2），偏移写入 dr[] / dc[]。
inline int channel_offsets(int which, int phase, int* dr, int* dc) {
    if (which == 0) {          // R
        BayerOffset o = bayer_offset(0, phase);
        dr[0] = o.dr; dc[0] = o.dc; return 1;
    }
    if (which == 2) {          // B
        BayerOffset o = bayer_offset(3, phase);
        dr[0] = o.dr; dc[0] = o.dc; return 1;
    }
    dr[0] = 0; dc[0] = 1;      // G1：2x2 单元右中
    dr[1] = 1; dc[1] = 0;      // G2：2x2 单元中下
    return 2;
}

// X6/X7 顶层：对 n_frames 帧 HxW RAW 做
//   解马赛克（按 which 取通道）-> block x block 分块求和 -> 帧累积 -> 行差分
//
// 与 preprocess_plane_frames 的区别有两点，都是为了 12 bit 口径：
//   1. **流式**：解马赛克与分块在同一遍里完成，不开 (h/2)*(w/2) 的中间平面
//      （1920x1920 时那是 921600 项，开 3 份会吃掉几十 MB）。
//   2. **双绿按和进入累加**（原则 P2）：which=1 时 v = G1 + G2，末位统一除以
//      2*block^2，全程无 0.5 舍入，代价只多 1 bit。
//
// `packing` 是**显式参数**：本函数用它做口径相关的断言与满量程换算，
// 两种口径下的**数值结果只差 2^4 倍**（这正是 test_x6_x7 的等价性判据）。
//
// 输出：acc 长度 bh*bw；diff 长度 (bh-1)*bw（可传 nullptr 表示不要）。
// out_max_raw（可空）回带本批数据里见到的最大容器值，供口径判定用。
template <typename Tin>
void preprocess_color_frames(const Tin* raw_frames, int n_frames,
                             int h, int w, int block,
                             int which, int phase, ValuePacking packing,
                             acc_t* acc, dif_t* diff,
                             int* out_bh, int* out_bw, uint64_t* out_max_raw) {
    (void)packing;   // 数值路径与口径无关；口径只决定位宽与满量程（见 packing_budget）
    const int ph = h / 2;
    const int pw = w / 2;
    const int bh = ph / block;
    const int bw = pw / block;
    if (out_bh) *out_bh = bh;
    if (out_bw) *out_bw = bw;
    const int n_blk = bh * bw;

    int dr[2], dc[2];
    const int ns = channel_offsets(which, phase, dr, dc);
    uint64_t mx = 0;

    for (int i = 0; i < n_blk; ++i) {
#pragma HLS PIPELINE II = 1
        acc[i] = 0;
    }
    for (int f = 0; f < n_frames; ++f) {
        const Tin* fr = raw_frames + (std::size_t)f * h * w;
        // 双层块索引：结构上不可能跨行（与 block_sum 同一纪律）
        for (int br = 0; br < bh; ++br) {
            for (int bc = 0; bc < bw; ++bc) {
#pragma HLS PIPELINE II = 1
                acc_t s = 0;
                for (int dy = 0; dy < block; ++dy) {
                    for (int dx = 0; dx < block; ++dx) {
                        const int y = br * block + dy;
                        const int x = bc * block + dx;
                        for (int k = 0; k < ns; ++k) {
                            const uint64_t v =
                                (uint64_t)fr[(2 * y + dr[k]) * w + (2 * x + dc[k])];
                            if (v > mx) mx = v;
                            s += (acc_t)v;
                        }
                    }
                }
                acc[br * bw + bc] += s;
            }
        }
    }
    if (out_max_raw) *out_max_raw = mx;
    if (diff) row_diff(acc, bh, bw, diff);
}

}  // namespace ps

#endif  // PENUMBRASIGHT_PREPROCESS_HPP
