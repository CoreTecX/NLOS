// ============================================================================
// PenumbraSight HLS 公共定点层 · ps_fixed.hpp
//
// 同一份源码在两个环境下都能工作
//   g++ 8.1   : 主机侧功能验证（不需要 Vivado）
//   Vitis HLS : 综合（工具自己定义 __SYNTHESIS__）
//
// 依据
// ---------------------------------------------------------------------------
//   方案 §6.3 定点格式 / §6.4 定位引擎 / §6.5 低秩重建与 TV / §9.2 定点误差判据；
//   configs/fixed_point_b.yaml 的 bit_chain 是这些常量的冻结来源。
//
//   本文件是 src/model/penumbrasight_model/quantize.py 在 C++（RTL/HLS）一侧的
//   同一份规则的实现，两边必须给出同一个码字：
//       round_half_even      <-> quantize.round_half_even   (np.rint，半值取偶)
//       saturate_code        <-> quantize.saturate          (饱和，禁止回绕)
//       FpChain              <-> quantize.FixedPointChain   (位宽链核算)
//       SharedExpScale       <-> quantize.shared_exponent   (scale = 2^(exp-16))
//   quantize.py 是「PC 浮点基线 -> 硬件定点」之间唯一的换算入口；这里是它的
//   硬件侧对偶。任一侧改了舍入或位宽，另一侧的判据就失效 —— 所以这里的每一行
//   都写得刻意冗余、逐个量点显式，不做「代数等价改写」。
//
// 一条硬规则（方案 §6.3 末段）
// ---------------------------------------------------------------------------
//   每个量化点使用「向最近值舍入、饱和」，**禁止截断回绕**。因此本文件不提供
//   `(int)x` 形式的强转量化：所有量化点必须走 sat_round_i64() 或
//   mul_shift_round() / narrow_format()。
//
// 关于 `#pragma HLS ...` 的包裹方式
// ---------------------------------------------------------------------------
//   本目录（src/fpga/hls）下所有综合专用 pragma 全部写在 `#ifdef __SYNTHESIS__`
//   里面。原因：g++ 看到未知 pragma 会在 -Wall 下报「ignoring #pragma ...」，
//   把 HLS 指令包起来后主机编译是干净的，而 Vitis HLS 综合时 pragma 照常生效。
//   注意 src/hardware/*.hpp 是既有代码、用的是裸 pragma；主机编译它们仍会产生
//   该类告警，构建脚本用 -Wno-unknown-pragmas 处理（见
//   src/fpga/scripts/build_hls_host.ps1 的说明），这不是本目录代码的问题。
// ============================================================================

#ifndef PENUMBRASIGHT_HLS_PS_FIXED_HPP
#define PENUMBRASIGHT_HLS_PS_FIXED_HPP

#include <cmath>
#include <cstdint>

namespace ps {

// ============================================================ Q 格式（§6.3）
//
// 格式名 / 总位宽 / 小数位，逐项对应 configs/fixed_point_b.yaml 的 bit_chain。
// 备注里写的是「为什么是这个位宽」，改动前先读方案 §6.3 那张表。

constexpr int kMeasBits   = 18;   // 归一化测量（竖直差分后）Q2.16
constexpr int kMeasFrac   = 16;
constexpr int kMeasInt    = kMeasBits - kMeasFrac;          // = 2（含符号位）

constexpr int kCoeffBits  = 16;   // U/V 系数 Q1.15，**每候选独立 scale**
constexpr int kCoeffFrac  = 15;   // 码字 16 bit + 独立 scale 即模型里的 int16

constexpr int kSingBits   = 16;   // 归一化奇异值 Q2.14（格式钉死，scale = 2^-14）
constexpr int kSingFrac   = 14;

constexpr int kImageBits  = 24;   // 图像状态 f 与对偶 q：Q6.18
constexpr int kImageFrac  = 18;
constexpr int kImageInt   = kImageBits - kImageFrac;        // = 6

constexpr int kMacAccBits = 48;   // MAC 累加器：signed 48 bit（§6.3）
constexpr int kScoreBits  = 64;   // 分数 / 残差：unsigned 48 或 64 bit，取 64

constexpr int kGainFrac   = 16;   // scale / 步长 / 正则权重的 Q16.16 定点增益

// 码字上界（对称格式取 2^(bits-1)-1，让 -max 与 +max 都有码字）
constexpr long long kCoeffCodeMax = (1LL << (kCoeffBits - 1)) - 1;   // 32767
constexpr long long kMeasCodeMax  = (1LL << (kMeasBits  - 1)) - 1;   // 131071
constexpr long long kImageCodeMax = (1LL << (kImageBits - 1)) - 1;   // 8388607
constexpr long long kMacAccCodeMax = (1LL << (kMacAccBits - 1)) - 1; // 2^47-1

// 增益安全区：|gain_q16| <= 2^16 = 65536。
// 取 2^16 而不是 2^16-1，是为了让「θ = 1.0」「gain = 1」这类**恰好为 2 的幂**
// 的系数能精确表示（否则 θ=1 会被钳到 65535，白丢 1.5e-5 的精度）。
// 上界成立性：(2^47-1)·2^16 = 2^63 - 2^16 < 2^63，int64 乘法安全。
constexpr long long kGainSafeMax   = (1LL << kGainFrac);             // 65536

// ============================================================ 饱和/溢出观测
//
// 三个核共用一份观测口径。方案 §6.6 要求「溢出必须体现为不可信」，所以这里
// 不只是计数：任何一个非零计数都要能被上层映射成 low_conf 或错误码。
struct FixFlags {
    int sat_events;      // 任一级发生饱和的总次数
    int mac_overflow;    // 48 bit MAC 累加器顶到边界
    int out_overflow;    // 输出格式（Q2.16 / Q6.18 等）收窄时饱和
    int score_overflow;  // 分数/残差累加器饱和或平方项溢出
    int gain_clamped;    // Q16.16 增益越出安全区被钳位（模型 scale 异常）
    long long max_abs_acc;   // 观测到的最大 |MAC 原始累加|，用于余量核算

    bool any() const {
        return (sat_events | mac_overflow | out_overflow
                | score_overflow | gain_clamped) != 0;
    }
};

inline void fix_flags_reset(FixFlags& f) {
    f.sat_events = 0;
    f.mac_overflow = 0;
    f.out_overflow = 0;
    f.score_overflow = 0;
    f.gain_clamped = 0;
    f.max_abs_acc = 0;
}

inline void fix_track_abs(FixFlags& f, long long v) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    const long long a = (v < 0) ? -v : v;
    if (a > f.max_abs_acc) f.max_abs_acc = a;
}

// 合并两套计数。分层核（如 pdhg_core 调用 lr_operator）各自维护 flags，
// 上层必须把子核的计数并进来，否则"溢出必须体现为不可信"这条会漏报。
inline void fix_flags_merge(FixFlags& dst, const FixFlags& src) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    dst.sat_events += src.sat_events;
    dst.mac_overflow += src.mac_overflow;
    dst.out_overflow += src.out_overflow;
    dst.score_overflow += src.score_overflow;
    dst.gain_clamped += src.gain_clamped;
    if (src.max_abs_acc > dst.max_abs_acc) dst.max_abs_acc = src.max_abs_acc;
}

// ============================================================ 舍入与饱和

// 向最近值舍入、半值取偶（banker's rounding），与 numpy.rint /np.rint 一致，
// 即 quantize.round_half_even。调用方保证 |x| < 2^62，否则 floor 会越出 long long。
//
// 为什么不用 (long long)(x + 0.5)：那会把 -2.5 舍成 -2、把 2.5 舍成 3，在
// 16 次迭代的重建里表现为符号相关的直流偏置（quantize.py 已就此写过警告）。
inline long long round_half_even(double x) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    const double fl = std::floor(x);
    const double frac = x - fl;                 // ∈ [0,1)
    long long i = (long long)fl;
    if (frac > 0.5) {
        ++i;
    } else if (frac == 0.5 && (i & 1LL)) {      // 恰在半格上：取偶
        ++i;
    }
    return i;
}

// 饱和（禁止回绕，§6.3）
inline long long saturate_i64(long long v, long long lo, long long hi) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 量化点：先按上界短路（避免 floor 越界），再偶数舍入。
inline long long sat_round_i64(double x, long long lo, long long hi) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    if (x >= (double)hi) return hi;
    if (x <= (double)lo) return lo;
    return round_half_even(x);
}

// ============================================================ 乘加与增益

// (a * b) >> shift，四舍五入到最近整数、半值取偶；shift < 0 表示左移。
//
// 这是「定点系数 × Q16.16 增益」的唯一入口。硬件上它就是一级乘法 + 移位，
// 所有量化点都走这里，保证与 Python 侧的 np.rint 逐码字对齐。
// 溢出前提：|a| < 2^47（48 bit MAC 政策）且 |b| <= 65535（增益安全区），
// 则 |a*b| < 2^63，int64 装得下 —— 两个前提都由调用方用 FixFlags 守住。
inline long long mul_shift_round(long long a, long long b, int shift) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    const long long p = a * b;
    if (shift == 0) return p;
    if (shift < 0) return p << (-shift);
    long long q = p >> shift;                   // 算术右移 = 向下取整
    const long long rem = p - (q << shift);     // 余数 ∈ [0, 2^shift)，负数也非负
    const long long half = 1LL << (shift - 1);
    if (rem > half) {
        ++q;
    } else if (rem == half && (q & 1LL)) {      // 恰在半格：取偶
        ++q;
    }
    return q;
}

// 增益安全化：Q16.16 增益限制在 ±65535。
// 越界意味着模型 scale 超出「Q1.15 系数 × Q2.16 测量」的必然范围（u_scale·d_scale
// 应当远小于 1），属于模型错误而不是运行时错误，故这里钳位并计数，由上层
// model_loader 的同一条判据拦截（方案 §8.2 ERROR_CODE）。
inline long long gain_safe(long long g, FixFlags& f) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    if (g > kGainSafeMax) { f.gain_clamped++; f.sat_events++; return kGainSafeMax; }
    if (g < -kGainSafeMax) { f.gain_clamped++; f.sat_events++; return -kGainSafeMax; }
    return g;
}

// 48 bit MAC 政策：累加结果超出声明位宽即饱和并记账。
// 只用一处判断，避免三个核各写一份宽窄不一的裁剪。
inline long long sat_mac48(long long acc, FixFlags& f) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    fix_track_abs(f, acc);
    if (acc > kMacAccCodeMax || acc < -kMacAccCodeMax) {
        f.mac_overflow++;
        f.sat_events++;
        return (acc > 0) ? kMacAccCodeMax : -kMacAccCodeMax;
    }
    return acc;
}

// 收窄到某个声明格式（Q2.16 / Q6.18 / …）。方案 §6.3：「MAC 累加 signed 48 bit，
// 最终舍入到 24 或 32 bit」——收窄点就在这几个调用处，全都是饱和。
inline int32_t narrow_format(long long v, long long code_max, FixFlags& f) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    if (v > code_max || v < -code_max) {
        f.out_overflow++;
        f.sat_events++;
        return (int32_t)((v > 0) ? code_max : -code_max);
    }
    return (int32_t)v;
}

// 分数/残差用 uint64 累加；顶上界即饱和并记账，绝不回绕（§6.3：防止候选排序溢出）
inline uint64_t sat_add_u64(uint64_t a, uint64_t b, FixFlags& f) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    const uint64_t s = a + b;
    if (s < a) {
        f.score_overflow++;
        f.sat_events++;
        return ~0ull;
    }
    return s;
}

// ============================================================ 整数平方根
//
// §6.5 要求对偶投影的范数与残差门控不依赖浮点；主机/硬件统一用精确整数平方根，
// 逐位算法（最多 32 轮），没有除法也没有循环次数不确定性。
inline uint64_t isqrt_u64(uint64_t v) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    uint64_t res = 0;
    uint64_t bit = 1ull << 62;
    while (bit > v) {
#ifdef __SYNTHESIS__
        // 给报告定界（bit 从 2^62 起每次右移 2 位，最多 32 轮）。
        // 只此一处给界：本项目里 isqrt_u64 仅被 pdhg_core 调用（对偶投影的范数与
        // ρ），projection_engine / lr_operator 两条链不用它，故不影响那两个核。
        // 该 pragma 只进报告，不参与 RTL 生成：v 的取值决定实际轮数（B 档
        // Q6.18 码字下 a²+b² ≤ 2^47，实际约 8 + 24 轮），min/max 是界不是实现。
#pragma HLS LOOP_TRIPCOUNT min=1 max=32
#endif
        bit >>= 2;
    }
    while (bit != 0) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=1 max=32
#endif
        if (v >= res + bit) {
            v -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

// ============================================================ 位宽链核算
//
// quantize.FixedPointChain 的 C++ 对偶。用途不是「算出一个位宽」，而是让三个核
// 的 testbench 能在**编译期常量**上核对「声明位宽 >= 需要位宽」，与 Python 侧
// 的 check() 给出同一组数字。派生量一律由字段算出，不允许手填。

inline int ceil_log2_u32(unsigned n) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    if (n <= 1) return 0;
    int k = 0;
    unsigned p = 1;
    while (p < n) { p <<= 1; ++k; }
    return k;
}

struct FpChain {
    int  raw_bits;        // 相机容器位宽（16）
    int  effective_bits;  // 相机有效位深（12）
    int  block;           // 分块边长（10）
    int  frames;          // 帧累积数（4）
    bool fullscale;       // true = 12 bit 左移 4 位（满量程口径）；false = 右对齐
    int  diff_bits;       // 声明：垂直差分 signed 24
    int  coeff_bits;      // 声明：U/V 系数 16
    int  mac_acc_bits;    // 声明：MAC 累加 48

    // —— 派生量（与 quantize.py 同式）——
    int src_bits() const { return fullscale ? raw_bits : effective_bits; }
    int block_sum_bits() const {
        return src_bits() + ceil_log2_u32((unsigned)(block * block));
    }
    int frame_acc_bits() const {
        return block_sum_bits() + ceil_log2_u32((unsigned)(frames < 1 ? 1 : frames));
    }
    int diff_bits_needed() const { return frame_acc_bits() + 1; }   // +1 符号
    int mac_acc_bits_needed() const {
        return 2 * coeff_bits + ceil_log2_u32((unsigned)(block * block));
    }

    bool ok_block_sum() const {
        const int declared = fullscale ? 23 : 20;                   // fixed_point_b.yaml
        return declared >= block_sum_bits();
    }
    bool ok_diff() const { return diff_bits >= diff_bits_needed(); }
    bool ok_mac_acc() const { return mac_acc_bits >= mac_acc_bits_needed(); }
    bool ok() const { return ok_block_sum() && ok_diff() && ok_mac_acc(); }
};

// B 档自采默认链：12 bit 源、10×10 分块、4 帧、右对齐（fixed_point_b.yaml 默认）
inline FpChain fp_chain_b() {
    FpChain c;
    c.raw_bits = 16;
    c.effective_bits = 12;
    c.block = 10;
    c.frames = 4;
    c.fullscale = false;
    c.diff_bits = 24;
    c.coeff_bits = kCoeffBits;
    c.mac_acc_bits = kMacAccBits;
    return c;
}

// ============================================================ 共享指数
//
// quantize.shared_exponent 的硬件侧记法：三通道（R/G/B）共用同一个指数，
// 于是通道间相对幅度被原样保留 —— 方案 §6.4 的联合分数 s_p 是三通道求和，
// 若各通道各自归一化，三个通道的相对权重就被悄悄改掉了。
//
//   scale = 2^(exp - frac_bits)，exp = ceil(log2(max|x|)) + 1（留 1 bit 余量）
struct SharedExpScale {
    int    exponent;      // 共享指数（每帧/每候选算一次，随模型头或帧头下发）
    double scale;         // = 2^(exponent - kMeasFrac)

    void init(double max_abs_x) {
        if (max_abs_x <= 0.0) {
            exponent = 0;
            scale = std::ldexp(1.0, -kMeasFrac);
            return;
        }
        exponent = (int)std::ceil(std::log2(max_abs_x)) + 1;
        scale = std::ldexp(1.0, exponent - kMeasFrac);
    }

    // 浮点 -> Q2.16 码字（偶数舍入 + 饱和，不允许截断）
    int32_t encode(double x) const {
        const double q = x / scale;
        return (int32_t)sat_round_i64(q, -kMeasCodeMax, kMeasCodeMax);
    }
};

// 浮点 scale 转 Q16.16 增益（模型头里的 u_scale / v_scale / s_scale 都走这里）。
//
// **已知约束（实测得出，不是推测）**：Q16.16 的增益分辨率是 2^-16 = 1.53e-5，
// 取整误差是**相对**误差，上界 = 0.5 / |gain_q16|：
//     gain_q16 >= 500（u_scale >= 7.6e-3）  -> 相对误差 <= 1e-3
//     gain_q16 >= 5000（u_scale >= 7.6e-2） -> 相对误差 <= 1e-4
// 而方案 §6.3 的 U/V 是「Q1.15 码字 + 每候选**自适应** scale」：若按
// max|U|/32767 取 scale，物理量级常在 1e-5，则 gain_q16 会落到 0 或个位数，
// 取整误差可达数十个百分点（lr_operator 的 testbench L7 用 4.6 -> 5 量出
// 8.7%，并给出「gain 应为 0.42 -> 取整到 0」的自适应例子）。
//
// 因此模型构建侧只有两条合规路线，必须二选一并在 model_loader 里校验：
//   a) scale 取 2 的幂（gain 精确），或
//   b) 对 U/V 用**跨候选共享指数**（与 §6.3 的测量通路同一做法），
// 否则该误差会整体缩放候选分数、破坏 §6.4 的 margin 门控，而它既不是 LSB
// 有界的、也不能靠加宽 MAC 或提高 rank 修好。这条判据与「每级 <= 1 LSB」
// 必须分开记在报告里（前者是相对误差，后者是绝对误差）。
inline int32_t scale_to_gain_q16(double scale) {
    return (int32_t)sat_round_i64(scale * (double)(1 << kGainFrac),
                                  -(1LL << 31) + 1, (1LL << 31) - 1);
}

inline double gain_q16_to_double(int32_t g) {
    return (double)g / (double)(1 << kGainFrac);
}

// ---------------------------------------------------------------- 增益：指数表示
// Q16.16 的增益解算有两个**结构性**问题，都不是靠加宽 MAC 能解决的：
//
//   1) Q16.16 的分辨率是 2^-16 = 1.53e-5，而真实 B 档模型的 U/V scale 落在
//      1e-6~1e-5 —— `scale_to_gain_q16` 会返回 0。实测 8 个量里 6 个取整到 0，
//      最好的那档也只有 1（相对误差上界 50%）。见 test_lr_operator.cpp 的 L7 段。
//   2) 增益取整带来的是**相对**误差（0.5 LSB / gain），不是 LSB 有界的，
//      所以「加宽累加器」一点用都没有 —— 要改的是增益的**表示**。
//
// 指数表示：scale = 2^exp。要求**生产端（quantize.py 的 quantize_symmetric_pow2）
// 保证 scale 取 2 的幂**，于是增益精确可表示，相对误差恒为 0；代价只是码字
// 动态范围最多少用 1 bit。
//
// 与旧 API 的关系：**两者并存**。旧的 `scale_to_gain_q16` / `gain_q16_to_double`
// 保留给已有的 test_*.cpp —— 它们仍在验 Q16.16 的非幂次误差特征，那是有价值的
// 反例，删掉就失去了这条判据的证据。新的数据通路应使用本节的指数 API。
//
// 指数位宽取 int8（[-128, 127]）：真实 B 档的指数是 -18~-16，余量 111 个 2 的幂。
// 装不下时返回 kGainExpInvalid 而**不是饱和** —— 饱和会悄悄改掉模型的幅度，
// 正是旧表示出问题的那种失败方式。
constexpr int kGainExpMin     = -128;
constexpr int kGainExpMax     =  127;
constexpr int kGainExpInvalid =  0x7FFFFFFF;

// 把 2 的幂 scale 转成指数。**非 2 的幂返回 kGainExpInvalid**（调用方必须处理），
// 这样生产端的 bug 会在入口暴露，而不是变成一个被悄悄改过的增益。
inline int scale_to_exp2(double scale) {
    if (!(scale > 0.0)) return kGainExpInvalid;
    int e = 0;
    const double m = std::frexp(scale, &e);      // scale = m * 2^e, m ∈ [0.5, 1)
    if (std::fabs(m - 0.5) > 1e-12) return kGainExpInvalid;   // 不是 2 的幂
    const int exp = e - 1;
    if (exp < kGainExpMin || exp > kGainExpMax) return kGainExpInvalid;
    return exp;
}

inline bool exp2_valid(int e) { return e != kGainExpInvalid; }

inline double exp2_to_scale(int e) { return std::ldexp(1.0, e); }

// 应用增益：code * 2^exp -> int64。用 ldexp 而不是移位，是为了让 exp 为负时
// 也走同一条路径（右移负数在 C++ 里是实现定义的）。
inline int64_t apply_exp2(int64_t code, int e) {
    return (int64_t)std::llround((double)code * std::ldexp(1.0, e));
}

// ------------------------------------------------ 指数 -> (gain, shift) 桥接
// 数据通路里的增益是「乘一个 gain 再右移 shift」的形式：
//     out = mul_shift_round(acc, gain, shift)
// 旧路径把 scale 编码成 Q16.16 的 gain、shift 用编译期常量；新路径改成
// **gain 恒为 2^kGainFrac，把指数折进 shift**：
//     scale = 2^exp  =>  gain = 2^kGainFrac,  shift = shift_const - exp
// 两者数学等价，但新路径的增益**精确**（不再有 Q16.16 的舍入）。
//
// 安全性两点：
//   1) `mul_shift_round` 已处理负 shift（见其定义处的 `if (shift < 0)`），
//      所以 exp 为正（scale > 1）时也正确，不会 UB。
//   2) gain 由 scale 值变成常量 2^16，乘积最大增长 2^16 倍。核内 acc 的
//      量级是 int16 码 × int32 码 × K(<=64) ≈ 6.9e10，乘 2^16 得 4.5e15，
//      远小于 int64 上界 9.2e18，**不会溢出**。
//
// exp 无效（kGainExpInvalid）时**退回旧的 Q16.16 路径**，所以既有测试
// （它们不设 exp 字段）行为完全不变 —— 这是本改动可安全落地的关键。
//
// ⚠ 但「exp 无效」不能是**生产路径**的常态：非 2 的幂（或过小）的 scale 折成
//   Q16.16 时 gain_q16 = round(scale·2^16) 常为 0，核输出整段归零且不报错。
//   现行纪律（A1）是**在入口拒绝**这样的模型，而不是靠这里的回退兜底：
//     模型 -> 增益 的唯一入口是 src/software/fpga_control/fix_core_driver.c 的
//     ps_fix_gain_from_candidate()，非 2 的幂的 u_scale/v_scale 返回
//     PS_FIX_DRV_ERR_GAIN_EXP（见 fix_core_driver.h 的「A1 入口拒绝」）。
//   本函数的回退只服务两类**合法**情形：
//     1) S 级乘数 = s_scale*s_max 一般不是 2 的幂（Q16.16 是合法支路）；
//     2) 主机侧测试显式传 kGainExpInvalid（test_fixed_top.cpp / test_lr_exp_gain.cpp）。
//   不要因为看到这里会回退，就把入口的校验删掉。
inline void exp_to_gain_shift(int exp, int32_t gain_q16, int shift_const,
                              long long& gain_out, int& shift_out) {
    if (exp == kGainExpInvalid) {
        gain_out  = (long long)gain_q16;
        shift_out = shift_const;
    } else {
        gain_out  = (long long)(1LL << kGainFrac);
        shift_out = shift_const - exp;
    }
}

}  // namespace ps

#endif  // PENUMBRASIGHT_HLS_PS_FIXED_HPP
