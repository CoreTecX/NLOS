// ============================================================================
// projection_engine 主机侧 testbench（方案 §6.2 / §6.4 / §6.3）
//
// 编译（在 src/fpga/hls/projection_engine 目录下）：
//   g++ -std=c++17 -Wall -Wextra -O2 projection_engine_fixed.cpp test_projection.cpp -o test_projection.exe
//   （实现文件名带 _fixed 后缀：同目录另有一层双精度参考实现占用了 projection_engine.cpp，
//     见 src/fpga/hls/readme.md「两层」一节）
// 或者直接用仓库脚本（推荐，会带 -Wno-unknown-pragmas）：
//   pwsh src/fpga/scripts/build_hls_host.ps1 -Only projection
//
// 这个 testbench **不依赖任何外部数据**：全部向量由固定种子的 splitmix64 现场生成，
// 因此任何机器上重跑都得到同一组数字（整数运算 + double 精确表示小整数之和）。
// 真实模型向量（model.psm / tests/vectors）的回归是 Task 6 Step 3 的活，
// 本文件先把「核自身是否正确」钉死。
//
// 检查项
//   U1 编译期定点格式与位宽链自洽（与 quantize.py 的 FixedPointChain 同式）
//   U2 候选网格映射是双射（与 x10_localize.cpp 的展开顺序同一约定）
//   U3 已知候选的合成测量 -> top-1 必须命中（方案 Task 4 Step 1 的定位判据）
//   U4 与双精度参考的分数**逐候选相等**（核内全整数 -> 参考可精确复算）
//   U5 与"未取整的浮点 scales"参考的相对误差（端到端定点误差量级）
//   U6 LANES=8/16/32 逐位相同（整数累加可结合，构造性成立，这里把它测出来）
//   U7 margin / 网格边界 / low_conf 门控
//   U8 极限码字下的饱和行为：不回绕；48 bit 在本档实测不溢出，但**不是不可溢出**
//      —— 理论最坏（K=8、M=9120）为 2^48.15，见下方口径
//   U9 带宽与算力核算：MAC 数、DDR 读数、分数通路余量
//   V-1 ε* 翻转裕度扫描（判据 6 的正确口径）：把「972/972 位号一致」这个结果量
//      升级成「误差再放大几倍 top-1 才翻转」的裕度倍数，判据 ε* > 2。
//      定义、分子/分母口径与不可扫描情形的处理见 V-1 段落内的注释。
//      本节只在主机侧运行（不参与综合），不改 HLS 核的算法。
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "projection_engine.hpp"

// ---------------------------------------------------------------- 自检框架
static int g_pass = 0;
static int g_fail = 0;

static void check(bool ok, const char* name, const char* detail) {
    std::printf("  [%s] %-46s %s\n", ok ? "ok  " : "FAIL", name, detail);
    if (ok) ++g_pass; else ++g_fail;
}

static void checkf(bool ok, const char* name, const char* fmt, double a, double b) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), fmt, a, b);
    check(ok, name, buf);
}

// ---------------------------------------------------------------- 确定性随机
// splitmix64：纯整数运算，跨平台逐位一致；取高 53 位给出 [0,1) 的 double。
static uint64_t g_rng = 0x243F6A8885A308D3ull;

static uint64_t rnd_u64() {
    g_rng += 0x9E3779B97F4A7C15ull;
    uint64_t z = g_rng;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static double rnd_uni() {                       // [0,1)
    return (double)(rnd_u64() >> 11) * (1.0 / 9007199254740992.0);
}

static double rnd_sym() { return 2.0 * rnd_uni() - 1.0; }   // [-1,1)

static int bit_width_u64(uint64_t v) {
    int b = 0;
    while (v != 0) { ++b; v >>= 1; }
    return b;
}

// ---------------------------------------------------------------- 打印小工具
static const char* fmt_bytes(double b, char* buf, size_t n) {
    if (b >= 1024.0 * 1024.0 * 1024.0) {
        std::snprintf(buf, n, "%.2f GiB", b / (1024.0 * 1024.0 * 1024.0));
    } else if (b >= 1024.0 * 1024.0) {
        std::snprintf(buf, n, "%.2f MiB", b / (1024.0 * 1024.0));
    } else {
        std::snprintf(buf, n, "%.1f KiB", b / 1024.0);
    }
    return buf;
}

// ------------------------------------------------------- V-1 ε* 翻转裕度扫描
//
// 定义（分子 m_k、分母 e_k 各自指什么，逐项写死，避免与「结果量」混淆）：
//
//   输入两条能量序列
//     R[p]  权威参考能量（本 testbench 用「未取整实值增益」的理想能量，双精度）
//     F[p]  定点核能量（projection_engine<16> 的输出）
//   有符号误差
//     δ[p] = F[p] - R[p]
//   top-1（按参考）
//     best = argmax_p R[p]
//   **分母** e_k（候选 k 的相对误差，无量纲）
//     e_k = |F[k] - R[k]| / R[k] = |δ[k]| / R[k]
//   **分子** m_k（候选 k 相对 top-1 的相对间隔，无量纲）
//     m_k = (R[best] - R[k]) / R[best]
//   沿判别方向放大误差：把误差向量整体放大 a 倍，得分数 R[p] + a·δ[p]。
//   候选 k 超过 best 的临界放大倍数（一阶，精确解）：
//     a_k = (R[best] - R[k]) / (δ[k] - δ[best])      仅当 δ[k] - δ[best] > 0
//   翻转裕度
//     ε* = min_k a_k
//   ε* 的含义：定点误差再放大 ε* 倍，top-1 才会首次翻转。故 ε* 是裕度倍数，
//   而「972/972 位号一致 / Spearman 1.000000」是结果量——两者不是一回事。
//   文档里给的比值口径 ε*_ratio = m_k / e_k 是 a_k 的保守下界（因 |δ_k-δ_b| <= e_k·R_k + e_b·R_b），
//   这里一并打印，便于与 sim/results/x10_localize_hls.md:15-23 的定义对照。
//
//   判据（交付定义 §5.2 的 V-1）：ε* > 2 通过；ε* ∈ [1,2] 裕度有限须标注；
//   ε* < 1 说明当前 top-1 一致是巧合。ε* = +inf 表示「误差恒零、无有限翻转点」。
//
//   不可扫描（如数列出，不跳过也不伪造）：
//     · δ ≡ 0（核与参考逐候选完全一致）-> 任何放大都不翻转，ε* = +inf；
//     · 不存在 δ[k] - δ[best] > 0 的对手 -> 误差方向一致地压低所有对手，无有限翻转点；
//     · 候选数 < 2 或 R[best] <= 0。
struct EpsScan {
    int    best;        // 参考 top-1 下标
    int    flip_k;      // 最先翻转时超过 best 的对手下标（-1 = 无）
    double eps_star;    // min_k a_k（+inf = 无有限翻转点）
    double eps_ratio;   // 文档比值口径 m_k / max(e_k, e_best)
    double margin;      // 到翻转对手的 m_k
    double e_best;      // top-1 的相对误差
    int    scannable;   // 1 = 存在有限翻转点
    double a_scan;      // 二分逐步放大得到的首次翻转倍数（与 eps_star 互校）
};

static EpsScan eps_flip_scan(const std::vector<double>& R,
                             const std::vector<double>& F) {
    EpsScan s;
    s.best = -1; s.flip_k = -1;
    s.eps_star = HUGE_VAL; s.eps_ratio = HUGE_VAL;
    s.margin = 0.0; s.e_best = 0.0; s.scannable = 0; s.a_scan = HUGE_VAL;

    const int n = (int)R.size();
    if (n < 2) return s;
    int best = 0;
    for (int i = 1; i < n; ++i) if (R[i] > R[best]) best = i;
    s.best = best;
    const double Rb = R[best];
    if (!(Rb > 0.0)) return s;

    std::vector<double> delta((size_t)n);
    for (int i = 0; i < n; ++i) delta[i] = F[i] - R[i];
    s.e_best = std::fabs(delta[best]) / Rb;

    // 精确翻转倍数：对每个对手取 a_k，再取最小。
    for (int k = 0; k < n; ++k) {
        if (k == best || R[k] >= Rb) continue;
        const double dd = delta[k] - delta[best];
        if (dd > 0.0) {
            const double a = (Rb - R[k]) / dd;
            if (a < s.eps_star) { s.eps_star = a; s.flip_k = k; }
        }
    }
    if (!std::isfinite(s.eps_star)) return s;   // 退化：无有限翻转点

    s.scannable = 1;
    s.margin = (Rb - R[s.flip_k]) / Rb;
    const double ek = std::fabs(delta[s.flip_k]) / R[s.flip_k];
    s.eps_ratio = (ek > s.e_best) ? (s.margin / ek) : (s.margin / s.e_best);

    // 逐步放大互校：二分定位 argmax(R + a·δ) 首次偏离 best 的 a。
    // 从 a=1（实际定点结果，仍应命中 best）放大到 a=4·ε*（必然翻转）。
    auto argmax_a = [&](double a) {
        int bi = 0; double v = -HUGE_VAL;
        for (int i = 0; i < n; ++i) {
            const double x = R[i] + a * delta[i];
            if (x > v) { v = x; bi = i; }
        }
        return bi;
    };
    double lo = 1.0;
    double hi = (s.eps_star * 4.0 > 2.0) ? (s.eps_star * 4.0) : 2.0;
    if (argmax_a(hi) != best) {
        for (int it = 0; it < 200 && (hi - lo) > 1e-15 * hi; ++it) {
            const double mid = 0.5 * (lo + hi);
            if (argmax_a(mid) == best) lo = mid; else hi = mid;
        }
        s.a_scan = hi;
    }
    return s;
}

// 理想实值投影能量（双精度）：与核用同一份整数 acc = Σ_m U·d，
// 但增益取**未取整**的 gain_true、平方不做整数取整。用于给出 V-1 的参考 R。
static double proj_ideal_score(const int32_t* meas, const int16_t* u_codes,
                               int p, int K, int M, double gain_true) {
    const int16_t* Uc = u_codes + (size_t)p * (size_t)K * (size_t)M;
    double tot = 0.0;
    for (int c = 0; c < 3; ++c) {
        const int32_t* dc = meas + (size_t)c * (size_t)M;
        for (int k = 0; k < K; ++k) {
            const int16_t* ur = Uc + (size_t)k * (size_t)M;
            double acc = 0.0;
            for (int m = 0; m < M; ++m) acc += (double)ur[m] * (double)dc[m];
            const double pz = acc * gain_true;
            tot += pz * pz;
        }
    }
    return tot;
}

int main() {
    std::printf("=== projection_engine 主机侧 testbench（方案 §6.2/§6.4）===\n");

    // ------------------------------------------------------------ U1 格式
    std::printf("\n=== U1 定点格式与位宽链（§6.3 / quantize.py）===\n");
    std::printf("  Q 格式：测量 Q%d.%d（%d bit）  系数 Q%d.%d  奇异值 Q%d.%d\n",
                ps::kMeasInt, ps::kMeasFrac, ps::kMeasBits,
                ps::kCoeffBits - ps::kCoeffFrac, ps::kCoeffFrac,
                ps::kSingBits - ps::kSingFrac, ps::kSingFrac);
    std::printf("         图像状态 Q%d.%d（%d bit）  MAC %d bit  分数 %d bit\n",
                ps::kImageInt, ps::kImageFrac, ps::kImageBits,
                ps::kMacAccBits, ps::kScoreBits);
    std::printf("          码字上界：系数 %lld  测量 %lld  图像 %lld  MAC %lld\n",
                ps::kCoeffCodeMax, ps::kMeasCodeMax, ps::kImageCodeMax,
                ps::kMacAccCodeMax);
    {
        const ps::FpChain c = ps::fp_chain_b();
        std::printf("  位宽链（B 档自采：%d bit 源 / %dx%d 分块 / %d 帧 / %s）：\n",
                    c.effective_bits, c.block, c.block, c.frames,
                    c.fullscale ? "fullscale(左移4位)" : "packed(右对齐)");
        std::printf("    块和 %d（声明 %d）-> 帧累积 %d -> 差分需 %d（声明 %d）；"
                    "MAC 需 %d（声明 %d）\n",
                    c.block_sum_bits(), c.fullscale ? 23 : 20, c.frame_acc_bits(),
                    c.diff_bits_needed(), c.diff_bits,
                    c.mac_acc_bits_needed(), c.mac_acc_bits);
        check(c.ok(), "位宽链自洽（与 FixedPointChain.check 同判据）",
              c.ok() ? "block_sum / diff / mac_acc 全部满足" : "存在不足项");
        check(c.block_sum_bits() == 19,
              "10x10 块和 = 12 + ceil(log2 100) = 19 bit", "与方案 §6.3 一致");
    }

    // ------------------------------------------------------------ 向量规模
    // 规模取小（M=64/K=8/P=27），但与真实布局完全同构：rank-major、通道优先、
    // 候选线性号按 (ix,iy,iz) 展开。真实规模 M=9120/K=8/P=243 的差别只在循环
    // 次数与内存大小，不影响任何一条判据。
    const int K = 8;
    const int M = 64;
    const int NX = 3, NY = 3, NZ = 3;
    const int P = NX * NY * NZ;                 // 27

    const int16_t U_CODE_MAX = 15000;           // 与真实模型 Q1.15 码字同量级
    const int32_t D_CODE_MAX = 16384;           // Q2.16 归一化测量的典型量级（|d|~0.25）

    std::vector<int16_t> u((size_t)P * K * M);
    for (size_t i = 0; i < u.size(); ++i) {
        u[i] = (int16_t)(rnd_sym() * (double)U_CODE_MAX);
    }
    std::vector<int32_t> d((size_t)3 * M);
    for (size_t i = 0; i < d.size(); ++i) {
        d[i] = (int32_t)(rnd_sym() * (double)D_CODE_MAX);
    }

    // ------------------------------------------------------------ U2 网格映射
    std::printf("\n=== U2 候选网格映射（与 x10_localize.cpp 同约定）===\n");
    {
        std::vector<int> seen(P, 0);
        bool bij = true;
        for (int k = 0; k < P; ++k) {
            int ix, iy, iz;
            ps::proj_grid_index(k, NX, NY, NZ, ix, iy, iz);
            const int back = (ix * NY + iy) * NZ + iz;
            if (back != k || ix < 0 || ix >= NX || iy < 0 || iy >= NY || iz < 0 || iz >= NZ) {
                bij = false;
            }
            seen[k] = 1;
        }
        for (int k = 0; k < P; ++k) if (!seen[k]) bij = false;
        std::printf("  网格 %dx%dx%d = %d 个候选；示例 k=13 -> ", NX, NY, NZ, P);
        int ix, iy, iz;
        ps::proj_grid_index(13, NX, NY, NZ, ix, iy, iz);
        std::printf("(ix,iy,iz)=(%d,%d,%d)\n", ix, iy, iz);
        check(bij, "映射是 [0,P) 上的双射（行主序 x->y->z）", "无越界、无重复");
        check(ps::proj_on_boundary(13, NX, NY, NZ) == 0, "内部候选不判边界",
              "k=13 -> (1,1,1) 三轴都在内部");
        check(ps::proj_on_boundary(0, NX, NY, NZ) == 1, "角点候选判边界",
              "k=0 -> (0,0,0)");
        check(ps::proj_on_boundary(13, 0, 0, 0) == 0, "无网格信息时不判边界",
              "nx=ny=nz=0（固定位置模式）");
    }

    // ------------------------------------------------------------ U3 已知候选
    // 合成测量：d_code ≈ a0 * U[t][0] + a1 * U[t][1]，a0 > a1 > 0。
    // 由 Cauchy-Schwarz，候选 t 的 |Σ_m U[t][k] d| 接近其上界，
    // 随机方向的其他候选重叠远小于它 -> 无噪时 top-1 必须命中（Task 4 Step 1）。
    std::printf("\n=== U3 已知候选的合成测量（无噪 top-1）===\n");
    const int T_TRUE = 13;                      // 三个轴都在内部的候选
    const double A0 = 0.60, A1 = 0.35;
    for (int m = 0; m < M; ++m) {
        const double v = A0 * (double)u[(size_t)T_TRUE * K * M + 0 * M + m]
                       + A1 * (double)u[(size_t)T_TRUE * K * M + 1 * M + m];
        d[m] = (int32_t)std::lround(v);         // 先做"无噪声可实现"的码字
    }
    // 三通道用同一份（灰度退化情形），另两通道给一点独立噪声以贴近彩色
    for (int c = 1; c < 3; ++c) {
        for (int m = 0; m < M; ++m) {
            const double v = A0 * (double)u[(size_t)T_TRUE * K * M + 0 * M + m]
                           + A1 * (double)u[(size_t)T_TRUE * K * M + 1 * M + m]
                           + 40.0 * rnd_sym();
            d[(size_t)c * M + m] = (int32_t)std::lround(v);
        }
    }

    // u_scale 取 2^-8（gain = 256，Q16.16 里精确），d_scale 取 2^0：
    // 二者都是 2 的幂，于是 gain_q16 精确可表示，核的误差只剩移位舍入 ——
    // 这正是我们想单独看的那一项。真实模型的非幂次 scale 误差在 U5 里量。
    const int32_t U_GAIN = 256;                 // = round(2^-8 * 2^16)
    std::vector<int32_t> gain_q16(P, U_GAIN);
    // 候选 5 故意用一个非幂次的 u_scale，检查非幂次增益只带来"相对"误差
    gain_q16[5] = 3;                            // round(4.6e-5 * 65536) = 3

    std::vector<uint64_t> score(P, 0), score_ch((size_t)P * 3, 0);
    ps::ProjResult res;
    ps::FixFlags flags;
    ps::fix_flags_reset(flags);

    const int st = ps::projection_engine<16>(d.data(), u.data(), gain_q16.data(),
                                             P, K, M, NX, NY, NZ, 0,
                                             score.data(), score_ch.data(),
                                             res, flags);
    std::printf("  u_scale = 2^-8 -> gain_q16 = %d（精确）；候选 5 用 gain_q16 = 3（非幂次）\n",
                U_GAIN);
    std::printf("  best_id = %d（真值 %d）  s1 = %llu  s2 = %llu  margin_q16 = %d  "
                "boundary = %d  low_conf = %d\n",
                res.best_id, T_TRUE,
                (unsigned long long)res.best_score,
                (unsigned long long)res.second_score,
                res.margin_q16, res.boundary, res.low_conf);
    check(st == ps::kProjOk, "核返回 kProjOk", "参数合法");
    check(res.best_id == T_TRUE, "无噪合成测量 top-1 命中", "best_id == 真值候选");
    check(res.margin_q16 > 0, "最优与次优可区分（margin > 0）", "分数存在间隔");

    // ------------------------------------------------------------ U4 精确复算
    std::printf("\n=== U4 与双精度参考逐候选对照（核内全整数，应当精确相等）===\n");
    {
        double worst = 0.0;
        int worst_p = -1;
        bool same_order = true;
        int best_ref = -1;
        double best_ref_v = -1.0;
        for (int p = 0; p < P; ++p) {
            const double ref = ps::projection_score_reference(
                d.data(), u.data(), gain_q16.data(), p, K, M);
            const double got = (double)score[p];
            const double denom = (ref > 0.0) ? ref : 1.0;
            const double rel = std::fabs(got - ref) / denom;
            if (rel > worst) { worst = rel; worst_p = p; }
            if (ref > best_ref_v) { best_ref_v = ref; best_ref = p; }
        }
        same_order = (best_ref == res.best_id);
        std::printf("  最坏相对误差 %.3e（候选 %d）；参考 best_id = %d\n",
                    worst, worst_p, best_ref);
        checkf(worst <= 1e-12, "分数与双精度参考一致（<=1e-12）",
               "最坏 %.3e", worst, 0.0);
        check(same_order, "排序与参考一致", "best_id 相同");
    }

    // ------------------------------------------------------------ U5 浮点参考
    // 端到端定点误差：把 gain 还原成"未取整"的 u_scale*d_scale 再算一遍。
    // 误差来源有三项，量级都可以预先说清：
    //   (a) 增益 Q16.16 取整：相对 2^-17 量级（gain=256 时约 4e-6）
    //   (b) 投影系数取整到整数：相对 1/|p|（|p| ~ 1e3 -> 1e-3 量级）
    //   (c) U 码字本身的量化：这里 U 码字直接来自整数，不引入额外误差
    // 故 (b) 是主导项，阈值按 1e-2 设。
    std::printf("\n=== U5 相对未取整 scale 的浮点参考：端到端定点误差 ===\n");
    {
        const double s_true = std::ldexp(1.0, -8);      // 未取整的 u_scale
        const double g_true = s_true;                   // d_scale = 1
        double worst = 0.0;
        for (int p = 0; p < P; ++p) {
            if (p == 5) continue;                       // 5 号故意用非幂次 gain，单列
            const int16_t* Uc = u.data() + (size_t)p * K * M;
            double tot = 0.0;
            for (int c = 0; c < 3; ++c) {
                for (int k = 0; k < K; ++k) {
                    double acc = 0.0;
                    for (int m = 0; m < M; ++m) {
                        acc += (double)Uc[k * M + m] * (double)d[(size_t)c * M + m];
                    }
                    const double pz = acc * g_true;
                    tot += pz * pz;
                }
            }
            const double got = (double)score[p];
            const double rel = std::fabs(got - tot) / ((tot > 0) ? tot : 1.0);
            if (rel > worst) worst = rel;
        }
        std::printf("  最坏相对误差 %.3e（不含候选 5）\n", worst);
        checkf(worst <= 1e-2, "端到端定点误差（<=1e-2）", "最坏 %.3e", worst, 0.0);
    }

    // ------------------------------------------------------------ U6 LANES
    std::printf("\n=== U6 LANES 不变性（8/16/32 必须逐位相同）===\n");
    {
        std::vector<uint64_t> s8(P), s16(P), s32(P), c8((size_t)P * 3);
        ps::ProjResult r8, r16, r32;
        ps::FixFlags f8, f16, f32;
        ps::fix_flags_reset(f8);
        ps::fix_flags_reset(f16);
        ps::fix_flags_reset(f32);
        ps::projection_engine<8>(d.data(), u.data(), gain_q16.data(), P, K, M,
                                 NX, NY, NZ, 0, s8.data(), c8.data(), r8, f8);
        ps::projection_engine<16>(d.data(), u.data(), gain_q16.data(), P, K, M,
                                  NX, NY, NZ, 0, s16.data(), 0, r16, f16);
        ps::projection_engine<32>(d.data(), u.data(), gain_q16.data(), P, K, M,
                                  NX, NY, NZ, 0, s32.data(), 0, r32, f32);
        bool eq = true;
        for (int p = 0; p < P; ++p) {
            if (s8[p] != s16[p] || s8[p] != s32[p]) eq = false;
        }
        for (int p = 0; p < P; ++p) {
            if (c8[(size_t)p * 3 + 0] + c8[(size_t)p * 3 + 1] + c8[(size_t)p * 3 + 2]
                != s8[p]) eq = false;              // 通道分数之和 == 总分数
        }
        check(eq, "8/16/32 分数逐位相同，且 3 通道和 == 总分",
              "整数累加可结合，构造性成立");
        check(r8.best_id == r16.best_id && r16.best_id == r32.best_id,
              "三种 LANES 的 best_id 相同", "排序一致");
        check(!f8.any() && !f16.any() && !f32.any(),
              "常规数据下无任何饱和", "FixFlags 全零");
    }

    // ------------------------------------------------------------ U7 门控
    std::printf("\n=== U7 margin / 边界 / low_conf 门控（§6.4 末条）===\n");
    {
        ps::ProjResult r;
        ps::FixFlags f;
        std::vector<uint64_t> s(P);
        ps::fix_flags_reset(f);

        // (a) 阈值 0、内部候选 -> 可信
        ps::projection_engine<16>(d.data(), u.data(), gain_q16.data(), P, K, M,
                                  NX, NY, NZ, 0, s.data(), 0, r, f);
        check(r.boundary == 0 && r.low_conf == 0,
              "阈值 0 且内部候选 -> 可信", "low_conf = 0");

        // (b) 阈值拉到最大 -> 一律低可信（禁止把低 margin 当精确自定位）
        ps::projection_engine<16>(d.data(), u.data(), gain_q16.data(), P, K, M,
                                  NX, NY, NZ, 65535, s.data(), 0, r, f);
        check(r.low_conf == 1, "阈值 65535 -> 一律低可信", "low_conf = 1");

        // (c) margin 的单调性：(s1-s2) 越小 margin 越小
        const uint64_t m_same = (uint64_t)ps::proj_margin_q16(1000000ull, 1000000ull);
        const uint64_t m_near = (uint64_t)ps::proj_margin_q16(1000000ull, 999000ull);
        const uint64_t m_far  = (uint64_t)ps::proj_margin_q16(1000000ull, 100000ull);
        std::printf("  margin(1e6,1e6)=%llu  (1e6,9.99e5)=%llu  (1e6,1e5)=%llu\n",
                    (unsigned long long)m_same, (unsigned long long)m_near,
                    (unsigned long long)m_far);
        check(m_same == 0 && m_near < m_far && m_far < 65536,
              "margin 随 (s1-s2) 单调增加且落在 [0,65536) 内", "比例关系成立");

        // (d) 边界候选 -> low_conf（构造：把真值放到角点上）
        {
            std::vector<int16_t> u2 = u;
            std::vector<int32_t> d2 = d;
            const int T2 = 0;                       // (ix,iy,iz) = (0,0,0)
            for (int m = 0; m < M; ++m) {
                const double v = 0.6 * (double)u2[(size_t)T2 * K * M + 0 * M + m]
                               + 0.35 * (double)u2[(size_t)T2 * K * M + 1 * M + m];
                d2[m] = (int32_t)std::lround(v);
            }
            for (int c = 1; c < 3; ++c) {
                for (int m = 0; m < M; ++m) {
                    const double v = 0.6 * (double)u2[(size_t)T2 * K * M + 0 * M + m]
                                   + 0.35 * (double)u2[(size_t)T2 * K * M + 1 * M + m];
                    d2[(size_t)c * M + m] = (int32_t)std::lround(v);
                }
            }
            ps::projection_engine<16>(d2.data(), u2.data(), gain_q16.data(), P, K, M,
                                      NX, NY, NZ, 0, s.data(), 0, r, f);
            std::printf("  角点真值：best_id = %d（应为 %d）  boundary = %d  low_conf = %d\n",
                        r.best_id, T2, r.boundary, r.low_conf);
            check(r.best_id == T2 && r.boundary == 1 && r.low_conf == 1,
                  "最佳落在网格边界 -> 边界位与低可信位同时置 1",
                  "禁止伪装成精确自定位");
        }
    }

    // ------------------------------------------------------------ U8 饱和行为
    std::printf("\n=== U8 极限码字下的饱和行为（禁止回绕，§6.3）===\n");
    {
        std::vector<int16_t> ux((size_t)P * K * M, (int16_t)ps::kCoeffCodeMax);
        std::vector<int32_t> dx((size_t)3 * M, (int32_t)ps::kMeasCodeMax);
        std::vector<int32_t> gx(P, (int32_t)ps::kGainSafeMax);
        std::vector<uint64_t> sx(P);
        ps::ProjResult rx;
        ps::FixFlags fx;
        ps::fix_flags_reset(fx);
        ps::projection_engine<16>(dx.data(), ux.data(), gx.data(), P, K, M,
                                  NX, NY, NZ, 0, sx.data(), 0, rx, fx);
        std::printf("  全最大码字：score[0] = %.6e（位宽 %d bit）  "
                    "score_overflow = %d  mac_overflow = %d  max|acc| = %.6e\n",
                    (double)sx[0], bit_width_u64(sx[0]),
                    fx.score_overflow, fx.mac_overflow, (double)fx.max_abs_acc);
        check(rx.overflow == 1 && fx.score_overflow > 0,
              "分数通路溢出被记账（不回绕）", "score_overflow > 0");
        check(sx[0] >= (1ull << 60), "分数停在饱和侧而不是绕回小值",
              "score 顶到上界附近");

        // 48 bit MAC 的解析余量。**两个口径都要写出来**（2026-10-02 更正）：
        //   · K=1（单 rank）：|acc| <= M · 系数上界 · 测量上界 = 3.92e13 = 2^45.15
        //   · K=8（PS_PSM_LOC_RANK，psm_format.h:58）：×8 = 3.13e14 = 2^48.15
        // 48 bit 有符号上界是 2^47 = 1.4074e14，**装不下 K=8 的理论最坏值**，
        // 即「不可溢出」这句是错的：真实数据下有余量（实测见下），理论最坏需 49 bit
        // 幅值（有符号即 50 bit）。此前只报 K=1 口径，故看起来「不可溢出」。
        const double acc_k1 = (double)9120 * (double)ps::kCoeffCodeMax
                            * (double)ps::kMeasCodeMax;
        const double acc_k8 = 8.0 * acc_k1;          // K=8（B 档定位秩）
        const double acc_max = std::ldexp(1.0, ps::kMacAccBits - 1) - 1.0;
        const double meas_headroom = std::log2(acc_max / (double)fx.max_abs_acc);
        std::printf("  |acc| 上界：K=1 口径 %.6e（2^%.2f）；K=8 口径 %.6e（2^%.2f）\n",
                    acc_k1, std::log2(acc_k1), acc_k8, std::log2(acc_k8));
        std::printf("  48 bit 有符号上界 2^47 = %.6e -> K=1 余量 %.2f bit，"
                    "K=8 理论最坏需 49 bit（48 bit 装不下）\n",
                    acc_max, std::log2(acc_max / acc_k1));
        std::printf("  本档（M=%d，全最大码字）实测 max|acc| = %.6e，实测余量 %.2f bit\n",
                    M, (double)fx.max_abs_acc, meas_headroom);
        check(fx.mac_overflow == 0 && meas_headroom > 5.0,
              "本档实测 max|acc| 未越 48 bit（余量 >= 5 bit）",
              "理论最坏 K=8 需 49 bit，不可声称不可溢出");
    }

    // ------------------------------------------------------------ U9 核算
    std::printf("\n=== U9 带宽与算力核算（方案 §5.3）===\n");
    {
        const double mac_real = 243.0 * 9120.0 * 8.0 * 3.0;
        char b1[32], b2[32];
        std::printf("  真实规模定位标量 MAC = 243 x 9120 x 8 x 3 = %.4e（方案 §5.3 写 53.2 M）\n",
                    mac_real);
        std::printf("  每帧只读 U 前 K 列：243 x 8 x 9120 x 2 B = %s\n",
                    fmt_bytes(243.0 * 8.0 * 9120.0 * 2.0, b1, sizeof(b1)));
        std::printf("  测量常驻 BRAM：3 x 9120 x 4 B = %s（Q2.16 存 18 bit 也按 32 bit 对齐）\n",
                    fmt_bytes(3.0 * 9120.0 * 4.0, b2, sizeof(b2)));
        std::printf("  本次实测最大分数位宽 = %d bit（64 bit 分数通路余量 %d bit）\n",
                    bit_width_u64(res.best_score), 64 - bit_width_u64(res.best_score));
        check(bit_width_u64(res.best_score) < 64,
              "实测分数位宽 < 64（分数通路不溢出）", "余量充足");
        check(std::fabs(mac_real - 53.2e6) / 53.2e6 < 0.01,
              "MAC 数与方案 §5.3 的 53.2 M 一致", "按同一定义复算");
    }

    // ------------------------------------------------------------ V-1 翻转裕度
    // 判据 6 的正确口径：把「结果量」（972/972 位号一致、Spearman 1.000000）
    // 升级成「裕度倍数」ε*。定义与分子/分母口径见 eps_flip_scan 上方注释。
    // 本段只跑主机侧，不参与综合，不改核。
    std::printf("\n=== V-1 ε* 翻转裕度扫描（判据 6 的正确口径）===\n");
    std::printf("  R=权威参考能量  F=定点核能量  δ=F-R\n");
    std::printf("  分子 m_k=(R[best]-R[k])/R[best]  分母 e_k=|δ_k|/R[k]\n");
    std::printf("  a_k=(R[best]-R[k])/(δ_k-δ_best)  ε*=min_k a_k  判据 ε* > 2\n");
    {
        // ---- (a) 扫描器自检：一维解析例（答案可手算）----
        {
            std::vector<double> R1{100.0, 99.0}, F1{99.9, 99.0};   // δ = {-0.1, 0}
            EpsScan e1 = eps_flip_scan(R1, F1);
            // a* = (100-99)/(0-(-0.1)) = 10
            std::printf("  [自检] R={100,99} F={99.9,99}: ε*=%.6f（解析 10）"
                        "  a_scan=%.6f  flip_k=%d\n",
                        e1.eps_star, e1.a_scan, e1.flip_k);
            check(e1.scannable == 1 && std::fabs(e1.eps_star - 10.0) < 1e-9 &&
                  std::fabs(e1.a_scan - e1.eps_star) <= 1e-9 * e1.eps_star,
                  "扫描器自检：解析例 ε*=10，逐步放大与解析一致",
                  "定义与实现一致");

            std::vector<double> R2{100.0, 99.0}, F2{100.0, 99.0};   // δ ≡ 0
            EpsScan e2 = eps_flip_scan(R2, F2);
            std::printf("  [自检] δ≡0: scannable=%d  ε* 为 +inf = %d\n",
                        e2.scannable, std::isinf(e2.eps_star) ? 1 : 0);
            check(e2.scannable == 0 && std::isinf(e2.eps_star),
                  "退化（δ≡0）判为不可扫描，如实列出而不伪造裕度",
                  "ε*=+inf");
        }

        // ---- (b) 流水线场景：核 vs 未取整实值参考 ----
        // 用未取整实值增益作 R（核 vs 理想），故 δ 非零、ε* 有限；
        // 这正是「误差再放大几倍翻转」的口径，与 x10 的 E_cpp vs E_ref 同定义。
        const double G_POW2 = std::ldexp(1.0, -8);     // 2^-8（gain_q16=256 精确）

        // U7 的角点真值测量（候选 0），复算一份供场景 S3 用。
        std::vector<int32_t> d_corner((size_t)3 * M);
        {
            const int T2 = 0;
            for (int m = 0; m < M; ++m) {
                const double v = 0.6 * (double)u[(size_t)T2 * K * M + 0 * M + m]
                               + 0.35 * (double)u[(size_t)T2 * K * M + 1 * M + m];
                d_corner[m] = (int32_t)std::lround(v);
            }
            for (int c = 1; c < 3; ++c)
                for (int m = 0; m < M; ++m) d_corner[(size_t)c * M + m] = d_corner[m];
        }

        // 求 (R,F) 对：use_ref=1 时用整数参考（同一已取整 gain），用于退化对照。
        std::vector<double> R((size_t)P), F((size_t)P);
        auto eval_pair = [&](const std::vector<int32_t>& dd, int32_t gq16,
                             double gtrue, int use_ref) {
            std::vector<int32_t> gg((size_t)P, gq16);
            std::vector<uint64_t> sc((size_t)P, 0);
            ps::ProjResult r; ps::FixFlags f; ps::fix_flags_reset(f);
            ps::projection_engine<16>(dd.data(), u.data(), gg.data(), P, K, M,
                                      NX, NY, NZ, 0, sc.data(), 0, r, f);
            for (int p = 0; p < P; ++p) {
                F[p] = (double)sc[p];
                R[p] = use_ref
                     ? ps::projection_score_reference(dd.data(), u.data(),
                                                      gg.data(), p, K, M)
                     : proj_ideal_score(dd.data(), u.data(), p, K, M, gtrue);
            }
        };

        struct Scen { const char* name; EpsScan e; };
        std::vector<Scen> scen;

        eval_pair(d, 256, G_POW2, 0);
        scen.push_back({"S1 U3无噪/幂次增益(gq=256)", eps_flip_scan(R, F)});
        eval_pair(d, 3, 4.6e-5, 0);
        scen.push_back({"S2 U3无噪/非幂次增益(gq=3)", eps_flip_scan(R, F)});
        eval_pair(d_corner, 256, G_POW2, 0);
        scen.push_back({"S3 角点真值/幂次增益(gq=256)", eps_flip_scan(R, F)});

        double min_eps = HUGE_VAL;
        int n_scan = 0;
        for (size_t i = 0; i < scen.size(); ++i) {
            const EpsScan& e = scen[i].e;
            if (e.scannable) {
                std::printf("  %-34s best=%2d flip_k=%2d  m_k=%.3e  e_best=%.3e"
                            "  ε*=%.6e  ε*_ratio=%.6e  a_scan=%.6e\n",
                            scen[i].name, e.best, e.flip_k, e.margin, e.e_best,
                            e.eps_star, e.eps_ratio, e.a_scan);
                if (e.eps_star < min_eps) min_eps = e.eps_star;
                ++n_scan;
            } else {
                std::printf("  %-34s best=%2d  ε*=+inf（不可扫描：δ≡0 或无翻转对手）\n",
                            scen[i].name, e.best);
            }
        }

        // S4 退化对照：参考取与核同一已取整 gain 的整数参考 -> δ≡0（列出并说明）。
        eval_pair(d, 256, G_POW2, 1);
        EpsScan e4 = eps_flip_scan(R, F);
        std::printf("  %-34s ε*=+inf  scannable=%d（δ≡0：核与整数参考逐候选相等）\n",
                    "S4 退化对照/整数参考", e4.scannable);

        std::printf("  可扫描场景数 = %d，最小 ε* = %.6e\n", n_scan, min_eps);
        check(n_scan >= 2, "至少 2 个场景给出有限翻转倍数",
              "其余场景不可扫描（无有限翻转点），已逐条列出");
        check(min_eps > 2.0, "可扫描场景的最小 ε* > 2（判据 6 的翻转裕度口径）",
              "定位结论稳，而非仅位号一致");
        check(e4.scannable == 0, "退化对照（δ≡0）不进 ε* 统计",
              "避免用无穷大冒充裕度");
    }

    std::printf("\n%s  PASS %d  FAIL %d\n",
                (g_fail == 0) ? "全部通过。" : "存在失败项。", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
