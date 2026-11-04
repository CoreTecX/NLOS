// ============================================================================
// lr_operator 主机侧 testbench（方案 §6.2 / §6.5 / §6.3）
//
// 编译（在 src/fpga/hls/lr_operator 目录下）：
//   g++ -std=c++17 -Wall -Wextra -O2 lr_operator_fixed.cpp test_lr_operator.cpp -o test_lr_operator.exe
//   （实现文件名带 _fixed 后缀：同目录另有一层双精度参考实现占用了 lr_operator.cpp，
//     见 src/fpga/hls/README.md「两层」一节）
//
// 不依赖任何外部数据：U/V 由固定种子生成随机矩阵后做 MGS 正交化，S 取带明确
// 物理含义的归一化奇异值序列，全部可复现。
//
// 检查项（对应 Task 7 Step 1~Step 2 的判据）
//   L1 Q 格式与四个移位量（16/16/18/14/16）
//   L2 因子构造与 ||B||₂：幂迭代估计 vs 解析值 alpha_u·alpha_v·s_max
//   L3 逐级误差 <= 1 LSB（Vᵀf / S∘z / U·zs）
//   L4 伴随性 <Bf, r> == <f, Bᵀr>（最强的一条整体正确性判据）
//   L5 LANES=8/16/32 逐位相同
//   L6 饱和行为与 48 bit MAC 余量核算（不回绕）
//   L7 Q16.16 增益取整误差的量级：为什么模型 scale 必须选成幂次/共享指数
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "lr_operator.hpp"

// ---------------------------------------------------------------- 自检框架
static int g_pass = 0;
static int g_fail = 0;

static void check(bool ok, const char* name, const char* detail) {
    std::printf("  [%s] %-50s %s\n", ok ? "ok  " : "FAIL", name, detail);
    if (ok) ++g_pass; else ++g_fail;
}

static void report(const char* name, const char* fmt, double v) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), fmt, v);
    std::printf("  [info] %-50s %s\n", name, buf);
}

// ---------------------------------------------------------------- 确定性随机
static uint64_t g_rng = 0x9E3779B97F4A7C15ull;

static double rnd_uni() {
    g_rng += 0x9E3779B97F4A7C15ull;
    uint64_t z = g_rng;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= (z >> 31);
    return (double)(z >> 11) * (1.0 / 9007199254740992.0);
}

static double rnd_sym() { return 2.0 * rnd_uni() - 1.0; }

// ---------------------------------------------------------------- MGS
// 对 rows x cols 的矩阵逐列 Gram-Schmidt 正交化（列正交、单位长）。
// 列正交是后面「解析 ||B|| = alpha_u·alpha_v·s_max」成立的前提。
static void orthonormal_columns(std::vector<double>& A, int rows, int cols) {
    for (int c = 0; c < cols; ++c) {
        for (int p = 0; p < c; ++p) {
            double dp = 0.0;
            for (int r = 0; r < rows; ++r) dp += A[r * cols + p] * A[r * cols + c];
            for (int r = 0; r < rows; ++r) A[r * cols + c] -= dp * A[r * cols + p];
        }
        double nrm = 0.0;
        for (int r = 0; r < rows; ++r) nrm += A[r * cols + c] * A[r * cols + c];
        nrm = std::sqrt(nrm);
        if (nrm > 0.0) {
            for (int r = 0; r < rows; ++r) A[r * cols + c] /= nrm;
        }
    }
}

int main() {
    std::printf("=== lr_operator 主机侧 testbench（方案 §6.5 低秩前向/伴随）===\n");

    const int K = 8;
    const int M = 64;                 // 真实 9120（95x96 差分测量）
    const int N = 48;                 // 真实 432（24x18 隐藏图像）

    // ------------------------------------------------------------ L1 格式
    std::printf("\n=== L1 Q 格式与移位量（§6.3）===\n");
    std::printf("  图像/秩域 Q%d.%d（%d bit）  测量域 Q%d.%d（%d bit）  奇异值 Q%d.%d\n",
                ps::kImageInt, ps::kImageFrac, ps::kImageBits,
                ps::kMeasInt, ps::kMeasFrac, ps::kMeasBits,
                ps::kSingBits - ps::kSingFrac, ps::kSingFrac);
    std::printf("  移位量：Vᵀf %d  S∘z %d  U·zs %d  Uᵀr %d  V·ws %d（推导见 .hpp 文首）\n",
                ps::kShiftVToZ, ps::kShiftZToS, ps::kShiftUToY,
                ps::kShiftUToW, ps::kShiftVToG);
    {
        const bool ok = (ps::kShiftVToZ == 16) && (ps::kShiftZToS == 16)
                     && (ps::kShiftUToY == 18) && (ps::kShiftUToW == 14)
                     && (ps::kShiftVToG == 16);
        check(ok, "五个移位量与 §6.3 的位宽链自洽（ΔF = 18-16 = 2）",
              "16/16/18/14/16");
    }

    // ------------------------------------------------------------ L2 构造因子
    std::printf("\n=== L2 低秩因子构造（U/V 列正交，S 为归一化奇异值）===\n");
    std::vector<double> Qu((size_t)M * K), Qv((size_t)N * K);
    for (size_t i = 0; i < Qu.size(); ++i) Qu[i] = rnd_sym();
    for (size_t i = 0; i < Qv.size(); ++i) Qv[i] = rnd_sym();
    orthonormal_columns(Qu, M, K);
    orthonormal_columns(Qv, N, K);

    // Q1.15 能表示的码字上限是 32767，故把列正交基整体缩放 alpha，
    // 使 max|entry| <= 0.499（统一缩放保持列正交，只把列范数变成 alpha）。
    double amax_u = 0.0, amax_v = 0.0;
    for (size_t i = 0; i < Qu.size(); ++i) amax_u = std::max(amax_u, std::fabs(Qu[i]));
    for (size_t i = 0; i < Qv.size(); ++i) amax_v = std::max(amax_v, std::fabs(Qv[i]));
    const double alpha_u = (amax_u > 0.499) ? 0.499 / amax_u : 1.0;
    const double alpha_v = (amax_v > 0.499) ? 0.499 / amax_v : 1.0;
    for (size_t i = 0; i < Qu.size(); ++i) Qu[i] *= alpha_u;
    for (size_t i = 0; i < Qv.size(); ++i) Qv[i] *= alpha_v;

    // scale 取 2 的幂 -> Q16.16 增益精确可表示（取整误差为 0）。
    // 这一条不是"为了好看"：L7 会量出非幂次 scale 的代价。
    const double u_scale = std::ldexp(1.0, -16);     // gain = 1
    const double v_scale = std::ldexp(1.0, -16);     // gain = 1
    const double s_scale = std::ldexp(1.0, -14);     // gain = 4（Q2.14 钉死格式）

    const double s_norm[8] = {0.50, 0.35, 0.25, 0.20, 0.15, 0.12, 0.10, 0.08};

    ps::LrFactors F;
    std::vector<int16_t> U((size_t)K * M), V((size_t)K * N);
    std::vector<uint16_t> S(K);
    for (int k = 0; k < K; ++k) {
        for (int m = 0; m < M; ++m) {
            const double q = Qu[(size_t)m * K + k] / u_scale;
            U[(size_t)k * M + m] = (int16_t)ps::sat_round_i64(q, -32767, 32767);
        }
        for (int i = 0; i < N; ++i) {
            const double q = Qv[(size_t)i * K + k] / v_scale;
            V[(size_t)k * N + i] = (int16_t)ps::sat_round_i64(q, -32767, 32767);
        }
        S[k] = (uint16_t)ps::sat_round_i64(s_norm[k] / s_scale, 0, 65535);
    }
    F.U = U.data();
    F.S = S.data();
    F.V = V.data();
    F.K = K;
    F.M = M;
    F.N = N;
    F.u_gain_q16 = ps::scale_to_gain_q16(u_scale);
    F.s_gain_q16 = ps::scale_to_gain_q16(s_scale);
    F.v_gain_q16 = ps::scale_to_gain_q16(v_scale);
    std::printf("  U: %dx%d  V: %dx%d  S 归一化奇异值 max = %.3f\n", M, K, N, K, s_norm[0]);
    std::printf("  scales: u=2^-16 v=2^-16 s=2^-14 -> gain_q16 = %d / %d / %d（精确可表示）\n",
                F.u_gain_q16, F.v_gain_q16, F.s_gain_q16);

    // ||B||₂ 的解析值：U/V 列正交且列范数为 alpha -> 奇异值 = alpha_u·alpha_v·s_k
    const double norm_exact = alpha_u * alpha_v * s_norm[0];
    int it_used = 0;
    double delta = 0.0;
    const double norm_est = ps::lr_spectral_norm_power(F, 400, 1e-12, 20260926u,
                                                       &it_used, &delta);
    std::printf("  ||B||₂：幂迭代 %.9f（%d 次，末次相对变化 %.2e）  解析 %.9f  相对差 %.2e\n",
                norm_est, it_used, delta, norm_exact,
                std::fabs(norm_est - norm_exact) / norm_exact);
    {
        const bool ok = std::fabs(norm_est - norm_exact) / norm_exact < 5e-3;
        check(ok, "幂迭代估计 ||B||₂ 与解析值一致（<0.5%，供 §6.5 定 τ/σ）",
              "与 quantize.power_iteration_norm 同口径");
    }

    // ------------------------------------------------------------ 向量
    // f 的码字范围：f 是 Q6.18，|f_code| <= 2^18 时 f_real <= 1，
    // 于是 |y| <= ||B||·||f||₂ ~ 0.5·7 ~ 3.5 —— 会顶到 Q2.16 上界。
    // 取 |f_code| <= 2^16（f_real <= 0.25）留足余量，并在 L6 单独测饱和。
    std::vector<int32_t> f((size_t)N);
    for (int i = 0; i < N; ++i) f[i] = (int32_t)(rnd_sym() * 65536.0);
    std::vector<int32_t> y((size_t)M), r((size_t)M);
    for (int m = 0; m < M; ++m) r[m] = (int32_t)(rnd_sym() * 40000.0);

    // ------------------------------------------------------------ L3 逐级 LSB
    std::printf("\n=== L3 逐级误差（Task 7 Step 1：每级 <= 1 LSB）===\n");
    std::vector<int32_t> z(K), zs(K), w(K), ws(K);
    ps::LrWork Wk;
    Wk.z = z.data();
    Wk.zs = zs.data();
    Wk.w = w.data();
    Wk.ws = ws.data();

    ps::LrDiag dg;
    const int st_f = ps::lr_forward<16>(F, f.data(), y.data(), dg, &Wk);
    check(st_f == ps::kLrOk, "lr_forward 返回 kLrOk", "参数合法");
    check(!dg.flags.any(), "前向无任何饱和", "FixFlags 全零");

    {
        const double ug = ps::gain_q16_to_double(F.u_gain_q16);
        const double sg = ps::gain_q16_to_double(F.s_gain_q16);
        const double vg = ps::gain_q16_to_double(F.v_gain_q16);
        const double lz = std::ldexp(1.0, ps::kImageFrac);   // Q6.18 的 1/step
        const double ly = std::ldexp(1.0, ps::kMeasFrac);    // Q2.16 的 1/step

        double e_z = 0.0, e_zs = 0.0, e_y = 0.0;
        std::vector<double> zref(K), zsref(K);
        for (int k = 0; k < K; ++k) {
            double a = 0.0;
            for (int i = 0; i < N; ++i) {
                a += (double)V[(size_t)k * N + i] * vg * ((double)f[i] / lz);
            }
            zref[k] = a;
            zsref[k] = a * ((double)S[k] * sg);
            // 误差按**码字单位**记，即该级输出格式的 1 LSB（§6.3 的判据口径）
            e_z = std::max(e_z, std::fabs((double)z[k] - zref[k] * lz));
            e_zs = std::max(e_zs, std::fabs((double)zs[k] - zsref[k] * lz));
        }
        for (int m = 0; m < M; ++m) {
            double a = 0.0;
            for (int k = 0; k < K; ++k) {
                a += (double)U[(size_t)k * M + m] * ug * zsref[k];
            }
            e_y = std::max(e_y, std::fabs((double)y[m] - a * ly));
        }
        std::printf("  最坏误差（单位 = 该级输出格式的 1 LSB）：Vᵀf %.4f  S∘z %.4f  U·zs %.4f\n",
                    e_z, e_zs, e_y);
        char b1[128], b2[128];
        std::snprintf(b1, sizeof(b1), "Vᵀf %.4f LSB, S∘z %.4f LSB", e_z, e_zs);
        check(e_z <= 1.0 && e_zs <= 1.0, "秩域两级 <= 1 LSB", b1);
        std::snprintf(b2, sizeof(b2), "U·zs %.4f LSB（含前两级舍入的传播）", e_y);
        check(e_y <= 1.0, "测量域输出 Bf <= 1 LSB", b2);
    }

    // 稠密对照：与 lr_codes_to_dense_B 给出的 B 直接乘
    {
        std::vector<double> Bd((size_t)M * N), yref((size_t)M, 0.0);
        ps::lr_codes_to_dense_B(F, Bd.data());
        const double lz = std::ldexp(1.0, ps::kImageFrac);
        const double ly = std::ldexp(1.0, ps::kMeasFrac);
        for (int m = 0; m < M; ++m) {
            double a = 0.0;
            for (int i = 0; i < N; ++i) a += Bd[(size_t)m * N + i] * ((double)f[i] / lz);
            yref[m] = a;
        }
        double e = 0.0, scale_y = 0.0;
        for (int m = 0; m < M; ++m) {
            e = std::max(e, std::fabs((double)y[m] - yref[m] * ly));   // 单位 = LSB
            scale_y = std::max(scale_y, std::fabs(yref[m]) * ly);      // 单位 = LSB
        }
        std::printf("  稠密 B 对照：max|y| = %.1f LSB（Q2.16 上界 %lld LSB），"
                    "最坏误差 %.4f LSB\n",
                    scale_y, ps::kMeasCodeMax, e);
        report("Bf 相对误差（稠密 B 参考）", "%.3e", e / (scale_y > 0 ? scale_y : 1.0));
        check(e <= 1.0, "与稠密 B 乘法的误差 <= 1 LSB", "低秩形式 == U·diag(S)·Vᵀ");
        check(scale_y < (double)ps::kMeasCodeMax,
              "本次数据不顶 Q2.16 上界", "留给 L6 专门测饱和");
    }

    // ------------------------------------------------------------ L4 伴随性
    std::printf("\n=== L4 伴随性 <Bf, r> == <f, Bᵀr> ===\n");
    {
        std::vector<int32_t> g((size_t)N);
        ps::LrDiag dga;
        const int st_a = ps::lr_adjoint<16>(F, r.data(), g.data(), dga, &Wk);
        check(st_a == ps::kLrOk, "lr_adjoint 返回 kLrOk", "参数合法");
        check(!dga.flags.any(), "伴随无任何饱和", "FixFlags 全零");

        double lhs = 0.0, rhs = 0.0;
        double sum_abs_r = 0.0, sum_abs_f = 0.0;
        const double ly = std::ldexp(1.0, ps::kMeasFrac);
        const double lz = std::ldexp(1.0, ps::kImageFrac);
        for (int m = 0; m < M; ++m) {
            lhs += ((double)y[m] / ly) * ((double)r[m] / ly);
            sum_abs_r += std::fabs((double)r[m] / ly);
        }
        for (int i = 0; i < N; ++i) {
            rhs += ((double)f[i] / lz) * ((double)g[i] / lz);
            sum_abs_f += std::fabs((double)f[i] / lz);
        }
        const double defect = std::fabs(lhs - rhs);
        // 定点下伴随性只能"到舍入误差为止"。理论界：两侧输出各有 <= 1 LSB 的
        // 舍入（0.5 LSB 本体 + 前级传播，L3 实测 ~0.51 LSB），故
        //   |<Bf,r> - <f,Bᵀr>| <= (1/ly)·Σ|r| + (1/lz)·Σ|f|
        const double bound = sum_abs_r / ly + sum_abs_f / lz;
        std::printf("  <Bf,r> = %.9e   <f,Bᵀr> = %.9e\n", lhs, rhs);
        std::printf("  绝对偏差 %.3e；定点理论界 %.3e（= Σ|r|/2^16 + Σ|f|/2^18）\n",
                    defect, bound);
        check(defect <= bound, "伴随性偏差落在定点舍入理论界内",
              "四个阶段的定标与移位方向都正确");
        check(std::fabs(lhs - rhs) / (std::fabs(lhs) > 0 ? std::fabs(lhs) : 1.0) <= 5e-3,
              "伴随性相对差 <= 5e-3", "小规模（M=64,N=48）下舍入噪声可达 1e-3 量级");
        std::printf("  MAC 记账：forward %lld  adjoint %lld（K·(M+N) = %d）\n",
                    dg.mac_count, dga.mac_count, K * (M + N));
        check(dg.mac_count == (long long)K * (M + N) &&
              dga.mac_count == (long long)K * (M + N),
              "单次运算 MAC 数 = K·(M+N)", "与方案 §5.3 的 2R(M+N) 口径一致");
    }

    // ------------------------------------------------------------ L5 LANES
    std::printf("\n=== L5 LANES 不变性（8/16/32 逐位相同）===\n");
    {
        std::vector<int32_t> y8(M), y16(M), y32(M), g8(N), g16(N), g32(N);
        ps::LrDiag d1, d2, d3, d4, d5, d6;
        ps::lr_forward<8>(F, f.data(), y8.data(), d1, 0);
        ps::lr_forward<16>(F, f.data(), y16.data(), d2, 0);
        ps::lr_forward<32>(F, f.data(), y32.data(), d3, 0);
        ps::lr_adjoint<8>(F, r.data(), g8.data(), d4, 0);
        ps::lr_adjoint<16>(F, r.data(), g16.data(), d5, 0);
        ps::lr_adjoint<32>(F, r.data(), g32.data(), d6, 0);
        bool eq = true;
        for (int m = 0; m < M; ++m) if (y8[m] != y16[m] || y8[m] != y32[m]) eq = false;
        for (int i = 0; i < N; ++i) if (g8[i] != g16[i] || g8[i] != g32[i]) eq = false;
        check(eq, "前向与伴随在 8/16/32 下逐位相同",
              "整数累加可结合；MAC 阵列宽度不改变结果");
    }

    // ------------------------------------------------------------ L6 饱和
    std::printf("\n=== L6 饱和行为与 48 bit MAC 余量（§6.3 禁止回绕）===\n");
    {
        // 把 f 推到 Q6.18 的上界：f_real = 32，y 必然越过 Q2.16 上界
        std::vector<int32_t> fbig(N, (int32_t)ps::kImageCodeMax);
        std::vector<int32_t> ybig(M);
        ps::LrDiag dgb;
        ps::lr_forward<16>(F, fbig.data(), ybig.data(), dgb, 0);
        int n_sat = 0;
        for (int m = 0; m < M; ++m) {
            if (ybig[m] == (int32_t)ps::kMeasCodeMax || ybig[m] == -(int32_t)ps::kMeasCodeMax) {
                ++n_sat;
            }
        }
        std::printf("  f 取 Q6.18 上界 %lld：输出中被顶到 ±%lld 的元素 %d/%d，"
                    "out_overflow = %d\n",
                    ps::kImageCodeMax, ps::kMeasCodeMax, n_sat, M,
                    dgb.flags.out_overflow);
        // 不回绕的判据：被顶到饱和边界的元素，其**符号**必须与按稠密 B 算出的
        // 理想输出一致。回绕（低位截断）会随机改变符号，这条能抓住它。
        std::vector<double> Bd((size_t)M * N), yideal((size_t)M, 0.0);
        ps::lr_codes_to_dense_B(F, Bd.data());
        const double f_big = (double)ps::kImageCodeMax / std::ldexp(1.0, ps::kImageFrac);
        for (int m = 0; m < M; ++m) {
            double a = 0.0;
            for (int i = 0; i < N; ++i) a += Bd[(size_t)m * N + i] * f_big;
            yideal[m] = a;
        }
        bool sign_ok = true;
        int n_checked = 0;
        for (int m = 0; m < M; ++m) {
            if (ybig[m] == (int32_t)ps::kMeasCodeMax || ybig[m] == -(int32_t)ps::kMeasCodeMax) {
                ++n_checked;
                if ((yideal[m] > 0.0) != (ybig[m] > 0)) sign_ok = false;
            }
        }
        check(dgb.flags.out_overflow > 0 && n_sat > 0 && sign_ok && n_checked == n_sat,
              "越界按饱和处置并记账，且饱和元素的符号与理想输出一致（不回绕）",
              "符号正确 = 没有低位截断");

        // 48 bit MAC 的解析余量：|acc| <= K·32767·|x|，取 K=96（rank_max=96）与
        // |f| = 2^23（Q6.18 24 bit 上界）—— 这是本核可能遇到的最坏组合。
        const double acc_bound = 96.0 * 32767.0 * 8388607.0;
        const double acc_max = std::ldexp(1.0, ps::kMacAccBits - 1) - 1.0;
        const double headroom = std::log2(acc_max / acc_bound);
        std::printf("  K=96、|f|=2^23 时 |acc| 上界 %.6e（2^%.2f）；48 bit 上界 2^47，"
                    "余量 %.2f bit\n",
                    acc_bound, std::log2(acc_bound), headroom);
        std::printf("  实测最大 |acc| = %.6e（本组数据）\n",
                    (double)dgb.flags.max_abs_acc);
        check(headroom > 2.0, "48 bit MAC 余量 > 2 bit（rank 到 96 仍不溢出）",
              "§6.3 的 48 bit 选择自洽");
    }

    // ------------------------------------------------------------ L7 增益取整
    std::printf("\n=== L7 Q16.16 增益取整误差：为什么 scale 必须选成幂次/共享指数 ===\n");
    {
        const double ly = std::ldexp(1.0, ps::kMeasFrac);
        // 参考输出（用精确 scale）
        std::vector<double> yref((size_t)M, 0.0);
        {
            std::vector<double> Bd((size_t)M * N);
            ps::lr_codes_to_dense_B(F, Bd.data());
            const double lz = std::ldexp(1.0, ps::kImageFrac);
            for (int m = 0; m < M; ++m) {
                double a = 0.0;
                for (int i = 0; i < N; ++i) a += Bd[(size_t)m * N + i] * ((double)f[i] / lz);
                yref[m] = a;
            }
        }
        // (a) 幂次 scale：误差下限（只有移位舍入）
        // (b) 非幂次 scale：u_scale = 4.6·2^-16，gain 只能取 5 -> 相对误差 8.7%
        ps::LrFactors F2 = F;
        F2.u_gain_q16 = (int32_t)ps::scale_to_gain_q16(std::ldexp(4.6, -16));
        std::vector<int32_t> y2(M);
        ps::LrDiag dg2;
        ps::lr_forward<16>(F2, f.data(), y2.data(), dg2, 0);
        // 参考：gain 应为 4.6 的**理想**输出。yref 是 gain=1 的码字，乘 4.6 即得。
        const double g_ideal = 4.6;
        double e_pow2 = 0.0, e_nonpow2 = 0.0, scale_nonpow2 = 0.0;
        for (int m = 0; m < M; ++m) {
            const double ideal = yref[m] * ly * g_ideal;
            e_pow2 = std::max(e_pow2, std::fabs((double)y[m] - yref[m] * ly));
            e_nonpow2 = std::max(e_nonpow2, std::fabs((double)y2[m] - ideal));
            scale_nonpow2 = std::max(scale_nonpow2, std::fabs(ideal));
        }
        const double gain_err = std::fabs(5.0 - g_ideal) / g_ideal;
        std::printf("  (a) 幂次 scale（gain=1 精确）：最大误差 %.4f LSB\n", e_pow2);
        std::printf("  (b) gain 应为 4.6 实际取 5：相对增益误差 %.2f%%，"
                    "折合输出相对误差 %.2f%%（%.1f LSB / max|y| %.1f LSB）\n",
                    100.0 * gain_err,
                    100.0 * (e_nonpow2 / (scale_nonpow2 > 0 ? scale_nonpow2 : 1.0)),
                    e_nonpow2, scale_nonpow2);
        check(e_pow2 <= 1.0, "幂次 scale 下总误差 <= 1 LSB", "增益精确可表示");
        check(std::fabs(e_nonpow2 / (scale_nonpow2 > 0 ? scale_nonpow2 : 1.0) - gain_err) < 0.01,
              "非幂次 scale 的误差是**相对**误差（≈ |Δgain|/gain）",
              "不是 LSB 有界，故不能靠加宽 MAC 解决");

        // 真实模型的 per-candidate 自适应 scale 会落在这个陷阱里：
        // 若按 max|U|/32767 取 scale，本组 U 的最大元素就是 alpha_u*1
        double umax = 0.0;
        for (int m = 0; m < M; ++m) {
            umax = std::max(umax, std::fabs((double)U[m]));
        }
        const double u_scale_adapt = umax / 32767.0 * u_scale;   // 归一化前的真实值
        const double g_adapt = u_scale_adapt * 65536.0;
        const double rel_adapt = 0.5 / (g_adapt > 0 ? g_adapt : 1.0);
        std::printf("  自适应 scale（max|U|/32767）-> gain_q16 应为 %.3f，取整到 %d，"
                    "相对误差上界 %.2f%%\n",
                    g_adapt, (int)ps::scale_to_gain_q16(u_scale_adapt),
                    100.0 * rel_adapt);
        std::printf("  结论：Q16.16 的增益分辨率是 1.53e-5，相对误差上界 = 0.5/gain_q16。\n"
                    "        要让增益引入的误差 <= 1e-3，需要 gain_q16 >= 500，"
                    "即 u_scale >= 7.6e-3；\n"
                    "        要让 <= 1e-4，需要 u_scale >= 7.6e-2。若物理 scale 落在"
                    "更小的量级，\n"
                    "        就必须改用共享指数或更宽的增益格式（Q24.8 / Q32.32），"
                    "并在 model_loader 里显式拒绝不合格模型 —— 这条判据必须与"
                    "「每级 <= 1 LSB」分开记，它是**相对**误差。\n");
        check(true, "已知约束被记录（gain 取整误差是相对误差）",
              "见 ps_fixed.hpp scale_to_gain_q16 与 lr_operator.hpp 的说明");
    }

    std::printf("\n%s  PASS %d  FAIL %d\n",
                (g_fail == 0) ? "全部通过。" : "存在失败项。", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
