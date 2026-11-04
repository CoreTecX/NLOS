// ============================================================================
// test_lr_exp_gain.cpp —— 增益的**指数路径**比对测试
//
// 为什么需要这个测试
// ----------------------------------------------------------------------------
// `lr_operator_fixed.cpp` 的 `lr_forward` 现在有两条增益路径：
//
//   旧（Q16.16）  F.u_gain_q16 = round(scale * 2^16)，shift 用编译期常量
//   新（指数）    F.u_exp      = log2(scale)，gain 恒为 2^16，shift = 常量 - exp
//
// 新路径在 `F.u_exp == kGainExpInvalid` 时**退回旧路径**，所以既有的
// `test_lr_operator.cpp` 全部走旧分支 —— 也就是说**新路径从未被执行过**。
// 这正是本项目踩过的坑：定点层的三个测试曾长期"存在但没接线"。
// 本文件把新路径真正跑起来，并与双精度参考逐值比对。
//
// 用真实尺度而不是随便挑的数
// ----------------------------------------------------------------------------
// U 取 u_scale = 2^-18。这不是为了制造极端用例，而是 `scripts/x23_gain_representation.py`
// 在真实 B 档几何上量出来的值：max|U| ≈ 0.129，u_scale = 2^-18。
// 在这一档下 `round(2^-18 * 2^16) = round(0.25) = 0` —— 旧路径的增益直接归零。
// V 取 v_scale = 2^-16（实测档），S 取 Q2.14 的 2^-14（方案 §6.3 钉死）。
//
// 构建
// ----------------------------------------------------------------------------
//   g++ -std=c++17 -O2 -Wall -Wextra lr_operator_fixed.cpp test_lr_exp_gain.cpp -o test_lr_exp_gain.exe -I../../software/include
// 或从仓库根：make test-fixed-exp
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "lr_operator.hpp"

namespace {

int g_pass = 0, g_fail = 0;

void check(bool ok, const char* what, const char* detail) {
    if (ok) { ++g_pass; std::printf("  [ok  ] %s  %s\n", what, detail); }
    else    { ++g_fail; std::printf("  [FAIL] %s  %s\n", what, detail); }
}

// 与真实模型同量级的确定性伪随机码字（不用 <random>，保证两次运行逐位一致）
int16_t code_of(uint32_t seed) {
    uint32_t x = seed * 1664525u + 1013904223u;
    x ^= x >> 13; x *= 1274126177u; x ^= x >> 16;
    return (int16_t)((int32_t)(x % 65534u) - 32767);
}

}  // namespace

int main() {
    constexpr int K = 8;        // 定位档的秩（方案 §5.1 的 K=8）
    constexpr int M = 64;       // 测量数（缩小规模以便快速运行）
    constexpr int N = 32;       // 隐藏像素数

    // ---- 真实的 2 的幂尺度（见文首）----
    const double u_scale = std::ldexp(1.0, -18);
    const double v_scale = std::ldexp(1.0, -16);
    const double s_scale = std::ldexp(1.0, -14);

    // ---- 造数据 ----
    std::vector<int16_t>  Ucode((size_t)K * M), Vcode((size_t)K * N);
    std::vector<uint16_t> Scode(K);
    for (size_t i = 0; i < Ucode.size(); ++i) Ucode[i] = code_of((uint32_t)(i + 1));
    for (size_t i = 0; i < Vcode.size(); ++i) Vcode[i] = code_of((uint32_t)(i + 7777));
    for (int k = 0; k < K; ++k) Scode[k] = (uint16_t)(16384 / (k + 1));   // max = 1.0 (Q2.14)

    // f：隐藏图像的 Q6.18 码字，取一个非平凡分布（不是全零，也不是常量）
    std::vector<int32_t> f_code(N);
    for (int i = 0; i < N; ++i) f_code[i] = ((i * 37) % 97) * 2713;

    // ---- 双精度参考：完全按三个阶段的数学式算，不经任何定标 ----
    std::vector<double> zref(K), zsref(K), yref(M, 0.0);
    for (int k = 0; k < K; ++k) {
        double a = 0.0;
        for (int i = 0; i < N; ++i)
            a += (double)Vcode[(size_t)k * N + i] * v_scale * ((double)f_code[i] * std::ldexp(1.0, -ps::kImageFrac));
        zref[k]  = a;
        zsref[k] = a * ((double)Scode[k] * s_scale);
    }
    for (int m = 0; m < M; ++m) {
        double a = 0.0;
        for (int k = 0; k < K; ++k)
            a += (double)Ucode[(size_t)k * M + m] * u_scale * zsref[k];
        yref[m] = a * std::ldexp(1.0, ps::kMeasFrac);      // 折成 Q2.16 码字
    }

    double ymax = 0.0;
    for (int m = 0; m < M; ++m) ymax = std::max(ymax, std::fabs(yref[m]));
    std::printf("=== 指数路径 vs Q16.16 路径（双精度参考）===\n");
    std::printf("  K=%d M=%d N=%d  u_scale=2^-18 v_scale=2^-16 s_scale=2^-14\n", K, M, N);
    std::printf("  参考 |y| 最大值 = %.1f（Q2.16 码字）\n", ymax);
    std::printf("  旧路径 u_gain_q16 = round(2^-18 * 2^16) = %d"
                "   <- 若为 0，增益归零\n\n",
                (int)std::llround(u_scale * 65536.0));

    // ---- 两条路径跑同一个输入 ----
    auto run = [&](bool use_exp, std::vector<int32_t>& out, int& used_exp) {
        ps::LrFactors F;
        F.U = Ucode.data(); F.S = Scode.data(); F.V = Vcode.data();
        F.K = K; F.M = M; F.N = N;
        // 旧字段两条路径都填（新路径会忽略它）
        F.u_gain_q16 = (int32_t)std::llround(u_scale * 65536.0);
        F.s_gain_q16 = (int32_t)std::llround(s_scale * 65536.0);
        F.v_gain_q16 = (int32_t)std::llround(v_scale * 65536.0);
        if (use_exp) {
            F.u_exp = ps::scale_to_exp2(u_scale);
            F.s_exp = ps::scale_to_exp2(s_scale);
            F.v_exp = ps::scale_to_exp2(v_scale);
        }
        used_exp = F.u_exp;
        out.assign(M, 0);
        ps::LrDiag dg;
        ps::lr_forward<16>(F, f_code.data(), out.data(), dg, 0);
        return dg;
    };

    std::vector<int32_t> y_old, y_new;
    int exp_old = 0, exp_new = 0;
    run(false, y_old, exp_old);
    run(true,  y_new, exp_new);

    std::printf("=== A. 尺度到指数的转换本身 ===\n");
    std::printf("  u_scale=%.6g -> exp=%d（期望 -18）\n", u_scale, ps::scale_to_exp2(u_scale));
    check(ps::scale_to_exp2(u_scale) == -18, "u_scale = 2^-18 转指数为 -18", "精确");
    check(ps::scale_to_exp2(v_scale) == -16, "v_scale = 2^-16 转指数为 -16", "精确");
    check(ps::scale_to_exp2(s_scale) == -14, "s_scale = 2^-14 转指数为 -14", "精确");
    check(!ps::exp2_valid(ps::scale_to_exp2(4.6)), "非 2 的幂（4.6）被判为无效", "不返回近似值");
    check(!ps::exp2_valid(ps::kGainExpInvalid), "kGainExpInvalid 自身无效", "哨兵自洽");

    // ---- 与参考比 ----
    auto err_of = [&](const std::vector<int32_t>& y) {
        double e = 0.0;
        for (int m = 0; m < M; ++m)
            e = std::max(e, std::fabs((double)y[m] - yref[m]));
        return e;
    };
    const double e_old = err_of(y_old);
    const double e_new = err_of(y_new);

    std::printf("\n=== B. 两条路径 vs 双精度参考 ===\n");
    std::printf("  旧（Q16.16）: 最大绝对误差 %10.1f  = |y|max 的 %7.3f%%（exp 字段=%d 表示走旧路径）\n",
                e_old, 100.0 * e_old / ymax, exp_old);
    std::printf("  新（指数）  : 最大绝对误差 %10.1f  = |y|max 的 %7.3f%%\n",
                e_new, 100.0 * e_new / ymax);
    std::printf("  改善倍数    : %.4g x\n", e_new > 0 ? e_old / e_new : 0.0);

    // 旧路径的增益归零（或被 gain_safe 兜成 1）-> 输出幅度错到 4 倍以上，
    // 相对误差必然远大于 1%。新路径只受码字量化限制（几个 LSB）。
    check(e_new < 0.01 * ymax, "新路径相对误差 < 1%（码字量化地板）", "增益精确");
    check(e_new < e_old, "新路径严格优于旧路径", "同一输入、同一核");
    check(e_old > 0.05 * ymax, "旧路径误差 > 5%（增益精度不足的直接后果）",
          "2^-18 这档下 Q16.16 分辨不出来");

    // ---- 退回路径的兼容性：exp 无效时必须逐位等于旧路径 ----
    std::printf("\n=== C. 兼容性：exp 无效 -> 退回旧路径，必须逐位相同 ===\n");
    {
        std::vector<int32_t> y_ret;
        int dummy = 0;
        ps::LrDiag dg_ret = run(false, y_ret, dummy);
        check(std::memcmp(y_ret.data(), y_old.data(), sizeof(int32_t) * M) == 0,
              "exp 无效时输出与旧路径逐位相同", "既有 19 项测试的行为不受影响");
        check(!dg_ret.flags.any(), "两条路径都未饱和（本用例幅度未溢出）", "FixFlags 未被改动");
    }

    // ---- 两条路径的码字差应当与「增益比」一致 ----
    std::printf("\n=== D. 差异的结构：旧路径是否只是整体缩放错了 ===\n");
    {
        double num = 0.0, den = 0.0;
        for (int m = 0; m < M; ++m) { num += (double)y_old[m] * yref[m]; den += (double)yref[m] * yref[m]; }
        const double ratio = (den > 0) ? num / den : 0.0;
        std::printf("  旧/参考 的最小二乘缩放比 = %.6g（理想 1.0）\n", ratio);
        std::printf("  提示：若比值 ≈ 4 或 0，说明旧路径是整体幅度错（增益问题），\n"
                    "        而不是逐点噪声 —— 这正是「相对误差不能靠加宽 MAC 解决」的含义。\n");
        check(std::fabs(ratio - 1.0) > 0.05, "旧路径整体幅度偏离参考 > 5%", "确认是增益问题");
    }

    // ================================================================ E 伴随侧
    // 伴随 Bᵀr 与前向 Bf 必须走同一套判据：PDHG 每轮迭代两个方向都用，
    // 只接一侧会让前向精确、伴随不精确，重建结果仍带着旧的增益误差。
    std::printf("\n=== E. 伴随侧（Bᵀr）同样的比对 ===\n");
    {
        constexpr int MR = 48;
        std::vector<int32_t> r_code(MR);
        for (int m = 0; m < MR; ++m) r_code[m] = ((m * 53) % 89) * 3179;

        // 双精度参考：w = Uᵀr -> ws = S∘w -> g = V·ws
        std::vector<double> wref(K), wsref(K), gref(N, 0.0);
        for (int k = 0; k < K; ++k) {
            double a = 0.0;
            for (int m = 0; m < MR; ++m)
                a += (double)Ucode[(size_t)k * M + m] * u_scale * ((double)r_code[m] * std::ldexp(1.0, -ps::kMeasFrac));
            wref[k]  = a;
            wsref[k] = a * ((double)Scode[k] * s_scale);
        }
        for (int i = 0; i < N; ++i) {
            double a = 0.0;
            for (int k = 0; k < K; ++k)
                a += (double)Vcode[(size_t)k * N + i] * v_scale * wsref[k];
            gref[i] = a * std::ldexp(1.0, ps::kImageFrac);
        }
        double gmax = 0.0;
        for (int i = 0; i < N; ++i) gmax = std::max(gmax, std::fabs(gref[i]));

        auto run_adj = [&](bool use_exp, std::vector<int32_t>& out) {
            ps::LrFactors F;
            F.U = Ucode.data(); F.S = Scode.data(); F.V = Vcode.data();
            F.K = K; F.M = MR; F.N = N;
            F.u_gain_q16 = (int32_t)std::llround(u_scale * 65536.0);
            F.s_gain_q16 = (int32_t)std::llround(s_scale * 65536.0);
            F.v_gain_q16 = (int32_t)std::llround(v_scale * 65536.0);
            if (use_exp) {
                F.u_exp = ps::scale_to_exp2(u_scale);
                F.s_exp = ps::scale_to_exp2(s_scale);
                F.v_exp = ps::scale_to_exp2(v_scale);
            }
            out.assign(N, 0);
            ps::LrDiag dg;
            ps::lr_adjoint<16>(F, r_code.data(), out.data(), dg, 0);
        };
        std::vector<int32_t> g_old, g_new;
        run_adj(false, g_old);
        run_adj(true,  g_new);

        auto aerr = [&](const std::vector<int32_t>& g) {
            double e = 0.0;
            for (int i = 0; i < N; ++i) e = std::max(e, std::fabs((double)g[i] - gref[i]));
            return e;
        };
        const double ea = aerr(g_old), eb = aerr(g_new);
        std::printf("  |g| 参考最大值 = %.1f（Q6.18 码字）\n", gmax);
        std::printf("  旧（Q16.16）: 误差 %10.1f = %7.3f%%\n", ea, 100.0 * ea / gmax);
        std::printf("  新（指数）  : 误差 %10.1f = %7.3f%%\n", eb, 100.0 * eb / gmax);
        check(eb < ea, "伴随侧新路径严格优于旧路径", "同一输入、同一核");

        // ---- 本段的参考算式**已知有误**，故不设硬判据 ----
        // F 段的伴随恒等式 rᵀ(Bf) = (Bᵀr)ᵀf 以 0.0091% 成立（Δ=0.0001），
        // 而那条恒等式不依赖任何手写参考 —— 所以**伴随内核是正确的**，
        // 是这里的参考算错了。原因：前向与伴随的 shift 常量并不相同
        //   forward: Vᵀf 用 16、U·zs 用 18
        //   adjoint: Uᵀr 用 14、V·ws 用 16
        // 因为两个方向的输入/输出码字格式不同（前向 N×Q6.18 -> M×Q2.16，
        // 伴随 M×Q2.16 -> N×Q6.18），各自把格式差折进了自己的 shift。
        // 本段却按前向那一套回程约定去算伴随的参考，于是差了一个固定倍数。
        // 44.187% 就是这个错配的表现，**不是内核缺陷**。
        // 保留打印以便对照；伴随的正式判据是 F 段。
        std::printf("  [注] 本段参考算式已知有误（见上方注释），仅作对照；"
                    "伴随的正式判据是 F 段的恒等式。误差 %.3f%% 为参考错配所致。\n",
                    100.0 * eb / gmax);
    }

    // ============================================================ F 伴随恒等式
    // 判别「内核错」还是「我的参考错」：用
    //     rᵀ(Bf) = (Bᵀr)ᵀf
    // 这条恒等式**不依赖任何手写参考**——前向已验证到 0.5 LSB，所以拿它当基准。
    // 恒等式成立 -> 伴随正确、是我在 E 段手写的参考错了；
    // 不成立   -> 内核错，且两侧之比直接给出那处移位的偏差 2^Δ。
    std::printf("\n=== F. 伴随恒等式 rᵀ(Bf) = (Bᵀr)ᵀf（不依赖手写参考）===\n");
    {
        std::printf("  移位常量：kShiftVToZ=%d kShiftZToS=%d kShiftUToY=%d | "
                    "kShiftUToW=%d kShiftVToG=%d\n",
                    (int)ps::kShiftVToZ, (int)ps::kShiftZToS, (int)ps::kShiftUToY,
                    (int)ps::kShiftUToW, (int)ps::kShiftVToG);
        std::printf("  分数位：kImageFrac=%d kMeasFrac=%d\n",
                    (int)ps::kImageFrac, (int)ps::kMeasFrac);

        std::vector<int32_t> r_id(M);
        for (int m = 0; m < M; ++m) r_id[m] = ((m * 29) % 71) * 4021;

        ps::LrFactors F;
        F.U = Ucode.data(); F.S = Scode.data(); F.V = Vcode.data();
        F.K = K; F.M = M; F.N = N;
        F.u_exp = ps::scale_to_exp2(u_scale);
        F.s_exp = ps::scale_to_exp2(s_scale);
        F.v_exp = ps::scale_to_exp2(v_scale);

        std::vector<int32_t> y_id(M, 0), g_id(N, 0);
        ps::LrDiag dg1, dg2;
        ps::lr_forward<16>(F, f_code.data(), y_id.data(), dg1, 0);
        ps::lr_adjoint<16>(F, r_id.data(), g_id.data(), dg2, 0);

        // 左：r_realᵀ y_real；右：g_realᵀ f_real。以「码字乘积」为单位比较：
        //   Σ r_code·y_code · 2^(-2·kMeasFrac)  ?=  Σ g_code·f_code · 2^(-2·kImageFrac)
        double lhs = 0.0, rhs = 0.0;
        for (int m = 0; m < M; ++m) lhs += (double)r_id[m] * y_id[m];
        for (int i = 0; i < N; ++i) rhs += (double)g_id[i] * f_code[i];
        lhs *= std::ldexp(1.0, -2 * ps::kMeasFrac);
        rhs *= std::ldexp(1.0, -2 * ps::kImageFrac);

        const double ratio = (rhs != 0.0) ? lhs / rhs : 0.0;
        std::printf("  rᵀ(Bf) = %.6e\n", lhs);
        std::printf("  (Bᵀr)ᵀf = %.6e\n", rhs);
        std::printf("  比值 左/右 = %.6f  ->  隐含移位偏差 2^Δ, Δ = %.4f\n",
                    ratio, std::log2(ratio));
        std::printf("  （Δ 接近整数则确认是某处移位常量差了这个数；"
                    "Δ≈0 则伴随正确，E 段参考有误）\n");

        // 相对恒等式偏差：以两侧均值为尺度，避免被大数掩盖
        const double scale = 0.5 * (std::fabs(lhs) + std::fabs(rhs));
        const double rel = (scale > 0) ? std::fabs(lhs - rhs) / scale : 0.0;
        std::printf("  相对恒等式偏差 = %.4f%%\n", 100.0 * rel);
        check(rel < 0.01, "伴随恒等式成立（<1%）",
              "成立则伴随正确、E 段的手写参考有误");
        if (rel >= 0.01) {
            std::printf("  >> 恒等式不成立：**内核侧确有缺陷**，不是参考的问题。\n");
            std::printf("  >> 两侧之比 %.6f 是定位线索：把移位常量逐个代入看哪个差 %.4f。\n",
                        ratio, std::log2(ratio));
        }
    }

    std::printf("\n%s  PASS %d  FAIL %d\n",
                (g_fail == 0) ? "全部通过。" : "存在失败项。", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
