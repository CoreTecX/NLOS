// ============================================================================
// X36 —— 主机侧定点 ADMM 求解器（论文链重建用）
//
// 目的：在**核的定点算术约定**下实现 x15g 定案的 ADMM（外层固定次 + 定内层
//       CG + 逐像素 TV 收缩 + 非负投影），验证定点能否达到双精度 ADMM 的重建
//       质量。本文件只是驱动/求解器，**不改 src/fpga/hls/** 任何源码**，只
//       include 并复用其定点原语：
//         ps::mul_shift_round / narrow_format / saturate_i64 / isqrt_u64 / FixFlags
//         ps::lr_forward / ps::lr_adjoint     （Bf、Bᵀr，Q1.15 码字 + 2^exp 增益）
//         ps::pdhg_gradient_i32 / ps::pdhg_divergence_i32  （TV 原语，同 recon.hpp 约定）
//
// 与 PDHG 核的关键差别（本轮的定量结论，见 sim/results/x36_admm_fixed.md）
//   1) ADMM 的 f-步正规方程算子要用 BᵀB。核的 lr_forward 把测量域收窄到 Q2.16
//      （±2.0），而 CG 的搜索方向远大于解，直接 lr_forward→lr_adjoint 会饱和
//      （实测 r(gt) 塌到 +0.19）。故此处实现**融合正规方程算子**：
//          BᵀB p = V ( S ∘ ( S ∘ (Vᵀ p) ) )      （等价 V·S²·Vᵀ，用两次 S 免新字段）
//   2) 该算子的**秩域中间量**需要比 Q6.18 更宽的收窄上限（WIDE_LIM），否则 CG
//      方向在秩域被裁（实测 +0.31 -> +0.35）。
//   3) 内层 CG 的标量（α、β）用整数比 (num<<16)/den 的**就近取整**；状态用
//      int64 码字，量化点全部走 ps 的舍入+饱和原语。
//
// 输入：case 二进制（build/scripts/x36_admm_fixed.py 写出）
// 输出：stdout 诊断 + 结果二进制
// ============================================================================

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "pdhg_core.hpp"      // ps::LrFactors / FixFlags / TV 原语 / isqrt / lr_*

namespace {

constexpr long long kWideLim = (1LL << 30) - 1;   // 秩域中间量收窄上限（Q6.18 之外放宽）

struct Case {
    int32_t K = 0, M = 0, N = 0, n_rows = 0, n_cols = 0;
    int32_t n_outer = 0, n_cg = 0, rho_q16 = 0, lambda_q16 = 0;
    int32_t u_exp = 0, v_exp = 0, s_exp = 0, wide_lim = 0, mode = 0;
    std::vector<int16_t> U;
    std::vector<uint16_t> S;
    std::vector<int16_t> V;
    std::vector<int32_t> d_code;
};

template <typename T>
bool rd(FILE* f, std::vector<T>& v, std::size_t n) {
    v.resize(n);
    return std::fread(v.data(), sizeof(T), n, f) == n;
}

bool read_case(const char* path, Case& c) {
    FILE* f = std::fopen(path, "rb");
    if (!f) { std::printf("cannot open %s\n", path); return false; }
    char magic[4];
    int32_t h[16];
    bool ok = (std::fread(magic, 1, 4, f) == 4) && (std::memcmp(magic, "X36C", 4) == 0) &&
              (std::fread(h, sizeof(int32_t), 16, f) == 16);
    if (!ok) { std::printf("bad header\n"); std::fclose(f); return false; }
    c.K = h[0]; c.M = h[1]; c.N = h[2]; c.n_rows = h[3]; c.n_cols = h[4];
    c.n_outer = h[5]; c.n_cg = h[6]; c.rho_q16 = h[7]; c.lambda_q16 = h[8];
    c.u_exp = h[9]; c.v_exp = h[10]; c.s_exp = h[11];
    c.wide_lim = h[12] ? h[12] : (int32_t)kWideLim;
    c.mode = h[13];   // 0 = fused normal-equation operator, 1 = lr round-trip
    ok = rd<int16_t>(f, c.U, (std::size_t)c.K * c.M) &&
         rd<uint16_t>(f, c.S, (std::size_t)c.K) &&
         rd<int16_t>(f, c.V, (std::size_t)c.K * c.N) &&
         rd<int32_t>(f, c.d_code, (std::size_t)c.M);
    std::fclose(f);
    return ok;
}

// 收益：把 int32 中间量收窄到一个显式上限（饱和 + 记账），与 pdhg 的 narrow_format 同风格。
inline int32_t narrow_lim(long long v, long long lim, ps::FixFlags& fl) {
    if (v > lim || v < -lim) {
        fl.out_overflow++;
        fl.sat_events++;
        return (int32_t)((v > 0) ? lim : -lim);
    }
    return (int32_t)v;
}

// 就近取整的整数比：round(num / den * 2^16)，用 __int128 避免中间溢出。
inline long long ratio_q16_near(long long num, long long den) {
    if (den == 0) return 0;
    __int128 v = ((__int128)num << 16);
    if (den > 0) v += (den >> 1); else v -= ((-den) >> 1);
    return (long long)(v / den);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) { std::printf("usage: %s <case.bin> <result.bin>\n", argv[0]); return 2; }
    Case c;
    if (!read_case(argv[1], c)) return 2;
    const int K = c.K, M = c.M, N = c.N;
    const long long wide = c.wide_lim;
    std::printf("=== X36 host-side fixed-point ADMM (operator=%s) ===\n",
                c.mode == 1 ? "lr_forward->lr_adjoint round-trip" : "fused V S^2 V^T");
    std::printf("K=%d M=%d N=%d shape=(%d,%d) n_outer=%d n_cg=%d rho_q16=%d lambda_q16=%d wide=%lld mode=%d\n",
                K, M, N, c.n_rows, c.n_cols, c.n_outer, c.n_cg, c.rho_q16, c.lambda_q16, wide, c.mode);

    ps::LrFactors F;
    F.U = c.U.data(); F.S = c.S.data(); F.V = c.V.data();
    F.K = K; F.M = M; F.N = N;
    F.u_gain_q16 = 0; F.s_gain_q16 = 0; F.v_gain_q16 = 0;
    F.u_exp = c.u_exp; F.v_exp = c.v_exp; F.s_exp = c.s_exp;

    ps::FixFlags fl; ps::fix_flags_reset(fl);
    const ps::ReconShape sh = {c.n_rows, c.n_cols};
    const int Vsh_v = 16 - c.v_exp;      // V 阶段移位（指数路径：gain=2^16, shift=16-exp）
    const int Vsh_s = 16 - c.s_exp;
    const long long GF = 1LL << 16;
    const long long lam = c.lambda_q16;
    const long long rho = c.rho_q16;
    const long long th_code = (lam << ps::kImageFrac) / (rho > 0 ? rho : 1);

    // ---- rhs0 = Bᵀ d（复用核的伴随；d 是 O(1) 测量，不会饱和）----
    std::vector<int32_t> rhs0(N, 0);
    std::vector<double> zeros_d(M, 0.0);
    {
        ps::LrDiag dg;
        ps::lr_adjoint<16>(F, c.d_code.data(), rhs0.data(), dg, 0);
        ps::fix_flags_merge(fl, dg.flags);
    }

    // ---- 融合正规方程算子 BᵀB p = V S² Vᵀ p（秩域中间量放宽到 wide）----
    std::vector<int64_t> tmpK(K, 0);
    auto btb_fused = [&](const int32_t* p, int32_t* out) {
        for (int k = 0; k < K; ++k) {
            int64_t acc = 0;
            const int16_t* vr = c.V.data() + (std::size_t)k * N;
            for (int i = 0; i < N; ++i) acc += (int64_t)vr[i] * (int64_t)p[i];
            tmpK[k] = ps::mul_shift_round(acc, GF, Vsh_v);
            tmpK[k] = narrow_lim(tmpK[k], wide, fl);
        }
        for (int k = 0; k < K; ++k) {
            tmpK[k] = ps::mul_shift_round(tmpK[k] * (int64_t)c.S[k], GF, Vsh_s);
            tmpK[k] = narrow_lim(tmpK[k], wide, fl);
        }
        for (int k = 0; k < K; ++k) {
            tmpK[k] = ps::mul_shift_round(tmpK[k] * (int64_t)c.S[k], GF, Vsh_s);
            tmpK[k] = narrow_lim(tmpK[k], wide, fl);
        }
        for (int i = 0; i < N; ++i) {
            int64_t acc = 0;
            for (int k = 0; k < K; ++k) {
                const int16_t* vr = c.V.data() + (std::size_t)k * N;
                acc += (int64_t)vr[i] * tmpK[k];
            }
            out[i] = narrow_lim(ps::mul_shift_round(acc, GF, Vsh_v), ps::kImageCodeMax, fl);
        }
    };

    // ---- 对照：核现有的 lr_forward→lr_adjoint 拼 BᵀB（测量域 Q2.16 往返）----
    // lr_forward 把输出收窄到 Q2.16（±2.0）；CG 搜索方向远大于解时 y 被裁，
    // 该算子即「不用融合形式、走测量域往返」的落地对照。mode=1 时启用。
    std::vector<int32_t> ybuf(M, 0);
    auto btb_lr = [&](const int32_t* p, int32_t* out) {
        ps::LrDiag dg;
        ps::lr_forward<16>(F, p, ybuf.data(), dg, 0);
        ps::fix_flags_merge(fl, dg.flags);
        ps::lr_adjoint<16>(F, ybuf.data(), out, dg, 0);
        ps::fix_flags_merge(fl, dg.flags);
    };
    auto btb = [&](const int32_t* p, int32_t* out) {
        if (c.mode == 1) btb_lr(p, out); else btb_fused(p, out);
    };

    // ---- op(p) = BᵀB p + rho·∇ᵀ∇p（∇ᵀ∇p = −div(grad p)，用核的 TV 原语）----
    std::vector<int32_t> gbuf(2 * N, 0), dbuf(N, 0);
    auto op = [&](const std::vector<int32_t>& p, std::vector<int32_t>& out) {
        btb(p.data(), out.data());
        ps::pdhg_gradient_i32(p.data(), sh, gbuf.data(), fl);
        ps::pdhg_divergence_i32(gbuf.data(), sh, dbuf.data(), fl);
        for (int i = 0; i < N; ++i) {
            const long long L = -(long long)dbuf[i];          // ∇ᵀ∇p
            const long long rl = ps::mul_shift_round(L, rho, ps::kGainFrac);
            out[i] = narrow_lim((long long)out[i] + rl, ps::kImageCodeMax, fl);
        }
    };

    // ---- ADMM 主循环 ----
    std::vector<int32_t> z(N, 0), x(N, 0), r(N, 0), p(N, 0), Ap(N, 0), rhs(N, 0);
    std::vector<int32_t> gS(2 * N, 0), qS(2 * N, 0), sS(2 * N, 0), gnS(2 * N, 0);
    std::vector<int32_t> gminusq(2 * N, 0);
    std::vector<double> res_sq_h(c.n_outer, 0.0), obj_h(c.n_outer, 0.0),
        split_h(c.n_outer, 0.0), zmax_h(c.n_outer, 0.0);
    std::vector<double> cg_h(c.n_cg, 0.0);
    double d_sq = 0.0;
    for (int m = 0; m < M; ++m) d_sq += (double)c.d_code[m] * (double)c.d_code[m];

    long long cg_iters_total = 0, cg_breaks = 0;
    for (int it = 0; it < c.n_outer; ++it) {
        // rhs = rhs0 − rho·div(gS − qS)   [div = −∇ᵀ]
        for (int i = 0; i < 2 * N; ++i) gminusq[i] = (int32_t)((long long)gS[i] - (long long)qS[i]);
        ps::pdhg_divergence_i32(gminusq.data(), sh, dbuf.data(), fl);
        for (int i = 0; i < N; ++i) {
            const long long t = ps::mul_shift_round((long long)dbuf[i], rho, ps::kGainFrac);
            rhs[i] = narrow_lim((long long)rhs0[i] - t, ps::kImageCodeMax, fl);
        }
        // ---- 内层 CG ----
        for (int i = 0; i < N; ++i) x[i] = z[i];
        op(x, Ap);
        for (int i = 0; i < N; ++i) r[i] = narrow_lim((long long)rhs[i] - (long long)Ap[i], ps::kImageCodeMax, fl);
        for (int i = 0; i < N; ++i) p[i] = r[i];
        long long rs = 0;
        for (int i = 0; i < N; ++i) rs += (long long)r[i] * (long long)r[i];
        int used = 0;
        for (int k = 0; k < c.n_cg; ++k) {
            op(p, Ap);
            long long den = 0;
            for (int i = 0; i < N; ++i) den += (long long)p[i] * (long long)Ap[i];
            if (den <= 0) { cg_breaks++; break; }
            const long long a = ratio_q16_near(rs, den);
            for (int i = 0; i < N; ++i) {
                const long long st = ps::mul_shift_round((long long)p[i], a, ps::kGainFrac);
                x[i] = narrow_lim((long long)x[i] + st, ps::kImageCodeMax, fl);
                const long long sr = ps::mul_shift_round((long long)Ap[i], a, ps::kGainFrac);
                r[i] = narrow_lim((long long)r[i] - sr, ps::kImageCodeMax, fl);
            }
            long long rsn = 0;
            for (int i = 0; i < N; ++i) rsn += (long long)r[i] * (long long)r[i];
            if (it == 0) cg_h[k] = (d_sq > 0) ? std::sqrt((double)rsn / d_sq) : 0.0;
            const long long b = (rs > 0) ? ratio_q16_near(rsn, rs) : 0;
            for (int i = 0; i < N; ++i) {
                const long long sp = ps::mul_shift_round((long long)p[i], b, ps::kGainFrac);
                p[i] = narrow_lim((long long)r[i] + sp, ps::kImageCodeMax, fl);
            }
            rs = rsn; used++;
        }
        cg_iters_total += used;
        // 非负投影
        for (int i = 0; i < N; ++i) z[i] = (x[i] > 0) ? x[i] : 0;
        // ---- TV 收缩 + 对偶更新 ----
        ps::pdhg_gradient_i32(z.data(), sh, gS.data(), fl);
        for (int i = 0; i < 2 * N; ++i) sS[i] = narrow_lim((long long)gS[i] + (long long)qS[i], ps::kImageCodeMax, fl);
        for (int i = 0; i < N; ++i) {
            const uint64_t a0 = (uint64_t)((long long)sS[i] * (long long)sS[i]);
            const uint64_t a1 = (uint64_t)((long long)sS[N + i] * (long long)sS[N + i]);
            const long long nrm = (long long)ps::isqrt_u64(a0 + a1);
            long long sc = 0;
            if (nrm > th_code) sc = ((nrm - th_code) << 16) / nrm;
            gnS[i] = narrow_lim(ps::mul_shift_round((long long)sS[i], sc, ps::kGainFrac), ps::kImageCodeMax, fl);
            gnS[N + i] = narrow_lim(ps::mul_shift_round((long long)sS[N + i], sc, ps::kGainFrac), ps::kImageCodeMax, fl);
        }
        for (int i = 0; i < 2 * N; ++i) {
            const long long t = (long long)gnS[i];
            qS[i] = narrow_lim((long long)qS[i] + (long long)gS[i] - t, ps::kImageCodeMax, fl);
        }
        // 诊断要用「∇z − split」；gS 即将被 split 覆盖，故先算分裂残差
        double sp = 0.0;
        for (int i = 0; i < 2 * N; ++i) {
            const long long dd = (long long)gS[i] - (long long)gnS[i];
            sp += (double)dd * dd;
        }
        for (int i = 0; i < 2 * N; ++i) gS[i] = gnS[i];
        // ---- 诊断：残差、目标、分裂残差 ----
        {
            ps::LrDiag dg;
            std::vector<int32_t> y(M, 0);
            ps::lr_forward<16>(F, z.data(), y.data(), dg, 0);
            ps::fix_flags_merge(fl, dg.flags);
            double rsq = 0.0, tv = 0.0, zmx = 0.0;
            for (int m = 0; m < M; ++m) {
                const double rr = (double)y[m] - (double)c.d_code[m];
                rsq += rr * rr;
            }
            for (int i = 0; i < N; ++i) {
                const uint64_t a0 = (uint64_t)((long long)gnS[i] * (long long)gnS[i]);
                const uint64_t a1 = (uint64_t)((long long)gnS[N + i] * (long long)gnS[N + i]);
                tv += (double)ps::isqrt_u64(a0 + a1);
                const double az = std::fabs((double)z[i]);
                if (az > zmx) zmx = az;
            }
            res_sq_h[it] = rsq;
            obj_h[it] = 0.5 * rsq / 4294967296.0 + (lam / 65536.0) * (tv / 262144.0);
            split_h[it] = (d_sq > 0) ? std::sqrt(sp / d_sq) : 0.0;
            zmax_h[it] = zmx / 262144.0;
        }
    }

    std::printf("[A fixed] n_outer=%d cg_iters=%lld cg_breaks=%lld res_sq=%.6g d_sq=%.6g\n",
                c.n_outer, cg_iters_total, cg_breaks, res_sq_h[c.n_outer - 1], d_sq);
    std::printf("[flags] sat=%d mac_ovf=%d out_ovf=%d score_ovf=%d gain_clamp=%d max_abs_acc=%lld\n",
                fl.sat_events, fl.mac_overflow, fl.out_overflow, fl.score_overflow,
                fl.gain_clamped, fl.max_abs_acc);

    FILE* fo = std::fopen(argv[2], "wb");
    if (!fo) { std::printf("cannot write %s\n", argv[2]); return 2; }
    std::fwrite("X36R", 1, 4, fo);
    const int32_t h2[8] = {N, c.n_outer, fl.sat_events, fl.mac_overflow, fl.out_overflow,
                           fl.score_overflow, fl.gain_clamped, (int32_t)cg_breaks};
    std::fwrite(h2, sizeof(int32_t), 8, fo);
    std::fwrite(res_sq_h.data(), sizeof(double), res_sq_h.size(), fo);
    std::fwrite(obj_h.data(), sizeof(double), obj_h.size(), fo);
    std::fwrite(split_h.data(), sizeof(double), split_h.size(), fo);
    std::fwrite(zmax_h.data(), sizeof(double), zmax_h.size(), fo);
    std::fwrite(cg_h.data(), sizeof(double), cg_h.size(), fo);
    std::fwrite(z.data(), sizeof(int32_t), N, fo);
    std::fclose(fo);
    std::printf("[out] %s\n", argv[2]);
    return 0;
}
