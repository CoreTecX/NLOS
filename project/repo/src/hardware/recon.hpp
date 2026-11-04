// ============================================================================
// PenumbraSight M5 重建核 · HLS 风格 C++
//
// 模块编号口径（以 X1 接口冻结为准）
// ---------------------------------------------------------------------------
//   M1 预处理核 / M2 几何算核 / M3 算子核 / M4 定位核 / M5 重建核。
//   W 系列规划那张表把 「M4 当重建核、M5 当主机端到端」，已停用。
//
// 设计目标：同一份源码在两个环境下都能工作
//   g++       : 主机侧功能验证（不需要 Vivado）
//   Vitis HLS : 综合（定义 __SYNTHESIS__）
//
// 本文件覆盖 X1 接口冻结第四节 M5 的**算子层**与**对外边界**。
//
// 边界已冻结、内部未冻结（X1 原话）
// ---------------------------------------------------------------------------
//   M5 的对外边界是「吃 A 算子 + λ + 迭代上限，吐出 u[N_SCENE]」，这一层现在
//   就可以写；**内部迭代算法必须等 X15 定案**。X15 的结论是「对比被 A 的误差
//   污染，必须在 X4 修正 A 之后重跑」，且 W 系列已否证 PDHG（无噪合成数据上
//   8000 次迭代只到 r=0.843，而精确最小二乘是 1.000000）。
//
//   因此本文件的做法是：
//     1. 把**所有候选求解器都要用的原语**写出来并验证 —— 梯度、散度、
//        TV 对偶投影、A_d 与 A_dᵀ 的图像布局应用、数据项梯度。
//        这些与「最后选哪个求解器」无关，不会因 X15 改线而返工。
//     2. 把 `m5_reconstruct` 的**签名冻结**下来，函数体明确返回
//        kReconSolverUndecided，并用注释写清两个候选各自的填充位置。
//        X15 定案后只改这个函数体，边界与全部原语不动。
//
// 为什么这一步现在做不浪费
// ---------------------------------------------------------------------------
//   X15 的两个候选（带回溯线搜索的 FISTA、CG 内迭代 + TV 分裂）都是
//   「A_d 应用 + 梯度/散度 + 逐像素投影」这三样东西的组合。线搜索要多次重算
//   A_d@x，CG 要 A_dᵀr，TV 分裂要逐像素投影与收缩。本文件的原语是它们的交集。
//
// 与 Python 参考的对应（reconstruction.py）
// ---------------------------------------------------------------------------
//   tv_gradient        <-> reconstruction.gradient
//   tv_divergence      <-> reconstruction.divergence
//   tv_project_l2_ball <-> reconstruction.project_l2_ball
//   tv_isotropic_norm  <-> reconstruction.tv_pdhg 内部的目标函数 TV 项
//   m5_apply_Ad        <-> localization.apply_row_difference_A(A) @ f
//   m5_apply_AdT       <-> localization.apply_row_difference_A(A).T @ r
//
// 布局约定（错了不会报错，只会整体错位）
// ---------------------------------------------------------------------------
//   隐藏场景 (H, W) = (29, 36)，**行 = z、列 = x**，扁平索引 = z*W + x。
//   导出脚本已用「mon_x 只随列变、mon_z 只随行变」逐值核对过这一条。
//   梯度场按 numpy (2,H,W) 展平：g[0*N + i] 为 z 方向分量，g[N + i] 为 x 方向。
//
//   边界为 Neumann：边缘差分为 0（reconstruction.py 用切片隐式实现，本文件
//   显式跳过 i = H-1 与 j = W-1）。
// ============================================================================

#ifndef PENUMBRASIGHT_RECON_HPP
#define PENUMBRASIGHT_RECON_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "geometry.hpp"
#include "transport.hpp"

namespace ps {

// ---------------------------------------------------------------- 图像布局

struct ReconShape {
    int n_rows;   // = 29，行索引对应 z
    int n_cols;   // = 36，列索引对应 x
};

inline int recon_n(const ReconShape& s) { return s.n_rows * s.n_cols; }

// ---------------------------------------------------------------- TV 微分算子

// g = gradient(f)，Neumann 边界。g 长度 2*N，布局见文首。
inline void tv_gradient(const double* f, const ReconShape& s, double* g) {
#pragma HLS INLINE off
    const int H = s.n_rows, W = s.n_cols;
    const int N = H * W;
    for (int i = 0; i < 2 * N; ++i) g[i] = 0.0;

    double* gz = g;          // 行方向（z）分量
    double* gx = g + N;      // 列方向（x）分量
    for (int i = 0; i + 1 < H; ++i) {
#pragma HLS PIPELINE II = 1
        const double* cur = f + (std::size_t)i * W;
        const double* nxt = f + (std::size_t)(i + 1) * W;
        double* dst = gz + (std::size_t)i * W;
        for (int j = 0; j < W; ++j) dst[j] = nxt[j] - cur[j];
    }
    for (int i = 0; i < H; ++i) {
#pragma HLS PIPELINE II = 1
        const double* cur = f + (std::size_t)i * W;
        double* dst = gx + (std::size_t)i * W;
        for (int j = 0; j + 1 < W; ++j) dst[j] = cur[j + 1] - cur[j];
    }
}

// out = divergence(q) = -grad^T(q)（负伴随），Neumann 边界。
// 满足 <grad f, q> = <f, divergence(q)> * (-1)，见 X1 契约脚本 D 组。
inline void tv_divergence(const double* q, const ReconShape& s, double* out) {
#pragma HLS INLINE off
    const int H = s.n_rows, W = s.n_cols;
    const int N = H * W;
    const double* qz = q;
    const double* qx = q + N;

    for (int i = 0; i < N; ++i) out[i] = 0.0;

    // out[:-1, :] -= qz[:-1, :] ; out[1:, :] += qz[:-1, :]
    for (int i = 0; i + 1 < H; ++i) {
#pragma HLS PIPELINE II = 1
        const double* src = qz + (std::size_t)i * W;
        double* up = out + (std::size_t)i * W;
        double* dn = out + (std::size_t)(i + 1) * W;
        for (int j = 0; j < W; ++j) {
            up[j] -= src[j];
            dn[j] += src[j];
        }
    }
    // out[:, :-1] -= qx[:, :-1] ; out[:, 1:] += qx[:, :-1]
    for (int i = 0; i < H; ++i) {
#pragma HLS PIPELINE II = 1
        const double* src = qx + (std::size_t)i * W;
        double* row = out + (std::size_t)i * W;
        for (int j = 0; j + 1 < W; ++j) {
            row[j] -= src[j];
            row[j + 1] += src[j];
        }
    }
    for (int i = 0; i < N; ++i) out[i] = -out[i];
}

// 逐像素把 (qz, qx) 投影到半径 radius 的 L2 球（各向同性 TV 的对偶投影）。
// 就地修改 q。这是 TV 邻近算子内层（Chambolle 类算法）的核心一步。
inline void tv_project_l2_ball(double* q, const ReconShape& s, double radius) {
#pragma HLS PIPELINE II = 1
    const int N = recon_n(s);
    double* qz = q;
    double* qx = q + N;
    for (int i = 0; i < N; ++i) {
        const double nrm = std::sqrt(qz[i] * qz[i] + qx[i] * qx[i]);
        if (nrm > radius) {
            const double sc = radius / nrm;
            qz[i] *= sc;
            qx[i] *= sc;
        }
    }
}

// 各向同性 TV 值 sum sqrt(gz^2 + gx^2)，用于目标函数记录（不上板判据）。
// gwork 为长度 2*N 的调用方暂存（G7：函数内不许有隐藏分配，故不用 static 数组）。
inline double tv_isotropic_norm(const double* f, const ReconShape& s,
                                double* gwork) {
#pragma HLS INLINE off
    const int N = recon_n(s);
    tv_gradient(f, s, gwork);
    double acc = 0.0;
    for (int i = 0; i < N; ++i) {
#pragma HLS PIPELINE II = 1
        acc += std::sqrt(gwork[i] * gwork[i] + gwork[N + i] * gwork[N + i]);
    }
    return acc;
}

// ---------------------------------------------------------------- M5 几何束

// 重建核需要的全部几何量。rect 由 M2 产出（与定位核同一份实现）。
struct M5Geometry {
    int n_scene;
    const OccRect* rect;
    const double* wall_x;
    const double* wall_z;
    const double* mon_x;
    const double* mon_z;
    TransportParams tp;
};

// 暂存区。G7：综合代码里不许用 new/delete，全部由调用方传入。
//   row_prev / row_cur : 各 wall_n   个 double
//   col                : (wall_n-1)*wall_n 个 double
//   tmp                : (wall_n-1)*wall_n 个 double（仅 m5_data_term_grad 需要）
struct M5Scratch {
    double* row_prev;
    double* row_cur;
    double* col;
    double* tmp;
};

// ---------------------------------------------------------------- 算子应用

// y = A_d @ f        y 长度 (n-1)*n，索引 a*n + b
inline void m5_apply_Ad(const M5Geometry& G, const double* f, double* y,
                        const M5Scratch& S) {
#pragma HLS INLINE off
    apply_Ad(G.n_scene, G.rect, G.wall_x, G.wall_z, G.mon_x, G.mon_z, G.tp,
             f, y, S.row_prev, S.row_cur, S.col);
}

// g = A_d^T @ r      r 长度 (n-1)*n，g 长度 n_scene
inline void m5_apply_AdT(const M5Geometry& G, const double* r, double* g,
                         const M5Scratch& S) {
#pragma HLS INLINE off
    apply_AdT(G.n_scene, G.rect, G.wall_x, G.wall_z, G.mon_x, G.mon_z, G.tp,
              r, g, S.row_prev, S.row_cur, S.col);
}

// out = A_d^T (A_d f - yd)：**数据项的梯度**，每个候选求解器都要用。
//   线搜索 FISTA 用它做下降方向；CG 内迭代用它做右端项；TV 分裂用它做 u 步。
// 需要 S.tmp（长度 (n-1)*n）。
inline void m5_data_term_grad(const M5Geometry& G, const double* f,
                              const double* yd, double* out,
                              const M5Scratch& S) {
#pragma HLS INLINE off
    m5_apply_Ad(G, f, S.tmp, S);
    const std::size_t Md = (std::size_t)(G.tp.wall_n - 1) * G.tp.wall_n;
    for (std::size_t k = 0; k < Md; ++k) {
#pragma HLS PIPELINE II = 1
        S.tmp[k] -= yd[k];                 // 残差 r = A_d f - yd
    }
    m5_apply_AdT(G, S.tmp, out, S);
}

// ============================================================================
// 冻结的对外边界（X1 接口冻结第四节 M5）
//
// 2026-09-25 更新：**求解器已由 X15g 定案**（此前留空是因为 X15 的对比被污染）。
// X15g 用自洽合成数据把「本征病态」与「模型误差」解耦，结论见
// sim/results/x15g_solver_tolerance.md。要点：
//
//   * δ=0（A 完全精确）时 ADMM（内层 CG 15 或 60）与论文线搜索 FISTA 都能复原，
//     sm_err 分别是 2.49e-03 与 3.29e-04；PDHG 直接溢出。
//   * 真实数据上 ADMM 的 sm_err 0.899 / r +0.3749，**两项都是候选里最好的**。
//   * **ρ 的标度决定成败**：ρ = 0.1·mean(diag(A_dᵀA_d)) 给 2.49e-03，
//     ρ = 1× 给 1.27e-01，ρ = 10× 给 3.61e-01。此前所有 ADMM 对比都没控制住它。
//   * 选 ADMM 而非 FISTA 的理由是**可定点化**：整条流水是固定次数 CG + 逐像素闭式
//     收缩，无数据依赖控制流；FISTA 的回溯线搜索要按数据反复重算 A_d@x。
//
// 因此边界由「吃 A 算子」改为「吃 Gram」——ADMM 迭代内只需要 A_dᵀA_d（上三角）与
// A_dᵀy，正是 M3 的 accum_gram 产出（X1 原文即写「吃 Gram 累加器（或 A 算子）」）。
// 这样与 X16 的「预存后迭代内不再访问 A」直接拼上，A 完全不进迭代。
//
// 本实现是**双精度参考版**，用于主机侧与 Python 逐值对齐。定点化（位宽、Q 格式、
// 除法）是 X18 的任务，判据是「定点轨迹与浮点一致」。
// ============================================================================

// ADMM 的输入：Gram 上三角 + A_dᵀy。
//   ata_tri 为行优先上三角，(i,j) i<=j 的偏移见 tri_offset()，长度 n(n+1)/2
struct M5Gram {
    int n_scene;
    const double* ata_tri;
    const double* aty;
};

// 上三角行优先偏移。i>j 时与 (j,i) 同一格（对称）。
inline std::size_t tri_offset(int i, int j, int n) {
    if (i > j) { const int t = i; i = j; j = t; }
    return (std::size_t)i * n - (std::size_t)i * (i - 1) / 2 + (std::size_t)(j - i);
}

// y = A_dᵀA_d @ x，直接从上三角取，不物化整矩阵。
inline void m5_ata_mul(const M5Gram& G, const double* x, double* y) {
#pragma HLS INLINE off
    const int n = G.n_scene;
    for (int i = 0; i < n; ++i) {
        double acc = 0.0;
        for (int j = 0; j < n; ++j) {
#pragma HLS PIPELINE II = 1
            acc += G.ata_tri[tri_offset(i, j, n)] * x[j];
        }
        y[i] = acc;
    }
}

// 迭代暂存（G7：全部由调用方传入，函数内不许有隐藏分配）。
//   u, rhs, cg_x, cg_r, cg_p, cg_ap : 各 n_scene
//   d, q, g, s                      : 各 2*n_scene（按 (2,H,W) 展平）
struct M5Work {
    double* u;
    double* rhs;
    double* cg_x;
    double* cg_r;
    double* cg_p;
    double* cg_ap;
    double* d;
    double* q;
    double* g;
    double* s;
};

// ---------------------------------------------------------------- 边界结构

enum ReconStatus {
    kReconOk              = 0,
    kReconSolverUndecided = 1,   // 已废弃：X15g 已定案，本文件不再返回
    kReconBadArgs         = 2
};

struct ReconParams {
    double lam;        // TV 正则化强度；定案值 160000（= REG_R / CSCALE_R）
    double rho;        // ADMM 罚参数；**必须按 0.1·mean(diag(A_dᵀA_d)) 定标**，
                       // 见 m5_default_rho()。这个标度决定成败
    int    n_outer;    // 外层次数；定案值 400（固定）
    int    n_cg;       // 内层 CG 次数；定案值 15（15 与 60 实测无差别）
};

struct ReconOut {
    double* u;         // 长度 n_scene，重建结果
    int     n_rec;     // 实际外层次数
    int     status;
};

// ρ 的定标规则。X15g 实测：0.1× 给 sm_err 2.49e-03，1× 给 1.27e-01，
// 10× 给 3.61e-01。此前所有 ADMM 对比都没控制住这个标度。
inline double m5_default_rho(const M5Gram& G) {
#pragma HLS INLINE off
    const int n = G.n_scene;
    double tr = 0.0;
    for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II = 1
        tr += G.ata_tri[tri_offset(i, i, n)];
    }
    return 0.1 * (tr / (double)n);
}

// 算 op(u) = A_dᵀA_d u + rho·(−div(grad u))，写入 dst。
inline void m5_admm_op(const M5Gram& G, const ReconShape& sh, double rho,
                       const double* u, double* dst, M5Work& W) {
#pragma HLS INLINE off
    m5_ata_mul(G, u, dst);
    tv_gradient(u, sh, W.g);
    tv_divergence(W.g, sh, W.s);          // tv_divergence = -gradᵀ
    const int N = recon_n(sh);
    for (int i = 0; i < N; ++i) {
#pragma HLS PIPELINE II = 1
        dst[i] += rho * (-W.s[i]);        // -div(grad u) = +gradᵀ grad u
    }
}

inline int m5_reconstruct(const M5Gram& G, const ReconShape& shape,
                          const ReconParams& P, M5Work& W, ReconOut& out) {
#pragma HLS INLINE off
    const int n = G.n_scene;
    const int N = recon_n(shape);
    if (n <= 0 || N != n || !G.ata_tri || !G.aty || !W.u || !out.u) {
        out.status = kReconBadArgs;
        out.n_rec = 0;
        return out.status;
    }
    if (P.rho <= 0.0) {
        out.status = kReconBadArgs;
        out.n_rec = 0;
        return out.status;
    }

    const double th = P.lam / P.rho;
    for (int i = 0; i < n; ++i) W.u[i] = 0.0;
    for (int i = 0; i < 2 * N; ++i) { W.d[i] = 0.0; W.q[i] = 0.0; }

    double* const dd = W.d;
    double* const qq = W.q;

    for (int it = 0; it < P.n_outer; ++it) {
#pragma HLS PIPELINE off
        // ---- rhs = aty - rho·div(d - q) ----
        for (int i = 0; i < 2 * N; ++i) {
#pragma HLS PIPELINE II = 1
            W.s[i] = dd[i] - qq[i];
        }
        tv_divergence(W.s, shape, W.rhs);
        for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II = 1
            W.rhs[i] = G.aty[i] - P.rho * W.rhs[i];
        }

        // ---- 内层固定次数 CG：解 (AᵀA + rho·Lop) u = rhs ----
        double* x = W.cg_x;
        for (int i = 0; i < n; ++i) x[i] = W.u[i];
        m5_admm_op(G, shape, P.rho, x, W.cg_ap, W);
        double rs = 0.0;
        for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II = 1
            W.cg_r[i] = W.rhs[i] - W.cg_ap[i];
            W.cg_p[i] = W.cg_r[i];
            rs += W.cg_r[i] * W.cg_r[i];
        }
        for (int c = 0; c < P.n_cg; ++c) {
#pragma HLS PIPELINE off
            m5_admm_op(G, shape, P.rho, W.cg_p, W.cg_ap, W);
            double den = 0.0, rs_new = 0.0;
            for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II = 1
                den += W.cg_p[i] * W.cg_ap[i];
            }
            if (!(den > 0.0)) break;
            const double al = rs / den;
            for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II = 1
                x[i] += al * W.cg_p[i];
                W.cg_r[i] -= al * W.cg_ap[i];
                rs_new += W.cg_r[i] * W.cg_r[i];
            }
            if (!(rs > 0.0)) break;
            const double be = rs_new / rs;
            for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II = 1
                W.cg_p[i] = W.cg_r[i] + be * W.cg_p[i];
            }
            rs = rs_new;
        }
        for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II = 1
            W.u[i] = x[i];
        }

        // ---- 逐像素收缩：d = s·max(0, 1 − th/|s|)，q += g − d ----
        tv_gradient(W.u, shape, W.g);
        for (int i = 0; i < N; ++i) {
#pragma HLS PIPELINE II = 1
            const double sz = W.g[i] + qq[i];
            const double sx = W.g[N + i] + qq[N + i];
            const double nrm = std::sqrt(sz * sz + sx * sx);
            double sc = 1.0 - th / (nrm > 1e-30 ? nrm : 1e-30);
            if (sc < 0.0) sc = 0.0;
            dd[i] = sz * sc;
            dd[N + i] = sx * sc;
            qq[i] = qq[i] + W.g[i] - dd[i];
            qq[N + i] = qq[N + i] + W.g[N + i] - dd[N + i];
        }
    }

    for (int i = 0; i < n; ++i) out.u[i] = W.u[i];
    out.n_rec = P.n_outer;
    out.status = kReconOk;
    return out.status;
}

}  // namespace ps

#endif  // PENUMBRASIGHT_RECON_HPP
