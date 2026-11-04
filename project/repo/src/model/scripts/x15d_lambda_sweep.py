# -*- coding: utf-8 -*-
"""X15 补充三：按量级扫 λ，找 ADMM 真正能出图的区间。

前两轮的问题
------------
solver_compare.py 的 ρ 扫描只在 1.45e-3 ~ 1.45e-1 之间（= 1~100 × mean(diag)），
而差分最小二乘解的振幅是 3.1e10、真值对应刻度只有 8.0e4 —— 放大 4e5 倍。
正则化强度差了四个数量级，收缩阈值 λ/ρ 远小于 |∇u|，TV 项完全没参与，
于是解退化成病态放大的散斑，却因为低阶结构碰巧相关把 r 抬到 0.449。

本脚本固定 ρ、把 λ 按量级扫开，用**目视 + IoU** 判断，不用 r。
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))
from penumbrasight_model.reconstruction import divergence, gradient, pearson  # noqa: E402

VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N, ROWS, COLS = 126, 29, 36
SHAPE = (ROWS, COLS)
NH = ROWS * COLS
CSCALE_R, REG_R = 312.5, 1e07 * 5.0
RHO = 0.1                      # 固定：λmax(AᵀA_n) = 1，取 0.1 让 ρ∇ᵀ∇ 与数据项同量级


def drows(M):
    o = np.empty_like(M); o[:-1, :] = M[:-1, :] - M[1:, :]; o[-1, :] = M[-1, :]
    return o


def dvec(v):
    o = np.empty_like(v); o[:-1] = v[:-1] - v[1:]; o[-1] = v[-1]
    return o


def rows_of(m2d, order):
    if order == "F":
        return np.ascontiguousarray(m2d.T).ravel(order="C")
    return np.ascontiguousarray(m2d).ravel(order="C")


def col_perm():
    return np.arange(NH).reshape(ROWS, COLS, order="F").ravel(order="C")


def cg(op, b, x0, n_iter):
    x = x0.copy()
    r = b - op(x)
    p = r.copy()
    rs = float(r @ r)
    for _ in range(n_iter):
        Ap = op(p)
        den = float(p @ Ap)
        if not np.isfinite(den) or den <= 0:
            break
        a = rs / den
        x = x + a * p
        r = r - a * Ap
        rs_new = float(r @ r)
        if not np.isfinite(rs_new):
            break
        p = r + (rs_new / rs) * p
        rs = rs_new
    return x


def admm(ATA, ATb, lam, rho=RHO, n_outer=600, n_cg=15):
    Lop = lambda u: -divergence(gradient(u.reshape(SHAPE))).reshape(-1)
    op = lambda u: ATA @ u + rho * Lop(u)
    u = np.zeros(NH)
    d = np.zeros((2,) + SHAPE)
    q = np.zeros((2,) + SHAPE)
    th = lam / rho
    for _ in range(n_outer):
        u = cg(op, ATb - rho * divergence(d - q).reshape(-1), u, n_cg)
        g = gradient(u.reshape(SHAPE))
        s_ = g + q
        nrm = np.sqrt(s_[0] ** 2 + s_[1] ** 2)
        d = s_ * np.maximum(0.0, 1.0 - th / np.maximum(nrm, 1e-30))
        q = q + g - d
    return u


def iou(u2d, ref_bool):
    frac = float(ref_bool.mean())
    thr = np.quantile(u2d.ravel(), 1.0 - frac)
    a = u2d.ravel() >= thr
    return float(np.logical_and(a, ref_bool).sum() /
                 max(np.logical_or(a, ref_bool).sum(), 1))


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    g = gt[:, :, 0].astype(float)
    refb = g.reshape(-1) > 0
    u_ref = g.reshape(-1)

    A3 = A0.reshape(N, N, A0.shape[1]).transpose(1, 0, 2)
    Ar = np.ascontiguousarray(A3).reshape(N * N, A0.shape[1])
    dA = drows(Ar)[:, col_perm()]
    dm = dvec(rows_of(meas[:, :, 0], "F"))
    AT = dA.T
    ATA = AT @ dA
    ATb = AT @ dm
    s2 = float(np.linalg.eigvalsh(ATA)[-1])
    s = np.sqrt(s2)
    ATAn, ATbn = ATA / s2, ATb / s
    u_ls = np.linalg.solve(ATAn, ATbn)
    print(f"归一化后 LS 解振幅 {np.abs(u_ls).max():.3e}；"
          f"真值对应刻度（未归一化）{255*CSCALE_R:.1e}")

    exps = np.arange(-4, 13, 1.0)          # λ = 10^k
    rows = []
    print(f"\n{'λ':>10}{'||u||∞':>12}{'r':>10}{'IoU':>9}{'TV(u)':>12}")
    for k in exps:
        lam = 10.0 ** k
        u = admm(ATAn, ATbn, lam) * s
        u2 = u.reshape(SHAPE)
        gg = gradient(u2)
        tv = float(np.sum(np.sqrt(gg[0] ** 2 + gg[1] ** 2)))
        r = pearson(u, u_ref)
        i = iou(u2, refb)
        rows.append((k, np.abs(u).max(), r, i, tv, u2.copy()))
        print(f"{'1e%+d' % k:>10}{np.abs(u).max():>12.3e}{r:>10.4f}{i:>9.4f}{tv:>12.3e}")

    best = max(rows, key=lambda t: t[3])
    print(f"\nIoU 最高：λ = 1e{int(best[0])}  IoU = {best[3]:.4f}  r = {best[2]:+.4f}")
    np.save(os.path.join(VEC, "x15_best_admm.npy"), best[5])

    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt
        show = [t for t in rows if t[3] > 0.30]
        if not show:
            show = sorted(rows, key=lambda t: -t[3])[:8]
        show = show[:9]
        n = len(show)
        fig, axes = plt.subplots(2, n + 1, figsize=(2.5 * (n + 1), 5.6))
        axes[0, 0].imshow(g, cmap="gray"); axes[0, 0].set_title("真值")
        axes[0, 0].axis("off"); axes[1, 0].axis("off")
        pin = os.path.join(VEC, "paper_recon_fista.npy")
        if os.path.exists(pin):
            pf = np.load(pin)
            axes[1, 0].imshow(pf, cmap="gray")
            axes[1, 0].set_title(f"论文 FISTA\nr={pearson(pf, g):+.3f}  "
                                 f"IoU={iou(pf, refb):.3f}")
        axes[1, 0].axis("off")
        for i, (k, mx, r, i_, tv, u2) in enumerate(show):
            axes[0, i + 1].imshow(u2, cmap="gray")
            axes[0, i + 1].set_title(f"ADMM λ=1e{int(k)}\n"
                                     f"r={r:+.3f}  IoU={i_:.3f}")
            axes[0, i + 1].axis("off")
            axes[1, i + 1].hist(u2.ravel(), bins=60)
            axes[1, i + 1].set_yscale("log"); axes[1, i + 1].set_title("值分布")
        fig.tight_layout()
        p = os.path.join(VEC, "x15_lambda_sweep.png")
        fig.savefig(p, dpi=100)
        print(f"已保存 {p}")
    except Exception as e:
        print(f"绘图跳过：{e}")


if __name__ == "__main__":
    main()
