# -*- coding: utf-8 -*-
"""X15 补充二：把 ADMM 在不同 λ 下的解画出来，判断 0.4494 到底是重建还是噪声。

同时给出一个客观指标：把解二值化后与二值真值的 IoU（真值本身就是二值图）。
噪声解即使相关系数不低，IoU 也会很低。
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


def admm(ATA, ATb, lam, rho, n_outer=600, n_cg=15, x0=None):
    Lop = lambda u: -divergence(gradient(u.reshape(SHAPE))).reshape(-1)
    op = lambda u: ATA @ u + rho * Lop(u)
    u = np.zeros(NH) if x0 is None else x0.copy()
    d = np.zeros((2,) + SHAPE)
    q = np.zeros((2,) + SHAPE)
    th = lam / rho if lam > 0 else 0.0
    for _ in range(n_outer):
        u = cg(op, ATb - rho * divergence(d - q).reshape(-1), u, n_cg)
        g = gradient(u.reshape(SHAPE))
        s_ = g + q
        if lam > 0:
            nrm = np.sqrt(s_[0] ** 2 + s_[1] ** 2)
            d = s_ * np.maximum(0.0, 1.0 - th / np.maximum(nrm, 1e-30))
        else:
            d = s_
        q = q + g - d
    return u


def iou_binarized(u2d, ref2d, frac=None):
    """按分位数阈值二值化后算 IoU。frac 默认取真值的白点占比。"""
    r = np.asarray(ref2d, float)
    if frac is None:
        frac = float((r.ravel() > 0).mean())
    thr = np.quantile(u2d.ravel(), 1.0 - frac)
    a = u2d >= thr
    b = r.ravel().reshape(r.shape) > 0
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return float(inter / union) if union else 0.0


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    g = gt[:, :, 0].astype(float)
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
    lam_paper = REG_R / CSCALE_R / s2 / s ** 2 * s ** 2   # = reg/cs/s2
    lam_paper = REG_R / CSCALE_R / s2

    u_ls = np.linalg.solve(ATAn, ATbn) * s          # 未归一化的最小二乘解
    print(f"最小二乘解 u_ls：范围 [{u_ls.min():.3e}, {u_ls.max():.3e}]  "
          f"真值对应量级 [0, {255*CSCALE_R:.1e}]")
    print(f"u_ls 的 r={pearson(u_ls, u_ref):+.4f}  IoU={iou_binarized(u_ls.reshape(SHAPE), g):.4f}")

    cases = [("λ = 0", 0.0), ("λ = 1e-4×", 1e-4), ("λ = 1×（论文）", 1.0),
             ("λ = 100×", 100.0), ("λ = 1e4×", 1e4)]
    imgs = []
    print(f"\n{'λ':<16}{'||u||∞':>12}{'r':>10}{'IoU':>10}{'TV(u)':>12}")
    for tag, fac in cases:
        lam = lam_paper * fac
        u = admm(ATAn, ATbn, lam, rho=10.0 * float(np.mean(np.diag(ATAn)))) * s
        u2 = u.reshape(SHAPE)
        gg = gradient(u2)
        tv = float(np.sum(np.sqrt(gg[0] ** 2 + gg[1] ** 2)))
        print(f"{tag:<16}{np.abs(u).max():>12.3e}{pearson(u, u_ref):>10.4f}"
              f"{iou_binarized(u2, g):>10.4f}{tv:>12.4e}")
        imgs.append((tag, u2))

    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 6, figsize=(21, 7.4))
        axes[0, 0].imshow(g, cmap="gray"); axes[0, 0].set_title("真值 R")
        axes[0, 0].axis("off"); axes[1, 0].axis("off")
        for i, (tag, u2) in enumerate(imgs):
            ax = axes[0, i + 1]
            ax.imshow(u2, cmap="gray")
            ax.set_title(f"{tag}\nr={pearson(u2.reshape(-1), u_ref):+.3f}  "
                         f"IoU={iou_binarized(u2, g):.3f}")
            ax.axis("off")
            ax2 = axes[1, i + 1]
            ax2.hist(u2.ravel(), bins=60)
            ax2.set_yscale("log"); ax2.set_title("解的值分布")
        fig.tight_layout()
        p = os.path.join(VEC, "x15_tv_activity.png")
        fig.savefig(p, dpi=100)
        print(f"\n已保存 {p}")
    except Exception as e:
        print(f"绘图跳过：{e}")


if __name__ == "__main__":
    main()
