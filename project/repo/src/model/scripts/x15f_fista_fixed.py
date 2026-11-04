# -*- coding: utf-8 -*-
"""X15 收尾：固定步长 FISTA（步长 = 1/L，无线搜索）能否等价于论文版。

为什么这一步决定 M4
------------------
ADMM（CG + 分裂）在本问题上是输的（见 x15e：IoU 上限 0.2297，低于论文 FISTA 的
0.2639）。那么 M4 只能用 FISTA。而论文版 fista_backtrack 的线搜索是数据依赖控制流，
硬件不友好。但归一化后 L = ||A_d||₂² = 1 是**已知量**，步长可以直接取 1/L，
线搜索变成多余的 —— 于是 FISTA 也变成固定循环次数。

本脚本回答：拿掉线搜索之后，FISTA 还剩下多少性能。
同时对比 TV 邻近算子内层迭代次数固定下来的影响。
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))
from penumbrasight_model.reconstruction import gradient, pearson  # noqa: E402

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


def co_prox_tv(b, lam, max_iter, tol):
    """Beck & Teboulle 2009 各向同性 TV 邻近算子；max_iter 固定即为定点化口径。"""
    def Lt(x):
        p = np.zeros_like(x); q = np.zeros_like(x)
        p[:-1, :] = -(x[1:, :] - x[:-1, :])
        q[:, :-1] = -(x[:, 1:] - x[:, :-1])
        return p, q

    def Lop(dx, dy):
        ix = np.zeros_like(dx); iy = np.zeros_like(dy)
        ix[1:, :] = dx[1:, :] - dx[:-1, :]
        iy[:, 1:] = dy[:, 1:] - dy[:, :-1]
        ix[0, :] = dx[0, :]; iy[:, 0] = dy[:, 0]
        return ix + iy

    r = np.zeros_like(b); s = np.zeros_like(b)
    p_old = r.copy(); q_old = s.copy()
    t_old = 1.0; old_val = 0.0
    sol = b.copy()
    for it in range(1, max_iter + 1):
        sol = b - lam * Lop(r, s)
        tmpr, tmps = Lt(sol)
        r = r + (1.0 / (8.0 * lam)) * tmpr
        s = s + (1.0 / (8.0 * lam)) * tmps
        w = np.maximum(1.0, np.sqrt(r ** 2 + s ** 2))
        p = r / w; q = s / w
        t = (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2)) / 2.0
        t_old = t
        r = p + ((t_old - 1.0) / t) * (p - p_old)
        s = q + ((t_old - 1.0) / t) * (q - q_old)
        p_old = p; q_old = q
        if tol is not None:
            cur = 0.5 * np.sum((b - sol) ** 2) + \
                lam * np.sum(np.sqrt(tmpr ** 2 + tmps ** 2))
            if cur <= 0 or abs(cur - old_val) / max(cur, 1e-300) < tol:
                break
            old_val = cur
    return sol


def fista_fixed(ATA, ATm, dA, dm, lam, step, n_iter, prox_kw):
    x_old = np.zeros(NH)
    y_old = np.zeros(NH)
    t_old = 1.0
    for _ in range(n_iter):
        g = ATA @ y_old - ATm
        x_new = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                           lam * step, **prox_kw).reshape(-1)
        t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2))
        y_new = x_new + (t_old - 1.0) / t_new * (x_new - x_old)
        x_old, t_old, y_old = x_new, t_new, y_new
    return x_new


def iou(u2d, ref_bool):
    thr = np.quantile(u2d.ravel(), 1.0 - float(ref_bool.mean()))
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
    ATm = AT @ dm
    s2 = float(np.linalg.eigvalsh(ATA)[-1])
    s = np.sqrt(s2)
    ATAn, ATmn, dAn, dmn = ATA / s2, ATm / s, dA / s, dm / s
    lam_p = REG_R / CSCALE_R / s2

    pin = os.path.join(VEC, "paper_recon_fista.npy")
    pf = np.load(pin)
    print(f"参照 论文 FISTA（含线搜索）：r={pearson(pf, g):+.4f}  "
          f"IoU={iou(pf, refb):.4f}\n")

    print(f"{'λ 倍数':>8}{'prox 口径':>16}{'r':>10}{'IoU':>9}{'||u||∞':>12}"
          f"{'耗时':>8}")
    out = []
    for fac in (0.1, 1.0, 10.0, 100.0):
        for tag, kw in (("收敛判据", dict(max_iter=20000, tol=1e-6)),
                        ("固定 200 次", dict(max_iter=200, tol=None)),
                        ("固定 50 次", dict(max_iter=50, tol=None))):
            t0 = time.time()
            u = fista_fixed(ATAn, ATmn, dAn, dmn, lam_p * fac, 1.0, 1100, kw) * s
            u2 = u.reshape(SHAPE)
            r = pearson(u, u_ref)
            i = iou(u2, refb)
            out.append((fac, tag, r, i, np.abs(u).max(), u2.copy()))
            print(f"{fac:>8g}{tag:>16}{r:>10.4f}{i:>9.4f}"
                  f"{np.abs(u).max():>12.3e}{time.time()-t0:>7.1f}s")

    best = max(out, key=lambda t: t[3])
    print(f"\nIoU 最高：λ×{best[0]:g} / {best[1]}  IoU={best[3]:.4f}  "
          f"r={best[2]:+.4f}")
    np.save(os.path.join(VEC, "x15_best_fista_fixed.npy"), best[5])

    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt
        n = len(out)
        fig, axes = plt.subplots(1, n + 2, figsize=(2.2 * (n + 2), 3.4))
        axes[0].imshow(g, cmap="gray"); axes[0].set_title("真值")
        axes[0].axis("off")
        axes[1].imshow(pf, cmap="gray")
        axes[1].set_title("论文 FISTA"); axes[1].axis("off")
        for i, (fac, tag, r, io, mx, u2) in enumerate(out):
            axes[i + 2].imshow(u2, cmap="gray")
            axes[i + 2].set_title(f"λ×{fac:g} {tag}\nr={r:+.3f} IoU={io:.2f}",
                                  fontsize=6.5)
            axes[i + 2].axis("off")
        fig.tight_layout()
        p = os.path.join(VEC, "x15_fista_fixed.png")
        fig.savefig(p, dpi=110)
        print(f"已保存 {p}")
    except Exception as e:
        print(f"绘图跳过：{e}")


if __name__ == "__main__":
    main()
