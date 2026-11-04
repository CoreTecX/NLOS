# -*- coding: utf-8 -*-
"""X15 补充四：ADMM 的内层 CG 迭代次数是不是瓶颈。

推理
----
u 步求解 (AᵀA + ρ∇ᵀ∇)u = Aᵀb − ρ·div(d−q)。归一化后 λmax(AᵀA)=1，
∇ᵀ∇（Neumann）的最小非零特征值约 (π/36)² ≈ 7.6e-3。
取 ρ=0.1 时复合算子的条件数约 1.8/(0.1×7.6e-3) ≈ 2370，CG 需要 O(√κ) ≈ 49 次
才有足够精度 —— 而前三轮每轮只给了 15 次，内层根本没收敛。

本脚本扫 (ρ, n_cg) 与 λ，看补足内层迭代后 ADMM 能否出图。
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))
from penumbrasight_model.reconstruction import divergence, gradient, pearson  # noqa: E402

VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N, ROWS, COLS = 126, 29, 36
SHAPE = (ROWS, COLS)
NH = ROWS * COLS
CSCALE_R = 312.5


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


def admm(ATA, ATb, lam, rho, n_outer, n_cg):
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
    ATb = AT @ dm
    s2 = float(np.linalg.eigvalsh(ATA)[-1])
    s = np.sqrt(s2)
    ATAn, ATbn = ATA / s2, ATb / s

    # 论文 FISTA 的参照（paper_recon_fista.npy）
    pin = os.path.join(VEC, "paper_recon_fista.npy")
    if os.path.exists(pin):
        pf = np.load(pin)
        print(f"参照 论文 FISTA：r={pearson(pf, g):+.4f}  IoU={iou(pf, refb):.4f}\n")

    out = []
    print(f"{'ρ':>6}{'n_cg':>6}{'轮数':>6}{'λ':>9}{'||u||∞':>12}{'r':>9}"
          f"{'IoU':>9}{'TV':>12}{'耗时':>8}")
    for rho in (1.0, 10.0):
        for n_cg, n_outer in ((60, 300),):
            for k in (-2, 0, 2, 4, 6):
                lam = 10.0 ** k
                t0 = time.time()
                u2 = (admm(ATAn, ATbn, lam, rho, n_outer, n_cg) * s).reshape(SHAPE)
                gg = gradient(u2)
                tv = float(np.sum(np.sqrt(gg[0] ** 2 + gg[1] ** 2)))
                r = pearson(u2.reshape(-1), u_ref)
                i = iou(u2, refb)
                out.append((rho, n_cg, k, np.abs(u2).max(), r, i, tv, u2.copy()))
                print(f"{rho:>6g}{n_cg:>6d}{n_outer:>6d}{'1e%+d' % k:>9}"
                      f"{np.abs(u2).max():>12.3e}{r:>9.4f}{i:>9.4f}{tv:>12.3e}"
                      f"{time.time()-t0:>7.1f}s")

    best = max(out, key=lambda t: t[5])
    print(f"\nIoU 最高：ρ={best[0]:g} n_cg={best[1]} λ=1e{best[2]}  "
          f"IoU={best[5]:.4f}  r={best[4]:+.4f}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt
        n = len(out)
        fig, axes = plt.subplots(1, n + 2, figsize=(2.4 * (n + 2), 3.6))
        axes[0].imshow(g, cmap="gray"); axes[0].set_title("真值"); axes[0].axis("off")
        if os.path.exists(pin):
            axes[1].imshow(np.load(pin), cmap="gray")
            axes[1].set_title("论文 FISTA")
        axes[1].axis("off")
        for i, (rho, ncg, k, mx, r, io, tv, u2) in enumerate(out):
            axes[i + 2].imshow(u2, cmap="gray")
            axes[i + 2].set_title(f"ρ={rho:g} n_cg={ncg}\nλ=1e{int(k)} "
                                  f"IoU={io:.2f}", fontsize=7)
            axes[i + 2].axis("off")
        fig.tight_layout()
        p = os.path.join(VEC, "x15_admm_cgfix.png")
        fig.savefig(p, dpi=110)
        print(f"已保存 {p}")
    except Exception as e:
        print(f"绘图跳过：{e}")


if __name__ == "__main__":
    main()
