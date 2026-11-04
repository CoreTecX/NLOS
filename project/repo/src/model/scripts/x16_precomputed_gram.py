# -*- coding: utf-8 -*-
"""X16：验证「预存 AᵀA + Aᵀb 后，重建迭代不再访问 A」，并核算 BRAM 与 MAC。

背景
----
ADMM 的 u 步求解 (AᵀA + ρ∇ᵀ∇) u = Aᵀb − ρ·div(d − q)。方程右边与算子都由
1044×1044 的 AᵀA、1044 长的 Aᵀb 以及逐像素差分构成 —— 结构上看不到 A。

本脚本做三件事：
1. 严格版 ADMM：函数体内完全不出现 dA / dm，跑完后才用 dA 算相关系数，
   与常规版结果对比，证明迭代确实不需要 A。
2. 核算 AᵀA 的定点范围与 BRAM 占用（整块 / 上三角，多种位宽）。
3. 核算每轮迭代的 MAC 数，与「每轮两次过 A」的方案对比，并按 A2 的 PL 算力折算耗时。
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
CSCALE_R, REG_R = 312.5, 1e07 * 5.0
BRAM_BITS = 32.1e6          # XCZU9EG 片上 BRAM 总位宽
PL_MAC_PER_S = 5e11         # A2 口径：1260 DSP @ 400 MHz


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


def admm_bare(ATA, ATb, lam, rho, n_outer=600, n_cg=15, x0=None):
    """严格版：函数体内不出现 A、dA、dm 中的任何一个。

    AᵀA 按「只存上三角、运行期镜像补齐」使用：取上三角后加自身转置、减一次对角。
    这一步不只是形式上的 —— 它同时说明片上只需要 545,490 个权重，
    而不是整块的 1,089,936 个。
    """
    tri = np.triu(ATA)
    sym = tri + tri.T - np.diag(np.diag(tri))
    Lop = lambda u: -divergence(gradient(u.reshape(SHAPE))).reshape(-1)
    op = lambda u: sym @ u + rho * Lop(u)
    u = np.zeros(NH) if x0 is None else x0.copy()
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


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    u_ref = gt[:, :, 0].astype(float).reshape(-1)

    A3 = A0.reshape(N, N, A0.shape[1]).transpose(1, 0, 2)
    Ar = np.ascontiguousarray(A3).reshape(N * N, A0.shape[1])
    dA = drows(Ar)[:, col_perm()]
    dm = dvec(rows_of(meas[:, :, 0], "F"))
    print(f"A {A0.shape}  dA {dA.shape}")

    AT = dA.T
    ATA = AT @ dA
    ATb = AT @ dm
    s2 = float(np.linalg.eigvalsh(ATA)[-1])
    s = np.sqrt(s2)
    ATAn, ATbn = ATA / s2, ATb / s
    lam_n = REG_R / CSCALE_R / s2
    rho = 10.0 * float(np.mean(np.diag(ATAn)))

    # ---------- 1. 严格版 ADMM ----------
    print("\n[1] 严格版 ADMM（函数体内不出现 A）")
    t0 = time.time()
    u_bare = admm_bare(ATAn, ATbn, lam_n, rho) * s
    print(f"    600×15 完成，用时 {time.time()-t0:.1f}s，"
          f"与真值 r = {pearson(u_bare, u_ref):+.4f}")
    print(f"    Aᵀb 的定点范围：[{np.abs(ATb).min():.3e}, {np.abs(ATb).max():.3e}]")

    # ---------- 2. AᵀA 的定点范围与 BRAM ----------
    print("\n[2] AᵀA 的定点范围与 BRAM 占用")
    at = np.abs(ATA)
    off = at[~np.eye(NH, dtype=bool)]
    print(f"    对称性偏差 max|AᵀA − (AᵀA)ᵀ| / max|AᵀA| = "
          f"{np.max(np.abs(ATA - ATA.T)) / at.max():.3e}")
    print(f"    全体元素  max {at.max():.4e}  median {np.median(at):.4e}  "
          f"动态范围 {at.max()/max(at[at>0].min(), 1e-30):.3e}")
    print(f"    对角元素  max {np.abs(np.diag(ATA)).max():.4e}  "
          f"min {np.abs(np.diag(ATA)).min():.4e}")
    print(f"    非对角    max {off.max():.4e}  median {np.median(off):.4e}")

    n_full, n_tri = NH * NH, NH * (NH + 1) // 2
    print(f"\n    元素数：整块 {n_full:,}   上三角（含对角） {n_tri:,}   "
          f"上三角（不含对角） {n_tri - NH:,}")
    print(f"    BRAM 总量 {BRAM_BITS/1e6:.1f} Mb")
    print(f"    {'位宽':<6}{'整块 Mb':>10}{'占 BRAM':>10}"
          f"{'上三角 Mb':>12}{'占 BRAM':>10}")
    for w in (16, 18, 20, 24, 32):
        f_mb = n_full * w / 1e6
        t_mb = n_tri * w / 1e6
        print(f"    {w:<6}{f_mb:>10.1f}{f_mb*1e6/BRAM_BITS*100:>9.1f}%"
              f"{t_mb:>12.1f}{t_mb*1e6/BRAM_BITS*100:>9.1f}%")

    # 需要多少整数位：以最大元素为基准给 6 位余量
    int_bits = int(np.ceil(np.log2(at.max()))) + 1
    print(f"\n    最大元素 {at.max():.4e} → 整数位约 {int_bits} bit（含符号）")
    for w in (18, 20, 24):
        print(f"    位宽 {w} → 小数位 {w - int_bits}，"
              f"最小可表示增量 {2.0**(-(w-int_bits)):.3e}，"
              f"相对最大元素 {2.0**(-(w-int_bits))/at.max():.2e}")

    # 定点量化后有多少元素被压成 0（按最大值归一化存储）
    print(f"\n    定点量化测试（按最大值归一化后存储，统计被压成 0 的元素）")
    print(f"    {'位宽':<6}{'归零元素数':>14}{'占比':>10}{'量化后相对 RMS 误差':>22}")
    amax = at.max()
    nz_total = int(np.count_nonzero(at))
    for w in (18, 20, 22, 24, 26):
        scale = 2.0 ** (w - 1) / amax
        q = np.round(ATA * scale) / scale
        n_zero = nz_total - int(np.count_nonzero(q))
        rms = float(np.linalg.norm(q - ATA) / np.linalg.norm(ATA))
        print(f"    {w:<6}{n_zero:>14,}{n_zero/nz_total*100:>9.2f}%{rms:>22.3e}")

    # ---------- 3. 每轮 MAC 与耗时 ----------
    print("\n[3] 每轮迭代的 MAC 与 PL 耗时")
    mac_gram = NH * NH                  # 一次 AᵀA 乘
    mac_pass = 2 * (N * N) * NH         # 一次「两次过 A」
    print(f"    1044² 矩阵乘          {mac_gram:,} MAC")
    print(f"    两次过 A（A@x + Aᵀ@r）{mac_pass:,} MAC")
    print(f"    比值                  {mac_pass/mac_gram:.1f} 倍")
    for n_cg, n_outer in ((15, 600), (10, 400), (20, 300)):
        total = n_cg * n_outer * mac_gram
        print(f"    {n_outer} 轮 × {n_cg} 次 CG → {total:.3e} MAC → "
              f"PL 约 {total/PL_MAC_PER_S*1e3:.1f} ms")
    total_ref = 600 * mac_pass
    print(f"    对照：同样 600 轮、每轮两次过 A → {total_ref:.3e} MAC → "
          f"PL 约 {total_ref/PL_MAC_PER_S*1e3:.1f} ms")


if __name__ == "__main__":
    main()
