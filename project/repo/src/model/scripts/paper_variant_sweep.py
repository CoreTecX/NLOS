# -*- coding: utf-8 -*-
"""行序/翻转 的四种自洽组合 × 重建质量 扫描。

背景
----
MATLAB 里两者都是列优先展平：
    A(:,k)          = flipud(image)(:)        （列优先）
    mr              = Diff * reshape(measurement(:,:,1),[],1)   （列优先）
而 paper_forward.py 保存的 A 是 C 序展平的。本脚本把 A 的行列重排到四种
自洽组合，各自跑一次 TV-PDHG 重建，看哪一种能把 BU 恢复出来。

关键观察（paper_recon_pdhg_fig.png）：C 序组合只恢复了水平条纹，
竖直方向完全塌掉 —— 说明差分方向很可能选错了。
"""
import itertools
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))

from penumbrasight_model.reconstruction import pearson, tv_pdhg  # noqa: E402

VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N = 126
ROWS, COLS = 29, 36
SHAPE = (ROWS, COLS)
CSCALE = np.array([312.5, 425.34722222, 496.54222222])
REG = 1e07 * np.array([5.0, 25.0, 25.0])


def rows_A(A, unflip, order):
    A3 = A.reshape(N, N, A.shape[1])
    if unflip:
        A3 = A3[::-1, :, :]
    if order == "F":
        A3 = A3.transpose(1, 0, 2)
    return np.ascontiguousarray(A3).reshape(N * N, A.shape[1])


def rows_m(m, unflip, order):
    if unflip:
        m = m[::-1, :]
    return np.ascontiguousarray(m).ravel(order=order)


def dvec(v):
    out = np.empty_like(v)
    out[:-1] = v[:-1] - v[1:]
    out[-1] = v[-1]
    return out


def drows(M):
    out = np.empty_like(M)
    out[:-1, :] = M[:-1, :] - M[1:, :]
    out[-1, :] = M[-1, :]
    return out


def col_perm():
    return np.arange(ROWS * COLS).reshape(ROWS, COLS, order="F").ravel(order="C")


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    gR = gt[:, :, 0].astype(float).ravel()          # 只有 R 通道有内容
    perm = col_perm()

    results = []
    for order, unflip in (("C", False), ("C", True), ("F", False), ("F", True)):
        t0 = time.time()
        Ar = rows_A(A0, unflip, order)[:, perm]     # 列换成 C 序以配合 tv_pdhg
        dA = drows(Ar)
        AT = dA.T
        ATA = AT @ dA
        s = np.sqrt(float(np.linalg.eigvalsh(ATA)[-1]))
        dAs = dA / s

        dm = dvec(rows_m(meas[:, :, 0], unflip, order))
        u_ls = np.linalg.solve(ATA, AT @ dm)
        r_ls = pearson(u_ls, gR)
        resid = float(np.linalg.norm(dA @ u_ls - dm) / np.linalg.norm(dm))
        print(f"\n===== 行序={order} 撤flip={unflip}  "
              f"(残差 {resid:.4f}, LS r={r_ls:+.4f}, 准备 {time.time()-t0:.0f}s) =====")

        best = (abs(r_ls), r_ls, u_ls / CSCALE[0], "LS", None)
        for factor in (0.1, 1.0, 10.0):
            lam = REG[0] / CSCALE[0] / (s ** 2) * factor
            t0 = time.time()
            res = tv_pdhg(dAs, dm / s, SHAPE, lam=lam, n_iter=800,
                          nonneg=False, record_every=800)
            u = np.asarray(res.image).reshape(-1) * s
            r = pearson(u, gR)
            print(f"   λ×{factor:<5g} r={r:+.4f}  ({time.time()-t0:.1f}s)")
            if abs(r) > best[0]:
                best = (abs(r), r, u / CSCALE[0], "PDHG", factor)

        img = best[2].reshape(SHAPE, order="C")
        results.append((order, unflip, resid, r_ls, best[3], best[4], best[1], img))
        print(f"  最佳：{best[3]} λ×{best[4]}  r={best[1]:+.4f}")

    print("\n" + "=" * 76)
    print(f"{'行序':<5}{'撤flip':<8}{'LS残差':>10}{'LS r':>10}{'方案':>6}{'λ×':>7}{'最佳 r':>10}")
    for o, uf, resid, r_ls, tag, fac, r, _ in sorted(results, key=lambda x: -x[6]):
        print(f"{o:<5}{str(uf):<8}{resid:>10.4f}{r_ls:>10.4f}{tag:>6}"
              f"{str(fac):>7}{r:>10.4f}")

    # 目视图
    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 5, figsize=(20, 4.6))
        axes[0].imshow(gt[:, :, 0], cmap="gray")
        axes[0].set_title("真值 R")
        axes[0].axis("off")
        for i, (o, uf, resid, r_ls, tag, fac, r, img) in enumerate(results):
            axes[i + 1].imshow(img, cmap="gray")
            axes[i + 1].set_title(f"序{o} 撤flip={uf}\nr={r:+.3f}")
            axes[i + 1].axis("off")
        fig.tight_layout()
        p = os.path.join(VEC, "paper_variant_sweep.png")
        fig.savefig(p, dpi=110)
        print(f"\n已保存 {p}")
    except Exception as e:
        print(f"绘图跳过：{e}")


if __name__ == "__main__":
    main()
