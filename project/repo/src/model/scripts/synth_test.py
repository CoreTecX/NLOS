# -*- coding: utf-8 -*-
"""隔离实验：用模型自己造的合成测量（无噪声）跑同一套重建管线。

分成两半来定位 bug：
  A. 合成测量 + 差分 + TV-PDHG → 能否恢复 gt？
     能  ⇒ 管线（差分/列序/求解器）没问题，问题出在 A 与真实测量不匹配
     不能 ⇒ 管线本身有 bug
  B. 合成测量直接最小二乘（不差分）
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))

from penumbrasight_model.reconstruction import pearson, tv_pdhg  # noqa: E402

VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N, ROWS, COLS = 126, 29, 36
SHAPE = (ROWS, COLS)


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
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    x_gt = gt[:, :, 0].astype(float).ravel(order="F")     # A 的列就是 F 序
    print(f"A {A0.shape}  x_gt {x_gt.shape}  非零块 {np.count_nonzero(x_gt)}")

    y = A0 @ x_gt                                          # 合成测量（A 的行序 = C 序翻转图）
    print(f"合成测量 y 范围 [{y.min():.3e}, {y.max():.3e}]  均值 {y.mean():.3e}")
    np.save(os.path.join(VEC, "paper_meas_synth.npy"), y)

    # ---- B) 不差分，直接最小二乘
    ATf = A0.T
    ATAf = ATf @ A0
    x_ls_full = np.linalg.solve(ATAf, ATf @ y)
    print(f"\n[B] 不差分 LS: r={pearson(x_ls_full, x_gt):+.6f}")

    # ---- A) 差分 + LS + PDHG
    dA = drows(A0)
    perm = col_perm()
    dAp = np.ascontiguousarray(dA[:, perm])
    AT = dAp.T
    ATA = AT @ dAp
    dm = dvec(y)
    s = np.sqrt(float(np.linalg.eigvalsh(ATA)[-1]))
    dAs = dAp / s
    dm_s = dm / s

    u_ls = np.linalg.solve(ATA, AT @ dm)
    print(f"[A] 差分 LS: r={pearson(u_ls, x_gt.reshape(ROWS, COLS, order='F').ravel(order='C')):+.6f}")

    for lam in (1e-4, 1e-3, 1e-2, 1e-1, 1.0):
        t0 = time.time()
        res = tv_pdhg(dAs, dm_s, SHAPE, lam=lam, n_iter=800, nonneg=False,
                      record_every=800)
        u = np.asarray(res.image).reshape(-1) * s
        r = pearson(u, x_gt.reshape(ROWS, COLS, order="F").ravel(order="C"))
        print(f"[A] 差分 PDHG λ={lam:.1e}: r={r:+.6f}  ({time.time()-t0:.1f}s)")

    # 目视
    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt
        res = tv_pdhg(dAs, dm_s, SHAPE, lam=1e-3, n_iter=1500, nonneg=False,
                      record_every=1500)
        fig, ax = plt.subplots(1, 3, figsize=(12, 4.4))
        ax[0].imshow(gt[:, :, 0], cmap="gray"); ax[0].set_title("真值"); ax[0].axis("off")
        ax[1].imshow(np.asarray(res.image), cmap="gray")
        ax[1].set_title("合成数据 PDHG 重建"); ax[1].axis("off")
        ax[2].imshow(np.asarray(res.image) - gt[:, :, 0], cmap="bwr")
        ax[2].set_title("残差"); ax[2].axis("off")
        fig.tight_layout()
        p = os.path.join(VEC, "synth_test.png")
        fig.savefig(p, dpi=110)
        print(f"已保存 {p}")
    except Exception as e:
        print(f"绘图跳过：{e}")


if __name__ == "__main__":
    main()
