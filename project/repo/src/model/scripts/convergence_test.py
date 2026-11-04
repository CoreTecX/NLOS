# -*- coding: utf-8 -*-
"""收敛性研究：PDHG 在（无噪声）合成数据上，随迭代次数能否收敛到最小二乘解。

若 r 随迭代单调升向 1，则此前所有重建失败的主因是「迭代不足」，
而不是 A 出错 —— 这对 FPGA 方案同样关键。
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
    o = np.empty_like(v); o[:-1] = v[:-1] - v[1:]; o[-1] = v[-1]
    return o


def drows(M):
    o = np.empty_like(M); o[:-1, :] = M[:-1, :] - M[1:, :]; o[-1, :] = M[-1, :]
    return o


def col_perm():
    return np.arange(ROWS * COLS).reshape(ROWS, COLS, order="F").ravel(order="C")


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    x_gt = gt[:, :, 0].astype(float).ravel(order="F")
    x_gtC = gt[:, :, 0].astype(float).reshape(-1)

    y = A0 @ x_gt
    dA = drows(A0)[:, col_perm()]
    AT = dA.T
    ATA = AT @ dA
    dm = dvec(y)
    s = np.sqrt(float(np.linalg.eigvalsh(ATA)[-1]))
    dAs, dms = dA / s, dm / s

    # 参考解：精确最小二乘
    x_ref = np.linalg.solve(ATA, AT @ dm)
    print(f"参考：差分最小二乘 r = {pearson(x_ref, x_gtC):+.6f}")

    # 用共轭梯度解 (ATA + tau I) x = AT dm，tau 很小 —— 看「几乎无正则」的解
    tau = 1e-6 * np.trace(ATA) / ATA.shape[0]
    n = ATA.shape[0]
    x = np.zeros(n)
    r = AT @ dm
    p = r.copy()
    rs = float(r @ r)
    for it in range(200):
        Ap = ATA @ p + tau * p
        alpha = rs / float(p @ Ap)
        x = x + alpha * p
        r = r - alpha * Ap
        rs_new = float(r @ r)
        if rs_new < 1e-24:
            break
        p = r + (rs_new / rs) * p
        rs = rs_new
    print(f"CG(200 次, tau={tau:.3e}) r = {pearson(x, x_gtC):+.6f}")

    print("\nPDHG 收敛性（合成无噪声数据）")
    for lam in (0.0, 1e-5):
        for nit in (500, 2000, 8000):
            t0 = time.time()
            res = tv_pdhg(dAs, dms, SHAPE, lam=lam, n_iter=nit,
                          nonneg=False, record_every=0)
            u = np.asarray(res.image).reshape(-1) * s
            print(f"  λ={lam:<8g} n_iter={nit:<6d} r={pearson(u, x_gtC):+.6f}"
                  f"  ({time.time()-t0:.1f}s)", flush=True)


if __name__ == "__main__":
    main()
