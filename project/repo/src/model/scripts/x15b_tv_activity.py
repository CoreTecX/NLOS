# -*- coding: utf-8 -*-
"""X15 补充：ADMM 里的 TV 项到底有没有起作用。

solver_compare.py 观察到：λ 相差 100 倍，r 却都是 +0.4494。若 TV 真的生效，
结果不可能一模一样。本脚本直接测收缩算子的活跃度与解对 λ 的敏感度。

判读：
- 若 λ=0 与 λ=论文值 的解几乎相同 → TV 没起作用，r 的改善来自 ρ∇ᵀ∇（拉普拉斯平滑）
  与提前停机，那么 M4 不需要 TV，只需正则化 CG。
- 若不同 → 只是 λ 的标定需要重算。
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
    th = lam / rho
    act = 0.0
    for _ in range(n_outer):
        u = cg(op, ATb - rho * divergence(d - q).reshape(-1), u, n_cg)
        g = gradient(u.reshape(SHAPE))
        s_ = g + q
        nrm = np.sqrt(s_[0] ** 2 + s_[1] ** 2)
        act = float(np.mean(nrm > th)) if lam > 0 else 0.0
        d = s_ * np.maximum(0.0, 1.0 - th / np.maximum(nrm, 1e-30)) if lam > 0 else s_
        q = q + g - d
    return u, act, th


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    u_ref = gt[:, :, 0].astype(float).reshape(-1)

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
    lam_paper = REG_R / CSCALE_R / s2
    rho = 10.0 * float(np.mean(np.diag(ATAn)))

    print(f"λ（论文折算）= {lam_paper:.4e}   ρ = {rho:.4e}")
    print(f"解 u 的量级：||u_ls||∞ = "
          f"{np.abs(np.linalg.solve(ATAn, ATbn)).max():.3e}\n")
    print(f"{'λ 倍数':<12}{'阈值 th':>12}{'收缩活跃像素比':>18}{'r':>10}{'TV(u)':>12}")

    for tag, fac in (("0（无 TV）", 0.0), ("1e-4×", 1e-4), ("0.01×", 0.01),
                     ("0.1×", 0.1), ("1×（论文）", 1.0), ("10×", 10.0),
                     ("100×", 100.0)):
        lam = lam_paper * fac
        t0 = time.time()
        u, act, th = admm(ATAn, ATbn, lam, rho)
        uu = u * s
        tv = float(np.sum(np.sqrt(gradient(uu.reshape(SHAPE))[0] ** 2 +
                                    gradient(uu.reshape(SHAPE))[1] ** 2)))
        print(f"{tag:<12}{th:>12.3e}{act*100:>17.2f}%"
              f"{pearson(uu, u_ref):>10.4f}{tv:>12.4e}"
              f"   ({time.time()-t0:.1f}s)")


if __name__ == "__main__":
    main()
