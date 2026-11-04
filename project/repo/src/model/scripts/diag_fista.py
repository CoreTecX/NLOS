# -*- coding: utf-8 -*-
"""X15 诊断：线搜索 FISTA 在归一化口径下为什么不动。

对照两种口径跑同样 40 轮，逐轮打印步长、数据项、位移、相关系数。
- 归一化：算子范数平方 ≈ 1，初始步长 1e-5 相对 1/L 偏小约 1e5 倍
- 未归一化：算子范数平方 ≈ 1.42e7，初始步长 1e-5 相对 1/L 偏大约 143 倍
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))
from penumbrasight_model.reconstruction import pearson  # noqa: E402

VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N, ROWS, COLS = 126, 29, 36
SHAPE = (ROWS, COLS)
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
    return np.arange(ROWS * COLS).reshape(ROWS, COLS, order="F").ravel(order="C")


def co_prox_tv(b, lam, tol=1e-6, max_iter=20000):
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
    n_it = 0
    for n_it in range(1, max_iter + 1):
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
        cur = 0.5 * np.sum((b - sol) ** 2) + lam * np.sum(np.sqrt(tmpr ** 2 + tmps ** 2))
        if cur <= 0 or abs(cur - old_val) / max(cur, 1e-300) < tol:
            break
        old_val = cur
    return sol, n_it


def run(tag, ATA, ATm, dA, dm, lam, u0, u_ref, n_iter=40, step0=1e-5):
    print(f"\n===== {tag}：λ = {lam:.4e}，初始步长 {step0:g}，"
          f"算子范数平方 ≈ {np.linalg.norm(ATA, 2):.3e} =====")
    print(f"{'轮':>4}{'步长':>14}{'数据项':>16}{'||Δx||':>14}{'r':>10}{'回捗':>6}")
    x_old = u0.copy(); y_old = u0.copy()
    t_old = 1.0; step = step0
    G = lambda v: 0.5 * float((dA @ v - dm) @ (dA @ v - dm))
    for it in range(1, n_iter + 1):
        g = ATA @ y_old - ATm
        x_new, nprox = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                                  lam * step)
        x_new = x_new.reshape(-1)
        t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2))
        y_new = x_new + (t_old - 1.0) / t_new * (x_new - x_old)
        c = G(x_new); oldc = G(y_old)
        n_ls = 0
        while c > oldc + float(g @ (x_new - y_old)) + \
                (1.0 / (2.0 * step)) * float(np.linalg.norm(x_new - y_old) ** 2):
            step *= 0.3
            x_new, nprox = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                                      lam * step)
            x_new = x_new.reshape(-1)
            y_new = x_new + (t_old - 1.0) / t_new * (x_new - x_old)
            c = G(x_new)
            n_ls += 1
            if n_ls > 40:
                break
        d = float(np.linalg.norm(x_new - x_old))
        if it <= 12 or it % 10 == 0:
            print(f"{it:>4}{step:>14.3e}{c:>16.6e}{d:>14.4e}"
                  f"{pearson(x_new, u_ref):>10.4f}{n_ls:>6}")
        x_old, t_old, y_old = x_new, t_new, y_new
        step *= 5.0
    print(f"  40 轮后 step={step:.3e}  r={pearson(x_new, u_ref):+.4f}")


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
    ATm = AT @ dm
    s2 = float(np.linalg.eigvalsh(ATA)[-1])
    s = np.sqrt(s2)
    u_ls = np.linalg.solve(ATA, ATm)
    print(f"未归一化：算子范数平方 {s2:.4e}  LS 解 r={pearson(u_ls, u_ref):+.4f}")

    # 未归一化口径（与 paper_fista.py 一致）
    run("未归一化", ATA, ATm, dA, dm, REG_R / CSCALE_R, u_ls.copy(), u_ref)
    # 归一化口径（与 solver_compare.py 一致）
    run("归一化", ATA / s2, ATm / s, dA / s, dm / s,
        REG_R / CSCALE_R / s2, u_ls.copy(), u_ref)
    # 归一化 + 步长按 1/L 起步
    run("归一化（步长按 1/L 起步）", ATA / s2, ATm / s, dA / s, dm / s,
        REG_R / CSCALE_R / s2, u_ls.copy(), u_ref, step0=1.0)


if __name__ == "__main__":
    main()
