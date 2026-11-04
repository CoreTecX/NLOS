# -*- coding: utf-8 -*-
"""论文求解器 fista_backtrack 的忠实复现（修正行列序之后）。

与 paper_recon.py 的区别
------------------------
1. 行序按「A 行与测量行同序」处理；差分方向用 -C / -F 两种分别试。
2. A 的列按 tv_pdhg 需要的 C 序排列，保证 TV 邻近算子作用在正确的图像朝向上。
3. 求解变量取 u = cscale * x，正则强度 λ = reg / cscale（推导见 docstring）。

为什么换掉 PDHG
---------------
convergence_test.py 实测：无噪声合成数据上，精确最小二乘 r=1.000000、CG(200) r=0.999992，
而固定步长 PDHG 迭代 8000 次才到 0.843 —— 该 Gram 矩阵条件数 6.4e5，PDHG 严重欠收敛。
论文的 fista_backtrack 有回溯线搜索，允许在平坦方向上取大步长，因此收敛快得多。
"""
import os
import time

import numpy as np

# 由本文件位置推导（见 docs/environment_ascii_build.md）：同一份源码要能在工作区检出
# 与 ASCII 构建区 D:\psight\penumbrasight 两处都跑，故不写死绝对路径。
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                     os.pardir, os.pardir, os.pardir))
VEC = os.path.join(_REPO, "sim", "vectors")
N, ROWS, COLS = 126, 29, 36
SHAPE = (ROWS, COLS)
CSCALE = np.array([312.5, 425.34722222, 496.54222222])
REG = 1e07 * np.array([5.0, 25.0, 25.0])


# ---------------------------------------------------------------- 算子
def drows(M):
    o = np.empty_like(M); o[:-1, :] = M[:-1, :] - M[1:, :]; o[-1, :] = M[-1, :]
    return o


def dvec(v):
    o = np.empty_like(v); o[:-1] = v[:-1] - v[1:]; o[-1] = v[-1]
    return o


def rows_of(M2d, order):
    """把 126x126 的图按指定展平序排成 15876 长的向量。"""
    if order == "F":
        return np.ascontiguousarray(M2d.T).ravel(order="C")
    return np.ascontiguousarray(M2d).ravel(order="C")


def col_perm():
    return np.arange(ROWS * COLS).reshape(ROWS, COLS, order="F").ravel(order="C")


# ---------------------------------------------------------------- TV 邻近算子
def co_prox_tv(b, lam, tol=1e-6, max_iter=20000):
    """复现 co_prox_tv.m（Beck & Teboulle 2009），输入为 (29,36) 图像。"""
    def Lt(x):
        p = np.zeros_like(x); q = np.zeros_like(x)
        p[:-1, :] = -(x[1:, :] - x[:-1, :])
        q[:, :-1] = -(x[:, 1:] - x[:, :-1])
        return p, q

    def Lop(dx, dy):
        ix = np.zeros_like(dx); iy = np.zeros_like(dy)
        ix[1:, :] = dx[1:, :] - dx[:-1, :]
        iy[:, 1:] = dy[:, 1:] - dy[:, :-1]
        ix[0, :] = dx[0, :]
        iy[:, 0] = dy[:, 0]
        return ix + iy

    r = np.zeros_like(b); s = np.zeros_like(b)
    p_old = r.copy(); q_old = s.copy()
    t_old = 1.0; old_val = 0.0
    sol = b.copy()
    for it in range(max_iter):
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
        if cur <= 0:
            break
        if abs(cur - old_val) / cur < tol:
            break
        old_val = cur
    return sol


# ---------------------------------------------------------------- FISTA
def fista(dA, dm, lam, u_init, max_iter=1100, tol=2.4e-6, step0=1e-5,
          alpha=1.0, verbose=True, tag=""):
    AT = dA.T
    ATA = AT @ dA
    ATm = AT @ dm
    n_el = u_init.size

    x_old = u_init.copy()
    y_old = u_init.copy()
    t_old = 1.0
    step = step0

    def G(v):
        r = dA @ v - dm
        return 0.5 * float(r @ r)

    x_new = x_old
    for it in range(max_iter):
        g = ATA @ y_old - ATm
        x_new = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                           lam * step).reshape(-1)
        t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2))
        y_new = x_new + alpha * (t_old - 1.0) / t_new * (x_new - x_old)

        c = G(x_new)
        oldc = G(y_old)
        n_ls = 0
        while c > oldc + float(g @ (x_new - y_old)) + \
                (1.0 / (2.0 * step)) * float(np.linalg.norm(x_new - y_old) ** 2):
            step *= 0.3
            x_new = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                               lam * step).reshape(-1)
            y_new = x_new + alpha * (t_old - 1.0) / t_new * (x_new - x_old)
            c = G(x_new)
            n_ls += 1
            if n_ls > 60:
                break

        e = float(np.linalg.norm(x_new - x_old)) / n_el
        if verbose and (it % 100 == 0 or it < 3):
            print(f"    {tag} iter {it+1:4d} step {step:.3e} 数据项 {c:.5e} e {e:.2e}"
                  f" 回溯 {n_ls}", flush=True)
        if it > 10 and e < tol:
            if verbose:
                print(f"    {tag} 第 {it+1} 轮收敛 (e={e:.3e})")
            break
        x_old = x_new; t_old = t_new; y_old = y_new
        step *= 5.0
    return x_new, it + 1


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    gC = gt[:, :, 0].astype(float).reshape(-1)
    perm = col_perm()

    results = {}
    for order in ("C", "F"):
        # A 行重排 + 列重排
        if order == "F":
            A3 = A0.reshape(N, N, A0.shape[1]).transpose(1, 0, 2)
            Ar = np.ascontiguousarray(A3).reshape(N * N, A0.shape[1])
        else:
            Ar = A0
        dA = drows(Ar)[:, perm]
        AT = dA.T
        ATA = AT @ dA

        m = rows_of(meas[:, :, 0], order)
        dm = dvec(m)
        cs = CSCALE[0]

        # 初始解 = 论文口径最小二乘
        u0 = np.linalg.solve(ATA, AT @ dm)
        print(f"\n===== 差分方向 {order}：初始 LS r={_pearson(u0, gC):+.4f} =====")

        for fac in (0.1, 1.0, 10.0):
            lam = REG[0] / cs * fac
            t0 = time.time()
            u, nit = fista(dA, dm, lam, u0, max_iter=1100, tag=f"{order}×{fac}")
            r = _pearson(u, gC)
            print(f"  λ×{fac:<5g} r={r:+.4f}  用时 {time.time()-t0:.1f}s 迭代 {nit}")
            results[(order, fac)] = (r, u.copy())

    print("\n" + "=" * 60)
    print("汇总（差分方向 × λ 因子 → 与真值的相关系数）")
    for (order, fac), (r, _) in sorted(results.items(), key=lambda kv: -abs(kv[1][0])):
        print(f"  方向 {order}  λ×{fac:<5g}  r={r:+.4f}")

    # 目视 + 保存
    best = max(results.items(), key=lambda kv: abs(kv[1][0]))
    (bo, bf), (br, bu) = best
    print(f"\n最佳：方向 {bo}，λ×{bf}，r={br:+.4f}")
    np.save(os.path.join(VEC, "paper_recon_fista.npy"),
            bu.reshape(SHAPE, order="C"))
    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt
        keys = sorted(results.keys())
        fig, axes = plt.subplots(1, len(keys) + 1, figsize=(3.2 * (len(keys) + 1), 4.2))
        axes[0].imshow(gt[:, :, 0], cmap="gray"); axes[0].set_title("真值 R"); axes[0].axis("off")
        for i, k in enumerate(keys):
            axes[i + 1].imshow(results[k][1].reshape(SHAPE, order="C"), cmap="gray")
            axes[i + 1].set_title(f"方向{k[0]} λ×{k[1]:g}\nr={results[k][0]:+.3f}")
            axes[i + 1].axis("off")
        fig.tight_layout()
        p = os.path.join(VEC, "paper_fista_grid.png")
        fig.savefig(p, dpi=110)
        print(f"已保存 {p}")
    except Exception as e:
        print(f"绘图跳过：{e}")


def _pearson(a, b):
    a = np.asarray(a, float).ravel() - np.mean(a)
    b = np.asarray(b, float).ravel() - np.mean(b)
    d = np.linalg.norm(a) * np.linalg.norm(b)
    return float(a @ b / d) if d > 0 else float("nan")


if __name__ == "__main__":
    main()
