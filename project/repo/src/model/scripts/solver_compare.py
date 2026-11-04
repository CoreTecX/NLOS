# -*- coding: utf-8 -*-
"""W4-1 求解器定案：线搜索 FISTA  vs  CG 内迭代 + TV 分裂（ADMM / 分裂 Bregman）。

问题（差分框架下，变量已按 u = cscale * x 折算）

    min_u  0.5 * || A_d u - b ||_2^2  +  lambda * TV_iso(u)

两种候选解法的本质差别
----------------------
- 线搜索 FISTA（论文原版）：每步梯度 + TV 邻近算子，步长靠回溯确定，
  **是数据依赖的控制流**。
- ADMM：把 TV 项分裂成 d = grad(u)，u 步是二次型，用固定次数 CG 解；
  d 步是逐像素收缩；全程固定循环次数，**没有数据依赖分支**。

判据：达到目标相关系数所需的外层迭代数、墙钟耗时，以及迭代结构对定点的友好度。
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
CSCALE_R = 312.5
REG_R = 1e07 * 5.0


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


# ---------------------------------------------------------------- 共轭梯度
def cg_solve(op, b, x0, n_iter):
    x = x0.copy()
    r = b - op(x)
    p = r.copy()
    rs = float(r @ r)
    rs0 = max(rs, 1e-300)
    for _ in range(n_iter):
        Ap = op(p)
        den = float(p @ Ap)
        if den <= 0:
            break
        a = rs / den
        x = x + a * p
        r = r - a * Ap
        rs_new = float(r @ r)
        if rs_new <= 1e-28 * rs0:
            break
        p = r + (rs_new / rs) * p
        rs = rs_new
    return x


# ---------------------------------------------------------------- FISTA
def co_prox_tv(b, lam, tol=1e-6, max_iter=20000):
    """各向同性 TV 邻近算子（Beck & Teboulle 2009），与论文 co_prox_tv.m 同构。"""
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
    for _ in range(max_iter):
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
        if cur <= 0 or abs(cur - old_val) / cur < tol:
            break
        old_val = cur
    return sol


def fista_ls(ATA, ATm, dA, dm, lam, u0, n_iter, u_ref, record_every=25,
             step0=1.0, tol_rel=1e-12):
    """线搜索 FISTA。

    与论文 fista_backtrack 的两处差别，都是诊断 X15 时被迫加的：

    1. **初始步长取 1/L**。归一化后 L = ||A_d||₂² = 1 是已知量，不必从 1e-5 爬起。
       论文从 1e-5 起步、每轮 ×5，在未归一化问题上恰好碰到合适的量级，
       归一化后就偏小 1e5 倍。
    2. **接受判据加相对容差**。原判据 `c > oldc + g·Δ + (1/2step)||Δ||²` 无容差，
       当迭代点落在最小二乘解附近时 g≈0、Δ≈1e-13，c 与 oldc 只差一个舍入位，
       判据靠浮点噪声决定，步长会一路 ×0.3 空转到 1e-22、位移归零。
    """
    n_el = u0.size
    x_old = u0.copy(); y_old = u0.copy()
    t_old = 1.0
    step = step0
    hist = [(0, pearson(u0, u_ref))]
    n_prox = 0
    n_back = 0

    def G(v):
        r = dA @ v - dm
        return 0.5 * float(r @ r)

    for it in range(1, n_iter + 1):
        g = ATA @ y_old - ATm
        x_new = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                           lam * step).reshape(-1); n_prox += 1
        t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2))
        y_new = x_new + (t_old - 1.0) / t_new * (x_new - x_old)
        c = G(x_new); oldc = G(y_old)
        tol = tol_rel * max(1.0, abs(oldc))
        n_ls = 0
        while c > oldc + float(g @ (x_new - y_old)) + \
                (1.0 / (2.0 * step)) * float(np.linalg.norm(x_new - y_old) ** 2) + tol:
            step *= 0.3
            x_new = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                               lam * step).reshape(-1); n_prox += 1
            y_new = x_new + (t_old - 1.0) / t_new * (x_new - x_old)
            c = G(x_new)
            n_ls += 1
            if n_ls > 40:
                break
        n_back += n_ls
        x_old = x_new; t_old = t_new; y_old = y_new
        step *= 5.0
        if it % record_every == 0:
            hist.append((it, pearson(x_new, u_ref)))
    return x_new, hist, n_prox, n_back


# ---------------------------------------------------------------- ADMM
def admm_tv(ATA, ATm, dA, dm, lam, rho, u0, n_outer, n_cg, u_ref, record_every=10):
    Lop = lambda u: -divergence(gradient(u.reshape(SHAPE))).reshape(-1)
    op = lambda u: ATA @ u + rho * Lop(u)
    u = u0.copy()
    d = np.zeros((2,) + SHAPE)
    q = np.zeros((2,) + SHAPE)
    th = lam / rho
    hist = [(0, pearson(u, u_ref))]
    n_mv = 0
    for k in range(1, n_outer + 1):
        rhs = ATm - rho * divergence(d - q).reshape(-1)
        u = cg_solve(op, rhs, u, n_cg); n_mv += n_cg
        g = gradient(u.reshape(SHAPE))
        s_ = g + q
        nrm = np.sqrt(s_[0] ** 2 + s_[1] ** 2)
        scale = np.maximum(0.0, 1.0 - th / np.maximum(nrm, 1e-30))
        d = s_ * scale
        q = q + g - d
        if k % record_every == 0:
            hist.append((k, pearson(u, u_ref)))
    return u, hist, n_mv


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    u_ref = gt[:, :, 0].astype(float).reshape(-1)

    A3 = A0.reshape(N, N, A0.shape[1]).transpose(1, 0, 2)      # F 序行（论文口径）
    Ar = np.ascontiguousarray(A3).reshape(N * N, A0.shape[1])
    dA = drows(Ar)[:, col_perm()]
    dm = dvec(rows_of(meas[:, :, 0], "F"))

    AT = dA.T
    ATA = AT @ dA
    ATm = AT @ dm
    ev = np.linalg.eigvalsh(ATA)
    s2 = float(ev[-1])
    print(f"A_d {dA.shape}  条件数 {ev[-1]/ev[0]:.3e}  算子范数平方 {s2:.4e}")

    # 归一化，避免 rho 的绝对尺度难调
    s = np.sqrt(s2)
    dAn, dmn = dA / s, dm / s
    ATAn, ATmn = ATA / s2, ATm / s
    u_ls = np.linalg.solve(ATAn, ATmn)
    print(f"最小二乘（不差分方向修正后）初始 r = {pearson(u_ls, u_ref):+.4f}")

    report = []
    for fac in (0.1, 1.0, 10.0):
        lam = REG_R / CSCALE_R / s2 * fac
        print(f"\n================ λ = {lam:.4e} （论文值 ×{fac}） ================")

        # ---- 线搜索 FISTA（步长按 1/L 起步，判据带容差）
        t0 = time.time()
        uf, hf, nprox, nback = fista_ls(ATAn, ATmn, dAn, dmn, lam, u_ls.copy(),
                                        1000, u_ref, record_every=20)
        tf = time.time() - t0
        rf = abs(pearson(uf, u_ref))
        print(f"FISTA：1000 轮，r={rf:+.4f}，耗时 {tf:.1f}s，"
              f"TV 邻近算子 {nprox} 次，回溯 {nback} 次")

        # ---- ADMM，ρ 向上扫
        best = None
        for rho_f in (1.0, 3.0, 10.0, 30.0, 100.0):
            rho = rho_f * float(np.mean(np.diag(ATAn)))
            t0 = time.time()
            ua, ha, nmv = admm_tv(ATAn, ATmn, dAn, dmn, lam, rho, u_ls.copy(),
                                  600, 15, u_ref)
            ta = time.time() - t0
            ra = abs(pearson(ua, u_ref))
            print(f"  ADMM ρ={rho_f:<6g} 600×15：r={ra:+.4f}  耗时 {ta:.1f}s  "
                  f"ATA 乘 {nmv} 次")
            if best is None or ra > best[0]:
                best = (ra, rho_f, ha, ta, nmv)
        ra, rho_f, ha, ta, nmv = best
        print(f"ADMM 最佳 ρ 倍数 {rho_f}，r={ra:+.4f}")

        def first_above(hist, thr):
            for k, r in hist:
                if abs(r) >= thr:
                    return k
            return None

        report.append(dict(
            fac=fac, lam=lam,
            fista=dict(r=rf, t=tf, nprox=nprox, h=hf,
                       k45=first_above(hf, 0.45), k50=first_above(hf, 0.50)),
            admm=dict(r=ra, t=ta, rho=rho_f, h=ha, nmv=nmv,
                      k45=first_above(ha, 0.45), k50=first_above(ha, 0.50)),
        ))

    print("\n" + "=" * 78)
    print("汇总：达到目标相关系数所需的外层迭代数")
    print(f"{'λ×':<6}{'FISTA r':>10}{'FISTA k(r≥0.45)':>18}{'ADMM r':>10}"
          f"{'ADMM k(r≥0.45)':>18}{'ADMM ρ×':>9}")
    for r in report:
        print(f"{r['fac']:<6g}{r['fista']['r']:>10.4f}"
              f"{str(r['fista']['k45']):>18}{r['admm']['r']:>10.4f}"
              f"{str(r['admm']['k45']):>18}{r['admm']['rho']:>9g}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, len(report), figsize=(5.2 * len(report), 4.4))
        if len(report) == 1:
            axes = [axes]
        for ax, r in zip(axes, report):
            for tag, h, c in (("线搜索 FISTA", r["fista"]["h"], "tab:red"),
                              ("ADMM", r["admm"]["h"], "tab:blue")):
                ks = [k for k, _ in h]
                rs = [abs(v) for _, v in h]
                ax.plot(ks, rs, "-o", ms=3, color=c, label=f"{tag}（终值 {rs[-1]:.3f}）")
            ax.axhline(0.45, ls="--", c="gray", lw=1)
            ax.set_xlabel("外层迭代数")
            ax.set_ylabel("|与真值相关系数|")
            ax.set_title(f"λ = 论文值 ×{r['fac']:g}")
            ax.legend(fontsize=8)
            ax.grid(alpha=0.3)
        fig.tight_layout()
        p = os.path.join(VEC, "solver_compare.png")
        fig.savefig(p, dpi=120)
        print(f"\n已保存 {p}")
    except Exception as e:
        print(f"绘图跳过：{e}")


if __name__ == "__main__":
    main()
