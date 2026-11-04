# -*- coding: utf-8 -*-
"""X15g：把「本征病态」与「A 的模型误差」解耦，据此定案求解器并导出 X4 的容忍度。

要解决的问题
------------
X15 的结论是「求解器对比被 A 的误差污染，必须等 X4」。这个说法把两件事混在一起：

  (a) **本征病态**：`A_dᵀA_d` 条件数 7.62e5。这是算子自身的性质，与我们的 A 对
      不对无关，X4 修完也还是 7.62e5。
  (b) **模型误差**：我们的 A 与真实模型差 ~31%（水平方向差分残差），X4 要修的就是它。

两者都会让解变坏，但机制不同。用**自洽合成数据**可以把 (a) 单独拎出来：

  dm = A_d(δ=0) @ u_true            ← 数据由 A 自己生成，模型误差**恰好为零**
  用 A_d(δ) 去重建                  ← δ 是人为注入的相对模型误差

于是 δ = 0 的结论是**无条件的**：任何在这里失败的求解器可以立刻排除，不必等 X4。
δ > 0 的扫描给出**容忍曲线**，直接对应 X4 的判据（相对误差 ≤ 1e-6）。

为什么用 paper A 而不是 build_transport_matrix
----------------------------------------------
条件数必须与 X15 相同（7.62e5），结论才是在讲同一个问题；而 X15 的那组数出自 paper A
这条既有流水线。另注：paper A 与 build_transport_matrix 之间还有 1.5e11 的量级差
（见 X16 订正），两者不可直接混用，故本脚本全程只用 paper A。

三类判据（沿用 X15 制度化的口径）
--------------------------------
  一级 目视可辨 BU —— 出图，人看。这是唯一没被骗过的判据。
  二级 结构指标 —— 标度不变误差 `sm_err`、振幅比、TV 比
  三级 r 与 IoU —— 只记录，不作判据

**为什么必须加 sm_err**：r、IoU、以及目视都可能被「标度错但方向对」的解误导。
`sm_err = min_α ‖α·u − u_true‖ / ‖u_true‖` 在对 u 做最优标度后比，因此能把
「量纲/归一化错」与「方向错」分开。首版没有这个量，差点把一次纯标度问题
（振幅比 ≈ ‖dA‖² = 1.42e7）读成算法失败。

λ 与 ρ 的口径（第二版修订）
--------------------------
首版沿用 x15c_render 的归一化写法（ATAn = ATA/s2 等，末尾乘 s 还原）。该写法在还原
时差一个 s² 因子，会把解的量纲整体放大 1.42e7 倍 —— 这正是 X15 报告里「最小二乘振幅
3.1e10」、「放大 4e5 倍」的来源。第二版改为**直接解未归一化问题**，不做任何量纲搬运：

    op(u) = ATA @ u + rho * Lop(u)
    rhs   = ATb - rho * div(d - q)
    th    = lam / rho

`rho` 以 `mean(diag(ATA))` 为基准扫 0.1 / 1 / 10 倍，`lam` 以 `REG_R/CSCALE_R` 为
基准扫 0.1 / 1 / 10 倍，按 `sm_err` 选优。这样就没有任何一处可能写错的换算。

用法
----
    python src/model/scripts/x15g_model_error_tolerance.py
    python src/model/scripts/x15g_model_error_tolerance.py --max-iter 120   # 快跑
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))

VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
OUT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "results"))

N, ROWS, COLS = 126, 29, 36
SHAPE = (ROWS, COLS)
NH = ROWS * COLS
CSCALE_R = 312.5
REG_R = 1e07 * 5.0


# ---------------------------------------------------------------- 既定口径的算子
# 与 paper_fista.py 一致，勿改。


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


def build_dA(A0):
    A3 = A0.reshape(N, N, A0.shape[1]).transpose(1, 0, 2)
    Ar = np.ascontiguousarray(A3).reshape(N * N, A0.shape[1])
    return drows(Ar)[:, col_perm()]


# ---------------------------------------------------------------- 度量


def pearson(a, b):
    a = np.asarray(a, float).ravel() - np.mean(a)
    b = np.asarray(b, float).ravel() - np.mean(b)
    d = np.linalg.norm(a) * np.linalg.norm(b)
    return float(a @ b / d) if d > 0 else float("nan")


def scale_matched_err(u, ref):
    """做最优标度后与 ref 的相对 L2 误差。标度不变，用来区分量纲错与方向错。"""
    u = np.asarray(u, float).ravel()
    ref = np.asarray(ref, float).ravel()
    den = float(u @ u)
    if den <= 0:
        return float("inf")
    a = float(u @ ref) / den
    nr = float(np.linalg.norm(ref))
    return float(np.linalg.norm(a * u - ref) / nr) if nr > 0 else float("nan")


def tv_value(u2d):
    g = np.zeros((2,) + u2d.shape)
    g[0, :-1, :] = u2d[1:, :] - u2d[:-1, :]
    g[1, :, :-1] = u2d[:, 1:] - u2d[:, :-1]
    return float(np.sum(np.sqrt(g[0] ** 2 + g[1] ** 2)))


def iou_binarized(u2d, ref2d):
    r = np.asarray(ref2d, float)
    frac = float((r.ravel() > 0).mean())
    if frac <= 0.0:
        return float("nan")
    thr = np.quantile(u2d.ravel(), 1.0 - frac)
    a = u2d >= thr
    b = r > 0
    u = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / u) if u else 0.0


def metrics(u, u_true, u_true2d, u_true_inf, u_true_tv):
    u2 = np.asarray(u, float).reshape(SHAPE)
    with np.errstate(invalid="ignore", divide="ignore"):
        return {
            "sm_err": scale_matched_err(u, u_true),
            "amp_ratio": float(np.max(np.abs(u)) / u_true_inf),
            "r": pearson(u, u_true),
            "iou": iou_binarized(u2, u_true2d),
            "tv_ratio": tv_value(u2) / u_true_tv,
            "u_inf": float(np.max(np.abs(u))) if np.all(np.isfinite(u)) else float("inf"),
            "finite": bool(np.all(np.isfinite(u))),
        }


def sel_key(m):
    """S1 选优键：标度不变误差，非有限一律视为极坏。"""
    v = m["sm_err"]
    return v if np.isfinite(v) else 1e30


# ---------------------------------------------------------------- 求解器


def co_prox_tv(b, lam, tol=1e-6, max_iter=100):
    """Beck & Teboulle 2009 FGP，复现 co_prox_tv.m。

    max_iter 取 100 —— X15 第三节实测「内层固定 50 次与收敛判据结果相同」。
    """
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
        if cur <= 0:
            break
        if abs(cur - old_val) / cur < tol:
            break
        old_val = cur
    return sol


def fista_ls(P, lam, u0, max_iter=900, tol=2.4e-6):
    """论文原版：带回溯线搜索的 FISTA（paper_fista.py 的忠实照搬）。"""
    ATA, ATm, dA, dm = P["ATA"], P["ATb"], P["dA"], P["dm"]
    x_old = u0.copy(); y_old = u0.copy()
    t_old = 1.0; step = 1e-5
    x_new = x_old
    it = 0

    def G(v):
        r = dA @ v - dm
        return 0.5 * float(r @ r)

    for it in range(max_iter):
        g = ATA @ y_old - ATm
        x_new = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                           lam * step).reshape(-1)
        t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2))
        y_new = x_new + (t_old - 1.0) / t_new * (x_new - x_old)

        c = G(x_new); oldc = G(y_old); n_ls = 0
        while c > oldc + float(g @ (x_new - y_old)) + \
                (1.0 / (2.0 * step)) * float(np.linalg.norm(x_new - y_old) ** 2):
            step *= 0.3
            x_new = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                               lam * step).reshape(-1)
            y_new = x_new + (t_old - 1.0) / t_new * (x_new - x_old)
            c = G(x_new); n_ls += 1
            if n_ls > 60:
                break

        e = float(np.linalg.norm(x_new - x_old)) / NH
        if it > 10 and e < tol:
            break
        x_old = x_new; t_old = t_new; y_old = y_new
        step *= 5.0
    return x_new, it + 1


def fista_fixed(P, lam, u0, max_iter=900, n_prox=100):
    """固定步长 FISTA，步长 = 1/‖A_d‖²（X15f 口径）。"""
    ATA, ATm = P["ATA"], P["ATb"]
    step = 1.0 / P["s2"]
    x_old = u0.copy(); y_old = u0.copy(); t_old = 1.0
    x_new = x_old
    for it in range(max_iter):
        g = ATA @ y_old - ATm
        x_new = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                           lam * step, max_iter=n_prox).reshape(-1)
        t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2))
        y_new = x_new + (t_old - 1.0) / t_new * (x_new - x_old)
        x_old = x_new; t_old = t_new; y_old = y_new
    return x_new, max_iter


def _grad(f2d):
    g = np.zeros((2,) + f2d.shape)
    g[0, :-1, :] = f2d[1:, :] - f2d[:-1, :]
    g[1, :, :-1] = f2d[:, 1:] - f2d[:, :-1]
    return g


def _div(g):
    out = np.zeros(g.shape[1:])
    out[:-1, :] -= g[0, :-1, :]
    out[1:, :] += g[0, :-1, :]
    out[:, :-1] -= g[1, :, :-1]
    out[:, 1:] += g[1, :, :-1]
    return -out


def _cg(op, b, x0, n_iter):
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


def admm_cg(P, lam, rho, n_outer=400, n_cg=15):
    """CG 内迭代 + TV 分裂，**在未归一化的问题上直接求解**。

    第二版修订：不再做 ATA/s2、ATb/s 的归一化与末尾乘 s 还原。首版沿用
    x15c_render 的写法，还原时差一个 s² 因子，会把解整体放大 1.42e7 倍。
    直接解原问题就没有任何可写错的换算。
    """
    ATA, ATb = P["ATA"], P["ATb"]
    Lop = lambda u: -_div(_grad(u.reshape(SHAPE))).reshape(-1)  # noqa: E731
    op = lambda u: ATA @ u + rho * Lop(u)                       # noqa: E731
    u = np.zeros(NH)
    d = np.zeros((2,) + SHAPE)
    q = np.zeros((2,) + SHAPE)
    th = lam / rho if rho > 0 else 0.0
    for _ in range(n_outer):
        u = _cg(op, ATb - rho * _div(d - q).reshape(-1), u, n_cg)
        g = _grad(u.reshape(SHAPE))
        s_ = g + q
        nrm = np.sqrt(s_[0] ** 2 + s_[1] ** 2)
        d = s_ * np.maximum(0.0, 1.0 - th / np.maximum(nrm, 1e-30))
        q = q + g - d
    return u, n_outer


def pdhg(P, lam, n_iter=900):
    """PDHG 固定步长（对照，W 系列已否证），同样在未归一化问题上跑。"""
    ATA, ATb = P["ATA"], P["ATb"]
    sigma = 1.0 / P["s"]
    tau = 1.0 / P["s"]
    f = np.zeros(NH); f_bar = f.copy()
    p = np.zeros(ATb.size)
    q = np.zeros((2,) + SHAPE)
    for _ in range(n_iter):
        p = (p + sigma * (ATA @ f_bar - ATb)) / (1.0 + sigma)
        q = q + sigma * _grad(f_bar.reshape(SHAPE))
        nrm = np.sqrt(q[0] ** 2 + q[1] ** 2)
        sc = np.ones_like(nrm)
        nz = nrm > lam
        sc[nz] = lam / nrm[nz]
        q = q * sc
        f_new = f - tau * (ATA.T @ p - _div(q).reshape(-1))
        np.maximum(f_new, 0.0, out=f_new)
        f_bar = 2.0 * f_new - f
        f = f_new
    return f, n_iter


# ---------------------------------------------------------------- 问题与扰动


def perturb(dA, delta, rng):
    """注入相对 Frobenius 误差 delta 的乘性扰动。

    真实误差是几何性的（可见性边界整体错位），是**结构化**的；这里用的是逐元素
    乘性随机扰动，属对结构不敏感的代理。δ 只能读作「A 的相对保真度」。
    """
    if delta <= 0:
        return dA
    D = dA * rng.standard_normal(dA.shape)
    e = float(np.linalg.norm(D)) / float(np.linalg.norm(dA))
    if e <= 0:
        return dA
    return dA + (delta / e) * D


def make_problem(dA_use, dm):
    """把「用某个 A 重建某份数据」整体打包。"""
    ATA = dA_use.T @ dA_use
    ATb = dA_use.T @ dm
    s2 = float(np.linalg.eigvalsh(ATA)[-1])
    s = float(np.sqrt(s2))
    return {
        "dA": dA_use, "dm": dm,
        "ATA": ATA, "ATb": ATb,
        "s2": s2, "s": s,
        "lam_ref": REG_R / CSCALE_R,
        "rho_ref": float(np.mean(np.diag(ATA))),
    }


def run(name, pr, P, u0, cap):
    """pr 为参数字典：FISTA/PDHG 用 lam_fac；ADMM 另用 rho_fac。"""
    t0 = time.time()
    lf = pr["lam_fac"]
    if name == "FISTA-LS":
        u, nit = fista_ls(P, P["lam_ref"] * lf, u0, max_iter=cap(50, 900))
    elif name == "FISTA-FX":
        u, nit = fista_fixed(P, P["lam_ref"] * lf, u0, max_iter=cap(50, 900))
    elif name == "ADMM-15":
        u, nit = admm_cg(P, P["lam_ref"] * lf, P["rho_ref"] * pr["rho_fac"],
                         n_outer=cap(30, 400), n_cg=15)
    elif name == "ADMM-60":
        u, nit = admm_cg(P, P["lam_ref"] * lf, P["rho_ref"] * pr["rho_fac"],
                         n_outer=cap(30, 400), n_cg=60)
    elif name == "PDHG":
        u, nit = pdhg(P, P["lam_ref"] * lf, n_iter=cap(50, 900))
    else:
        raise ValueError(name)
    return u, nit, time.time() - t0


def ls_init(P):
    try:
        return np.linalg.solve(P["ATA"], P["ATb"])
    except np.linalg.LinAlgError:
        return np.linalg.lstsq(P["ATA"], P["ATb"], rcond=None)[0]


SOLVERS = ["FISTA-LS", "FISTA-FX", "ADMM-15", "ADMM-60", "PDHG"]
LAM_GRID = (0.1, 1.0, 10.0)
RHO_GRID = (0.1, 1.0, 10.0)


# ---------------------------------------------------------------- 主流程


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-iter", type=int, default=0,
                    help=">0 时覆盖各求解器迭代上限（快跑）")
    ap.add_argument("--seed", type=int, default=20260925)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    print("=" * 78)
    print("X15g（第二版）：本征病态 vs 模型误差 —— 求解器容忍度实验")
    print("=" * 78)

    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))

    u_true2d = gt[:, :, 0].astype(float)
    u_true = (CSCALE_R * u_true2d).reshape(-1)
    u_true_inf = float(np.max(np.abs(u_true)))
    # TV 基准用**放大到 u 量纲**的真值，否则 TV 比恒等于 CSCALE_R = 312.5
    u_true_tv = tv_value(u_true2d * CSCALE_R)

    dA = build_dA(A0)
    P0 = make_problem(dA, dA @ u_true)          # 自洽：数据由这个 A 生成
    P_real = make_problem(dA, dvec(rows_of(meas[:, :, 0], "F")))

    ev = np.linalg.eigvalsh(P0["ATA"])
    print(f"  dA {dA.shape}   ‖dA‖² = {P0['s2']:.6e}   mean(diag) = {P0['rho_ref']:.4e}")
    print(f"  cond(A_dᵀA_d) = {float(ev[-1] / ev[0]):.4e}"
          f"   (λmax {float(ev[-1]):.6e} / λmin {float(ev[0]):.6e})")
    print("  （X15 / w4_1 报的条件数 7.62e5，此处应对上）")
    print(f"  u_true 范围 [{u_true.min():.3e}, {u_true.max():.3e}]  "
          f"TV(u_true) = {u_true_tv:.4e}")

    u_ls0 = ls_init(P0)
    m_ls = metrics(u_ls0, u_true, u_true2d, u_true_inf, u_true_tv)
    print(f"  自洽性核对：δ=0 精确 LS  sm_err = {m_ls['sm_err']:.3e}  "
          f"r = {m_ls['r']:+.6f}  振幅比 = {m_ls['amp_ratio']:.4f}")

    it_cap = args.max_iter

    def cap(lo, hi):
        return min(hi, it_cap) if it_cap > 0 else hi

    def M(u):
        return metrics(u, u_true, u_true2d, u_true_inf, u_true_tv)

    rng = np.random.default_rng(args.seed)
    records = []
    store = {}

    # ================================================================
    # S1：δ = 0 上扫参，定下每个求解器自己的最优 (lam, 必要时 rho)
    # ================================================================
    print("\n" + "-" * 78)
    print("S1  参数选优（δ = 0，自洽无噪）")
    print("-" * 78)
    u0 = u_ls0
    best_pr = {}
    for nm in SOLVERS:
        grid = ([(lf, None) for lf in LAM_GRID] if nm.startswith("FISTA") or nm == "PDHG"
                else [(lf, rf) for lf in LAM_GRID for rf in RHO_GRID])
        cand = []
        for lf, rf in grid:
            pr = {"lam_fac": lf, "rho_fac": rf}
            u, nit, el = run(nm, pr, P0, u0, cap)
            m = M(u)
            cand.append((pr, m, nit, el))
            extra = "" if rf is None else f" ρ×{rf:<5g}"
            print(f"  {nm:<9} λ×{lf:<5g}{extra} sm_err={m['sm_err']:.3e} "
                  f"r={m['r']:+.4f} 振幅比={m['amp_ratio']:.3e} {el:.1f}s")
            sys.stdout.flush()
        pick = min(cand, key=lambda c: sel_key(c[1]))
        best_pr[nm] = pick[0]
        print(f"  -> {nm} 选 {pick[0]}（sm_err {sel_key(pick[1]):.3e}）")

    # ================================================================
    # S2：δ 扫描
    # ================================================================
    deltas = [0.0, 1e-6, 1e-3, 1e-2, 0.05, 0.31]
    print("\n" + "-" * 78)
    print("S2  δ 扫描（模型误差容忍度）")
    print("-" * 78)
    print(f"  {'δ':<8}{'求解器':<10}{'sm_err':>11}{'r':>9}{'振幅比':>11}"
          f"{'TV比':>9}{'IoU':>7}{'秒':>7}")
    for dl in deltas:
        dAp = perturb(dA, dl, rng)
        Pp = make_problem(dAp, P0["dm"])       # 数据始终来自 δ=0 的自洽生成
        Pp["rel_err"] = float(np.linalg.norm(dAp - dA) / np.linalg.norm(dA))
        u0p = ls_init(Pp)
        m0 = M(u0p)
        print(f"  {dl:<8g}{'[LS]':<10}{m0['sm_err']:>11.3e}{m0['r']:>9.4f}"
              f"{m0['amp_ratio']:>11.3e}{m0['tv_ratio']:>9.3f}{m0['iou']:>7.3f}")
        store[(dl, "LS")] = u0p.reshape(SHAPE)
        for nm in SOLVERS:
            u, nit, el = run(nm, best_pr[nm], Pp, u0p, cap)
            m = M(u)
            m.update({"delta": dl, "rel_err_injected": Pp["rel_err"],
                      "solver": nm, "params": best_pr[nm],
                      "iters": nit, "seconds": el, "case": "synth"})
            records.append(m)
            store[(dl, nm)] = u.reshape(SHAPE)
            print(f"  {dl:<8g}{nm:<10}{m['sm_err']:>11.3e}{m['r']:>9.4f}"
                  f"{m['amp_ratio']:>11.3e}{m['tv_ratio']:>9.3f}"
                  f"{m['iou']:>7.3f}{el:>7.1f}")
            sys.stdout.flush()

    # ================================================================
    # S3：真实数据锚点（核对本脚本与 X15 一致）
    # ================================================================
    print("\n" + "-" * 78)
    print("S3  真实数据锚点")
    print("-" * 78)
    u0r = ls_init(P_real)
    m0 = M(u0r)
    store[("real", "LS")] = u0r.reshape(SHAPE)
    print(f"  {'真实':<10}{'LS':<10}sm_err={m0['sm_err']:.3e}  r={m0['r']:+.4f}  "
          f"振幅比={m0['amp_ratio']:.3e}  振幅={m0['u_inf']:.3e}")
    print("  （X15 报告写的是「LS 振幅 3.1e10，放大 4e5 倍」；本脚本第二版应为 1e3 量级）")
    for nm in SOLVERS:
        u, nit, el = run(nm, best_pr[nm], P_real, u0r, cap)
        m = M(u)
        m.update({"delta": None, "solver": nm, "params": best_pr[nm],
                  "iters": nit, "seconds": el, "case": "real"})
        records.append(m)
        store[("real", nm)] = u.reshape(SHAPE)
        print(f"  {'真实':<10}{nm:<10}sm_err={m['sm_err']:.3e}  r={m['r']:+.4f}  "
              f"振幅比={m['amp_ratio']:.3e}  TV比={m['tv_ratio']:.3f}  {el:.1f}s")
        sys.stdout.flush()
    store[("truth", "")] = u_true2d

    # ================================================================
    # 落盘
    # ================================================================
    os.makedirs(OUT, exist_ok=True)
    payload = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "version": 2,
        "note": "自洽合成 + δ 扫描；δ 为乘性随机扰动的相对 Frobenius 误差；"
                "ADMM/PDHG 在未归一化问题上求解",
        "cond_ATA": float(ev[-1] / ev[0]),
        "norm_sq_dA": P0["s2"],
        "mean_diag_ATA": P0["rho_ref"],
        "u_true_inf": u_true_inf,
        "u_true_tv": u_true_tv,
        "ls_selfcheck": m_ls,
        "ls_real": m0,
        "best_params": best_pr,
        "records": records,
    }
    jp = os.path.join(OUT, "x15g_tolerance.json")
    with io.open(jp, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print(f"\n已保存 {jp}")

    if not args.no_plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
            matplotlib.rcParams["axes.unicode_minus"] = False
            import matplotlib.pyplot as plt

            def show(ax, img):
                # 用分位裁剪做显示，避免个别离群像素把整幅冲成灰（首版吃过这个亏）
                lo, hi = np.percentile(img, [1.0, 99.5])
                if hi <= lo:
                    hi = lo + 1e-30
                ax.imshow(img, cmap="gray", vmin=lo, vmax=hi)
                ax.set_xticks([]); ax.set_yticks([])

            cols = 2 + len(SOLVERS)
            rows = len(deltas)
            fig, axes = plt.subplots(rows, cols, figsize=(1.95 * cols, 2.05 * rows))
            for r_i, dl in enumerate(deltas):
                show(axes[r_i][0], u_true2d * CSCALE_R)
                axes[r_i][0].set_ylabel(f"δ={dl:g}", fontsize=10)
                if r_i == 0:
                    axes[r_i][0].set_title("真值", fontsize=10)
                show(axes[r_i][1], store[(dl, "LS")])
                if r_i == 0:
                    axes[r_i][1].set_title("最小二乘", fontsize=10)
                for c_i, nm in enumerate(SOLVERS):
                    ax = axes[r_i][c_i + 2]
                    show(ax, store[(dl, nm)])
                    if r_i == 0:
                        ax.set_title(nm, fontsize=10)
            fig.tight_layout()
            p1 = os.path.join(OUT, "x15g_tolerance_grid.png")
            fig.savefig(p1, dpi=100)
            print(f"已保存 {p1}")

            fig2, axes2 = plt.subplots(2, 2 + len(SOLVERS),
                                       figsize=(1.95 * (2 + len(SOLVERS)), 4.6))
            show(axes2[0][0], u_true2d * CSCALE_R)
            axes2[0][0].set_title("真值", fontsize=9)
            show(axes2[1][0], store[("real", "LS")])
            axes2[1][0].set_title("真实数据 LS", fontsize=9)
            for c_i, nm in enumerate(SOLVERS):
                show(axes2[0][c_i + 1], store[("real", nm)])
                rr = [x for x in records
                      if x["solver"] == nm and x.get("case") == "real"]
                axes2[0][c_i + 1].set_title(
                    f"{nm}\nsm_err={rr[0]['sm_err']:.2e} r={rr[0]['r']:+.3f}" if rr else nm,
                    fontsize=9)
                axes2[1][c_i + 1].axis("off")
            axes2[0][-1].axis("off"); axes2[1][-1].axis("off")
            fig2.tight_layout()
            p2 = os.path.join(OUT, "x15g_real_anchor.png")
            fig2.savefig(p2, dpi=100)
            print(f"已保存 {p2}")
        except Exception as e:  # noqa: BLE001
            print(f"绘图跳过：{e}")

    print("\n完成。")


if __name__ == "__main__":
    main()
