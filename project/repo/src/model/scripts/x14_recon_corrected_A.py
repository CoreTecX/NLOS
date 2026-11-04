# -*- coding: utf-8 -*-
"""X14 —— 用**修正后的 A** 重跑重建，出可辨认的 BU。

台账定义（`01_分析报告/00_分步任务与优先级.md` §四 第 17 行，**以该文件为准**）
--------------------------------------------------------------------------
| 编号 | 任务 | 依赖 | 判据 | 人日 |
|---|---|---|---|---|
| **X14** | 用修正后的 A 重跑重建，出可辨认的 BU | X4 | 相关系数高于当前 0.373 | 0.5 |

同一台账 §八 又给了判据口径：**一级目视可辨 BU > 二级结构指标 > 三级 r / IoU**，
且明文写「这条口径同时适用于 X14」。所以本脚本两个判据都报：
  一级：目视出图（`x14_recon_corrected.png`）
  数字：r（台账写的那条）+ sm_err / IoU / 振幅比（X15g 制度化的二级指标）

「修正后的 A」是谁 —— 这是一个必须写明的取舍
--------------------------------------------
台账把 X14 挂在 **X4**（"修正 forward.py 的几何、遮挡与视角因子"，判据是与 `simA`
相对误差 ≤ 1e-6）之后。但 2026-09-26 的总表订正已写明 **X4 尚未完成**，
所以「X4 修正后的 A」**不存在**。

现在仓库里唯一被实测支持的模型修正，是 **X4l 找到的「去掉视角因子的 z 镜像配对」**
（`sim/results/x4l_switch_ablation.md`）：它是 9 个开关配置里**唯一跨场景一致**的杠杆，
4 组拟合（2 域 × 2 场景）全部改善，方案口径定位从 0.1081/0.0529 m 推到 0.0438 m。

本脚本采用的修正就是这一条。它作用在**论文前向模型**（`forward_paper.py` 复现的
`simulate_A.m` 口径），而不是 `forward.py`（`forward_loc.py` 本轮被明确划为禁改）。
两处代码里的对应开关分别是：

  - `forward_paper.viewing_angle_factor(..., mz_reversed=True)`：复现原文的
    `wall_matr(3, end:-1:1)`（反向 z）
  - `forward_loc` 的 `mz_mirrored`（X4l 扫的那个开关）

**这条修正的代价要写清楚（T7）**：原文的 MATLAB 保存数组 `proj_yOnRangeA`
是按 z 镜像口径算的，去镜像之后**不可能**再维持 x4i 量到的 ±1% 一致。
即「复现论文数值」与「拟合实测图像」二者只能取一。见 `x14_recon_corrected.md`。

怎么做到「不重算 132 MB 的 A」
------------------------------
`simulate_block` 对视角因子 `MM` 是**严格线性**的：

    image = Σ_i MM ⊙ (vs2/‖v_i‖⁴) ⊙ vis_i
            └── 只有这一项含 MM，`vs2/tmp` 与可见性 `occ_img` 都不含 MM

而 `MM = outer(mz, mx)`，`mx`（x 向、指数 1）在两种口径下**相同**，只有 `mz`
（z 向、指数 18）换了下标。于是逐墙点比值

    ratio(vec_z) = mz_fix(vec_z) / mz_rev(vec_z)      ← 与 x 下标、与子采样点都无关

可以**逐列**乘到已存的 `paper_A_va1.npy` 上，得到与「整张重算 no-mirror A」在
浮点舍入内相同的矩阵。脚本里对 3 列做了直接重算核对（判据 1e-12）。

用法
----
    python src/model/scripts/x14_recon_corrected_A.py            # 完整
    python src/model/scripts/x14_recon_corrected_A.py --quick    # 冒烟
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
_MODEL = os.path.abspath(os.path.join(HERE, os.pardir))
if _MODEL not in sys.path:
    sys.path.insert(0, _MODEL)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

# 用**脚本版** paper_forward（它就是产出 paper_A_va1.npy 的那一份：含 D11 几何常量、
# make_geometry / make_occ_corner / simulate_A）。模块版 penumbrasight_model.forward_paper
# 是另一条独立实现（吃 forward.OpticalSetup），两者不要混用。
import paper_forward as FP                                    # noqa: E402

REPO = os.path.abspath(os.path.join(_MODEL, os.pardir, os.pardir))
VEC = os.path.join(REPO, "sim", "vectors")
OUT = os.path.join(REPO, "sim", "results")

# ---- 与 paper_fista.py / x15g / x15c_render 完全一致的口径常量（勿改） ----
N, ROWS, COLS = 126, 29, 36
SHAPE = (ROWS, COLS)
NH = ROWS * COLS
CSCALE_R = 312.5
REG_R = 1e07 * 5.0
LAM_REF = REG_R / CSCALE_R                 # = 160000.0，x15g 的 lam_ref

BASELINE_R = 0.373                         # 台账写的「当前 0.373」（paper_fista 口径）


# ================================================================ 既定口径算子


def drows(M):
    o = np.empty_like(M)
    o[:-1, :] = M[:-1, :] - M[1:, :]
    o[-1, :] = M[-1, :]
    return o


def dvec(v):
    o = np.empty_like(v)
    o[:-1] = v[:-1] - v[1:]
    o[-1] = v[-1]
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


# ================================================================ 度量


def pearson(a, b):
    a = np.asarray(a, float).ravel() - np.mean(a)
    b = np.asarray(b, float).ravel() - np.mean(b)
    d = np.linalg.norm(a) * np.linalg.norm(b)
    return float(a @ b / d) if d > 0 else float("nan")


def scale_matched_err(u, ref):
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
    un = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / un) if un else 0.0


# ================================================================ 求解器（照搬 x15g）


def co_prox_tv(b, lam, tol=1e-6, max_iter=100):
    """Beck & Teboulle 2009 FGP；x15g 用 max_iter=100（与收敛判据同结果）。"""
    def Lt(x):
        p = np.zeros_like(x)
        q = np.zeros_like(x)
        p[:-1, :] = -(x[1:, :] - x[:-1, :])
        q[:, :-1] = -(x[:, 1:] - x[:, :-1])
        return p, q

    def Lop(dx, dy):
        ix = np.zeros_like(dx)
        iy = np.zeros_like(dy)
        ix[1:, :] = dx[1:, :] - dx[:-1, :]
        iy[:, 1:] = dy[:, 1:] - dy[:, :-1]
        ix[0, :] = dx[0, :]
        iy[:, 0] = dy[:, 0]
        return ix + iy

    r = np.zeros_like(b)
    s = np.zeros_like(b)
    p_old = r.copy()
    q_old = s.copy()
    t_old = 1.0
    old_val = 0.0
    sol = b.copy()
    for _ in range(max_iter):
        sol = b - lam * Lop(r, s)
        tmpr, tmps = Lt(sol)
        r = r + (1.0 / (8.0 * lam)) * tmpr
        s = s + (1.0 / (8.0 * lam)) * tmps
        w = np.maximum(1.0, np.sqrt(r ** 2 + s ** 2))
        p = r / w
        q = s / w
        t = (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2)) / 2.0
        t_old = t
        r = p + ((t_old - 1.0) / t) * (p - p_old)
        s = q + ((t_old - 1.0) / t) * (q - q_old)
        p_old = p
        q_old = q
        cur = 0.5 * np.sum((b - sol) ** 2) + lam * np.sum(np.sqrt(tmpr ** 2 + tmps ** 2))
        if cur <= 0:
            break
        if abs(cur - old_val) / cur < tol:
            break
        old_val = cur
    return sol


def fista_ls(P, lam, u0, max_iter=1100, tol=2.4e-6):
    """论文原版：带回溯线搜索的 FISTA（paper_fista.py 的忠实照搬）。

    `paper_fista.py` 是「当前 0.373」这条基线的产出脚本，故这里的步长初值、
    回退/放大系数、迭代上限与它逐项一致，基线才可复算。
    """
    ATA, ATm, dA, dm = P["ATA"], P["ATb"], P["dA"], P["dm"]
    x_old = u0.copy()
    y_old = u0.copy()
    t_old = 1.0
    step = 1e-5
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

        c = G(x_new)
        oldc = G(y_old)
        n_ls = 0
        while c > oldc + float(g @ (x_new - y_old)) + \
                (1.0 / (2.0 * step)) * float(np.linalg.norm(x_new - y_old) ** 2):
            step *= 0.3
            x_new = co_prox_tv((y_old - step * g).reshape(SHAPE, order="C"),
                               lam * step).reshape(-1)
            y_new = x_new + (t_old - 1.0) / t_new * (x_new - x_old)
            c = G(x_new)
            n_ls += 1
            if n_ls > 60:
                break

        e = float(np.linalg.norm(x_new - x_old)) / NH
        if it > 10 and e < tol:
            break
        x_old = x_new
        t_old = t_new
        y_old = y_new
        step *= 5.0
    return x_new, it + 1


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
    """X15g 定案的 M5 求解器：ADMM（固定内层 CG + 逐像素收缩），未归一化直解。"""
    ATA, ATb = P["ATA"], P["ATb"]
    Lop = lambda u: -_div(_grad(u.reshape(SHAPE))).reshape(-1)   # noqa: E731
    op = lambda u: ATA @ u + rho * Lop(u)                        # noqa: E731
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


def make_problem(dA_use, dm):
    ATA = dA_use.T @ dA_use
    ATb = dA_use.T @ dm
    return {"dA": dA_use, "dm": dm, "ATA": ATA, "ATb": ATb,
            "lam_ref": LAM_REF, "rho_ref": float(np.mean(np.diag(ATA)))}


def ls_init(P):
    try:
        return np.linalg.solve(P["ATA"], P["ATb"])
    except np.linalg.LinAlgError:
        return np.linalg.lstsq(P["ATA"], P["ATb"], rcond=None)[0]


# ================================================================ 「修正后的 A」


def mz_ratio(geom, walln):
    """返回 (ratio, diag)。ratio 形状 (ROWS, walln)：第 mr 行是 z 索引上的乘性修正。

    对应 `forward_paper.viewing_angle_factor` 的 z 向量实参：
        原文（mz_reversed=True）: fov_z = wall_matr[2, ::-1]   ← 反向
        修正（no mirror）        : fov_z = wall_matr[2, :]     ← 不反向
    """
    wall_x = geom["wall_matr"][0, :]
    wall_z = geom["wall_matr"][2, :]
    mon_zdiscr = geom["mon_zdiscr"]
    blk = geom["illum_block"]
    D = FP.D

    ratio = np.empty((ROWS, walln), dtype=np.float64)
    diag = []
    for mr in range(ROWS):
        ref_z = float(mon_zdiscr[mr] - blk[1] / 2)
        mz_rev = np.cos(np.arctan((ref_z - wall_z[::-1]) / D)) ** 18
        mz_fix = np.cos(np.arctan((ref_z - wall_z) / D)) ** 18
        ratio[mr, :] = mz_fix / mz_rev
        diag.append({"mr": mr, "ref_z": ref_z,
                     "mz_rev_min": float(mz_rev.min()),
                     "mz_fix_min": float(mz_fix.min()),
                     "ratio_min": float((mz_fix / mz_rev).min()),
                     "ratio_max": float((mz_fix / mz_rev).max())})
    _ = wall_x
    return ratio, diag


def apply_mz_ratio(A0, ratio):
    """把 z 镜像修正乘到已存的 A 上。

    列序：`blockcount = (n_col-1-mc)*n_row + mr` → `mr = col % n_row`。
    行序：`A[:, c] = image[::-1, :].ravel()`（C 序）→ 线性下标 `idx = a'*walln + b`，
    其中 `a' = idx // walln`，而它对应的 image 行是 `a = walln-1-a'`。
    """
    walln = int(round(np.sqrt(A0.shape[0])))
    ncol_blk = A0.shape[1] // ROWS
    assert walln * walln == A0.shape[0], A0.shape
    assert ncol_blk * ROWS == A0.shape[1], A0.shape

    A2 = A0.reshape(walln, walln, -1)              # (a', b, col)
    rr = ratio[:, ::-1]                            # 行下标 a' = walln-1-a
    for col in range(A0.shape[1]):
        A2[:, :, col] *= rr[col % ROWS][:, None]
    return A0


def simulate_A_direct(geom, occ, n_discr, mz_reversed, wanted):
    """对**指定若干列**直接重算（照搬 forward_paper.simulate_A 的循环，只改 z 向量）。

    `wanted` = [(mc, mr), ...]，返回 {col_index: (15876,) 数组}。
    """
    wall_matr = geom["wall_matr"]
    wall_point = geom["wall_point"]
    wall_normal = geom["wall_normal"]
    walln = geom["walln_points"]
    mon_xdiscr = geom["mon_xdiscr"]
    mon_zdiscr = geom["mon_zdiscr"]
    blk = geom["illum_block"]
    masker = FP.PolyMasker(walln)

    fov_z = wall_matr[2, :] if not mz_reversed else wall_matr[2, ::-1]
    out = {}
    for mc, mr in wanted:
        mm = FP.viewing_angle_factor(
            np.array([mon_xdiscr[mc] - blk[0] / 2, 0.0, mon_zdiscr[mr] - blk[1] / 2]),
            wall_matr[0, :], fov_z, FP.D)
        lx = mon_xdiscr[mc] - np.arange(n_discr) * (blk[0] / n_discr)
        lz = mon_zdiscr[mr] - np.arange(n_discr) * (blk[1] / n_discr)
        gx, gz = np.meshgrid(lx, lz)
        light_pos = np.column_stack([gx.ravel(), np.zeros(n_discr * n_discr), gz.ravel()])
        img = FP.simulate_block(wall_matr, wall_point, geom, walln,
                                light_pos, occ, mm, masker)
        col = (COLS - 1 - mc) * ROWS + mr
        out[col] = img[::-1, :].ravel()
    return out


# ================================================================ 主流程


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="冒烟：压迭代上限")
    ap.add_argument("--identity-only", action="store_true",
                    help="只做「比值法 == 直接重算」这一条不变式的核对"
                         "（供 make test 用；缺向量文件时 SKIP 而不是失败）")
    ap.add_argument("--fista-cap", type=int, default=1100)
    ap.add_argument("--admm-cap", type=int, default=400)
    args = ap.parse_args()
    if args.quick:
        args.fista_cap = 60
        args.admm_cap = 40

    t_all = time.time()
    print("=" * 86)
    print("X14  用修正后的 A 重跑重建（修正 = X4l 的「去掉视角因子 z 镜像配对」）")
    print("=" * 86)

    a_path = os.path.join(VEC, "paper_A_va1.npy")
    if not os.path.exists(a_path):
        print("SKIP  缺少 %s（论文口径 A），本机无法核对" % a_path)
        return 0

    A0 = np.load(a_path)
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    geom = FP.make_geometry()
    occ = FP.make_occ_corner()

    u_true2d = gt[:, :, 0].astype(float)
    u_true = (CSCALE_R * u_true2d).reshape(-1)
    u_true_inf = float(np.max(np.abs(u_true)))
    u_true_tv = tv_value(u_true2d * CSCALE_R)

    ratio, rdiag = mz_ratio(geom, geom["walln_points"])
    print("\n[1/5] z 镜像修正表")
    print("  ref_z ∈ [%.4f, %.4f]  （mon_zdiscr − blk_z/2）"
          % (rdiag[0]["ref_z"], rdiag[-1]["ref_z"]))
    print("  ratio = mz_fix/mz_rev ∈ [%.6f, %.6f]，全表极值 [%.6f, %.6f]"
          % (rdiag[0]["ratio_min"], rdiag[0]["ratio_max"],
             float(ratio.min()), float(ratio.max())))

    # ---------------- 修正的正确性：与直接重算核对 ----------------
    print("\n[2/5] 修正构造的正确性核对（逐列直接重算 vs 比值法）")
    wanted = [(35, 0), (35, 14), (35, 28), (17, 7), (0, 20)]
    t0 = time.time()
    base_cols = simulate_A_direct(geom, occ, 6, True, wanted)
    fix_cols = simulate_A_direct(geom, occ, 6, False, wanted)
    A1 = A0.copy()
    apply_mz_ratio(A1, ratio)
    chk = []
    for mc, mr in wanted:
        col = (COLS - 1 - mc) * ROWS + mr
        b_ref = base_cols[col]
        f_ref = fix_cols[col]
        f_ratio = A1[:, col]
        rb = float(np.linalg.norm(A0[:, col] - b_ref) / max(np.linalg.norm(b_ref), 1e-300))
        rf = float(np.linalg.norm(f_ratio - f_ref) / max(np.linalg.norm(f_ref), 1e-300))
        chk.append({"col": col, "mc": mc, "mr": mr,
                    "rel_base": rb, "rel_fix": rf})
        print("  col %4d (mc=%2d, mr=%2d)  存值vs重算(base) %.3e   比值法vs重算(fix) %.3e"
              % (col, mc, mr, rb, rf))
    rel_worst = max(c["rel_fix"] for c in chk)
    rel_base_worst = max(c["rel_base"] for c in chk)
    identity_ok = rel_worst < 1e-12 and rel_base_worst < 1e-12
    print("  -> 比值法逐列最大相对偏差 %.3e（判据 1e-12）%s（用时 %.1f s）"
          % (rel_worst, "PASS" if identity_ok else "FAIL", time.time() - t0))

    A_fix = A1

    if args.identity_only:
        print("\n完成（--identity-only：只核对不变式，不跑重建）。")
        return 0 if identity_ok else 1

    # ---------------- 两套 A 上的重建 ----------------
    dm = dvec(rows_of(meas[:, :, 0], "F"))
    variants = [("A_paper (z 镜像, 现状)", A0), ("A_nomirror (X4l 修正)", A_fix)]
    lam_facs = (0.1, 1.0, 10.0)
    rows = []
    sols = {}

    print("\n[3/5] 差分算子与正则化矩阵")
    probs = {}
    for name, A in variants:
        dA = build_dA(A)
        P = make_problem(dA, dm)
        probs[name] = P
        ev = np.linalg.eigvalsh(P["ATA"])
        print("  %-26s ‖dA‖²=%.5e  mean(diag ATA)=%.4e  cond=%.4e"
              % (name, float(ev[-1]), P["rho_ref"], float(ev[-1] / ev[0])))

    print("\n[4/5] 重建（求解器与 X15g 定案一致；λ 以论文 REG_R/CSCALE_R 为基准扫 0.1/1/10）")
    for name, A in variants:
        P = probs[name]
        u0 = ls_init(P)
        m0 = {"r": pearson(u0, u_true), "sm_err": scale_matched_err(u0, u_true),
              "iou": iou_binarized(u0.reshape(SHAPE), u_true2d),
              "amp_ratio": float(np.max(np.abs(u0)) / u_true_inf), "u_inf": float(np.max(np.abs(u0)))}
        rows.append(dict(a=name, solver="LS(精确最小二乘)", lam_fac=None,
                         seconds=0.0, **m0))
        sols[(name, "LS(精确最小二乘)", None)] = u0.reshape(SHAPE)
        print("  %-26s %-16s lam=—        r=%+.4f  sm_err=%.3e"
              % (name, "LS", m0["r"], m0["sm_err"]))

        for lf in lam_facs:
            u, nit = fista_ls(P, LAM_REF * lf, u0, max_iter=args.fista_cap)
            m = {"r": pearson(u, u_true), "sm_err": scale_matched_err(u, u_true),
                 "iou": iou_binarized(u.reshape(SHAPE), u_true2d),
                 "amp_ratio": float(np.max(np.abs(u)) / u_true_inf),
                 "u_inf": float(np.max(np.abs(u))), "iters": nit}
            rows.append(dict(a=name, solver="FISTA-LS(论文)", lam_fac=lf, seconds=0.0, **m))
            sols[(name, "FISTA-LS(论文)", lf)] = u.reshape(SHAPE)
            print("  %-26s %-16s lam×%-5g r=%+.4f  sm_err=%.3e  iters=%d"
                  % (name, "FISTA-LS", lf, m["r"], m["sm_err"], nit))

        t0 = time.time()
        for lf in lam_facs:
            u, nit = admm_cg(P, LAM_REF * lf, P["rho_ref"] * 0.1,
                             n_outer=args.admm_cap, n_cg=15)
            m = {"r": pearson(u, u_true), "sm_err": scale_matched_err(u, u_true),
                 "iou": iou_binarized(u.reshape(SHAPE), u_true2d),
                 "amp_ratio": float(np.max(np.abs(u)) / u_true_inf),
                 "u_inf": float(np.max(np.abs(u))), "iters": nit}
            rows.append(dict(a=name, solver="ADMM(CG15, 定案)", lam_fac=lf, seconds=0.0, **m))
            sols[(name, "ADMM(CG15, 定案)", lf)] = u.reshape(SHAPE)
            print("  %-26s %-16s lam×%-5g r=%+.4f  sm_err=%.3e"
                  % (name, "ADMM-15", lf, m["r"], m["sm_err"]))
        print("     （%s 的 ADMM 三档共 %.1f s）" % (name, time.time() - t0))

    # ---------------- 判据 ----------------
    print("\n[5/5] 判据")
    base_paper = [x for x in rows if x["a"] == variants[0][0]]
    base_fix = [x for x in rows if x["a"] == variants[1][0]]
    best_base = max(base_paper, key=lambda x: x["r"])
    best_fix = max(base_fix, key=lambda x: x["r"])
    best_base_sm = min(base_paper, key=lambda x: x["sm_err"])
    best_fix_sm = min(base_fix, key=lambda x: x["sm_err"])
    # 同配置逐一对比（消除「不同配置互比」的口径风险）
    paired = []
    for key in [(r["solver"], r["lam_fac"]) for r in base_paper]:
        pb = next((x for x in base_paper if (x["solver"], x["lam_fac"]) == key), None)
        pf = next((x for x in base_fix if (x["solver"], x["lam_fac"]) == key), None)
        if pb and pf:
            paired.append({"solver": key[0], "lam_fac": key[1],
                           "r_base": pb["r"], "r_fix": pf["r"],
                           "dr": pf["r"] - pb["r"],
                           "sm_base": pb["sm_err"], "sm_fix": pf["sm_err"],
                           "improved_r": pf["r"] > pb["r"]})
    n_pair = len(paired)
    n_win = sum(1 for p in paired if p["improved_r"])

    print("  r 最佳（按 r 最高，逐 A 各自最优配置）")
    print("    A_paper    : %.4f  （%s，λ×%s）" % (best_base["r"], best_base["solver"], best_base["lam_fac"]))
    print("    A_nomirror : %.4f  （%s，λ×%s）" % (best_fix["r"], best_fix["solver"], best_fix["lam_fac"]))
    print("  台账判据：r 高于当前 %.3f -> %s（%.4f vs %.3f，Δ=%+.4f）"
          % (BASELINE_R, "PASS" if best_fix["r"] > BASELINE_R else "FAIL",
             best_fix["r"], BASELINE_R, best_fix["r"] - BASELINE_R))
    print("  同配置配对（%d 组）：A_nomirror 在 %d 组上 r 更高、%d 组更低"
          % (n_pair, n_win, n_pair - n_win))
    print("    r 变化 Δ 中位 %+.4f，最小 %+.4f，最大 %+.4f"
          % (float(np.median([p["dr"] for p in paired])),
             float(min(p["dr"] for p in paired)),
             float(max(p["dr"] for p in paired))))
    print("  sm_err 最佳（X15g 二级判据，越小越好）")
    print("    A_paper    : %.4e  （%s，λ×%s）" % (best_base_sm["sm_err"], best_base_sm["solver"], best_base_sm["lam_fac"]))
    print("    A_nomirror : %.4e  （%s，λ×%s）" % (best_fix_sm["sm_err"], best_fix_sm["solver"], best_fix_sm["lam_fac"]))

    # ---------------- 出图 ----------------
    png = os.path.join(OUT, "x14_recon_corrected.png")
    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt

        def show(ax, img, title):
            lo, hi = np.percentile(img, [1.0, 99.5])
            ax.imshow(img, cmap="gray", vmin=lo, vmax=hi if hi > lo else lo + 1e-30)
            ax.set_title(title, fontsize=9)
            ax.set_xticks([]); ax.set_yticks([])

        up_b = sols[(variants[0][0], best_base["solver"], best_base["lam_fac"])]
        up_f = sols[(variants[1][0], best_fix["solver"], best_fix["lam_fac"])]
        us_b = sols[(variants[0][0], best_base_sm["solver"], best_base_sm["lam_fac"])]
        us_f = sols[(variants[1][0], best_fix_sm["solver"], best_fix_sm["lam_fac"])]

        fig, axes = plt.subplots(2, 4, figsize=(12.4, 6.6))
        show(axes[0][0], u_true2d, "真值 R 通道（29×36）")
        show(axes[0][1], up_b, "A_paper %s λ×%g\nr=%+.4f  sm_err=%.2e"
             % (best_base["solver"], best_base["lam_fac"], best_base["r"], best_base["sm_err"]))
        show(axes[0][2], up_f, "A_nomirror %s λ×%g\nr=%+.4f  sm_err=%.2e"
             % (best_fix["solver"], best_fix["lam_fac"], best_fix["r"], best_fix["sm_err"]))
        axes[0][3].axis("off")
        axes[0][3].text(0.0, 0.9, "r 最高的一组\n\nA_paper    %+.4f\nA_nomirror %+.4f\nΔ=%+.4f"
                        % (best_base["r"], best_fix["r"], best_fix["r"] - best_base["r"]),
                        fontsize=10, va="top")
        show(axes[1][0], u_true2d, "真值 R 通道（同上）")
        show(axes[1][1], us_b, "A_paper %s λ×%g\nsm_err 最小：%.3e  r=%+.4f"
             % (best_base_sm["solver"], best_base_sm["lam_fac"], best_base_sm["sm_err"], best_base_sm["r"]))
        show(axes[1][2], us_f, "A_nomirror %s λ×%g\nsm_err 最小：%.3e  r=%+.4f"
             % (best_fix_sm["solver"], best_fix_sm["lam_fac"], best_fix_sm["sm_err"], best_fix_sm["r"]))
        show(axes[1][3], up_f - up_f.mean(), "A_nomirror 解（去均值后）")
        fig.tight_layout()
        fig.savefig(png, dpi=110)
        plt.close(fig)
        print("\n  已保存 %s" % png)

        # 与台账基线（paper_fista_grid.png）**同显示口径**的对照图：
        # 基线那张用的是裸 imshow（自动按 min/max 拉伸），所以这里也不做分位裁剪，
        # 这样「目视是否可辨 BU」这句话才是在同一个显示口径下说的。
        png2 = os.path.join(OUT, "x14_recon_corrected_vs_baseline.png")
        cfg = ("FISTA-LS(论文)", 1.0)      # 基线 0.373 就是这一档（方向 F、λ×1）
        fig2, axes2 = plt.subplots(1, 6, figsize=(19.5, 3.9))
        panels = [
            (u_true2d, "真值 R"),
            (sols[(variants[0][0],) + cfg], "A_paper FISTA-LS λ×1\nr=%+.3f（= 台账基线）"
             % next(x["r"] for x in rows if x["a"] == variants[0][0]
                    and (x["solver"], x["lam_fac"]) == cfg)),
            (sols[(variants[1][0],) + cfg], "A_nomirror FISTA-LS λ×1\nr=%+.3f"
             % next(x["r"] for x in rows if x["a"] == variants[1][0]
                    and (x["solver"], x["lam_fac"]) == cfg)),
            (sols[(variants[0][0], "LS(精确最小二乘)", None)], "A_paper 最小二乘\nr=%+.3f"
             % next(x["r"] for x in rows if x["a"] == variants[0][0]
                    and x["solver"].startswith("LS"))),
            (sols[(variants[1][0], "LS(精确最小二乘)", None)], "A_nomirror 最小二乘\nr=%+.3f"
             % next(x["r"] for x in rows if x["a"] == variants[1][0]
                    and x["solver"].startswith("LS"))),
            (sols[(variants[0][0], "ADMM(CG15, 定案)", 1.0)],
             "A_paper ADMM(CG15, 定案) λ×1\nr=%+.4f（现状下最高）"
             % next(x["r"] for x in rows if x["a"] == variants[0][0]
                    and x["solver"] == "ADMM(CG15, 定案)" and x["lam_fac"] == 1.0)),
        ]
        for i, (im, tt) in enumerate(panels):
            axes2[i].imshow(im, cmap="gray")     # 裸 imshow，与基线同口径
            axes2[i].set_title(tt, fontsize=9)
            axes2[i].set_xticks([]); axes2[i].set_yticks([])
        fig2.tight_layout()
        fig2.savefig(png2, dpi=110)
        plt.close(fig2)
        print("  已保存 %s" % png2)
    except Exception as e:  # noqa: BLE001
        print("\n  绘图跳过：%s" % e)

    # ---------------- 落盘 ----------------
    np.save(os.path.join(OUT, "x14_mz_ratio.npy"), ratio)
    payload = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "quick": bool(args.quick),
        "task": "X14 用修正后的 A 重跑重建；修正 = X4l 的去掉视角因子 z 镜像配对",
        "baseline_r_from_ledger": BASELINE_R,
        "correction": {
            "what": "forward_paper.viewing_angle_factor 的 z 向量由 wall_matr[2, ::-1] 改为 wall_matr[2, :]",
            "where_forward_loc": "对应 forward_loc 的 mz_mirrored=False（X4l）",
            "provenance": "sim/results/x4l_switch_ablation.md",
            "linear_in_MM": "simulate_block 对 MM 严格线性，故可逐列乘比值",
            "ratio_min": float(ratio.min()), "ratio_max": float(ratio.max()),
            "per_mr_ratio_range": [[d["ratio_min"], d["ratio_max"]] for d in rdiag],
            "ref_z_first": rdiag[0]["ref_z"], "ref_z_last": rdiag[-1]["ref_z"],
        },
        "identity_check": {"cols": chk, "rel_worst_fix": rel_worst,
                           "rel_worst_base": rel_base_worst, "ok": bool(identity_ok),
                           "threshold": 1e-12},
        "u_true_inf": u_true_inf, "u_true_tv": u_true_tv,
        "rows": rows, "paired": paired,
        "best_by_r": {"base": best_base, "fix": best_fix},
        "best_by_sm_err": {"base": best_base_sm, "fix": best_fix_sm},
        "criterion": {
            "ledger": "相关系数高于当前 0.373",
            "r_fix_best": best_fix["r"], "r_base_best": best_base["r"],
            "pass": bool(best_fix["r"] > BASELINE_R),
        },
    }
    jp = os.path.join(OUT, "x14_recon_corrected.json")
    with io.open(jp, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print("  已保存 %s" % jp)
    print("\n总用时 %.1f s" % (time.time() - t_all))
    return 0


if __name__ == "__main__":
    sys.exit(main())
