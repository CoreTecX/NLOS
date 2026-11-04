# -*- coding: utf-8 -*-
"""x4k / T1 —— 定位差分重测：按老师方案 `d = D_y y`、`B(p) = D_y A(p)`、`K = 8` 重跑。

为什么做这一轮
--------------
x4j3 结尾我给的结论是「退到 C（定位降级）」。读完老师的技术实现方案后**撤销**了那条建议，
因为方案 §2 的定式是

    y = A(p_o) f + b        d = D_y y        B(p_o) = D_y A(p_o)
    p̂_o = argmax_p Σ_c || U_{p,K}^T d_c ||²      K = 8

而我此前测的 11 个判据（x4j / x4j2 / x4j3）**没有一个**是这个形式：
它们全部作用在**未差分**的 `y` 上。差分这一层正是用来消掉直流亮场 `b` 的
——x4j2 已经量出「定位信息在交流分量里，被直流亮场挡住」。所以「退到 C」下早了。

本轮的职责只有一个：**判定差分 + K=8 这条方案口径到底活不活**。

关于 D_y 的一处必须先说清的差异（写进报告，不"顺手修正"）
--------------------------------------------------------
论文仓库里**没有给定位做差分**。唯一存在的差分算子在 `reconstruct_tv_it_cbg.m:4-12`：

    Diff = diag(ones(1,N),0) - diag(ones(1,N-1),1);     % N = size(A,1) = 墙像素数
    mr   = Diff * reshape(measurement(:,:,1),[],1);
    dA   = Diff * A;

它是作用在**列优先展平向量**上的 N×N 全局差分：`(Diff*x)_i = x_i - x_{i+1}`。
因为展平索引 `i = iz + npx*ix`，这个算子在**每一列的边界上会串到下一列的首行**
（`i = npx-1` 时差的是 `y[0, ix+1]`），是一条**环绕伪边**；而且输出维度仍是 N。
方案 §2 写的是「**真正的相邻行差分**，96×96 输入得到 95×96 = 9120 个差分测量」，
即把行数减一、不产生伪边 —— 这是**方案对论文那处粗率的修正**，本脚本实现的是方案口径。

预登记（先写下来，再看结果）
--------------------------
网格与 x4j 完全一致（9×7×5 = 315，Ndiscr_mon = 2，ds = 4），所以数字与 x4j 直接可比。
网格上离真值最近的点 = (0.475, 0.560, 0.215)，距离 **0.0157 m** —— 这是我能不能分辨
「≤ 0.03 m」的物理地板。

| 编号 | 数据 | 算子 | K | 通道合成 | 预登记预测 |
|---|---|---|---|---|---|
| `B0_paper_K1`  | `y`    | 无 | 1 | 逐通道取 argmax 再平均 | 失败（x4j 复现值） |
| `B1_K8_nodiff` | `y`    | 无 | 8 | 能量求和后 argmax | 失败（K 不是病根，直流才是） |
| **`T1a_diff_K8`** | `D_y y` | `L_z` | **8** | **能量求和后 argmax** | **待判 —— 本轮主角** |
| `T1b_diff_K8_mean` | `D_y y` | `L_z` | 8 | 逐通道取 argmax 再平均 | 待判（bu 场景内容只在 r，可能会更好） |
| `T1c_diff_r_K8` | `D_y y` | `L_z` | 8 | 只用 r 通道 | 待判 |
| `T1d_diff_K1`  | `D_y y` | `L_z` | 1 | 能量求和后 argmax | 待判（**差分单独的效果**，与 K 解耦） |
| `T1e_diff_sigma` | `D_y y` | `L_z` | σ=0.75 规则 | 能量求和后 argmax | 待判（差分后 K 会不会自己涨上去） |
| `T1f_diffx_K8` | `D_x y` | `L_x` | 8 | 能量求和后 argmax | 消融：方向错了会怎样 |

另有两条不依赖场景未知的 oracle 诊断（固定 `ground_truth`，只扫遮挡物位置）：

| 编号 | 预登记预测 |
|---|---|
| `O1_oracle_y` | 应复现 x4j2 的 **0.0108 m**（未差分时信息在交流分量里） |
| `O2_oracle_diff` | **关键**：差分后信息还在不在。若 `O2` 也落在网格地板上 → 差分没毁信息，只是估计器问题；若 `O2` 散掉 → 差分把信息削掉了 |

成功线沿用 x4j：**≤ 0.03 m 成功 / ≤ 0.05 m 边缘 / > 0.05 m 失败**。

一条解析注记（省一次实验）
------------------------
`B^T d = A^T D_y^T D_y y = A^T (L_z y)`，而截断最小二乘的残差
`||d - B x̂_K||² = ||d||² - Σ_{k≤K} (v_k^T B^T d)² / λ_k`。
所以**「残差判据」与「子空间能量判据」的 argmax 逐点相同**，不必单列。

成本
----
每个候选建一张 A（3969×1044）+ 3 个 Gram（`AᵀA`、`Aᵀ(L_z⊗I)A`、`Aᵀ(I⊗L_x)A`）
+ 3 次 24 阶子空间特征分解。315 个候选约 10 分钟。
`mz_mirrored` / `stick` / `view_angle` 全部保持 x4i 验过的忠实取值，**不动**。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(_HERE, os.pardir))
if _MODEL_DIR not in sys.path:
    sys.path.insert(0, _MODEL_DIR)
from penumbrasight_model import forward_loc as FL   # noqa: E402

REPO = os.path.abspath(os.path.join(_MODEL_DIR, os.pardir, os.pardir))
RESULTS = os.path.join(REPO, "sim", "results")

DATA_ROOT = r"D:\学习\26秋嵌入式\论文数据集\Ordinary-Camera-master\Data\TestPosD11"
SCENES = {
    # 场景名 -> (文件名, 论文 table_S1.m 里该场景的 n_iter / sigma_th2，仅供报告参考)
    "bu":       ("image_test_bur20.mat",      3, 0.5),
    "mushroom": ("image_test_mushroom20.mat", 3, 0.2),
    "rgb":      ("image_test_colbar20.mat",   4, 0.2),
    "smilehat": ("image_test_smilehat20.mat", 3, 0.5),
}

_TOPK = 64          # 特征分解请求的高端阶数。24 只够看谱形；64 才够做 **K 扫描**
                    # （差分后 σ=0.75 的 K 已经涨到 13，24 会把扫描截在半路）
_K_DEFAULT = 8      # 方案冻结值
_SIGMA = 0.75       # 论文全局轮的 σ


# ======================================================================
# D_y 的实现：1-D 相邻行差分，绝不物化成 (npx*(npx-1)) × npx² 的大矩阵
# ======================================================================
def diff_gram_1d(n: int) -> np.ndarray:
    """`DᵀD`，其中 `D` 是 n×n 的一阶相邻差分（`(Dx)[i] = x[i+1] - x[i]`，输出 n-1 维）。

    逐项推：`(DᵀD)[i,i] = 1`（两端）/ `2`（中间），`[i,i+1] = [i+1,i] = -1`。
    这与 `reconstruct_tv_it_cbg.m` 那个 N×N 环绕版**不同**（见模块 docstring）。
    """
    L = np.zeros((n, n), dtype=np.float64)
    idx = np.arange(n - 1)
    L[idx, idx] += 1.0
    L[idx + 1, idx + 1] += 1.0
    L[idx, idx + 1] -= 1.0
    L[idx + 1, idx] -= 1.0
    return L


def topk_eigh(G: np.ndarray, k: int):
    """对称半正定 G 的**最大** k 个特征对，按降序返回 (lam, V)。"""
    n = G.shape[0]
    k = int(min(k, n))
    try:
        from scipy.linalg import eigh as _eigh
        lam, V = _eigh(G, subset_by_index=[n - k, n - 1], driver="evr")
    except Exception:
        lam, V = np.linalg.eigh(G)
        lam, V = lam[-k:], V[:, -k:]
    return np.maximum(lam[::-1], 0.0), V[:, ::-1]


def sub_energy(lam: np.ndarray, V: np.ndarray, b: np.ndarray, K: int) -> float:
    """`||U(:,1:K)ᵀ b||²` 的 Gram 等价写法 `Σ_{k≤K} (v_kᵀb)² / λ_k`。

    推导：`A = U S Vᵀ` ⇒ `Uᵀb = S^{-1} Vᵀ (Aᵀb)`。这里传进来的 `b` 已经是 `Aᵀ·(·)`。
    """
    if K <= 0 or lam.size == 0:
        return 0.0
    k = int(min(K, lam.size))
    m = lam[:k] > 0.0
    if not m.any():
        return 0.0
    c = V[:, :k][:, m].T @ b
    return float(np.sum((c * c) / lam[:k][m]))


def sigma_K(lam: np.ndarray, sigma: float) -> int:
    """论文规则 `K = #{s_k / s_1 > sigma}`（在请求的 `_TOPK` 阶内）。"""
    if lam.size == 0 or lam[0] <= 0:
        return 0
    s = np.sqrt(lam)
    return int(np.sum(s / s[0] > sigma))


def fista_nnls(G: np.ndarray, b: np.ndarray, d2: float, L: float,
               iters: int = 1500):
    """`min_{x>=0} ||B x - d||²` 的加速投影梯度解，返回 (x, 残差, 收敛诊断)。

    展开：`||Bx-d||² = x'Gx - 2x'b + d2`（`G = B'B`、`b = B'd`）。
    步长 `1/L`（`L = λ_max(G)`）。加 Nesterov 外推 + 函数值回升即重启。

    收敛诊断 = 最后 200 步的相对函数值下降量。**投影梯度的解落在真解上界**，
    所以残差是"至少这么小"，不是精确 NNLS。这一点在报告里如实标注。
    """
    n = G.shape[0]
    x = np.zeros(n, dtype=np.float64)
    z = x.copy()
    t = 1.0
    hist = []
    for it in range(int(iters)):
        g = G @ z - b
        x_new = np.maximum(0.0, z - g / L)
        t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t * t))
        z = x_new + ((t - 1.0) / t_new) * (x_new - x)
        x, t = x_new, t_new
        if it >= int(iters) - 200:
            hist.append(float(x @ (G @ x) - 2.0 * (x @ b) + d2))
    r2 = float(x @ (G @ x) - 2.0 * (x @ b) + d2)
    f_prev = hist[0] if hist else r2
    drop = abs(f_prev - r2) / max(abs(f_prev), 1e-300)
    return x, max(r2, 0.0) ** 0.5, float(drop)


# ======================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="bu", choices=sorted(SCENES))
    ap.add_argument("--data-root", default=DATA_ROOT)
    ap.add_argument("--nx", type=int, default=9)
    ap.add_argument("--ny", type=int, default=7)
    ap.add_argument("--nz", type=int, default=5)
    ap.add_argument("--xlo", type=float, default=0.33)
    ap.add_argument("--xhi", type=float, default=0.62)
    ap.add_argument("--ylo", type=float, default=0.42)
    ap.add_argument("--yhi", type=float, default=0.70)
    ap.add_argument("--zlo", type=float, default=0.13)
    ap.add_argument("--zhi", type=float, default=0.30)
    ap.add_argument("--nmon", type=int, default=2, help="Ndiscr_mon（论文全局轮 = 2）")
    ap.add_argument("--downsample", type=int, default=4)
    ap.add_argument("--k", type=int, default=_K_DEFAULT)
    ap.add_argument("--sigma", type=float, default=_SIGMA)
    ap.add_argument("--nnls-iters", type=int, default=1200)
    # X4l 消融发现的开关：默认全部保持「忠实原文」，只在消融/复核时翻转
    ap.add_argument("--mz-mirror", action=argparse.BooleanOptionalAction, default=True,
                    help="视角因子的 z 轴镜像（忠实原文 = True；X4l 发现 False 拟合更好）")
    ap.add_argument("--x-reversed", action=argparse.BooleanOptionalAction, default=True,
                    help="块→显示器 x 序号反向（忠实原文 = True）")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    fn, n_iter_paper, sig2_paper = SCENES[args.scene]
    mat = os.path.join(args.data_root, fn)
    ds = args.downsample

    st = FL.d11_loc_setup()
    truth = np.asarray(st.occ_ll_true, dtype=float)
    meas, gt = FL.load_measurement(mat, ds)

    npx = FL.num_pixels_for(st, ds)
    nrow, ncol = st.num_blocks
    nb = nrow * ncol
    N = args.nmon

    gx = np.linspace(args.xlo, args.xhi, args.nx)
    gy = np.linspace(args.ylo, args.yhi, args.ny)
    gz = np.linspace(args.zlo, args.zhi, args.nz)
    shp = (args.nx, args.ny, args.nz)

    tag = args.tag or ("x4k_diff_%s" % args.scene)
    print("=" * 78)
    print("x4k / T1  定位差分重测   场景 = %s" % args.scene)
    print("=" * 78)
    print("测量   : %s  downsamp=%d  nMon=%d" % (fn, ds, N))
    print("墙网格 : npx=%d（%d 个像素）  差分后 %d×%d = %d 个测量"
          % (npx, npx * npx, npx - 1, npx, (npx - 1) * npx))
    print("显示器 : %d 块（%d×%d）" % (nb, nrow, ncol))
    print("真值   : [%.4f %.4f %.4f]（搜索坐标）" % tuple(truth))
    bp = np.array([gx[np.argmin(abs(gx - truth[0]))], gy[np.argmin(abs(gy - truth[1]))],
                   gz[np.argmin(abs(gz - truth[2]))]])
    floor = float(np.linalg.norm(bp - truth))
    print("网格   : %d×%d×%d = %d 个候选" % (args.nx, args.ny, args.nz, args.nx * args.ny * args.nz))
    print("         步长 %.4f / %.4f / %.4f" % (gx[1] - gx[0], gy[1] - gy[0], gz[1] - gz[0]))
    print("         离真值最近的网格点 %.4f %.4f %.4f，距离 %.6f m  <- 物理地板"
          % (bp[0], bp[1], bp[2], floor))
    print("口径   : K=%d  σ=%.2f（论文全局轮）" % (args.k, args.sigma))
    print()

    # ---- 测量侧：y、L_z y、L_x y -------------------------------------
    Lz = diff_gram_1d(npx)      # 作用在 z（快轴，A 行序里的 % npx）
    Lx = diff_gram_1d(npx)      # 作用在 x（慢轴）

    Y3 = np.stack([np.ascontiguousarray(meas[:, :, c].T) for c in range(3)], axis=-1)
    # Y3[ix, iz, c]：A 的行号 = ix*npx + iz，所以这样摆才能和 A 的行序对齐
    #
    # 两个方向的算子作用在不同轴上，这里极容易写错（首版就把 Lx 也乘到了 iz 上，
    # 于是"水平差分"与"垂直差分"给出**逐位相同**的结果）：
    #   L_z 作用在快轴 iz（A3 的第 1 轴），L_x 作用在慢轴 ix（A3 的第 0 轴）。
    LY_z = np.einsum("ibc,ab->iac", Y3, Lz)
    LY_x = (Lx @ Y3.reshape(npx, -1)).reshape(npx, npx, 3)

    Y0 = Y3.reshape(-1, 3)
    YZ = np.ascontiguousarray(LY_z).reshape(-1, 3)
    YX = np.ascontiguousarray(LY_x).reshape(-1, 3)

    # 差分后的测量能量（与遮挡物位置无关，一次算好）
    dnorm2 = np.array([float(Y0[:, c] @ YZ[:, c]) for c in range(3)])
    ynorm2 = np.array([float(Y0[:, c] @ Y0[:, c]) for c in range(3)])
    print("测量侧能量占比 ||D_y y||² / ||y||²  r/g/b = %s"
          % np.array2string(dnorm2 / ynorm2, precision=4))
    print("（这个比值很小是**好事**：说明直流亮场确实被差分吃掉了）")
    print()

    # ---- oracle 用：场景真值 -----------------------------------------
    xgt = np.stack([np.asarray(gt[:, :, c], dtype=float).reshape(-1, order="F")
                    for c in range(3)], axis=-1)          # (nb, 3)
    nz_gt = [int(np.count_nonzero(xgt[:, c])) for c in range(3)]
    print("oracle : ground_truth 非零块数 r/g/b = %s / %d" % (nz_gt, nb))

    plan = FL.make_plan(st, N, downsamp_factor=ds,
                        mz_mirrored=args.mz_mirror, x_reversed=args.x_reversed)
    print("plan   : npx=%d blocks=%d chunk=%d  mz_mirrored=%s x_reversed=%s"
          % (plan.npx, plan.n_blocks, plan.block_chunk,
             plan.mz_mirrored, plan.x_reversed))
    print()

    # ---- 判据表（预登记的顺序，逐字对应模块 docstring 的表）----------
    CRIT = [
        # id, 哪种数据/算子, K（-1 = σ 规则）
        ("B0_paper_K1",     "none", 1),
        ("B1_K8_nodiff",    "none", args.k),
        ("T1a_diff_K8",     "z",    args.k),
        ("T1b_diff_K8_mean", "z",   args.k),
        ("T1c_diff_r_K8",   "z",    args.k),
        ("T1d_diff_K1",     "z",    1),
        ("T1e_diff_sigma",  "z",    -1),
        ("T1f_diffx_K8",    "x",    args.k),
    ]
    surf = {c[0]: np.zeros(shp + (3,)) for c in CRIT}
    # 预登记之外、但由 oracle 结果（§O4 差分后残差判据命中网格地板）直接推出来的
    # 第二条路：**非负最小二乘残差**。它是截断 SVD 家族**之外**的判据 ——
    # 残差 = ‖d‖² − Σ_{k≤K}(v_kᵀb)²/λ_k 与子空间能量逐点同 argmax，所以
    # 只要还待在截断 SVD 家族里，换 K / 换加权都救不了；唯一没被这层等价性
    # 覆盖的先验就是「f ≥ 0」。x4j 在**未差分**域试过（C3），这里在差分域重试。
    EXTRA = [("T1g_diff_nnls", False)]          # (名字, 是否求最大)
    surf["T1g_diff_nnls"] = np.zeros(shp)       # 残差（越小越好）
    nnls_drop = np.zeros(shp)                   # 收敛诊断
    nnls_nnz = np.zeros(shp, dtype=int)

    Ksig = np.zeros(shp, dtype=int)
    Ksig_nodiff = np.zeros(shp, dtype=int)
    # K 扫描用的投影缓存（沿用 X11 那条「早停轨迹 = 长轨迹的前缀」口径）：
    # 存下所有 λ_k 与 v_kᵀb，于是**任意 K、任意通道合成、任意加权都事后可算**，
    # 不必为了每个 K 重跑一遍。OPS 顺序 = ["none"(未差分), "z"(垂直差分), "x"(水平差分)]
    OPS = ["none", "z", "x"]
    proj = np.zeros((3,) + shp + (3, _TOPK))
    lamv = np.zeros((3,) + shp + (_TOPK,))
    # oracle 曲面
    or_corr_nodiff = np.zeros(shp)
    or_relres_nodiff = np.zeros(shp)
    or_corr_diff = np.zeros(shp)
    or_relres_diff = np.zeros(shp)
    # 真值位置上的谱（用于报告）
    spec_truth = {}
    Ksig_truth = 0
    Ksig_truth_nodiff = 0
    seen_truth = False

    t0 = time.time()
    n = 0
    for ix in range(args.nx):
        for iy in range(args.ny):
            for iz in range(args.nz):
                p = [gx[ix], gy[iy], gz[iz]]
                A = FL.build_A_from_plan(plan, p)
                A3 = A.reshape(npx, npx, nb)            # [ix, iz, j]
                Ar = A3.reshape(npx * npx, nb)
                Az = np.ascontiguousarray(
                    np.einsum("ibj,ab->iaj", A3, Lz)).reshape(npx * npx, nb)
                # L_x 乘在第 0 轴（ix）上，不能照抄 L_z 那个 einsum
                Ax = (Lx @ A3.reshape(npx, npx * nb)).reshape(npx * npx, nb)

                G0 = Ar.T @ Ar
                Gz = Az.T @ Ar
                Gx = Ax.T @ Ar
                for G in (G0, Gz, Gx):
                    G += G.T
                    G *= 0.5

                b0 = Ar.T @ Y0
                bz = Ar.T @ YZ
                bx = Ar.T @ YX

                lam0, V0 = topk_eigh(G0, _TOPK)
                lamz, Vz = topk_eigh(Gz, _TOPK)
                lamx, Vx = topk_eigh(Gx, _TOPK)

                Ks = sigma_K(lamz, args.sigma)
                Ks0 = sigma_K(lam0, args.sigma)
                Ksig[ix, iy, iz] = Ks
                Ksig_nodiff[ix, iy, iz] = Ks0

                for name, which, K in CRIT:
                    if which == "none":
                        lam, V, b = lam0, V0, b0
                    elif which == "z":
                        lam, V, b = lamz, Vz, bz
                    else:
                        lam, V, b = lamx, Vx, bx
                    Kx = K if K >= 0 else sigma_K(lam, args.sigma)
                    for c in range(3):
                        surf[name][ix, iy, iz, c] = sub_energy(lam, V, b[:, c], Kx)

                # ---- 投影缓存（供事后 K 扫描）----
                for oi, (lam, V, b) in enumerate(
                        ((lam0, V0, b0), (lamz, Vz, bz), (lamx, Vx, bx))):
                    lamv[oi, ix, iy, iz, :] = lam
                    for c in range(3):
                        proj[oi, ix, iy, iz, c, :] = V.T @ b[:, c]

                # ---- 非负最小二乘残差（差分域，只用 r 通道：bu 场景内容全在 r）----
                _xh, _rr, _dp = fista_nnls(Gz, bz[:, 0], float(dnorm2[0]),
                                           float(lamz[0]), iters=args.nnls_iters)
                surf["T1g_diff_nnls"][ix, iy, iz] = _rr
                nnls_drop[ix, iy, iz] = _dp
                nnls_nnz[ix, iy, iz] = int(np.count_nonzero(_xh))

                # ---- oracle ----
                yp = Ar @ xgt[:, 0]                       # A(p) @ x_gt（未差分）
                P3 = yp.reshape(npx, npx)                 # [ix, iz]
                dp = np.diff(P3, axis=1)                  # D_y(A x_gt)
                dm = np.diff(Y3[:, :, 0], axis=1)         # D_y y
                if np.linalg.norm(yp) > 0:
                    or_corr_nodiff[ix, iy, iz] = float(np.corrcoef(yp, Y0[:, 0])[0, 1])
                    al = float(yp @ Y0[:, 0] / (yp @ yp))
                    or_relres_nodiff[ix, iy, iz] = float(
                        np.linalg.norm(Y0[:, 0] - al * yp) / np.linalg.norm(Y0[:, 0]))
                else:
                    or_corr_nodiff[ix, iy, iz] = -1.0
                    or_relres_nodiff[ix, iy, iz] = 1.0
                if np.linalg.norm(dp) > 0:
                    or_corr_diff[ix, iy, iz] = float(
                        np.corrcoef(dp.ravel(), dm.ravel())[0, 1])
                    al = float(dp.ravel() @ dm.ravel() / (dp.ravel() @ dp.ravel()))
                    or_relres_diff[ix, iy, iz] = float(
                        np.linalg.norm(dm - al * dp) / np.linalg.norm(dm))
                else:
                    or_corr_diff[ix, iy, iz] = -1.0
                    or_relres_diff[ix, iy, iz] = 1.0

                # 真值最近网格点上记一次谱
                if not seen_truth and abs(gx[ix] - bp[0]) < 1e-12 \
                        and abs(gy[iy] - bp[1]) < 1e-12 and abs(gz[iz] - bp[2]) < 1e-12:
                    seen_truth = True
                    Ksig_truth = Ks
                    Ksig_truth_nodiff = Ks0
                    spec_truth = dict(
                        A_s_ratio=(np.sqrt(lam0) / np.sqrt(lam0[0])).tolist(),
                        B_s_ratio=(np.sqrt(lamz) / np.sqrt(lamz[0])).tolist(),
                        K_sigma_B=Ks, K_sigma_A=Ks0,
                        # 列间相关：A 的近秩 1 本质残不残
                        A_colcos=float(np.mean([np.dot(A[:, j], A[:, j + 1]) /
                                                (np.linalg.norm(A[:, j]) * np.linalg.norm(A[:, j + 1]))
                                                for j in range(0, nb - 1, 37)])),
                        B_colcos=float(np.mean([np.dot(Az[:, j], Az[:, j + 1]) /
                                                (np.linalg.norm(Az[:, j]) * np.linalg.norm(Az[:, j + 1]))
                                                for j in range(0, nb - 1, 37)])),
                    )

                n += 1
        print("  x=%d/%d  %.1f s  （%d 个候选）" % (ix + 1, args.nx, time.time() - t0, n))

    print("\n完成，%d 个候选，用时 %.1f s\n" % (n, time.time() - t0))

    # ==================================================================
    # 汇总
    # ==================================================================
    def _argmax(s, ch=None):
        a = s if ch is None else s[..., ch]
        return np.unravel_index(int(np.argmax(a)), shp)

    def _pos(idx):
        return np.array([gx[idx[0]], gy[idx[1]], gz[idx[2]]])

    def _gap(s):
        f = np.sort(s.ravel())[::-1]
        return float((f[0] - f[1]) / abs(f[0])) if f[0] else float("nan")

    def _verdict(d):
        return "成功" if d <= 0.03 else ("边缘" if d <= 0.05 else "失败")

    # 离真值最近的网格点（0 基索引），供各表报「真值网格点取值」
    ti = (int(np.argmin(abs(gx - truth[0]))), int(np.argmin(abs(gy - truth[1]))),
          int(np.argmin(abs(gz - truth[2]))))

    rows = []
    print("=" * 78)
    print("结果（成功线 ≤ 0.03 m；边缘 ≤ 0.05 m；> 0.05 m 记失败；物理地板 %.4f m）" % floor)
    print("=" * 78)
    hdr = "%-18s %-26s %-10s %-10s %s" % ("判据", "argmax（搜索坐标）", "到真值(m)", "top1/top2", "判定")
    print(hdr)
    print("-" * len(hdr))

    for name, which, K in CRIT:
        s = surf[name]
        # 方案 §2 口径：三通道能量求和后单次 argmax
        p_sum = _pos(_argmax(s.sum(axis=-1)))
        d_sum = float(np.linalg.norm(p_sum - truth))
        g_sum = _gap(s.sum(axis=-1))
        # 论文风格：逐通道 argmax 再平均
        p_ch = np.stack([_pos(_argmax(s, c)) for c in range(3)]).mean(axis=0)
        d_ch = float(np.linalg.norm(p_ch - truth))
        # 只用 r 通道
        p_r = _pos(_argmax(s, 0))
        d_r = float(np.linalg.norm(p_r - truth))

        print("%-18s [%.4f %.4f %.4f]   %-10.6f %-10.3g %s"
              % (name + "_sum", p_sum[0], p_sum[1], p_sum[2], d_sum, g_sum, _verdict(d_sum)))
        print("%-18s [%.4f %.4f %.4f]   %-10.6f %-10s %s"
              % ("  └ mean-ch", p_ch[0], p_ch[1], p_ch[2], d_ch, "-", _verdict(d_ch)))
        print("%-18s [%.4f %.4f %.4f]   %-10.6f %-10s %s"
              % ("  └ r-only", p_r[0], p_r[1], p_r[2], d_r, "-", _verdict(d_r)))
        rows.append(dict(name=name, data=which, K=(K if K >= 0 else "sigma"),
                         p_sum=p_sum.tolist(), d_sum=d_sum, gap_sum=g_sum,
                         p_mean=p_ch.tolist(), d_mean=d_ch,
                         p_r=p_r.tolist(), d_r=d_r,
                         verdict_sum=_verdict(d_sum), verdict_mean=_verdict(d_ch),
                         verdict_r=_verdict(d_r)))

    # ---- 第二族：非负最小二乘残差（越小越好）----
    for name, maximize in EXTRA:
        s = surf[name]
        i = np.unravel_index(int(np.argmin(s)), shp)
        p_sum = _pos(i)
        d_sum = float(np.linalg.norm(p_sum - truth))
        vals = np.sort(s.ravel())
        gap = float((vals[1] - vals[0]) / abs(vals[0])) if vals[0] else float("nan")
        print("%-18s [%.4f %.4f %.4f]   %-10.6f %-10.3g %s   （残差 %.6f / 最小值 %.6f）"
              % (name + "_argmin", p_sum[0], p_sum[1], p_sum[2], d_sum, gap,
                 _verdict(d_sum), float(s[ti]), float(s.min())))
        print("%-18s 收敛诊断（末 200 步相对下降）中位 %.3e  最大 %.3e ；非零解块数 中位 %d"
              % ("  └ diag", float(np.median(nnls_drop)), float(nnls_drop.max()),
                 int(np.median(nnls_nnz))))
        rows.append(dict(name=name, data="z", K="nnls", maximize=False,
                         p_sum=p_sum.tolist(), d_sum=d_sum, gap_sum=gap,
                         verdict_sum=_verdict(d_sum),
                         truth_pt=float(s[ti]), grid_min=float(s.min()),
                         drop_median=float(np.median(nnls_drop)),
                         drop_max=float(nnls_drop.max()),
                         nnz_median=int(np.median(nnls_nnz))))

    # ---- oracle 汇总 ----
    print()
    print("=" * 78)
    print("oracle（固定 ground_truth，只扫遮挡物位置 —— 与估计器无关）")
    print("=" * 78)
    orc = []
    for nm, cf in (("O1_oracle_y     未差分 corr", or_corr_nodiff),
                   ("O2_oracle_diff  差分后 corr", or_corr_diff)):
        maximize = True
        i = np.unravel_index(int(np.argmax(cf) if maximize else np.argmin(cf)), shp)
        p = _pos(i)
        d = float(np.linalg.norm(p - truth))
        tv = float(cf[ti])
        best = float(cf.max() if maximize else cf.min())
        print("%-28s argmax=[%.4f %.4f %.4f]  距真值 %.6f m" % (nm, p[0], p[1], p[2], d))
        print("     真值网格点 %.6f  vs  全网格最优 %.6f  -> 真值位置%s最优  %s"
              % (tv, best, "是" if abs(tv - best) < 1e-12 else "不是",
                 "✅ 信息在" if abs(tv - best) < 1e-12 else "❌ 信息不在这个判据里"))
        print("     全网格 corr: min %.6f  max %.6f  mean %.6f" % (cf.min(), cf.max(), cf.mean()))
        orc.append(dict(name=nm, p=p.tolist(), d=d, truth_pt=tv, grid_best=best,
                        is_truth_best=bool(abs(tv - best) < 1e-12)))
    for nm, rf, maximize in (("O3_relres 未差分", or_relres_nodiff, False),
                             ("O4_relres 差分后", or_relres_diff, False)):
        i = np.unravel_index(int(np.argmin(rf)), shp)
        p = _pos(i)
        d = float(np.linalg.norm(p - truth))
        tv = float(rf[ti])
        best = float(rf.min())
        print("%-28s argmin=[%.4f %.4f %.4f]  距真值 %.6f m" % (nm, p[0], p[1], p[2], d))
        print("     真值网格点 %.6f  vs  全网格最优 %.6f  -> 真值位置%s最优"
              % (tv, best, "是" if abs(tv - best) < 1e-12 else "不是"))
        orc.append(dict(name=nm, p=p.tolist(), d=d, truth_pt=tv, grid_best=best,
                        is_truth_best=bool(abs(tv - best) < 1e-12)))

    # ---- 谱 ----
    print()
    print("=" * 78)
    print("谱（真值最近网格点上）")
    print("=" * 78)
    print("A  的 s_k/s_1 前 12 个: %s" % np.array2string(
        np.array(spec_truth["A_s_ratio"][:12]), precision=4, max_line_width=200))
    print("B  的 s_k/s_1 前 12 个: %s" % np.array2string(
        np.array(spec_truth["B_s_ratio"][:12]), precision=4, max_line_width=200))
    print("σ=%.2f 规则的 K ： A -> %d ； B -> %d    （σ=%.2f）"
          % (args.sigma, Ksig_truth_nodiff, Ksig_truth, args.sigma))
    print("列间 cos 抽样均值： A %.6f  ->  B %.6f" % (spec_truth["A_colcos"], spec_truth["B_colcos"]))
    print("K(σ) 在 315 个候选上的取值：未差分 %s ；差分 %s"
          % (np.unique(Ksig_nodiff).tolist(), np.unique(Ksig).tolist()))

    out = os.path.join(RESULTS, tag + ".json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(dict(scene=args.scene, mat=mat, downsample=ds, nmon=N,
                       npx=npx, nblocks=nb, truth=truth.tolist(),
                       grid=dict(gx=gx.tolist(), gy=gy.tolist(), gz=gz.tolist()),
                       grid_floor=floor, K=args.k, sigma=args.sigma,
                       dnorm2_ratio=(dnorm2 / ynorm2).tolist(),
                       spec=spec_truth,
                       K_sigma_unique=dict(nodiff=np.unique(Ksig_nodiff).tolist(),
                                           diff=np.unique(Ksig).tolist()),
                       rows=rows, oracle=orc), f, ensure_ascii=False, indent=2)
    np.savez_compressed(out.replace(".json", "_surf.npz"),
                        **{("surf_" + k): v for k, v in surf.items()},
                        Ksig=Ksig, Ksig_nodiff=Ksig_nodiff,
                        nnls_drop=nnls_drop, nnls_nnz=nnls_nnz,
                        # K 扫描用的缓存：lamv[op, ix, iy, iz, k]、proj[op, ix, iy, iz, c, k]
                        lamv=lamv, proj=proj,
                        ops=np.array(",".join(OPS)),
                        gx=gx, gy=gy, gz=gz,
                        truth=truth, dnorm2=dnorm2, ynorm2=ynorm2,
                        or_corr_nodiff=or_corr_nodiff, or_relres_nodiff=or_relres_nodiff,
                        or_corr_diff=or_corr_diff, or_relres_diff=or_relres_diff)
    print("\n结果: %s" % out)
    print("      %s（含 K 扫描缓存 lamv/proj，可事后算任意 K）"
          % out.replace(".json", "_surf.npz"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
