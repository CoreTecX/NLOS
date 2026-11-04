# -*- coding: utf-8 -*-
"""遮挡物定位：差分框架下的粗到细投影能量搜索（T0-05）。

对应论文式(2)与 Methods 的定位段：

    p_o_hat = argmax_{p_o} || U_K(p_o)^T y ||_2^2

其中 U_K 是 A(p_o) 的显著左奇异向量（按阈值 kappa 截断）。

**与论文的差异（由 T0-04 的实测结论确定）**：

论文正文式(2)写的是直接对 y 做投影，但 S011 说明重建前要对测量与传输矩阵的
对应行做垂直差分以抵消缓慢变化的背景。T0-04 的诊断给出定量证据：
不做差分时，kappa=0.75 的截断在官方数据上**只保留 1 个奇异成分**，能量评价退化
为单一主方向的投影，失去分辨能力；做差分后保留 17~28 个成分，且能量景观与官方
公布的位置估计方向一致（官方估计得分为真值的 2.03 倍）。

因此本实现的定位一律在**差分框架**下进行：

    y_d = D y,   A_d = D A,   能量在 A_d 上进行

其中 D 为垂直相邻差分算子（沿测量图的 z 方向，即行方向）。

搜索网格直接采用官方配置文件中的 xhatvals / yhatvals / zhatvals 系列，
使本实现的搜索结果与官方 best_pos 系列可逐轮直接比对。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    from . import forward as F
except ImportError:  # pragma: no cover
    import forward as F  # type: ignore


# ---------------------------------------------------------------- 差分算子


def row_difference_1d(n: int) -> np.ndarray:
    """沿行方向的相邻差分算子 D，形状 (n-1, n)。"""
    D = np.zeros((n - 1, n), dtype=np.float64)
    idx = np.arange(n - 1)
    D[idx, idx] = -1.0
    D[idx, idx + 1] = 1.0
    return D


def apply_row_difference_img(y2d: np.ndarray) -> np.ndarray:
    """对二维测量图按行差分并展平，与 A 的行块顺序一致。"""
    return (y2d[1:, :] - y2d[:-1, :]).reshape(-1)


def apply_row_difference_A(A: np.ndarray, n: int) -> np.ndarray:
    """对光传输矩阵按测量图的行结构差分，形状 (n*(n-1), N)。"""
    Ar = A.reshape(n, n, -1)
    return (Ar[1:, :, :] - Ar[:-1, :, :]).reshape(-1, A.shape[1])


# ---------------------------------------------------------------- 投影能量


@dataclass
class EnergyResult:
    """单个候选位置的能量计算明细。"""

    ll: np.ndarray
    energy: float
    n_kept: int
    s1: float

    def as_row(self) -> Dict[str, object]:
        return {"ll": self.ll.tolist(), "energy": self.energy,
                "n_kept": self.n_kept, "s1": self.s1}


def projection_energy_detail(A: np.ndarray, y: np.ndarray,
                             kappa: float = 0.75) -> Tuple[float, int, float]:
    """投影能量及其明细，返回 (energy, 保留成分数, 最大奇异值)。

    原理（避免显式左奇异向量）：
        设 A = U S V^T，则 U_K U_K^T = A V_K S_K^{-2} V_K^T A^T，
        故 ||U_K U_K^T y||^2 = sum_{k in K} (v_k^T A^T y)^2 / s_k^2。
    v_k、s_k 由 A^T A 的特征分解得到，规模从 M x N 降到 N x N。
    """
    A = np.asarray(A)
    y = np.asarray(y, dtype=np.float64).reshape(-1)
    if A.shape[0] != y.size:
        raise ValueError(f"A 行数 {A.shape[0]} 与 y 长度 {y.size} 不符")

    z = A.T @ y
    G = A.T @ A
    G = 0.5 * (G + G.T)
    lam, V = np.linalg.eigh(G)
    del G
    lam = lam[::-1]
    V = V[:, ::-1]
    lam = np.maximum(lam, 0.0)
    s = np.sqrt(lam)
    if s.size == 0 or s[0] <= 0:
        return 0.0, 0, 0.0
    keep = (s >= kappa * s[0]) if kappa > 0 else np.ones(s.size, dtype=bool)
    c = V.T @ z
    e = float(np.sum((c[keep] ** 2) / lam[keep])) if keep.any() else 0.0
    return e, int(keep.sum()), float(s[0])


def evaluate_candidate(setup: F.OpticalSetup, y2d: np.ndarray, ll: Sequence[float],
                       differenced: bool = True, kappa: float = 0.75,
                       mu_mode: str = "none", mu_k: float = 1.0) -> EnergyResult:
    """评估一个候选遮挡物位置的能量。"""
    s2 = setup.with_occluder_ll(ll)
    A = F.build_transport_matrix(s2, mu_mode=mu_mode, mu_k=mu_k)
    if differenced:
        y = apply_row_difference_img(y2d)
        A = apply_row_difference_A(A, setup.wall.n)
    else:
        y = np.asarray(y2d, dtype=np.float64).reshape(-1)
    e, nk, s1 = projection_energy_detail(A, y, kappa)
    del A
    return EnergyResult(ll=np.asarray(ll, dtype=np.float64), energy=e,
                        n_kept=nk, s1=s1)


# ---------------------------------------------------------------- 网格搜索


@dataclass
class SearchRound:
    """一轮搜索结果。"""

    index: int
    kappa: float
    x_values: np.ndarray
    y_values: np.ndarray
    z_values: np.ndarray
    energies: np.ndarray            # (nx, ny, nz)
    best_ll: np.ndarray
    best_energy: float
    n_candidates: int
    elapsed: float
    n_kept_at_best: int = 0

    def as_dict(self) -> Dict[str, object]:
        return {
            "round": self.index, "kappa": self.kappa,
            "grid": (self.x_values.size, self.y_values.size, self.z_values.size),
            "n_candidates": self.n_candidates,
            "best_ll": self.best_ll.tolist(),
            "best_energy": self.best_energy,
            "elapsed_s": self.elapsed,
            "n_kept_at_best": self.n_kept_at_best,
        }


def search_grid(setup: F.OpticalSetup, y2d: np.ndarray,
                x_values: Sequence[float], y_values: Sequence[float],
                z_values: Sequence[float], kappa: float = 0.75,
                differenced: bool = True, index: int = 1,
                progress: bool = True) -> SearchRound:
    """在给定三维网格上穷举候选位置，返回能量体与最优位置。

    参数顺序与官方 proj_yOnRangeA 的维度一致：(x, y, z)，其中 y 为深度方向。
    """
    import time
    xv = np.asarray(x_values, dtype=np.float64)
    yv = np.asarray(y_values, dtype=np.float64)
    zv = np.asarray(z_values, dtype=np.float64)
    E = np.full((xv.size, yv.size, zv.size), np.nan)
    n_total = E.size
    t0 = time.time()
    best = (-np.inf, None, 0)

    k = 0
    for i, x in enumerate(xv):
        for j, ydep in enumerate(yv):
            for m, z in enumerate(zv):
                ll = np.array([x, ydep, z])
                r = evaluate_candidate(setup, y2d, ll, differenced=differenced,
                                       kappa=kappa)
                E[i, j, m] = r.energy
                if r.energy > best[0]:
                    best = (r.energy, ll, r.n_kept)
                k += 1
        if progress:
            print(f"      [{index}] x={x:.4f}  {k}/{n_total}  "
                  f"当前最优 E={best[0]:.6g} ll={best[1]}  "
                  f"{time.time()-t0:.0f}s", flush=True)

    return SearchRound(index=index, kappa=kappa, x_values=xv, y_values=yv,
                       z_values=zv, energies=E, best_ll=best[1],
                       best_energy=best[0], n_candidates=n_total,
                       elapsed=time.time() - t0, n_kept_at_best=best[2])


def load_search_grids(config_path: str) -> Dict[str, np.ndarray]:
    """从官方配置文件读取四轮搜索网格。

    官方字段命名：xhatvals -> x（横向），yhatvals -> y（深度），
    zhatvals -> z（横向）。带后缀 2/3/4 的为后续轮次的细化网格。
    """
    d = F.load_mat(config_path)
    out: Dict[str, np.ndarray] = {}
    for tag, suffix in (("r1", ""), ("r2", "2"), ("r3", "3"), ("r4", "4")):
        try:
            out[f"{tag}_x"] = np.asarray(d[f"xhatvals{suffix}"], dtype=np.float64).reshape(-1)
            out[f"{tag}_y"] = np.asarray(d[f"yhatvals{suffix}"], dtype=np.float64).reshape(-1)
            out[f"{tag}_z"] = np.asarray(d[f"zhatvals{suffix}"], dtype=np.float64).reshape(-1)
        except KeyError:
            pass
    return out


def coarse_to_fine(setup: F.OpticalSetup, y2d: np.ndarray, grids: Dict[str, np.ndarray],
                   kappas: Sequence[float] = (0.75, 0.5, 0.05, 0.05),
                   max_rounds: int = 4, progress: bool = True) -> List[SearchRound]:
    """按官方四轮结构执行粗到细搜索。

    每轮使用官方对应的网格与截断阈值，便于与官方 best_pos 系列逐轮比对。
    """
    rounds: List[SearchRound] = []
    for i in range(1, max_rounds + 1):
        tag = f"r{i}"
        if f"{tag}_x" not in grids:
            break
        kappa = kappas[min(i - 1, len(kappas) - 1)]
        if progress:
            print(f"   第 {i} 轮：网格 {grids[f'{tag}_x'].size} x "
                  f"{grids[f'{tag}_y'].size} x {grids[f'{tag}_z'].size}，κ={kappa}")
        rounds.append(search_grid(setup, y2d, grids[f"{tag}_x"],
                                  grids[f"{tag}_y"], grids[f"{tag}_z"],
                                  kappa=kappa, index=i, progress=progress))
    return rounds
