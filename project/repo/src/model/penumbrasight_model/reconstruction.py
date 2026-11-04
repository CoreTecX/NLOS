# -*- coding: utf-8 -*-
"""隐藏场景重建：TV 正则化 PDHG 求解器（T0-05）。

对应论文式(7)与 Methods 的 TV 反演段。在差分框架下求解

    f_hat = argmin_{f >= 0}  0.5 * || D A f - D y ||_2^2  +  lambda * || f ||_TV

其中 D 为垂直相邻差分算子（抵消缓慢变化的多跳反射与环境背景），
||f||_TV 为各向同性总变差。

**与论文的差异**：论文 Methods 使用 FISTA，但 FISTA 需要 TV 的邻近算子，
而各向同性 TV 的邻近算子没有闭式解（只能再套一层内迭代），不利于后续
FPGA 定点化。本实现改用 **PDHG（Chambolle-Pock 原始-对偶混合梯度）**：

    min_{f>=0} max_{p, ||q_i||_2 <= lambda}
        <D A f, p> - 0.5||p||^2 - <D y, p> + <grad f, q>

迭代格式：
    p^{k+1} = ( p^k + sigma (D A f_bar - D y) ) / (1 + sigma)
    q^{k+1} = proj( q^k + sigma grad f_bar )              # 逐像素投影到 L2 球
    f^{k+1} = max( 0, f^k - tau ( (DA)^T p^{k+1} + grad^T q^{k+1} ) )

PDHG 的优势是每步只含矩阵向量乘与逐像素运算，**无内层迭代、无非线性函数**
（对比 FISTA 需要收缩阈值与 TV 邻近算子），定点化后的误差传播更容易界定，
迭代次数固定即可满足流水线时序要求。

约定：图像按 (n_rows, n_cols) = (29, 36) 排布，行索引对应 z、列索引对应 x。
梯度采用 Neumann 边界（边缘差分为 0）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------- 梯度算子


def gradient(f2d: np.ndarray) -> np.ndarray:
    """各向同性 TV 的前向差分梯度，返回 (2, H, W)。"""
    h, w = f2d.shape
    g = np.zeros((2, h, w), dtype=f2d.dtype)
    g[0, :-1, :] = f2d[1:, :] - f2d[:-1, :]
    g[1, :, :-1] = f2d[:, 1:] - f2d[:, :-1]
    return g


def divergence(g: np.ndarray) -> np.ndarray:
    """梯度的负伴随，返回 (H, W)。

    实现为 div = -(grad^T)，与 gradient 满足 <grad f, g> = <f, -div g>。
    """
    _, h, w = g.shape
    out = np.zeros((h, w), dtype=g.dtype)
    out[:-1, :] -= g[0, :-1, :]
    out[1:, :] += g[0, :-1, :]
    out[:, :-1] -= g[1, :, :-1]
    out[:, 1:] += g[1, :, :-1]
    return -out


def project_l2_ball(q: np.ndarray, radius: float) -> np.ndarray:
    """逐像素把二维向量投影到半径 radius 的 L2 球（各向同性 TV 的对偶投影）。"""
    norm = np.sqrt(q[0] ** 2 + q[1] ** 2)
    scale = np.ones_like(norm)
    nz = norm > radius
    scale[nz] = radius / norm[nz]
    return q * scale


# ---------------------------------------------------------------- 算子范数


def operator_norm_sq(A: np.ndarray, shape: Tuple[int, int],
                     n_iter: int = 25, seed: int = 0) -> float:
    """估计 K = [A; grad] 的算子范数平方（幂迭代）。

    K^T K = A^T A + grad^T grad，故 L^2 为其最大特征值。
    """
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(A.shape[1])
    v /= np.linalg.norm(v)
    L2 = 0.0
    for _ in range(n_iter):
        w = A.T @ (A @ v) - divergence(gradient(v.reshape(shape))).reshape(-1)
        L2 = float(np.linalg.norm(w))
        n = np.linalg.norm(w)
        if n == 0:
            break
        v = w / n
    return L2


# ---------------------------------------------------------------- 求解器


@dataclass
class ReconstructResult:
    """重建结果。"""

    image: np.ndarray            # (H, W) 单通道重建
    objective: list              # 每若干次迭代记录的目标函数分量
    n_iter: int
    lam: float
    elapsed: float

    def as_dict(self) -> Dict[str, object]:
        return {"n_iter": self.n_iter, "lam": self.lam,
                "elapsed_s": self.elapsed, "final_objective": self.objective[-1]}


def tv_pdhg(A: np.ndarray, b: np.ndarray, shape: Tuple[int, int],
            lam: float = 0.01, n_iter: int = 300, nonneg: bool = True,
            L2: Optional[float] = None, record_every: int = 25) -> ReconstructResult:
    """用 PDHG 求解差分 TV 问题。

    参数：
        A        光传输矩阵（差分后），形状 (M, N)
        b        差分后的测量，长度 M
        shape    隐藏图像的 (H, W)
        lam      TV 正则化强度
        n_iter   迭代次数
        nonneg   是否施加非负约束（物理上辐射度非负，应保持 True）
        L2       算子范数平方；None 时用幂迭代现算

    返回 ReconstructResult。
    """
    import time
    A = np.asarray(A, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64).reshape(-1)
    if A.ndim != 2 or not np.all(np.isfinite(A)) or not np.all(np.isfinite(b)):
        raise ValueError("A and b must be finite arrays")
    if not np.isfinite(lam) or lam < 0 or n_iter < 1 or record_every < 0:
        raise ValueError("invalid regularization or iteration parameters")
    if A.shape[0] != b.size:
        raise ValueError(f"A 行数 {A.shape[0]} 与 b 长度 {b.size} 不符")
    if A.shape[1] != shape[0] * shape[1]:
        raise ValueError(f"A 列数 {A.shape[1]} 与 shape {shape} 不符")

    if L2 is None:
        L2 = operator_norm_sq(A, shape)
    if not np.isfinite(L2) or L2 <= 0:
        raise ValueError("算子范数估计失败")
    # Strict safety margin: tau*sigma*||K||^2 < 1.
    sigma = np.sqrt(0.99 / L2)
    tau = sigma

    f = np.zeros(A.shape[1], dtype=np.float64)
    f_bar = f.copy()
    p = np.zeros(b.size, dtype=np.float64)
    q = np.zeros((2,) + shape, dtype=np.float64)

    objective = []
    t0 = time.time()
    for k in range(1, n_iter + 1):
        # 对偶：数据项
        p = (p + sigma * (A @ f_bar - b)) / (1.0 + sigma)
        # 对偶：TV 项，逐像素投影到 L2 球
        q = project_l2_ball(q + sigma * gradient(f_bar.reshape(shape)), lam)
        # 原始更新
        f_new = f - tau * (A.T @ p - divergence(q).reshape(-1))
        if nonneg:
            np.maximum(f_new, 0.0, out=f_new)
        f_bar = 2.0 * f_new - f
        f = f_new

        if k == n_iter or (record_every and k % record_every == 0):
            r = A @ f - b
            data_term = 0.5 * float(r @ r)
            g = gradient(f.reshape(shape))
            tv = float(np.sum(np.sqrt(g[0] ** 2 + g[1] ** 2)))
            objective.append({"iter": k, "data": data_term,
                              "tv": tv, "obj": data_term + lam * tv})

    return ReconstructResult(image=f.reshape(shape), objective=objective,
                             n_iter=n_iter, lam=lam, elapsed=time.time() - t0)


def reconstruct_channel(setup, y2d: np.ndarray, ll, lam: float = 0.01,
                        n_iter: int = 300, mu_mode: str = "none",
                        mu_k: float = 1.0, differenced: bool = True,
                        A: Optional[np.ndarray] = None) -> ReconstructResult:
    """对单个颜色通道完成「构造矩阵 + 差分 + 求解」全流程。

    A 可外部传入以复用（同一遮挡位置下三个通道共用同一个矩阵）。
    """
    from . import forward as F  # noqa: F401
    from .localization import apply_row_difference_A, apply_row_difference_img

    shape = setup.monitor.shape
    if A is None:
        s2 = setup.with_occluder_ll(ll)
        A = F.build_transport_matrix(s2, mu_mode=mu_mode, mu_k=mu_k)
    if differenced:
        A_use = apply_row_difference_A(A, setup.wall.n)
        b = apply_row_difference_img(np.asarray(y2d, dtype=np.float64))
    else:
        A_use = A
        b = np.asarray(y2d, dtype=np.float64).reshape(-1)
    return tv_pdhg(A_use, b, shape, lam=lam, n_iter=n_iter)


# ---------------------------------------------------------------- 质量度量


def scale_matched(a: np.ndarray, ref: np.ndarray) -> Tuple[np.ndarray, float]:
    """最小二乘标度匹配，使 a 在尺度上可比于 ref。返回 (标度后数组, 标度系数)。"""
    a = np.asarray(a, dtype=np.float64).reshape(-1)
    r = np.asarray(ref, dtype=np.float64).reshape(-1)
    denom = float(a @ a)
    alpha = float(a @ r) / denom if denom > 0 else 0.0
    return (a * alpha).reshape(np.asarray(ref).shape), alpha


def psnr_scale_matched(a: np.ndarray, ref: np.ndarray,
                       data_range: Optional[float] = None) -> float:
    """标度匹配后的 PSNR（dB）。

    重建结果与真值之间只差一个未知的全局增益（辐射度标定未知），
    因此先用最小二乘估出该增益再做 PSNR，否则 PSNR 无意义。
    """
    ref = np.asarray(ref, dtype=np.float64)
    am, _ = scale_matched(a, ref)
    mse = float(np.mean((am - ref) ** 2))
    if mse <= 0:
        return float("inf")
    dr = data_range if data_range is not None else float(ref.max() - ref.min())
    if dr <= 0:
        return float("-inf")
    return float(10.0 * np.log10(dr * dr / mse))


def pearson(a: np.ndarray, b: np.ndarray) -> float:
    x = np.asarray(a, dtype=np.float64).reshape(-1)
    y = np.asarray(b, dtype=np.float64).reshape(-1)
    x = x - x.mean()
    y = y - y.mean()
    den = np.sqrt(float(x @ x) * float(y @ y))
    return float(x @ y / den) if den > 0 else float("nan")
