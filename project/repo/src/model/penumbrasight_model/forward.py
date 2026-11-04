# -*- coding: utf-8 -*-
"""几何前向模型（T0-04）。

物理结构（官方配置中的实际约定，已由参数值确认）：

    y 轴为深度方向，三个平面互相平行：
        显示器（隐藏场景）平面   y = 0        <- 隐藏场景所在
        遮挡物平面               y = 0.594    <- 尺寸 7.5cm x 7.5cm 的矩形
        可见成像墙               y = 1.03     <- 相机观察的朗伯面
    相机与显示器之间无直接视线，只拍墙。

前向模型（论文式(1)的离散化）：

    I(p_w) = sum_i A[j, i] * f_i + b_j
    A[j, i] = [cos th_w * cos th_m / ||w_j - m_i||^2] * V(i,j) * mu(m_i, w_j) * dA_i

其中 j 索引墙面测量点（126 x 126 = 15,876 个），i 索引隐藏场景像素（29 x 36 = 1,044 个）。

**关键几何简化**：三个平面互相平行且法向均为 ±y，因此
    cos th_w = cos th_m = Y_w / ||w_j - m_i||
两个正向因子相等，权重退化为 Y_w^2 / ||v||^4。

**可见性的闭式解**：从显示器点 m（y=0）到墙点 w（y=Y_w）的线段与遮挡物平面
（y=Y_o）的交点为
    q = (1 - t) * m + t * w,     t = Y_o / Y_w
因此可见性退化为「q 是否落在遮挡物矩形内」的二维点包含判断，不需要论文 Methods
中 L = 64 的步进采样。这是等价加速，已由 validate_geometry.py 的 V4 交叉验证。

**图像方向约定**：墙点与隐藏像素均按行优先展平，行索引对应 z、列索引对应 x，
与官方测量图 (126,126) 和真值 (29,36) 的排列一致。实际方向由 V1 在 8 种约定中
实测选定，不依赖此处假设。

本模块不复制官方 MATLAB 代码，按论文公开的数学定义独立实现。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

try:
    from .io_mat import load_mat
except ImportError:  # pragma: no cover
    from io_mat import load_mat  # type: ignore


# ---------------------------------------------------------------- 几何数据类


@dataclass(frozen=True)
class WallPlane:
    """可见成像墙。法向为 -y（朝向显示器一侧）。"""

    y: float
    x_min: float
    x_max: float
    z_min: float
    z_max: float
    n: int

    @property
    def x_axis(self) -> np.ndarray:
        """n 个分块中心的 x 坐标（一维轴）。"""
        return self.x_min + (np.arange(self.n) + 0.5) * (self.x_max - self.x_min) / self.n

    @property
    def z_axis(self) -> np.ndarray:
        return self.z_min + (np.arange(self.n) + 0.5) * (self.z_max - self.z_min) / self.n

    @property
    def n_points(self) -> int:
        """测量点总数 M = n^2。"""
        return self.n * self.n

    @property
    def spacing(self) -> Tuple[float, float]:
        return ((self.x_max - self.x_min) / self.n, (self.z_max - self.z_min) / self.n)

    def grid(self) -> Tuple[np.ndarray, np.ndarray]:
        """展平为 (M,) 的 x、z 坐标，行优先（行索引 = z，列索引 = x）。"""
        xx, zz = np.meshgrid(self.x_axis, self.z_axis)
        return xx.reshape(-1), zz.reshape(-1)


@dataclass(frozen=True)
class MonitorPlane:
    """隐藏场景所在平面（显示器）。法向为 +y（朝向墙）。"""

    y: float
    x_centers: np.ndarray      # (n_cols,) = 36
    z_centers: np.ndarray      # (n_rows,) = 29，降序（与图像行序一致）
    block_w: float             # 单个隐藏像素物理宽度 (IlluminationBlock_Size[0])
    block_h: float

    @property
    def shape(self) -> Tuple[int, int]:
        """(n_rows, n_cols) = (29, 36)，与官方 ground_truth 排列一致。"""
        return (self.z_centers.size, self.x_centers.size)

    @property
    def n_pixels(self) -> int:
        return self.x_centers.size * self.z_centers.size

    @property
    def block_area(self) -> float:
        return float(self.block_w * self.block_h)

    def flat_grid(self) -> Tuple[np.ndarray, np.ndarray]:
        """展平为 (N,) 的 x、z 坐标，行优先。"""
        xx, zz = np.meshgrid(self.x_centers, self.z_centers)
        return xx.reshape(-1), zz.reshape(-1)


@dataclass(frozen=True)
class Occluder:
    """矩形遮挡物，位于与显示器、墙平行的平面上。"""

    ll: np.ndarray      # (3,) 参考角 (x, y, z)
    size: np.ndarray    # (2,) (dx, dz)

    @property
    def y(self) -> float:
        return float(self.ll[1])

    @property
    def x_range(self) -> Tuple[float, float]:
        return (float(self.ll[0]), float(self.ll[0] + self.size[0]))

    @property
    def z_range(self) -> Tuple[float, float]:
        return (float(self.ll[2]), float(self.ll[2] + self.size[1]))

    @property
    def center(self) -> np.ndarray:
        return np.array([self.ll[0] + self.size[0] / 2,
                         self.ll[1],
                         self.ll[2] + self.size[1] / 2])


@dataclass(frozen=True)
class OpticalSetup:
    """完整几何配置。"""

    wall: WallPlane
    monitor: MonitorPlane
    occluder: Occluder
    view_angle_correction: bool = True

    def with_occluder_ll(self, ll) -> "OpticalSetup":
        """以新的遮挡物参考角构造同构配置（用于位置搜索）。"""
        return OpticalSetup(
            wall=self.wall, monitor=self.monitor,
            occluder=Occluder(ll=np.asarray(ll, dtype=np.float64),
                              size=self.occluder.size),
            view_angle_correction=self.view_angle_correction,
        )

    def summary(self) -> Dict[str, object]:
        return {
            "wall_plane_y": self.wall.y,
            "wall_fov_x": (self.wall.x_min, self.wall.x_max),
            "wall_fov_z": (self.wall.z_min, self.wall.z_max),
            "wall_samples_1d": self.wall.n,
            "wall_measurements_M": self.wall.n_points,
            "monitor_plane_y": self.monitor.y,
            "monitor_shape": self.monitor.shape,
            "monitor_pixels_N": self.monitor.n_pixels,
            "monitor_block_area_m2": self.monitor.block_area,
            "occluder_plane_y": self.occluder.y,
            "occluder_x": self.occluder.x_range,
            "occluder_z": self.occluder.z_range,
            "occluder_size": (float(self.size_dx), float(self.size_dz)),
            "view_angle_correction": self.view_angle_correction,
        }

    @property
    def size_dx(self) -> float:
        return float(self.occluder.size[0])

    @property
    def size_dz(self) -> float:
        return float(self.occluder.size[1])


# ---------------------------------------------------------------- 配置加载


def load_setup(config_path: str, occluder_ll: Optional[np.ndarray] = None) -> OpticalSetup:
    """从官方配置文件构造几何配置。

    occluder_ll 为 None 时使用配置中的真实遮挡物位置；传入 (3,) 数组可构造任意候选位置。
    """
    d = load_mat(config_path)

    fov = np.asarray(d["FOV_cord"], dtype=np.float64).reshape(2, 2)
    wall_n = int(np.asarray(d["walln_points"]).reshape(-1)[0])
    wall = WallPlane(
        y=float(np.asarray(d["wall_point"]).reshape(-1)[1]),
        x_min=float(fov[0, 0]), x_max=float(fov[1, 0]),
        z_min=float(fov[0, 1]), z_max=float(fov[1, 1]),
        n=wall_n,
    )

    blk = np.asarray(d["IlluminationBlock_Size"], dtype=np.float64).reshape(-1)
    monitor = MonitorPlane(
        y=float(np.asarray(d["Monitor_y"]).reshape(-1)[0]),
        x_centers=np.asarray(d["Mon_xdiscr"], dtype=np.float64).reshape(-1),
        z_centers=np.asarray(d["Mon_zdiscr"], dtype=np.float64).reshape(-1),
        block_w=float(blk[0]), block_h=float(blk[1]),
    )

    occ_cfg = np.asarray(d["Occluder"], dtype=np.float64)
    size = np.asarray(d["Occ_size"], dtype=np.float64).reshape(-1)
    ll = occ_cfg[0] if occluder_ll is None else np.asarray(occluder_ll, dtype=np.float64)
    occluder = Occluder(ll=ll, size=np.array([size[0], size[2]]))

    vac = bool(np.asarray(d.get("viewAngleCorrection", np.array([[1]]))).reshape(-1)[0])
    return OpticalSetup(wall=wall, monitor=monitor, occluder=occluder,
                        view_angle_correction=vac)


# ---------------------------------------------------------------- 可见性


def crossing_points(setup: OpticalSetup, dtype=np.float32) -> Tuple[np.ndarray, np.ndarray, float]:
    """所有（墙点, 隐藏像素）连线在遮挡物平面上的交点坐标。

    返回 (qx, qz, t)，形状均为 (M_wall, N_scene)：
        qx[j, i] = (1 - t) * m_x[i] + t * w_x[j],   t = y_occluder / y_wall
    """
    t = setup.occluder.y / setup.wall.y
    wx, wz = setup.wall.grid()
    mx, mz = setup.monitor.flat_grid()
    wx = wx.astype(dtype)[:, None]
    wz = wz.astype(dtype)[:, None]
    mx = mx.astype(dtype)[None, :]
    mz = mz.astype(dtype)[None, :]
    omt = dtype(1.0 - t)
    td = dtype(t)
    qx = omt * mx + td * wx
    qz = omt * mz + td * wz
    return qx, qz, t


def visibility(setup: OpticalSetup) -> np.ndarray:
    """可见性函数 V，形状 (M_wall, N_scene) 的布尔数组。

    V[j, i] = True 表示从墙点 j 能看到隐藏像素 i（连线不穿过遮挡物）。
    """
    qx, qz, _ = crossing_points(setup)
    x0, x1 = setup.occluder.x_range
    z0, z1 = setup.occluder.z_range
    inside = (qx >= x0) & (qx <= x1) & (qz >= z0) & (qz <= z1)
    del qx, qz
    return ~inside


def visible_fraction(setup: OpticalSetup) -> np.ndarray:
    """每个墙点能看到多少比例的隐藏场景，形状 (n, n)，取值 [0, 1]。"""
    V = visibility(setup)
    return V.mean(axis=1).reshape(setup.wall.n, setup.wall.n)


# ---------------------------------------------------------------- 光传输矩阵


def angle_response(setup: OpticalSetup, cos_theta: np.ndarray,
                   mode: str = "none", k: float = 1.0) -> np.ndarray:
    """显示器角度辐射响应 mu(m, w)。

    官方只公开 viewAngleCorrection = 1（启用）这一开关，未给出函数形式，
    因此这里提供参数化实现，指数 k 由 validate_geometry.py 在官方数据上拟合。

    mode:
        'none'    mu = 1（关闭角度修正）
        'cos_pow' mu = cos(theta_m)^k
        'poly'    mu = 1 + k * (1 - cos(theta_m))
    """
    if mode == "none":
        return np.ones_like(cos_theta, dtype=np.float32)
    if mode == "cos_pow":
        return np.power(np.clip(cos_theta, 1e-6, 1.0), np.float32(k))
    if mode == "poly":
        return 1.0 + np.float32(k) * (1.0 - cos_theta)
    raise ValueError(f"未知角度响应模式 {mode}")


def build_transport_matrix(setup: OpticalSetup, mu_mode: str = "none",
                           mu_k: float = 1.0, with_occluder: bool = True,
                           dtype=np.float64) -> np.ndarray:
    """构造光传输矩阵 A，形状 (M_wall, N_scene)。

    A[j, i] = [Y_w^2 / ||v||^4] * V * mu * dA
    （三平面平行使两个余弦因子相等，故为 Y_w^2 / ||v||^4）

    with_occluder=False 时 V 恒为 1，用于复现论文 S008「无遮挡物时 A 的行过于相似」。
    """
    wx, wz = setup.wall.grid()
    mx, mz = setup.monitor.flat_grid()
    wx = wx[:, None]; wz = wz[:, None]
    mx = mx[None, :]; mz = mz[None, :]

    y_w = float(setup.wall.y - setup.monitor.y)

    d2 = (wx - mx) ** 2
    d2 += (wz - mz) ** 2
    d2 += y_w * y_w                       # (M, N) 距离平方

    if setup.view_angle_correction and mu_mode != "none":
        cos_theta = (y_w / np.sqrt(d2)).astype(np.float32)
        mu = angle_response(setup, cos_theta, mu_mode, mu_k).astype(np.float64)
        del cos_theta
    else:
        mu = None

    np.square(d2, out=d2)                 # d2 = ||v||^4
    A = (y_w * y_w) / d2
    del d2
    if mu is not None:
        A *= mu
        del mu
    A *= setup.monitor.block_area

    if with_occluder:
        A *= visibility(setup)
    return A.astype(dtype, copy=False)


def forward_render(setup: OpticalSetup, scene: Optional[np.ndarray] = None,
                   mu_mode: str = "none", mu_k: float = 1.0) -> np.ndarray:
    """正向渲染墙面图像，形状 (n, n)。

    scene 为 None 时用全 1（均匀发光显示器），用于观察遮挡物投射的半影结构；
    否则形状为 (29, 36) 或 (N,)。
    """
    A = build_transport_matrix(setup, mu_mode=mu_mode, mu_k=mu_k)
    if scene is None:
        f = np.ones(A.shape[1], dtype=np.float64)
    else:
        f = np.asarray(scene, dtype=np.float64).reshape(-1)
        if f.size != A.shape[1]:
            raise ValueError(f"scene 长度 {f.size} 与隐藏场景维数 {A.shape[1]} 不符")
    out = A @ f
    return out.reshape(setup.wall.n, setup.wall.n)


# ---------------------------------------------------------------- 定位评分


def projection_energy(A: np.ndarray, y: np.ndarray, kappa: float = 0.75) -> float:
    """y 在 A 值域上的投影能量（论文式(2)），按阈值 kappa 截断奇异值。

    不显式计算左奇异向量。设 A = U S V^T，则
        ||U_K U_K^T y||^2 = sum_{k: s_k/s_1 >= kappa} (v_k^T A^T y)^2 / s_k^2
    v_k、s_k 来自 A^T A 的特征分解。这把规模从 M x N 的 SVD 降为 N x N 的特征分解，
    是位移搜索能在本机跑完的关键。

    kappa = 0 表示不截断。官方论文在连续三次搜索中分别使用 0.75、0.5、0.05。
    """
    A = np.asarray(A)
    y = np.asarray(y, dtype=np.float64).reshape(-1)
    if A.shape[0] != y.size:
        raise ValueError(f"A 的行数 {A.shape[0]} 与 y 长度 {y.size} 不符")

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
        return 0.0
    keep = (s >= kappa * s[0]) if kappa > 0 else np.ones(s.size, dtype=bool)
    c = V.T @ z
    return float(np.sum((c[keep] ** 2) / lam[keep]))


def singular_spectrum(A: np.ndarray, top: int = 40) -> np.ndarray:
    """A 的前 top 个奇异值。"""
    G = A.T @ A
    G = 0.5 * (G + G.T)
    lam = np.linalg.eigvalsh(G)[::-1][:top]
    return np.sqrt(np.maximum(lam, 0.0))


def mean_abs_column_correlation(A: np.ndarray, max_cols: int = 200) -> float:
    """A 各列之间平均绝对相关系数（越低说明各列越可区分）。"""
    A = np.asarray(A, dtype=np.float64)
    n = A.shape[1]
    idx = np.linspace(0, n - 1, min(n, max_cols)).astype(int)
    B = A[:, idx]
    B = B - B.mean(axis=0, keepdims=True)
    norms = np.linalg.norm(B, axis=0)
    norms[norms == 0] = 1.0
    B = B / norms
    C = np.abs(B.T @ B)
    m = ~np.eye(C.shape[0], dtype=bool)
    return float(C[m].mean())


def condition_number(A: np.ndarray) -> float:
    s = singular_spectrum(A, top=min(A.shape) - 1)
    s = s[s > 0]
    return float(s[0] / s[-1]) if s.size else float("inf")
