# -*- coding: utf-8 -*-
"""MATLAB ``poly2mask`` 的等价栅格化（X4 收尾）。

为什么需要它
------------
论文重建链的遮挡物是**先把 4 个角点投到墙面、换算成墙图像像素坐标、取凸包，
再用 `poly2mask` 栅格化**（`Functions/simulate_occluder.m`）::

    occ_image_coords_1 = (proj(:,1) - (wall_point(1)-wall_vector_1(1))) * walln/(wall_vector_1(1)*2);
    occ_image_coords_2 = (proj(:,3) - (wall_point(3)-wall_vector_2(3))) * walln/(wall_vector_2(3)*2);
    occluder_image = ~poly2mask(occ_image_coords_1(k), occ_image_coords_2(k), walln, walln);

即多边形坐标落在 **[0, n] × [0, n]** 的「像素坐标」里（FOV 下角 → 0、上角 → n），
而判定点是**整数像素中心 1..n**（1-based）。这与「把墙采样点代入交点判据」不是一回事：
墙采样点射到同一像素坐标里是 ``(i-1)*n/(n-1)``，与整数网格相差最多 1 个像素 ——
这是 X4 之前 4.57e-2 残差的**唯一**来源。

判定规则（可实测反推，见 sim/results/x4_poly2mask_alignment.md 与对照脚本）
--------------------------------------------------------------------------
1. 评估点 = 整数像素中心 ``(col, row) = (1..n, 1..m)``；
2. 多边形顶点按原值使用（**不做** 5×5 子像素网格吸附，见下）；
3. 像素在多边形内 ⇔ 自该点向左（−x）的射线与多边形边界交点数**为奇**（even-odd），
   采用半开 y 区间 ``[min(y0,y1), max(y0,y1))`` 消除顶点/水平边的二义性。

关于 5×5 子像素网格：老版 MATLAB 文档描述 `poly2mask` 先把顶点吸附到 5×5 子像素
网格（步长 0.2）再沿网格边扫掠。`snap=True` 开关实现了「顶点吸附到该网格」的变体，
但在本项目的数据上 **snap=True 与 snap=False 给出逐元素完全相同的结果**（0.2 步长的
吸附位移 ≤0.1 像素，不足以跨过任何一个整数像素中心；实测 1044 列 ×36 子采样 ×2 多边形
全部一致），而 ceil/floor 式吸附会立刻破坏一致性（相对误差 1.4e-2 ~ 1.7e-2）。
所以默认 ``snap=False``：**按原值多边形 + 整数像素中心判定**，这正是复现 MATLAB
`simA` 到机器精度（相对 2.0e-16）的那一条。
"""
from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

__all__ = ["poly2mask", "region_mask", "visible_from_shadow",
           "wall_pixel_grid", "project_occluder_corners"]


def poly2mask(x: Sequence[float], y: Sequence[float], m: int, n: int,
              snap=False) -> np.ndarray:
    """``BW = poly2mask(x, y, m, n)`` 的等价实现。

    参数
    ----
    x, y : 多边形顶点的像素坐标（1-based；x 对应列、y 对应行）。
    m, n : 输出掩码的行数、列数（对应 ``BW(m, n)``，行由 y 索引、列由 x 索引）。
    snap : 是否把顶点吸附到 5×5 子像素网格（老版 MATLAB 文档口径），可取
           False / "round" / "ceil" / "floor"。本项目实测不需要且默认关闭
           （round/ceil/floor 三种吸附在本数据上的判别见报告 §4）。
    """
    m = int(m)
    n = int(n)
    xs = np.asarray(x, dtype=np.float64).ravel()
    ys = np.asarray(y, dtype=np.float64).ravel()
    if xs.size != ys.size:
        raise ValueError("x 与 y 长度不一致")
    if xs.size < 3:
        return np.zeros((m, n), dtype=bool)

    # 自动闭合（MATLAB: poly2mask closes the polygon automatically）
    if xs[0] != xs[-1] or ys[0] != ys[-1]:
        xs = np.append(xs, xs[0])
        ys = np.append(ys, ys[0])

    if snap:
        how = "round" if snap is True else str(snap)
        xs = _snap_subpixel(xs, how)
        ys = _snap_subpixel(ys, how)

    # 行中心 1..m；半开 y 区间 [y_lo, y_hi) 决定该边贡献哪些扫描行
    x0, y0 = xs[:-1], ys[:-1]
    x1, y1 = xs[1:], ys[1:]

    # 差分开销数组：D[r, p] 取反 ⇒ 前缀异或后，列 c >= p 的奇偶翻转
    D = np.zeros((m, n + 1), dtype=np.uint8)
    for ex0, ey0, ex1, ey1 in zip(x0, y0, x1, y1):
        if ey0 == ey1:
            continue                                  # 水平边不参与扫描
        lo, hi = (ey0, ey1) if ey0 < ey1 else (ey1, ey0)
        r_lo = int(np.ceil(lo))
        r_hi = int(np.ceil(hi)) - 1                   # 半开上端
        r_lo = max(r_lo, 1)
        r_hi = min(r_hi, m)
        if r_lo > r_hi:
            continue
        rows = np.arange(r_lo, r_hi + 1, dtype=np.float64)
        xcross = ex0 + (rows - ey0) * (ex1 - ex0) / (ey1 - ey0)
        # 该交点贡献给「像素中心列 c_pix > xcross」的全部列；差分数组下标 = c_pix-1
        idx = np.floor(xcross).astype(np.int64) + 1     # c_pix 的 1-based 起点
        np.clip(idx, 1, n + 1, out=idx)
        D[rows.astype(np.int64) - 1, idx - 1] ^= 1

    # 沿列做前缀异或：mask[r, c] = 自像素中心 (c+1, r+1) 向左的边界交点数是否为奇
    mask = np.bitwise_xor.accumulate(D[:, :n], axis=1).astype(bool)
    return mask


def _snap_subpixel(a: np.ndarray, how: str = "round") -> np.ndarray:
    """把像素坐标吸附到 5×5 子像素网格节点（0.2 步长、偏移 0.1）。

    网格节点 = 0.1 + 0.2k（像素 ``c`` 覆盖 [c-0.5, c+0.5]，内部四条网格线
    在 c-0.3, c-0.1, c+0.1, c+0.3）。对应 MATLAB 老文档的
    「Adjust the position of the vertices to be on the intersections of the
    subpixel grid」。``how`` 取 "round"（最近节点，文档口径）/ "ceil" / "floor"。
    """
    t = 5.0 * (a - 0.5)
    if how == "round":
        u = np.round(t)
    elif how == "ceil":
        u = np.ceil(t)
    elif how == "floor":
        u = np.floor(t)
    else:
        raise ValueError("snap 取 False / 'round' / 'ceil' / 'floor'，收到 %r" % (how,))
    return 0.5 + u / 5.0


def region_mask(vertices: np.ndarray, m: int, n: int,
                snap: bool = False) -> np.ndarray:
    """便捷入口：``vertices`` 为 (k, 2) 的 (x, y) 像素坐标数组。"""
    v = np.asarray(vertices, dtype=np.float64)
    if v.ndim != 2 or v.shape[1] != 2:
        raise ValueError("vertices 需为 (k, 2) 的 (x, y) 数组")
    return poly2mask(v[:, 0], v[:, 1], m, n, snap=snap)


def visible_from_shadow(vertices: np.ndarray, m: int, n: int,
                        snap: bool = False) -> np.ndarray:
    """``simulate_occluder`` 的输出口径：``~poly2mask(...)``（1=可见）。"""
    return ~region_mask(vertices, m, n, snap=snap)


def wall_pixel_grid(setup, n: int) -> Tuple[np.ndarray, np.ndarray]:
    """``poly2mask`` 评估点对应的墙面世界坐标（整数像素中心 1..n）。

    ``c1 = (x - FOV_x0) * n / FOV_w`` ⇒ ``x = FOV_x0 + c1 * FOV_w / n``。
    注意这与墙**采样点**网格 ``FOV_x0 + (i-1)*FOV_w/(n-1)`` 不同：前者右端
    恰好落在 FOV 上沿、左端内缩 1/n，两者最多相差 1 个像素。
    """
    ix = np.arange(1, n + 1, dtype=np.float64)
    x = setup.wall.x_min + ix * (setup.wall.x_max - setup.wall.x_min) / n
    z = setup.wall.z_min + ix * (setup.wall.z_max - setup.wall.z_min) / n
    return x, z


def project_occluder_corners(corners: np.ndarray, light_xyz: np.ndarray,
                             wall_y: float, fov_x0: float, fov_w: float,
                             fov_z0: float, fov_h: float,
                             n: int, light_y: float = 0.0) -> np.ndarray:
    """把遮挡物角点沿「光源→角点」射线投到墙面，再换算成像素坐标。

    与 ``ray_plane_intersect_y`` 等价：三个平面平行 ⇒ 投影是以光源为中心的位似，
    比例 ``s = (wall_y - light_y) / (corner_y - light_y)``（光源默认 y = 0）。
    """
    c = np.asarray(corners, dtype=np.float64)
    out = np.empty((c.shape[0], 2), dtype=np.float64)
    for i in range(c.shape[0]):
        denom = c[i, 1] - light_y
        s = (wall_y - light_y) / denom if denom != 0 else 0.0
        px = light_xyz[0] + s * (c[i, 0] - light_xyz[0])
        pz = light_xyz[2] + s * (c[i, 2] - light_xyz[2])
        out[i, 0] = (px - fov_x0) * n / fov_w
        out[i, 1] = (pz - fov_z0) * n / fov_h
    return out
