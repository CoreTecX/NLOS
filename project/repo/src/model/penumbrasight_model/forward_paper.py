"""论文口径的光传输矩阵（用于与 MATLAB simA 逐值对标）

本模块**刻意不复用** `forward.build_transport_matrix`，而是照论文
`export_ref_for_python.m` 的 `ps_simulate_A_cpu` / `ps_simulate_block_cpu` /
`ps_viewing_angle_factor` 逐行重写。命名与原文一一对应，便于对照。

**状态（2026-09-26，X4 收尾）**：与 MATLAB `simA` 的相对误差已到 **2.15e-16（va0）/
3.34e-16（va1）**，判据 1e-6 达标，非零元集合与参照完全相同。
见 `sim/results/x4_poly2mask_alignment.md`。

与现有实现的关键差异（见 sim/results/x4_forward_spec.md）：
  1. 行序：k = z + n·x（z 最快），并有一个 flipud
  2. 列序：k = (mr−1) + (ncol−mc)·nrow（z 行最快、x 降序）
  3. 视角因子 MM = Mx(x) · Mz(z)，指数分别是 1 与 18；Mz 用**反向**的 z 数组
  4. 块积分：Ndiscr_mon² 点子采样求和，不是乘块面积
  5. Mon_xdiscr / Mon_zdiscr 是块的**边界**，块中心 = 边界 − 半个块
  6. 遮挡物是**两个**多边形的乘积（可见性取交集）
  7. **遮挡栅格化走 `poly2mask` 口径**（`occluder_mask="poly2mask"`，默认）：
     角点投到墙面 → 换算成 `[0, n]` 的像素坐标 → 在**整数像素中心**上 even-odd 判定。
     旧口径（墙采样点 + 交点闭区间）保留为 `occluder_mask="wall_samples"` 作 A/B 对照，
     它给出 4.57e-2 的残差，正是 X4 之前那个数。

用法：
    from penumbrasight_model import forward_paper as FP
    A, meta = FP.build_paper_A(setup)
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import forward as F
from . import poly2mask as P2M


# ---------------------------------------------------------------- 遮挡物


def _rect_from_corners(c: np.ndarray) -> Tuple[float, float, float, float]:
    """从一个多边形的 4 个角点取出它的 x、z 范围。"""
    return (float(c[:, 0].min()), float(c[:, 0].max()),
            float(c[:, 2].min()), float(c[:, 2].max()))


def default_polygons(setup: F.OpticalSetup,
                     occ_corner: Optional[np.ndarray] = None
                     ) -> List[Tuple[float, float, float, float]]:
    """返回遮挡物的矩形列表 [(x0, x1, z0, z1), ...]。

    论文的 occ_corner 有 2 个多边形：正方形挡板 + 椅腿。若未提供 occ_corner，
    退化为单一矩形（即现有实现的口径），便于做对照。
    """
    if occ_corner is None:
        return [(setup.occluder.x_range[0], setup.occluder.x_range[1],
                 setup.occluder.z_range[0], setup.occluder.z_range[1])]
    return [_rect_from_corners(occ_corner[:, :, k])
            for k in range(occ_corner.shape[2])]


def corner_polygons(setup: F.OpticalSetup,
                    polygons: Optional[Sequence] = None
                    ) -> List[np.ndarray]:
    """把 ``polygons`` 规范化为「遮挡物平面上的 (4, 3) 角点数组」列表。

    接受两种输入：
      · (4, 3) 角点数组（逐多边形）—— 直接使用；
      · (x0, x1, z0, z1) 矩形 —— 在遮挡物平面上展开成 4 个角点。
    """
    polys = default_polygons(setup) if polygons is None else list(polygons)
    y = setup.occluder.y
    out: List[np.ndarray] = []
    for p in polys:
        a = np.asarray(p, dtype=np.float64)
        if a.shape == (4, 3):
            out.append(a.copy())
            continue
        x0, x1, z0, z1 = (float(v) for v in a.ravel()[:4])
        out.append(np.array([[x0, y, z0], [x1, y, z0],
                             [x1, y, z1], [x0, y, z1]], dtype=np.float64))
    return out


# ---------------------------------------------------------------- 视角因子


def viewing_angle_factor(ref_xyz: Sequence[float], wall_x: np.ndarray,
                         wall_z: np.ndarray, D: float,
                         mz_reversed: bool = True) -> np.ndarray:
    """照抄 `ps_viewing_angle_factor`。

        Mz = cos(atan((mon_z − FOV_z)/D)).^18
        Mx = cos(atan((mon_x − FOV_x)/D)).^1
        M  = (Mx' * Mz)'   →  M(j,k) = Mx(k) · Mz(j)

    mz_reversed=True 时 FOV_z 取反向的 wall_z（原文传的是 wall_matr(3,end:-1:1)）。
    """
    ref_x, _, ref_z = ref_xyz[0], ref_xyz[1], ref_xyz[2]
    fz = wall_z[::-1] if mz_reversed else wall_z
    Mz = np.cos(np.arctan((ref_z - fz) / D)) ** 18          # (n,)
    Mx = np.cos(np.arctan((ref_x - wall_x) / D)) ** 1       # (n,)
    return np.outer(Mz, Mx)                                 # (j=z 索引, k=x 索引)


# ---------------------------------------------------------------- 主函数


def build_paper_A(setup: F.OpticalSetup,
                  ndiscr: int = 6,
                  view_angle: bool = True,
                  mz_reversed: bool = True,
                  polygons: Optional[List] = None,
                  light_y: float = 0.0,
                  block_center_from_edge: bool = True,
                  occluder_mask: str = "poly2mask",
                  snap: bool = False
                  ) -> Tuple[np.ndarray, Dict[str, object]]:
    """按论文口径构造 A，形状 (n², nrow·ncol)。

    返回 (A, meta)。A 的列序与行序都与 MATLAB 的 `simA` 一致。

    occluder_mask 决定遮挡栅格化口径（这是 X4 收尾的核心开关）：
      · "poly2mask"（**默认**）：照 `simulate_occluder.m` 的口径 —— 角点投到墙面后
        换算成像素坐标，用 `poly2mask` 的判定规则栅格化（评估点 = 整数像素中心
        1..n，even-odd，不做 5×5 子像素吸附）。实测与 MATLAB `simA` 相对误差
        2e-16（机器精度）。
      · "wall_samples"：X4 之前的旧口径 —— 把**墙采样点**代入「交点是否落在遮挡物
        矩形内」的闭区间判据。评估网格与 poly2mask 的整数网格最多差 1 个像素，
        残留相对误差 4.6e-2。
    """
    n = setup.wall.n
    nrow, ncol = setup.monitor.shape                  # (29, 36)
    blkw = setup.monitor.block_w
    blkh = setup.monitor.block_h
    wall_y = setup.wall.y
    D = wall_y                                        # simuParams.D

    # 墙网格：FOV 端点（原文 wall_vec = -1 : 2/(n-1) : 1）
    wall_x = setup.wall.x_min + np.arange(n) * (setup.wall.x_max - setup.wall.x_min) / (n - 1)
    wall_z = setup.wall.z_min + np.arange(n) * (setup.wall.z_max - setup.wall.z_min) / (n - 1)

    # 显示器：Mon_xdiscr / Mon_zdiscr 是块的边界
    mx_edge = np.asarray(setup.monitor.x_centers, dtype=np.float64)
    mz_edge = np.asarray(setup.monitor.z_centers, dtype=np.float64)

    yw2 = float((wall_y - light_y) ** 2)

    # 遮挡物：规范化为 (4, 3) 角点（供 poly2mask 口径使用）
    corner_polys = corner_polygons(setup, polygons)
    # 旧口径用的矩形范围 [(x0, x1, z0, z1), ...]
    rect_polys: List[Tuple[float, float, float, float]] = []
    for p in (default_polygons(setup) if polygons is None else list(polygons)):
        a = np.asarray(p, dtype=np.float64)
        rect_polys.append(_rect_from_corners(a) if a.shape == (4, 3)
                          else tuple(float(v) for v in a.ravel()[:4]))

    t = setup.occluder.y / wall_y                     # 交点参数 t = y_occ / y_wall
    if occluder_mask not in ("poly2mask", "wall_samples"):
        raise ValueError(f"未知的 occluder_mask 口径 {occluder_mask}")

    def visibility_for(light_xyz: Sequence[float]) -> np.ndarray:
        """返回 (z 行, x 列) 的可见掩码。"""
        if occluder_mask == "poly2mask":
            vis = np.ones((n, n), dtype=bool)
            for corners in corner_polys:
                pix = P2M.project_occluder_corners(
                    corners, light_xyz,
                    wall_y=wall_y, light_y=light_y,
                    fov_x0=setup.wall.x_min, fov_w=setup.wall.x_max - setup.wall.x_min,
                    fov_z0=setup.wall.z_min, fov_h=setup.wall.z_max - setup.wall.z_min,
                    n=n)
                vis &= ~P2M.poly2mask(pix[:, 0], pix[:, 1], n, n, snap=snap)
            return vis
        # 旧口径：墙采样点 + 交点闭区间
        lx, lz = float(light_xyz[0]), float(light_xyz[2])
        qx = (1.0 - t) * lx + t * wall_x
        qz = (1.0 - t) * lz + t * wall_z
        vis = np.ones((n, n), dtype=bool)
        for (x0, x1, z0, z1) in rect_polys:
            in_x = (qx >= x0) & (qx <= x1)
            in_z = (qz >= z0) & (qz <= z1)
            vis &= ~(in_z[:, None] & in_x[None, :])
        return vis

    A = np.zeros((n * n, nrow * ncol), dtype=np.float64)
    mz_used = None

    kindex = 0
    for mc in range(ncol - 1, -1, -1):                # 原文 mc = ncol:-1:1
        for mr in range(nrow):                        # 原文 mr = 1:nrow
            # 视角因子的参考点：块的**中心**
            ref_x = mx_edge[mc] - blkw / 2 if block_center_from_edge else mx_edge[mc]
            ref_z = mz_edge[mr] - blkh / 2 if block_center_from_edge else mz_edge[mr]
            if view_angle:
                MM = viewing_angle_factor([ref_x, light_y, ref_z],
                                          wall_x, wall_z, D, mz_reversed)
                mz_used = wall_z[::-1] if mz_reversed else wall_z
            else:
                MM = np.ones((n, n), dtype=np.float64)

            # 36 个子采样位置：从块边界向内
            sx = mx_edge[mc] - (np.arange(ndiscr) / ndiscr) * blkw
            sz = mz_edge[mr] - (np.arange(ndiscr) / ndiscr) * blkh
            SX, SZ = np.meshgrid(sx, sz)              # (ndiscr, ndiscr)
            lx_all = SX.reshape(-1)
            lz_all = SZ.reshape(-1)

            img = np.zeros((n, n), dtype=np.float64)
            for i in range(lx_all.size):
                lx = float(lx_all[i])
                lz = float(lz_all[i])
                dx2 = (wall_x - lx) ** 2              # (n,) 配 x 索引 = 列 b
                dz2 = (wall_z - lz) ** 2              # (n,) 配 z 索引 = 行 a
                tmp = dz2[:, None] + yw2 + dx2[None, :]   # (a=z, b=x)
                inten = MM * yw2 / (tmp * tmp)
                img += inten * visibility_for([lx, light_y, lz])

            A[:, kindex] = np.flipud(img).reshape(-1, order="F")
            kindex += 1

    meta = {
        "n": n, "nrow": nrow, "ncol": ncol, "ndiscr": ndiscr,
        "view_angle": view_angle, "mz_reversed": mz_reversed,
        "n_polygons": len(corner_polys), "t_occluder": t,
        "wall_x": wall_x, "wall_z": wall_z, "yw2": yw2,
        "mz_used": mz_used,
        "occluder_mask": occluder_mask, "snap": snap,
    }
    return A, meta
