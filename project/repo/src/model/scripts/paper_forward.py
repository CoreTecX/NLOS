# -*- coding: utf-8 -*-
"""论文前向模型的 Python 逐行复现（D11 + bu 配置）。

用途
----
1. 与 PenumbraSight 自有实现（forward.py）逐值比对，量化四个差异点；
2. 作为「修正后的 A」，用于重跑重建，检验能否复现论文的 BU 重建结果。

对应 MATLAB 源文件
------------------
    Functions/simulate_A.m
    Functions/simulate_block.m
    Functions/simulate_occluder.m
    Functions/ray_plane_intersect_y.m
    Functions/load_image1.m
    Functions/load_experiment_config_data_reconstruction.m  (D11 分支)

关键差异（相对 PenumbraSight 原实现）
-------------------------------------
1. 每个场景块离散为 Ndiscr_mon^2 个点光源，逐点算遮挡后叠加（半影建模）
2. 视角因子为 cos(atan(dx/D))^1 * cos(atan(dz/D))^18（各向异性）
3. 测量为 8x8 块平均（非求和）；A 的标度随之不同
4. simulate_block 输出后 flipud，且视角因子用翻转的 z 坐标
"""
import os
import warnings

import numpy as np
from scipy.spatial import ConvexHull

# matplotlib 是可选依赖（pyproject.toml 的 optional-dependencies）。
# 点-多边形判定是算法通路（遮挡可见性），不是绘图：缺包时走纯 numpy 的
# 射线法兜底（_points_in_polygon），有 matplotlib 时仍走 Path.contains_points，
# 保持此前已验证口径不变。
try:
    from matplotlib.path import Path
except ImportError:  # pragma: no cover - 取决于环境是否装 matplotlib
    Path = None

# ============================================================ D11 配置
# 来源：load_experiment_config_data_reconstruction.m 的 case 'D11'
D = 1.03
FOV_SIZE = np.array([0.4372, 0.4372])
FOV_LLCORNER = np.array([0.521, 0.048])
OCC_SIZE = np.array([0.075, 0.0, 0.075])
OCC_LLCORNER_BU = np.array([0.4733, 0.5661, 0.2072])   # fig4_column_c.m 的 bu 分支
NUMBLOCKS_ROW = 29
NUMBLOCKS_COL = 36
SCREEN_SIZE = np.array([0.408, 0.3085])
SCREEN_RESOLUTION = np.array([1280, 1024])
NUM_PIXELS_PER_BLOCK = 35
NUM_PIXELS = 1008
DOWNSAMP_FACTOR = 3

DATA_DIR = (r"D:\学习\26秋嵌入式\论文数据集\Ordinary-Camera-master"
            r"\Data\TestPosD11")


def make_geometry():
    """复现 fig4_column_c.m 的 setup 段（不含数据加载）。"""
    wall_point = np.array([FOV_LLCORNER[0] + FOV_SIZE[0] / 2, D,
                           FOV_LLCORNER[1] + FOV_SIZE[1] / 2])
    wall_vector_1 = np.array([FOV_SIZE[0] / 2, 0.0, 0.0])
    wall_vector_2 = np.array([0.0, 0.0, FOV_SIZE[1] / 2])
    wall_normal = np.cross(wall_vector_1, wall_vector_2)
    wall_normal = wall_normal / np.linalg.norm(wall_normal)

    walln_points = int(np.floor(NUM_PIXELS / (2 ** DOWNSAMP_FACTOR)))

    wall_vec = np.linspace(-1.0, 1.0, walln_points)
    wall_matr = np.zeros((3, walln_points))
    wall_matr[0, :] = wall_point[0] + wall_vec * wall_vector_1[0] + wall_vec * wall_vector_2[0]
    wall_matr[1, :] = wall_point[1] + wall_vec * wall_vector_1[1] + wall_vec * wall_vector_2[1]
    wall_matr[2, :] = wall_point[2] + wall_vec * wall_vector_1[2] + wall_vec * wall_vector_2[2]

    # 显示器分块
    pixel_size_m = SCREEN_SIZE / SCREEN_RESOLUTION
    mon_offset = np.array([0.0210, 0.136])
    mon_offset[1] = mon_offset[1] + pixel_size_m[1] * np.mod(SCREEN_RESOLUTION[1], NUMBLOCKS_ROW)
    mon_offset[0] = mon_offset[0] + 1 * pixel_size_m[0] * np.mod(SCREEN_RESOLUTION[0], NUMBLOCKS_COL)

    illum_block = pixel_size_m * NUM_PIXELS_PER_BLOCK

    mon_xlim = np.array([0.0, NUMBLOCKS_COL]) * illum_block[0] + mon_offset[0]
    mon_zlim = np.array([0.0, NUMBLOCKS_ROW]) * illum_block[1] + mon_offset[1]
    mon_xdiscr = np.linspace(mon_xlim[0], mon_xlim[1], NUMBLOCKS_COL)
    # 注意：z 方向是反向的（linspace(zlim[2], zlim[1], ...)）
    mon_zdiscr = np.linspace(mon_zlim[1], mon_zlim[0], NUMBLOCKS_ROW)

    return dict(
        wall_point=wall_point,
        wall_vector_1=wall_vector_1,
        wall_vector_2=wall_vector_2,
        wall_normal=wall_normal,
        walln_points=walln_points,
        wall_matr=wall_matr,
        mon_xdiscr=mon_xdiscr,
        mon_zdiscr=mon_zdiscr,
        illum_block=illum_block,
        mon_offset=mon_offset,
    )


def make_occ_corner(occ_llcorner=None):
    """复现 fig4_column_c.m 里 occ_corner 的构造（两个凸多边形）。"""
    ll = OCC_LLCORNER_BU if occ_llcorner is None else np.asarray(occ_llcorner, float)
    occ = np.zeros((4, 3, 2))
    occ[0, :, 0] = ll
    occ[1, :, 0] = ll + np.array([OCC_SIZE[0], 0, 0])
    occ[2, :, 0] = ll + np.array([OCC_SIZE[0], 0, OCC_SIZE[2]])
    occ[3, :, 0] = ll + np.array([0, 0, OCC_SIZE[2]])

    occ[0, :, 1] = ll + np.array([OCC_SIZE[0] / 2 - 0.00275, 0, 0])
    occ[1, :, 1] = ll + np.array([OCC_SIZE[0] / 2 + 0.00275, 0, 0])
    occ[2, :, 1] = ll + np.array([OCC_SIZE[0] / 2 - 0.00275, 0, -ll[2]])
    occ[3, :, 1] = ll + np.array([OCC_SIZE[0] / 2 + 0.00275, 0, -ll[2]])
    return occ


# ============================================================ 几何基元
def ray_plane_intersect_y(ray_start, ray_end, plane_point, plane_normal):
    """复现 ray_plane_intersect_y.m（针对 y 对齐平面优化）。"""
    ray_dir = ray_end - ray_start
    ray_dir = ray_dir / np.linalg.norm(ray_dir)
    dist = (ray_start[1] - plane_point[1]) / (-ray_dir[1])
    if dist > 0:
        return ray_start + dist * ray_dir
    point = ray_start - dist * ray_dir
    return point + point[1] * plane_normal + plane_point


def _points_in_polygon(points, polygon):
    """纯 numpy 点-多边形判定（水平向右射线法），matplotlib 缺失时的兜底。

    与 ``matplotlib.path.Path.contains_points`` 等价的内点判定；多边形按
    ``polygon[i] -> polygon[(i+1) % n]`` 逐边处理（含闭合边）。
    边界点（查询点恰好落在边上）的归类可能与 matplotlib 不同：本工程查询点
    是整数像素中心、多边形顶点来自几何计算，实测没有点恰好落在边上。
    返回布尔数组，长度 == points.shape[0]，在内为 True。
    """
    px = points[:, 0, None]
    py = points[:, 1, None]
    x1 = polygon[:, 0][None, :]
    y1 = polygon[:, 1][None, :]
    x2 = np.roll(polygon[:, 0], -1)[None, :]
    y2 = np.roll(polygon[:, 1], -1)[None, :]
    # 边跨过点的水平射线（端点规则：下端计入、上端不计入）
    crosses_y = (y1 > py) != (y2 > py)
    with np.errstate(divide="ignore", invalid="ignore"):
        xint = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
    return ((crosses_y & (px < xint)).sum(axis=1) % 2).astype(bool)


class PolyMasker:
    """poly2mask 的等价款，预计算墙点坐标以加速。

    MATLAB 的 poly2mask(x, y, m, n) 以像素中心 (x=col, y=row)（1-based）做判定，
    落在多边形内为 1。simulate_occluder 取反，得到「可见」掩码。
    """

    def __init__(self, walln):
        rows, cols = np.mgrid[0:walln, 0:walln]
        # 1-based 像素坐标：(x, y) = (col + 1, row + 1)
        self.pts = np.column_stack([(cols.ravel() + 1.0), (rows.ravel() + 1.0)])
        self.shape = (walln, walln)
        self.walln = walln

    def visible_mask(self, x, y):
        """返回可见掩码：(walln x walln)，可见为 1、被遮挡为 0。"""
        v = np.column_stack([x, y])
        if v.shape[0] >= 3:
            try:
                hull = ConvexHull(v)
                v = v[hull.vertices]
            except Exception:
                pass
        if v.shape[0] >= 3:
            if Path is not None:
                # matplotlib 的 Path(closed=True) 会把**最后一个顶点**的 code 设为
                # CLOSEPOLY（其坐标被忽略），n 个顶点会丢掉最后一个角、多边形退化。
                # 正确做法：首尾重复第一个顶点凑 n+1 个顶点再 closed。
                closed_v = np.vstack([v, v[:1]])
                path = Path(closed_v, closed=True)
                inside = path.contains_points(self.pts)
            else:
                warnings.warn(
                    "matplotlib 不可用，遮挡掩膜改用纯 numpy 射线法兜底"
                    "（内点分类与 matplotlib 一致；边界点除外）",
                    stacklevel=2)
                inside = _points_in_polygon(self.pts, np.asarray(v, dtype=np.float64))
            return (~inside).reshape(self.shape).astype(np.float64)
        return np.ones(self.shape, dtype=np.float64)


def viewing_angle_factor(monitor_pixel_xyz, fov_xdiscr, fov_zdiscr, dd):
    """复现 simulate_A.m 内的局部函数 ViewingAngleFactor。"""
    powexp = 18
    ang = np.arctan((monitor_pixel_xyz[2] - fov_zdiscr) / dd)
    mz = np.cos(ang) ** powexp
    mx = np.cos(np.arctan((monitor_pixel_xyz[0] - fov_xdiscr) / dd)) ** 1
    # MATLAB: M = (Mx'*Mz)';  M(m, k) = Mx(k) * Mz(m)
    return np.outer(mz, mx)


def simulate_A(geom, occ_corner, n_discr=6, view_angle=True, verbose=True):
    """复现 simulate_A.m。返回 (A, light_pos_info)。"""
    wall_matr = geom["wall_matr"]
    wall_point = geom["wall_point"]
    wall_normal = geom["wall_normal"]
    walln = geom["walln_points"]
    mon_xdiscr = geom["mon_xdiscr"]
    mon_zdiscr = geom["mon_zdiscr"]
    blk = geom["illum_block"]

    masker = PolyMasker(walln)

    n_col = NUMBLOCKS_COL
    n_row = NUMBLOCKS_ROW
    A = np.zeros((walln * walln, n_row * n_col))
    blockcount = 0
    n_src = n_discr * n_discr

    for mc in range(n_col - 1, -1, -1):            # MATLAB: NumBlocks_col:-1:1
        for mr in range(n_row):                     # MATLAB: 1:NumBlocks_row
            light_y = 0.0

            if view_angle:
                mm = viewing_angle_factor(
                    np.array([mon_xdiscr[mc] - blk[0] / 2, light_y,
                              mon_zdiscr[mr] - blk[1] / 2]),
                    wall_matr[0, :], wall_matr[2, ::-1], D)
            else:
                mm = np.ones((walln, walln))

            # 块内离散点光源（等价 meshgrid + 冒号表达式）
            lx = mon_xdiscr[mc] - np.arange(n_discr) * (blk[0] / n_discr)
            lz = mon_zdiscr[mr] - np.arange(n_discr) * (blk[1] / n_discr)
            gx, gz = np.meshgrid(lx, lz)
            light_pos = np.column_stack([gx.ravel(), np.full(n_src, light_y), gz.ravel()])

            image = simulate_block(wall_matr, wall_point, geom, walln, light_pos,
                                   occ_corner, mm, masker)
            A[:, blockcount] = image[::-1, :].ravel()   # MATLAB: flipud(...)(:) 
            blockcount += 1

        if verbose and (mc % 6 == 0):
            print(f"    A 组装进度 {blockcount}/{n_row * n_col}", flush=True)

    return A


def simulate_block(wall_matr, wall_point, geom, walln, light_pos, occ_corner, mm, masker):
    """复现 simulate_block.m（2D 版，重建阶段使用）。"""
    blk = geom["illum_block"]
    wall_vector_1 = geom["wall_vector_1"]
    wall_vector_2 = geom["wall_vector_2"]
    wall_normal = geom["wall_normal"]
    n_src = light_pos.shape[0]
    light_y = light_pos[0, 1]

    # vec: 墙点 (x, z) 到光源 (x, z) 的差
    dx = wall_matr[0, :][:, None] - light_pos[:, 0][None, :]      # (W, N)
    dz = wall_matr[2, :][:, None] - light_pos[:, 2][None, :]      # (W, N)

    # y 分量对所有墙点与所有光源都是常数（光源 y=0，墙 y=D）
    vs2 = (wall_matr[1, 0] - light_y) ** 2

    # tmp[m, k, i] = dx[k,i]^2 + vs2 + dz[m,i]^2  = ||v||^2
    dist2 = np.empty((walln, walln, n_src))
    for i in range(n_src):
        # 第 1 维对应 dz（z 索引 m），第 2 维对应 dx（x 索引 k）
        dist2[:, :, i] = (dz[:, i][:, None] + vs2) + dx[:, i][None, :]
    tmp = dist2 ** 2

    intensity = mm[:, :, None] * (vs2 / tmp)

    image = np.zeros((walln, walln))
    n_poly = occ_corner.shape[2]
    for i in range(n_src):
        lp = light_pos[i, :]
        occ_img = None
        for o in range(n_poly):
            corners = occ_corner[:, :, o]
            proj = np.array([
                ray_plane_intersect_y(lp, corners[c], wall_point, wall_normal)
                for c in range(corners.shape[0])
            ])
            # 投到 FOV 坐标
            c1 = (proj[:, 0] - (wall_point[0] - wall_vector_1[0])) * walln / (wall_vector_1[0] * 2)
            c2 = (proj[:, 2] - (wall_point[2] - wall_vector_2[2])) * walln / (wall_vector_2[2] * 2)
            m = masker.visible_mask(c1, c2)
            occ_img = m if occ_img is None else occ_img * m
        image = image + intensity[:, :, i] * occ_img

    return image


# ============================================================ 测量加载
def load_measurement(mat_name="image_test_bur20.mat", downsamp=DOWNSAMP_FACTOR,
                     data_dir=DATA_DIR, apply_shutter=True):
    """复现 load_image1.m：Bayer 去马赛克 + 逐级 2x2 平均下采样。

    返回 (126, 126, 3) 的测量矩阵与 (29, 36, 3) 的 ground truth。
    """
    import scipy.io as sio

    iml = sio.loadmat(os.path.join(data_dir, mat_name))
    im = iml["image"].astype(np.float64)
    gt = iml["ground_truth"]

    # get_shutter_speed：优先 averaged_iterations
    speed = float(iml["shutter_speed"].ravel()[0])
    if "averaged_iterations" in iml:
        measure_shut = speed * float(iml["averaged_iterations"].ravel()[0])
    else:
        measure_shut = speed * float(iml["iterations"].ravel()[0])

    # 复现 load_image1 里的 calshut 逻辑：首次调用时 calshut = measure_shut，比值恒为 1
    factor = 1.0
    if apply_shutter and "calshut" in iml:
        calshut = float(iml["calshut"].ravel()[0])
        factor = calshut / measure_shut

    # get_color_image（RGGB 相位，双绿取平均）
    red = im[0::2, 0::2]
    green = (im[1::2, 0::2] + im[0::2, 1::2]) / 2.0
    blue = im[1::2, 1::2]

    chans = []
    for ch in (red, green, blue):
        # downsample_2：逐级 2x2 平均（注意是先下采样再赋值，形状会缩小）
        x = ch
        for _ in range(downsamp):
            a = x[0::2, 0::2]
            b = x[0::2, 1::2]
            c2 = x[1::2, 0::2]
            d = x[1::2, 1::2]
            x = (a + b + c2 + d) / 4.0
        chans.append(factor * x)

    out = np.stack(chans, axis=2)
    return out, gt


if __name__ == "__main__":
    import time

    print("=" * 70)
    print("论文前向模型 Python 复现 · D11 + bu")
    print("=" * 70)

    geom = make_geometry()
    print(f"墙网格: {geom['walln_points']} x {geom['walln_points']} "
          f"= {geom['walln_points']**2}")
    print(f"场景块: {NUMBLOCKS_ROW} x {NUMBLOCKS_COL} = {NUMBLOCKS_ROW * NUMBLOCKS_COL}")
    print(f"墙点 (x) 范围: [{geom['wall_matr'][0].min():.6f}, {geom['wall_matr'][0].max():.6f}]")
    print(f"墙点 (z) 范围: [{geom['wall_matr'][2].min():.6f}, {geom['wall_matr'][2].max():.6f}]")
    print(f"Mon_x 范围: [{geom['mon_xdiscr'].min():.6f}, {geom['mon_xdiscr'].max():.6f}]")
    print(f"Mon_z 范围: [{geom['mon_zdiscr'].min():.6f}, {geom['mon_zdiscr'].max():.6f}]")
    print(f"照明块尺寸: {geom['illum_block']}")

    t0 = time.time()
    meas, gt = load_measurement()
    print(f"\n测量矩阵: {meas.shape}  范围 [{meas.min():.1f}, {meas.max():.1f}]  "
          f"({time.time() - t0:.1f}s)")
    print(f"ground truth: {gt.shape}")

    occ = make_occ_corner()
    print(f"遮挡物四角 shape: {occ.shape}")

    t0 = time.time()
    A = simulate_A(geom, occ, n_discr=6, view_angle=True, verbose=True)
    print(f"\nA 矩阵: {A.shape}  用时 {time.time() - t0:.1f}s")
    print(f"A 范围 [{A.min():.6e}, {A.max():.6e}]  "
          f"动态范围 {A[A > 0].max() / A[A > 0].min():.3e}")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "..", "..", "sim", "vectors", "paper_A_va1.npy")
    out = os.path.abspath(out)
    np.save(out, A)
    np.save(out.replace("paper_A_va1.npy", "paper_meas.npy"), meas)
    np.save(out.replace("paper_A_va1.npy", "paper_gt.npy"), gt)
    print(f"\n已保存 {out}")
