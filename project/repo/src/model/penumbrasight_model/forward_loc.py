# -*- coding: utf-8 -*-
"""定位链前向模型的逐行 Python 移植（论文 `SimulateForwardModelPerBlock` +
`SimulateA_OccluderEstimation`）。

**为什么需要单独一个模块**（详见 `sim/results/x4h_localization_forward_model.md`）：
论文仓库里有**两套互不相同的前向模型**：

  重建链  fig4_column_c.m → simulate_A  → simulate_block
          → simulate_occluder（poly2mask 栅格化）；视角指数 Mz^18 * Mx^1
  定位链  table_S1.m     → SimulateA_OccluderEstimation
          → SimulateForwardModelPerBlock（**解析阴影**，零栅格化）；视角指数 Mz^20 * Mx^0

本模块移植的是**定位链**那一套。它不是「同一份代码换参数」，
所以不能靠 `forward.py` / `forward_paper.py` 加个开关实现。

移植时**刻意保留**的几处原代码特征（都对结果有实质影响，不能"顺手修正"）：

1. 显示器是**整屏全局离散**再按块挑选，不是逐块局部采样：
   `Mon_xdiscr = linspace(off_x, off_x + ncol*blk_x, N*ncol)`，
   `Mon_zdiscr = linspace(off_z + nrow*blk_z, off_z, N*nrow)`（**降序**）。
   注意 N*ncol 个点铺在 ncol*blk_x 上，间距 ≠ blk_x/N，**采样点与块边界不对齐**。
2. 块 → 显示器点的映射里，**块的 x 序号与显示器 x 序号反向**（源码注释
   "this not so natural ordering used below, due to the file naming convention"）。
3. 遮挡是**解析阴影**：`X1 = D*(e1-x)/occ_d + x`、`X2 = D*(e2-x)/occ_d + x`，
   区间内为 0（被挡）、区间外为 1（可见）；x/z 两个一维可见性做**外积**即精确解
   （因为遮挡物是 x-z 平面内的矩形，投影到墙上仍是轴对齐矩形）。
4. 另有一根**独立立柱** occstick（半宽 0.0035，z 从 0 到遮挡物下沿），
   与矩形阴影用 **OR（并集）** 合并——重建链那边是逐元素乘积（交集）。
5. 辐射项 `temp1 .* temp2 = D^2 / d^6`（`d^2 = (mx-wx)^2 + D^2 + (mz-wz)^2`），
   注意这与重建链的 `y^2/d^4` **不同**。
6. 视角因子 `ViewingAngleFactor` 内部用的是**降序**的 `FOV_xdiscr / FOV_zdiscr`
   （`linspace(FOV(2,:), FOV(1,:))`），而墙网格 `Wall_*discr` 是**升序**，
   两者由 `(Mx*Mz')` 外积后按元素相乘 ⇒ **视角因子被当成 z 的镜像在使用**。
   x 侧因指数为 0（Mx ≡ 1）无影响。这个镜像开关由 `mz_mirrored` 控制，
   默认 True（忠实原代码）。
7. A 的行序：`Image = reshape(sum(LL,2), npx, npx).'`（行=z、列=x），
   再 `flipud` + `(:)`（列优先）⇒ 行号 `k` 对应 `ix = k//npx`、`iz = npx-1-(k%npx)`。
   `row_to_wall_index()` 给出这个映射，不必再靠猜。
"""

from __future__ import annotations

import dataclasses
from typing import Optional, Tuple

import numpy as np

__all__ = [
    "LocSetup",
    "d11_loc_setup",
    "num_pixels_for",
    "wall_grid",
    "monitor_grid",
    "block_monitor_indices",
    "build_A",
    "row_to_wall_index",
    "load_measurement",
    "loc_energy_grid",
]

_OCC_STICK_HALFWIDTH = 0.0035   # SimulateForwardModelPerBlock.m:43
_MZ_POW = 20                    # SimulateForwardModelPerBlock.m:162
_MX_POW = 0                     # SimulateForwardModelPerBlock.m:166


# ======================================================================
# 参数
# ======================================================================
@dataclasses.dataclass(frozen=True)
class LocSetup:
    """对应论文 `load_experiment_config_data_localization.m` 里那几个变量。

    注意该文件**没有 switch，整份硬编码 D11**；`table_S1.m` 里四个场景
    (rgb / bu / mushroom / tommy) 都是**同一个 D11 位置上的不同物体**。
    """

    num_blocks: Tuple[int, int]     # (NumBlocks_row, NumBlocks_col) = (29, 36)
    num_pixels_raw: int             # simuParams.numPixels = 1008
    D: float                        # 显示器平面 ↔ 可见墙的距离
    fov_cord: np.ndarray            # (2,2) [[x_lo, z_lo], [x_hi, z_hi]]
    illum_block_size: np.ndarray    # (2,) 显示器块的 (x, z) 尺寸
    mon_offset: np.ndarray          # (2,) 可用屏区的左下角 (x, z)
    occ_size: np.ndarray            # (3,) 遮挡物尺寸 (x, y, z)

    # ---- 非物理、纯由加载器算出来的量，一并记下来便于核对 ----
    occ_ll_true: np.ndarray = dataclasses.field(default=None)
    occ_ll_est: np.ndarray = dataclasses.field(default=None)


def d11_loc_setup() -> LocSetup:
    """按 `load_experiment_config_data_localization.m` 逐行算出 D11 定位档参数。

    该文件里这几行是顺序执行的、且 Mon_Offset 两个分量互相独立：

        NumBlocks_sim = [29 36];  D = 1.03;
        Occ_size = [0.077 0 0.075];
        Occ_LLcorner = [0.470  D-0.460  0.2040];      % useEstimatedOccPos = false
        FOV_size = [0.4372 0.4372];  FOV_LLCorner = [0.5128 0.0482];
        Mon_Offset = [0.0210 0.136];
        NumBlocks_col = NumBlocks_sim(2);  NumBlocks_row = NumBlocks_sim(1);
        ScreenSize = [0.408 0.3085];  ScreenResolution = [1280 1024];
        NumPixelsPerMonitorBlock = 35;
        PixelSize_m = ScreenSize./ScreenResolution;
        Mon_Offset(2) += PixelSize_m(2)*mod(ScreenResolution(2), NumBlocks_sim(1));
        Mon_Offset(1) += PixelSize_m(1)*mod(ScreenResolution(1), NumBlocks_sim(2));
        IlluminationBlock_Size = PixelSize_m.*NumPixelsPerMonitorBlock;
    """
    num_blocks = (29, 36)
    D = 1.03
    occ_size = np.array([0.077, 0.0, 0.075], dtype=float)

    # 真值：注释 "Please note here that the occluder's y-position is stated as (D-p_y)"
    # 所以 Occ_LLcorner(2) 存的是「显示器 → 遮挡物」的距离，直接用进前向模型。
    occ_ll_true = np.array([0.470, D - 0.460, 0.2040], dtype=float)
    # useEstimatedOccPos = true 的那一组（论文里四个场景的估计值，注释掉的）
    occ_ll_est = np.array([0.4583, D - 0.4892, 0.2026], dtype=float)   # Mushroom

    fov_size = np.array([0.4372, 0.4372], dtype=float)
    fov_ll = np.array([0.5128, 0.0482], dtype=float)
    fov_cord = np.vstack([fov_ll, fov_ll + fov_size])

    mon_offset = np.array([0.0210, 0.136], dtype=float)
    screen_size = np.array([0.408, 0.3085], dtype=float)
    screen_res = np.array([1280, 1024], dtype=float)
    nppmb = 35.0

    pixel_size = screen_size / screen_res
    mon_offset = mon_offset.copy()
    mon_offset[1] = mon_offset[1] + pixel_size[1] * (screen_res[1] % num_blocks[0])
    mon_offset[0] = mon_offset[0] + 1.0 * pixel_size[0] * (screen_res[0] % num_blocks[1])

    illum = pixel_size * nppmb

    return LocSetup(
        num_blocks=num_blocks,
        num_pixels_raw=1008,
        D=float(D),
        fov_cord=fov_cord,
        illum_block_size=illum,
        mon_offset=mon_offset,
        occ_size=occ_size,
        occ_ll_true=occ_ll_true,
        occ_ll_est=occ_ll_est,
    )


def b_loc_setup() -> LocSetup:
    """B 档交付几何的定位档参数（`configs/geometry_b.yaml` 的冻结值）。

    ## num_pixels_raw 的口径与 d11 不同 —— 这是刻意的，不是笔误

    d11 档里 `num_pixels_raw = 1008`，含义是「**下采样前**的墙网格边长」，
    走 `SimulateA_OccluderEstimation.m:36` 的 `floor(npr / 2^ds)`：
    墙网格 = 1008 / 2^3 = 126（ds=3）、1008 / 2^4 = 63（ds=4）。
    1008 这个数来自 2016 原始像素经 Bayer 2x2 合并后的边长，能被 2 的幂整除。

    **B 档的分块是 10，不是 2 的幂**，没法折进 `2^ds` 这条规则里：
    B 档墙网格 96 = 1920 原始像素 / 2（Bayer 合并）/ 10（分块）。
    所以对 B 档，`num_pixels_raw` 取**分块之后**的边长 96，
    即「ds=0 时的墙网格边长」，`ds` 变成在此之上的**额外** 2 的幂级缩减。
    这么定是为了让 `num_pixels_for()` 这个既有接口继续成立。

    于是 B 档建模应取 `ds = 0`（得 96x96），而不是照搬论文档的 ds=4。

    ## 待标定项（标定前生成的模型只能用于通路联调）

    `mon_offset` 与 `screen_size` 依赖实验台实测，本函数取以下保守值并显式标注：

    - `screen_size`：取 geometry_b.yaml 的隐藏面范围 [0.40, 0.30]。
      物理上显示屏就是隐藏面，所以这两个量应当相等；若实测屏幕可视区不同，
      改 geometry_b.yaml 的 `hidden.extent`，不要只改这里。
    - `mon_offset`：**取 [0, 0]**。d11 档用的是 [0.0210, 0.136]，但那是论文
      实验台的实测偏移量，我们没有对应的测量。标定前不得用于报指标。
    - `occ_ll_true`：由几何中心 (-0.0375, 0.57, -0.0375) 推得，
      即「显示中心 - 75mm/2」。`occluder.nominal_center.calibrated` 目前是
      **false**，标定后要用实测值覆盖。
    """
    # ---- 冻结值：全部来自 configs/geometry_b.yaml，改这里之前先改 YAML ----
    wall_extent = 0.44          # wall.extent
    hidden_extent = np.array([0.40, 0.30], dtype=float)   # hidden.extent
    D = 1.03                    # planes.wall_y
    occ_y = 0.57                # planes.occluder_y
    occ_side = 0.075            # occluder.size （75 mm 见方）

    # 隐藏网格：geometry_b.yaml 的 hidden.n = [nx, nz] = [24, 18]，
    # 而 LocSetup.num_blocks 的口径是 (行, 列) = (nz, nx) —— 与 d11 一致
    # （d11 的 NumBlocks_sim = [29 36] 对应 29 行 36 列，29 是 z 向）。
    num_blocks = (18, 24)

    # 墙网格边长（分块后），见文首「num_pixels_raw 的口径」
    num_pixels_raw = 96

    # FOV：geometry_b.yaml 以显示中心为原点，墙视场 0.44 见方 -> [-0.22, +0.22]
    half = wall_extent / 2.0
    fov_ll = np.array([-half, -half], dtype=float)
    fov_size = np.array([wall_extent, wall_extent], dtype=float)
    fov_cord = np.vstack([fov_ll, fov_ll + fov_size])

    # 遮挡板：occ_size 沿用 d11 的三元组排布 [x, unused, z]
    occ_size = np.array([occ_side, 0.0, occ_side], dtype=float)
    c = occ_side / 2.0          # 0.0375
    occ_ll_true = np.array([-c, occ_y, -c], dtype=float)

    # 显示器
    screen_size = hidden_extent.copy()      # 待标定，见文首
    screen_res = np.array([1280, 1024], dtype=float)
    # 每显示器块的像素数：d11 是 35。B 档没有对应实测值，
    # 这里按「块物理边长 / 像素物理边长」反推，使亮块物理尺寸与 d11 同量级。
    nppmb = 35.0

    mon_offset = np.zeros(2, dtype=float)   # 待标定，见文首

    pixel_size = screen_size / screen_res
    mon_offset = mon_offset.copy()
    # 与 d11 逐行同构的取整补偿：把屏分辨率的余数折进偏移
    mon_offset[1] = mon_offset[1] + pixel_size[1] * (screen_res[1] % num_blocks[0])
    mon_offset[0] = mon_offset[0] + pixel_size[0] * (screen_res[0] % num_blocks[1])

    illum = pixel_size * nppmb

    return LocSetup(
        num_blocks=num_blocks,
        num_pixels_raw=int(num_pixels_raw),
        D=float(D),
        fov_cord=fov_cord,
        illum_block_size=illum,
        mon_offset=mon_offset,
        occ_size=occ_size,
        occ_ll_true=occ_ll_true,
        occ_ll_est=None,        # B 档没有「论文估计值」这种东西
    )


def b_candidate_grid():
    """B 档候选网格：方案 §5.1 的 x/y ±40 mm（10 mm 步长）、z {-20, 0, +20} mm。

    返回 `(lo, hi, n)` 三元组，与 `build_model.PAPER_GRID` 同结构：
        nx=9, ny=9, nz=3  ->  243 个候选
    注意这里的 y 是「显示器 -> 遮挡板」的距离方向，与 LocSetup 里
    `occ_ll[1]` 用的是同一个量（越靠近显示面 y 越小）。
    """
    nx = ny = 9
    nz = 3
    half_x, half_y = 0.040, 0.040
    half_z = 0.020
    # 候选网格以标称遮挡板位置为中心
    cx = 0.0
    cy = 0.57
    cz = 0.0
    return (dict(lo=(cx - half_x, cy - half_y, cz - half_z),
                 hi=(cx + half_x, cy + half_y, cz + half_z),
                 n=(nx, ny, nz)), 243)


def num_pixels_for(setup: LocSetup, downsamp_factor: int) -> int:
    """`SimulateA_OccluderEstimation.m:36`：numPixels = floor(numPixels / 2^ds)。"""
    return int(np.floor(setup.num_pixels_raw / (2.0 ** downsamp_factor)))


# ======================================================================
# 网格
# ======================================================================
def wall_grid(setup: LocSetup, npx: int) -> Tuple[np.ndarray, np.ndarray]:
    """可见墙的离散坐标（**FOV 端点**口径，长度各 npx）。

    `SimulateForwardModelPerBlock.m:58-59`：
        Wall_xdiscr = linspace(FOV(1,1), FOV(2,1), numPixels);   % 升序
        Wall_zdiscr = linspace(FOV(1,2), FOV(2,2), numPixels);   % 升序
    注意这与我们 `forward.py` 的「单元中心」口径不同。
    """
    wall_x = np.linspace(setup.fov_cord[0, 0], setup.fov_cord[1, 0], npx)
    wall_z = np.linspace(setup.fov_cord[0, 1], setup.fov_cord[1, 1], npx)
    return wall_x, wall_z


def monitor_grid(setup: LocSetup, n_discr_mon: int) -> Tuple[np.ndarray, np.ndarray]:
    """显示器**整屏**离散（`SimulateForwardModelPerBlock.m:80-86`）。

    返回长度分别为 N*ncol（升序）与 N*nrow（**降序**）的两个一维数组。
    """
    nrow, ncol = setup.num_blocks
    N = int(n_discr_mon)
    off_x, off_z = setup.mon_offset
    blk_x, blk_z = setup.illum_block_size
    mon_x = np.linspace(off_x, off_x + ncol * blk_x, N * ncol)
    mon_z = np.linspace(off_z + nrow * blk_z, off_z, N * nrow)     # 降序
    return mon_x, mon_z


def block_monitor_indices(setup: LocSetup, row0: int, col0: int,
                          n_discr_mon: int,
                          x_reversed: bool = True) -> Tuple[np.ndarray, np.ndarray]:
    """块 `(row0, col0)`（1 基）那 N^2 个显示器点在全局网格里的下标（0 基）。

    `SimulateForwardModelPerBlock.m:90-95` 的 `kron` + `fliplr` + 转置 + `(:)`
    逐步推下来（推导见 x4h 报告 §3.2）：

        ix ∈ [(ncol-col0)*N, (ncol-col0+1)*N)      % 注意 col0 越大 ix 越小
        iz ∈ [(row0-1)*N,    row0*N)

    `col0 = 1` 落在 ix 的最大端 —— 这就是源码注释里
    "this not so natural ordering ... due to the file naming convention" 的含义。

    `x_reversed=False` 把这一处反过来（`ix ∈ [(col0-1)*N, col0*N)`），
    **仅用于 X4l 的开关消融**，默认 `True` 才是忠实原文。
    """
    nrow, ncol = setup.num_blocks
    N = int(n_discr_mon)
    if not (1 <= row0 <= nrow and 1 <= col0 <= ncol):
        raise ValueError("block index out of range")
    jj = (ncol - col0) if x_reversed else (col0 - 1)
    ix = np.arange(jj * N, (jj + 1) * N)
    iz = np.arange((row0 - 1) * N, row0 * N)
    return ix, iz


def row_to_wall_index(setup: LocSetup, downsamp_factor: int = 4
                      ) -> Tuple[np.ndarray, np.ndarray]:
    """A 的第 k 行对应墙上的哪个点。返回 0 基的 (ix, iz)。

    `SimulateA_OccluderEstimation.m:63-64` 是 `Im = flipud(Im); simA(:,j) = Im(:)`，
    而 `Im` 行=z、列=x，所以 `k` 按列优先展平后 z 快、x 慢：

        ix = k // npx
        iz = npx - 1 - (k % npx)
    """
    npx = num_pixels_for(setup, downsamp_factor)
    k = np.arange(npx * npx)
    ix = k // npx
    iz = npx - 1 - (k % npx)
    return ix, iz


# ======================================================================
# 前向模型
# ======================================================================
def _occluder_shadow(x_locs: np.ndarray, xx: np.ndarray, D: float,
                     occ_d: float, edges: np.ndarray) -> np.ndarray:
    """`ComputeOccluderShadow`（`SimulateForwardModelPerBlock.m:171-205`）。

    返回 1 = 可见、0 = 被挡，形状 `x_locs.shape + (len(xx),)`。

        X1 = D*(edges(1) - x_locs)/occ_d + x_locs
        X2 = D*(edges(2) - x_locs)/occ_d + x_locs
        Vx = 1;  Vx(xx ∈ [X1, X2]) = 0
    """
    xl = np.asarray(x_locs, dtype=float)[..., None]
    lo = D * (edges[0] - xl) / occ_d + xl
    hi = D * (edges[1] - xl) / occ_d + xl
    xx = np.asarray(xx, dtype=float)
    shadowed = (xx >= lo) & (xx <= hi)
    return (~shadowed).astype(np.float64)


def _stick_geometry(occ_ll: np.ndarray, occ_size_x: float
                    ) -> Tuple[np.ndarray, np.ndarray]:
    """`SimulateForwardModelPerBlock.m:43-45` 的 occstick。

        occstick(:,1) = [-0.0035; +0.0035] + 0.5*(Occluder(1,1)+Occluder(2,1));
        occstick(:,3) = [0; Occluder(1,3)];      % 从地面到遮挡物下沿
        occstick(:,2) = Occluder(2,2);           % 与遮挡物同面
    返回 (x 边界, z 边界)。
    """
    cx = 0.5 * (occ_ll[0] + (occ_ll[0] + occ_size_x))
    return (np.array([cx - _OCC_STICK_HALFWIDTH, cx + _OCC_STICK_HALFWIDTH]),
            np.array([0.0, occ_ll[2]]))


@dataclasses.dataclass
class LocPlan:
    """与遮挡物位置**无关**的那部分准备工作。

    复现 Table S1 要建三百多张 A，逐次重算网格和块→显示器映射既慢又容易出错，
    所以把它们抽出来复用。只有阴影端点、立柱、以及 `occ_y` 随候选位置变。
    """

    setup: LocSetup
    n_discr_mon: int
    downsamp_factor: int
    mz_mirrored: bool
    npx: int
    nrow: int
    ncol: int
    wall_x: np.ndarray          # (npx,)
    wall_z: np.ndarray          # (npx,) 升序
    blk_mx: np.ndarray          # (nrow*ncol, N^2)
    blk_mz: np.ndarray          # (nrow*ncol, N^2)
    WX: np.ndarray              # (npx, npx)  [iz, ix] = wall_x[ix]
    WZ: np.ndarray              # (npx, npx)  [iz, ix] = wall_z[iz]
    z_for_mm: np.ndarray        # (npx,) 视角因子的 z（镜像或不镜像）
    block_chunk: int
    x_reversed: bool = True      # 块→显示器 x 序号是否反向（忠实原文 = True）

    @property
    def n_blocks(self) -> int:
        return self.nrow * self.ncol

    @property
    def n_pixels(self) -> int:
        return self.npx * self.npx


def make_plan(setup: LocSetup, n_discr_mon: int, *, downsamp_factor: int = 4,
              mz_mirrored: bool = True, block_chunk: int = 0,
              x_reversed: bool = True) -> LocPlan:
    nrow, ncol = setup.num_blocks
    N = int(n_discr_mon)
    npx = num_pixels_for(setup, downsamp_factor)
    wall_x, wall_z = wall_grid(setup, npx)
    mon_x, mon_z = monitor_grid(setup, N)

    blk_mx = np.empty((nrow * ncol, N * N), dtype=float)
    blk_mz = np.empty((nrow * ncol, N * N), dtype=float)
    for col0 in range(1, ncol + 1):
        for row0 in range(1, nrow + 1):
            jj = (row0 - 1) + nrow * (col0 - 1)          # 0 基列号
            ix, iz = block_monitor_indices(setup, row0, col0, N,
                                           x_reversed=x_reversed)
            MX, MZ = np.meshgrid(mon_x[ix], mon_z[iz])   # (N_nz, N_nx)
            blk_mx[jj] = MX.ravel()
            blk_mz[jj] = MZ.ravel()

    if block_chunk <= 0:
        # 让每个分块的临时数组大约 8e6 个元素（约 64 MB），
        # 避免 Ndiscr_mon=10 时爆内存，也不至于分块太小而 Python 层开销占主导。
        block_chunk = max(2, int(8.0e6 / max(1, N * N * npx * npx)))

    return LocPlan(
        setup=setup, n_discr_mon=N, downsamp_factor=int(downsamp_factor),
        mz_mirrored=bool(mz_mirrored), npx=npx, nrow=nrow, ncol=ncol,
        wall_x=wall_x, wall_z=wall_z, blk_mx=blk_mx, blk_mz=blk_mz,
        WX=np.tile(wall_x, (npx, 1)),
        WZ=np.tile(wall_z[:, None], (1, npx)),
        z_for_mm=(wall_z[::-1] if mz_mirrored else wall_z).copy(),
        block_chunk=int(block_chunk),
        x_reversed=bool(x_reversed),
    )


def build_A_from_plan(plan: LocPlan, occ_ll, *, view_angle: bool = True,
                      stick: bool = True) -> np.ndarray:
    """用 `make_plan` 的结果建一张 A（`npx^2 × nrow*ncol`）。"""
    setup = plan.setup
    D = setup.D
    npx, nrow, ncol, N = plan.npx, plan.nrow, plan.ncol, plan.n_discr_mon

    occ_ll = np.asarray(occ_ll, dtype=float).reshape(3)
    occ_so = occ_ll + setup.occ_size
    occ_y = float(occ_ll[1])                       # Occluder(1,2)，矩形与立柱共用
    edges_x = np.array([occ_ll[0], occ_so[0]])
    edges_z = np.array([occ_ll[2], occ_so[2]])
    stk_x, stk_z = _stick_geometry(occ_ll, float(setup.occ_size[0]))

    A = np.zeros((npx * npx, plan.n_blocks), dtype=np.float64)
    step = plan.block_chunk

    for start in range(0, plan.n_blocks, step):
        stop = min(start + step, plan.n_blocks)
        mx = plan.blk_mx[start:stop]                  # (nb, N^2)
        mz = plan.blk_mz[start:stop]

        tx = mx[:, :, None, None] - plan.WX[None, None, :, :]   # (nb, N^2, nz, nx)
        tz = mz[:, :, None, None] - plan.WZ[None, None, :, :]
        d2 = tx * tx + tz * tz + D * D
        rad = (D * D) / (d2 * d2 * d2)                # temp1*temp2 = D^2/d^6

        vx = _occluder_shadow(mx, plan.wall_x, D, occ_y, edges_x)   # (nb, N^2, nx)
        vz = _occluder_shadow(mz, plan.wall_z, D, occ_y, edges_z)   # (nb, N^2, nz)

        sh = (vz < 0.5)[..., :, None] & (vx < 0.5)[..., None, :]
        if stick:
            vxs = _occluder_shadow(mx, plan.wall_x, D, occ_y, stk_x)
            vzs = _occluder_shadow(mz, plan.wall_z, D, occ_y, stk_z)
            sh = sh | ((vzs < 0.5)[..., :, None] & (vxs < 0.5)[..., None, :])
        vis = np.where(sh, 0.0, 1.0)

        if view_angle:
            mzfac = np.cos(np.arctan(
                (mz[:, :, None] - plan.z_for_mm[None, None, :]) / D)) ** _MZ_POW
            col = (rad * vis * mzfac[..., :, None]).sum(axis=1)     # (nb, nz, nx)
        else:
            col = (rad * vis).sum(axis=1)

        # Image[iz, ix] = col[iz, ix]；再 flipud + 列优先展平 ⇒ A 的行
        Im = col[:, ::-1, :]                                        # (nb, nz, nx)
        A[:, start:stop] = np.transpose(Im, (0, 2, 1)).reshape(stop - start, -1).T

    return A


def build_A(setup: LocSetup, occ_ll, n_discr_mon: int, *,
            downsamp_factor: int = 4,
            view_angle: bool = True,
            mz_mirrored: bool = True,
            stick: bool = True) -> np.ndarray:
    """便利封装：建计划 + 建 A。要批量建多张时请自己复用 `make_plan`。

    参数
    ----
    occ_ll : (3,) 遮挡物「下左」角，**搜索坐标**（y = 显示器 → 遮挡物距离）
    n_discr_mon : 每块每维的显示器采样点数（table_S1：全局轮 2，x/z 轮 1，y 轮 10）
    view_angle : 是否用视角因子（`simuParams.viewAngleCorrection`）
    mz_mirrored : 是否复刻原代码那处 z 轴镜像（默认 True = 忠实）
    stick : 是否并入立柱（原代码恒为 True，留开关用于消融）
    """
    plan = make_plan(setup, n_discr_mon, downsamp_factor=downsamp_factor,
                     mz_mirrored=mz_mirrored)
    return build_A_from_plan(plan, occ_ll, view_angle=view_angle, stick=stick)


# ======================================================================
# 测量（`load_image1.m` 的移植）
# ======================================================================
def _get_color_image(im: np.ndarray) -> np.ndarray:
    """`load_image1.m:39-47`。Bayer 解马赛克：R=(0,0)、G=(1,0)/(0,1)、B=(1,1)。"""
    red = im[0::2, 0::2]
    green = (im[1::2, 0::2] + im[0::2, 1::2]) / 2.0
    blue = im[1::2, 1::2]
    return np.stack([red, green, blue], axis=-1)


def _downsample_2(im: np.ndarray, iterations: int) -> np.ndarray:
    """`load_image1.m:49-65`：每级取 2x2 四点求均值，共 iterations 级。"""
    for _ in range(int(iterations)):
        im1 = im[0::2, 0::2]
        im2 = im[0::2, 1::2]
        im3 = im[1::2, 0::2]
        im4 = im[1::2, 1::2]
        im = (im1 + im2 + im3 + im4) / 4.0
    return im


def load_measurement(mat_path: str, downsamp_factor: int = 4
                     ) -> Tuple[np.ndarray, np.ndarray]:
    """读一个数据集 `.mat`，复刻 `load_image1`。

    这几份数据集是 **MAT v5/v7**（不是 v7.3），`scipy.io.loadmat` 可直接读。
    返回 (camera_capture, ground_truth)：
      camera_capture : (npx, npx, 3)
      ground_truth   : (nrow, ncol, 3)

    `load_image1` 里那个 `calshut/measure_shut` 比例恒为 1：函数内
    `if ~exist('calshut','var')` 判断的是**局部**变量，而它从未被赋值，
    所以 `calshut = measure_shut`。文件里那个同名 `calshut` 字段与它无关。
    """
    import scipy.io as sio

    d = sio.loadmat(mat_path)
    if "image" not in d:
        raise KeyError("no 'image' field in %s; keys=%s" % (mat_path, sorted(d)))
    imload = np.asarray(d["image"], dtype=np.float64)
    if imload.ndim != 2:
        raise ValueError("unexpected image shape %s" % (imload.shape,))
    gt = d.get("ground_truth", None)
    if gt is not None:
        gt = np.asarray(gt)

    imcol = _get_color_image(imload)
    ds = [_downsample_2(imcol[:, :, c], downsamp_factor) for c in range(3)]
    return np.stack(ds, axis=-1), gt


# ======================================================================
# 定位能量（`occluderposgridsearch` 的内部量）
# ======================================================================
def _top_eigen(G: np.ndarray, k_top: int):
    """G 的**最大** `k_top` 个特征对，按特征值降序返回 (lam_desc, V_desc)。

    只算高端即可：`K` 的判据是 `sqrt(lam)/sqrt(lam_max) > sigma_th`，
    sigma_th 取 0.5 / 0.75 这种「相对最大值」的量级，只涉及大奇异值。
    若这 `k_top` 个**全部**过阈值，调用方必须回退到全谱才能报准 `npass`。
    """
    n = G.shape[0]
    k = int(min(k_top, n))
    if k <= 0:
        return np.zeros(0), np.zeros((n, 0))
    try:
        from scipy.linalg import eigh as _sp_eigh
        lam, V = _sp_eigh(G, subset_by_index=[n - k, n - 1], driver="evr")
    except Exception:
        lam, V = np.linalg.eigh(G)
        lam, V = lam[-k:], V[:, -k:]
    return np.maximum(lam[::-1], 0.0), V[:, ::-1]


_K_TOP = 32


def _spectrum(G: np.ndarray, sigma_th: float):
    """返回 (lam_desc, V_desc)，保证足以判定 `sum(s/s_max > sigma_th)`。"""
    lam, V = _top_eigen(G, _K_TOP)
    if lam.size:
        s = np.sqrt(lam)
        if np.all(s / s[0] > sigma_th):
            lam_f, V_f = np.linalg.eigh(G)
            return np.maximum(lam_f[::-1], 0.0), V_f[:, ::-1]
    return lam, V


def loc_energy_grid(A_of, meas_channels: np.ndarray, grid_x, grid_y, grid_z,
                    sigma_th: float, k_max: int = 10, return_K: bool = True):
    """逐候选算 `|| U(:,1:K)' y_c ||_2` 与 K（论文 `occluderposgridsearch.m:78-90`）。

    用 Gram 等价的写法代替 `svd(simA,'econ')`：
        A = U S V'  ⇒  U' y = S^{-1} V' (A' y)，||U(:,1:K)' y||^2 = Σ (v_k' A'y)^2 / λ_k
    两条路给出同一个 K（s = sqrt(λ)，且阈值只涉及大奇异值，平方条件数不影响它们）。

    A_of(x, y, z) -> (npx^2, nblocks) 的 A 生成器。

    返回 (E, K_pass)：E 形状 `(len(x), len(y), len(z), 3)`，
    K_pass 是**未截断**的 `sum(vecS > sigma_th)`（论文里的 `K_vals`）。
    """
    nx, ny, nz = len(grid_x), len(grid_y), len(grid_z)
    E = np.zeros((nx, ny, nz, 3), dtype=np.float64)
    K_pass = np.zeros((nx, ny, nz), dtype=np.int64)

    ch = [np.asarray(meas_channels[..., c], dtype=np.float64).reshape(-1) for c in range(3)]

    for iix in range(nx):
        for iiy in range(ny):
            for iiz in range(nz):
                A = A_of(grid_x[iix], grid_y[iiy], grid_z[iiz])
                G = A.T @ A
                G = 0.5 * (G + G.T)
                lam, V = _spectrum(G, sigma_th)
                s = np.sqrt(lam)
                vecS = s / s[0]
                npass = int(np.sum(vecS > sigma_th))
                K_pass[iix, iiy, iiz] = npass
                K = min(npass, k_max)
                if K <= 0:
                    continue
                W = V[:, :K]
                inv_s = 1.0 / s[:K]
                for c in range(3):
                    coef = W.T @ (A.T @ ch[c])
                    E[iix, iiy, iiz, c] = np.linalg.norm(coef * inv_s)

    if not return_K:
        return E, K_pass
    return E, K_pass
