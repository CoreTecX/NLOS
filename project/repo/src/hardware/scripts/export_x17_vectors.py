# -*- coding: utf-8 -*-
"""X17 测试向量导出：给 M5 重建核算子层的 C++ 主机测试用。

模块编号口径（以 X1 接口冻结为准）
----------------------------------
    M3 = 算子核，M4 = 定位核，M5 = 重建核。
    W 系列规划那张表（M4=重建核、M5=主机端到端）已停用，见 00_当前进程与任务总表。

导出内容
--------
    x17_params.txt       纯文本参数清单（人可读）
    x17_wall_x.bin       float32[126]   墙网格 x 单元中心
    x17_wall_z.bin       float32[126]   墙网格 z 单元中心
    x17_mon_x.bin        float32[1044]  隐藏场景 x 坐标
    x17_mon_z.bin        float32[1044]  隐藏场景 z 坐标
    x17_rect.bin         int32[4*1044]  真值候选的遮挡矩形（x_lo x_hi z_lo z_hi）
    x17_f_ramp.bin       float32[1044]  重建核的试验图像 f（斜坡 + 鼓包）
    x17_r_rand.bin       float32[15876] 残差向量 r
    x17_q_rand.bin       float32[2*1044] TV 对偶变量 q
    x17_g_in.bin         float32[2*1044] 散度算子的输入场
    x17_ref_grad.bin     float64[2*1044] 参考 gradient(f_ramp)
    x17_ref_div.bin      float64[1044]   参考 divergence(g_in)
    x17_ref_proj.bin     float64[2*1044] 参考 project_l2_ball(q_rand, radius)
    x17_ref_Adf.bin      float64[15876]  参考 A_d @ f_ramp
    x17_ref_AdTr.bin     float64[1044]   参考 A_d^T @ r_rand
    x17_ref_dgrad.bin    float64[1044]   参考 A_d^T (A_d @ f_ramp - r_rand)（数据项梯度）

为什么输入导出为 float32、参考输出导出为 float64
------------------------------------------------
    输入导出 float32 是为了让 C++ 与 Python 用**位完全相同**的输入：脚本把
    浮点数组先降到 float32、再升回 float64 去算参考输出，因此参考值对应的
    正是 C++ 将读到的那组数。输出的参考值保留 float64，是为了让比对反映
    「流式实现 vs 矩阵乘」的**算法差异**，而不是被 float32 舍入掩盖。

    注意 A_d 的比对不能期望 1e-15：差分是近似相等量相减，相对误差按 X8 实测
    放大约 82 倍。X8 用 float32 参考时 A_d 抽查列为 3.22e-06，X17 的判据是
    1e-4，留有余量。

用法
----
    python src/hardware/scripts/export_x17_vectors.py \
        --data-root "D:\\学习\\26秋嵌入式\\论文数据集\\Ordinary-Camera-master\\Data"
"""

from __future__ import annotations

import argparse
import io
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_HW = os.path.dirname(_HERE)
_SRC = os.path.dirname(_HW)
_SRC_MODEL = os.path.join(_SRC, "model")
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import forward as F  # noqa: E402
from penumbrasight_model import localization as L  # noqa: E402
from penumbrasight_model import reconstruction as RC  # noqa: E402


def rect_from_occlusion(occ_col: np.ndarray, n: int):
    """由参考实现的被遮挡集合导出矩形索引（与 export_x8_vectors.py 同一约定）。"""
    idx = np.nonzero(occ_col)[0]
    if idx.size == 0:
        return (1, 0, 1, 0)
    zs = idx // n
    xs = idx % n
    return (int(xs.min()), int(xs.max()), int(zs.min()), int(zs.max()))


def as_f32(a: np.ndarray) -> np.ndarray:
    """降到 float32 并保证 C 连续。"""
    return np.ascontiguousarray(a, dtype=np.float32)


def back(a32: np.ndarray) -> np.ndarray:
    """升回 float64 —— C++ 读到的正是这组值。"""
    return a32.astype(np.float64)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default=os.path.join(_HW, "hwdata"))
    ap.add_argument("--seed", type=int, default=20260925)
    args = ap.parse_args()

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    setup = F.load_setup(cfg)
    n = setup.wall.n
    N = setup.monitor.n_pixels
    shape = tuple(int(v) for v in setup.monitor.shape)
    H, W = shape
    truth = np.asarray(setup.occluder.ll, dtype=np.float64)[:3]

    # ---------------------------------------------------------------- 布局核对
    if H * W != N:
        raise SystemExit("隐藏场景 shape %s 与 n_pixels %d 不符" % (shape, N))

    wx, wz = setup.wall.grid()
    wall_x = as_f32(wx.reshape(n, n)[0, :])
    wall_z = as_f32(wz.reshape(n, n)[:, 0])
    if not np.allclose(wx.reshape(n, n), wall_x.astype(np.float64)[None, :]):
        raise SystemExit("墙 x 坐标不是一维可分的")
    if not np.allclose(wz.reshape(n, n), wall_z.astype(np.float64)[:, None]):
        raise SystemExit("墙 z 坐标不是一维可分的")

    mon_x32, mon_z32 = setup.monitor.flat_grid()
    mon_x32 = as_f32(mon_x32)
    mon_z32 = as_f32(mon_z32)

    # 关键核对：扁平序必须是「行 = z、列 = x」，即 mon_x 只随列变、mon_z 只随行变。
    # 这条错了，recon.hpp 的 H/W 拆分会整体错位，且不会报错。
    MX = mon_x32.astype(np.float64).reshape(H, W)
    MZ = mon_z32.astype(np.float64).reshape(H, W)
    if not np.allclose(MX, MX[0:1, :], rtol=0, atol=1e-9):
        raise SystemExit("mon_x 随行变化，扁平序不是行=z 列=x")
    if not np.allclose(MZ, MZ[:, 0:1], rtol=0, atol=1e-9):
        raise SystemExit("mon_z 随列变化，扁平序不是行=z 列=x")
    print("[info] 布局核对通过：shape=(%d,%d) 行=z 列=x，N=%d" % (H, W, N))

    # ---------------------------------------------------------------- 候选与矩形
    s = setup.with_occluder_ll(truth)
    occ = ~F.visibility(s)
    rects = np.zeros((N, 4), dtype=np.int32)
    n_empty = 0
    for i in range(N):
        rects[i] = rect_from_occlusion(occ[:, i], n)
        if rects[i, 0] > rects[i, 1]:
            n_empty += 1
    print("[info] 真值候选 ll=%s  t=%.6f  空矩形 %d/%d"
          % (np.round(truth, 5).tolist(), truth[1] / setup.wall.y, n_empty, N))

    A = F.build_transport_matrix(s)           # (n*n, N) float64，未差分
    Ad = L.apply_row_difference_A(A, n)       # (n*(n-1), N)
    Md = Ad.shape[0]

    # ---------------------------------------------------------------- 试验输入
    zz = np.arange(H, dtype=np.float64)[:, None] / max(H - 1, 1)
    xx = np.arange(W, dtype=np.float64)[None, :] / max(W - 1, 1)
    f_ramp = 0.5 + 0.40 * zz + 0.35 * xx \
        + 0.25 * np.sin(2.0 * np.pi * xx) * np.cos(2.0 * np.pi * zz)
    f_ramp32 = as_f32(f_ramp.reshape(-1))

    rng = np.random.default_rng(args.seed)
    r_rand32 = as_f32(rng.standard_normal(Md))
    q_rand32 = as_f32(rng.standard_normal((2,) + shape)).reshape(-1)
    g_in32 = as_f32(rng.standard_normal((2,) + shape)).reshape(-1)

    # ---------------------------------------------------------------- 参考输出
    f64 = back(f_ramp32)
    r64 = back(r_rand32)
    q64 = back(q_rand32).reshape((2,) + shape)
    g64 = back(g_in32).reshape((2,) + shape)

    # 投影半径取 q 的 60 分位模长，使被裁与未裁两种分支都被覆盖
    q_norm = np.sqrt(q64[0] ** 2 + q64[1] ** 2)
    radius = float(np.percentile(q_norm, 60.0))
    n_clip = int(np.count_nonzero(q_norm > radius))

    ref_grad = np.ascontiguousarray(RC.gradient(f64.reshape(shape)), dtype=np.float64)
    # divergence 返回 (H, W)，展平后再落盘与做内积
    ref_div = np.ascontiguousarray(RC.divergence(g64).reshape(-1), dtype=np.float64)
    ref_proj = np.ascontiguousarray(RC.project_l2_ball(q64.copy(), radius),
                                    dtype=np.float64)

    Adf = Ad @ f64
    AdTr = Ad.T @ r64
    dgrad = Ad.T @ (Adf - r64)

    for tag, v in (("ref_grad", ref_grad), ("ref_div", ref_div),
                   ("ref_proj", ref_proj), ("ref_Adf", Adf),
                   ("ref_AdTr", AdTr), ("ref_dgrad", dgrad)):
        if not np.all(np.isfinite(v)):
            raise SystemExit("参考量 %s 含非有限值" % tag)

    # 自检：负伴随 <grad f, q> == -<f, div q>
    # 注意两边必须用**同一个** q。首版这里左边用 q_rand、右边用 g_in，是两个不同
    # 的随机场，于是量出一个 31% 的假差异（见 x17_recon_ops.md 第一节）。
    div_q = RC.divergence(q64).reshape(-1)
    lhs = float(ref_grad.reshape(-1) @ q64.reshape(-1))
    rhs = -float(f64 @ div_q)
    scale = max(abs(lhs), abs(rhs), 1e-300)
    print("[info] 负伴随自检 <grad f, q>=%.10e  -<f, div q>=%.10e  相对差 %.3e"
          % (lhs, rhs, abs(lhs - rhs) / scale))

    # ---------------------------------------------------------------- 落盘
    os.makedirs(args.out_dir, exist_ok=True)

    def w(name: str, arr: np.ndarray) -> None:
        p = os.path.join(args.out_dir, name)
        np.ascontiguousarray(arr).tofile(p)
        print("  -> %-22s %-10s %s" % (name, str(arr.dtype), list(arr.shape)))

    w("x17_wall_x.bin", wall_x)
    w("x17_wall_z.bin", wall_z)
    w("x17_mon_x.bin", mon_x32)
    w("x17_mon_z.bin", mon_z32)
    w("x17_rect.bin", rects.reshape(-1))
    w("x17_f_ramp.bin", f_ramp32)
    w("x17_r_rand.bin", r_rand32)
    w("x17_q_rand.bin", q_rand32)
    w("x17_g_in.bin", g_in32)
    w("x17_ref_grad.bin", ref_grad.reshape(-1))
    w("x17_ref_div.bin", ref_div.reshape(-1))
    w("x17_ref_proj.bin", ref_proj.reshape(-1))
    w("x17_ref_Adf.bin", Adf)
    w("x17_ref_AdTr.bin", AdTr)
    w("x17_ref_dgrad.bin", dgrad)

    lines = [
        "# X17 向量参数清单（由 export_x17_vectors.py 生成）",
        "# 模块编号：M3 算子核 / M4 定位核 / M5 重建核（口径以 X1 接口冻结为准）",
        "# 浮点用 %.17g：double 经文本往返必须逐位可逆。首版用 %.12g 写 proj_radius，",
        "# 让 tv_project_l2_ball 量出 2.07e-12 的假差异（阈值 1e-12），见 x17_recon_ops.md。",
        "wall_n            = %d" % n,
        "n_rows            = %d" % H,
        "n_cols            = %d" % W,
        "n_scene           = %d" % N,
        "M_wall            = %d" % (n * n),
        "M_wall_diff       = %d" % Md,
        "wall_y            = %.17g" % setup.wall.y,
        "monitor_y         = %.17g" % setup.monitor.y,
        "wall_x_min        = %.17g" % setup.wall.x_min,
        "wall_x_max        = %.17g" % setup.wall.x_max,
        "wall_z_min        = %.17g" % setup.wall.z_min,
        "wall_z_max        = %.17g" % setup.wall.z_max,
        "occ_size_dx       = %.17g" % setup.occluder.size[0],
        "occ_size_dz       = %.17g" % setup.occluder.size[1],
        "monitor_block_area= %.17g" % setup.monitor.block_area,
        "proj_radius       = %.17g" % radius,
        "n_clipped         = %d" % n_clip,
        "npix              = %d" % q_norm.size,
        "truth_ll          = %.17g %.17g %.17g" % tuple(truth),
        "truth_t           = %.17g" % (truth[1] / setup.wall.y),
        "rect_empty_count  = %d" % n_empty,
        "",
        "# 文件偏移约定",
        "x17_rect.bin      : int32,   索引 = i*4 + {0:x_lo,1:x_hi,2:z_lo,3:z_hi}",
        "x17_q_rand.bin    : float32, 索引 = p*n_scene + i，p=0 为 z 分量、p=1 为 x 分量",
        "x17_g_in.bin      : 同上",
        "x17_ref_grad.bin  : float64, 同上",
        "x17_ref_proj.bin  : float64, 同上",
        "x17_ref_Adf.bin   : float64, 索引 = a*n + b（a 为 z 方向差分索引）",
        "x17_ref_AdTr.bin  : float64, 索引 = i",
        "x17_ref_dgrad.bin : float64, 索引 = i",
    ]
    with io.open(os.path.join(args.out_dir, "x17_params.txt"), "w",
                 encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("  -> x17_params.txt")
    print("[done] 输出目录 %s" % args.out_dir)


if __name__ == "__main__":
    main()
