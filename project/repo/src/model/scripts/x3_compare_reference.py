"""X3 —— MATLAB 参照量 vs 我们 Python 的 A：分段核对与归因

参照量来自 export_ref_for_python.m 的裸二进制 dump（ref_out/raw_chair_va*__*.f64），
它是论文**原始 MATLAB 代码**算出来的，所以是第三方基准，不是我们自己代码的复述。

分五段：
  1  几何输入对照：墙网格、显示器离散、遮挡物角点
  2  A 的规模与形状：先分清「差一个常数倍」与「差形状」
  3  A 逐值：两种墙网格口径（单元中心 / FOV 端点）各自比一遍
  4  列对应关系：MATLAB 的列序与我们的列序是不是同一个
  5  归因：差异里有多少是「半个单元」的约定差，有多少是模型真的不同

用法：
  python x3_compare_reference.py --data-root "D:/学习/.../Ordinary-Camera-master/Data" \
                                 --raw-dir   "D:/学习/.../Ordinary-Camera-master/ref_out"
"""
from __future__ import annotations

import argparse
import dataclasses
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(HERE, ".."))
if _MODEL_DIR not in sys.path:
    sys.path.insert(0, _MODEL_DIR)
try:
    from penumbrasight_model import forward as F
except Exception:  # pragma: no cover
    import forward as F  # type: ignore


# ----------------------------------------------------------------- 读取


def load_raw(raw_dir: str, tag: str, name: str) -> np.ndarray:
    """读裸二进制副本：列优先 float64 + shape 文本。"""
    with open(os.path.join(raw_dir, "raw_%s__%s.shape" % (tag, name))) as fh:
        shape = tuple(int(s) for s in fh.read().split())
    a = np.fromfile(os.path.join(raw_dir, "raw_%s__%s.f64" % (tag, name)), dtype="<f8")
    if a.size != int(np.prod(shape)):
        raise ValueError("%s 大小不符：%d vs %s" % (name, a.size, shape))
    return a.reshape(shape, order="F")


def rel_err(a: np.ndarray, b: np.ndarray) -> float:
    """相对 Frobenius 误差（不是范数之比）。"""
    return float(np.linalg.norm(a - b) / np.linalg.norm(b))


def best_scale(a: np.ndarray, b: np.ndarray) -> tuple:
    """把 a 用单一常数 alpha 拟合到 b：alpha = <a,b>/<a,a>。

    返回 (alpha, 缩放后的相对误差)。用来区分「只差一个常数倍」与「形状不同」。
    """
    num = float(np.sum(a * b))
    den = float(np.sum(a * a))
    alpha = num / den if den > 0 else float("nan")
    return alpha, rel_err(alpha * a, b)


# ----------------------------------------------------------------- 第 1 段


def compare_geometry(setup, ml, out):
    p = lambda *a: print(*a, file=out)
    p("=" * 78)
    p("第 1 段  几何输入对照")
    p("=" * 78)

    # ---- 墙平面
    wall_matr = ml["wall_matr"]                       # (3, 126)
    n = setup.wall.n
    x_ml = wall_matr[0]
    y_ml = wall_matr[1]
    z_ml = wall_matr[2]
    ours_x = setup.wall.x_axis
    ours_z = setup.wall.z_axis

    p("\n[墙平面]")
    p("  n（1D 采样数）              MATLAB %d   我们 %d    %s"
      % (x_ml.size, n, "一致" if x_ml.size == n else "**不一致**"))
    p("  wall y                      MATLAB %.9f   我们 %.9f   差 %.3e"
      % (y_ml[0], setup.wall.y, abs(y_ml[0] - setup.wall.y)))
    p("  MATLAB y 是否为常数         max|y-y0| = %.3e" % float(np.abs(y_ml - y_ml[0]).max()))
    p("  x 端点                      MATLAB [%.9f, %.9f]" % (x_ml[0], x_ml[-1]))
    p("  我们 FOV                    [%.9f, %.9f]"
      % (setup.wall.x_min, setup.wall.x_max))
    p("  我们首/末单元中心           [%.9f, %.9f]" % (ours_x[0], ours_x[-1]))
    p("  x 步长                      MATLAB %.12e   我们 %.12e   差 %.3e"
      % (x_ml[1] - x_ml[0], ours_x[1] - ours_x[0],
         abs((x_ml[1] - x_ml[0]) - (ours_x[1] - ours_x[0]))))
    p("  首点偏移（我们 - MATLAB）   %.9e   （半个单元 = %.9e）"
      % (ours_x[0] - x_ml[0], (setup.wall.x_max - setup.wall.x_min) / n / 2))
    p("  z 端点                      MATLAB [%.9f, %.9f]" % (z_ml[0], z_ml[-1]))
    p("  我们首/末单元中心           [%.9f, %.9f]" % (ours_z[0], ours_z[-1]))
    p("  首点偏移（我们 - MATLAB）   %.9e" % (ours_z[0] - z_ml[0]))

    # ---- 显示器离散
    mx_ml = ml["Mon_xdiscr"].reshape(-1)
    mz_ml = ml["Mon_zdiscr"].reshape(-1)
    p("\n[显示器离散]")
    p("  Mon_Offset                  MATLAB %s   我们 monitor.y = %.9f"
      % (np.array2string(ml["Mon_Offset"].reshape(-1), precision=6), setup.monitor.y))
    p("  x 中心个数                  MATLAB %d   我们 %d" % (mx_ml.size, setup.monitor.x_centers.size))
    p("  z 中心个数                  MATLAB %d   我们 %d" % (mz_ml.size, setup.monitor.z_centers.size))
    p("  x 范围                      MATLAB [%.9f, %.9f]   我们 [%.9f, %.9f]"
      % (mx_ml.min(), mx_ml.max(), setup.monitor.x_centers.min(), setup.monitor.x_centers.max()))
    p("  z 范围                      MATLAB [%.9f, %.9f]   我们 [%.9f, %.9f]"
      % (mz_ml.min(), mz_ml.max(), setup.monitor.z_centers.min(), setup.monitor.z_centers.max()))
    p("  x 是否单调递增              MATLAB %s   我们 %s"
      % (bool(np.all(np.diff(mx_ml) > 0)), bool(np.all(np.diff(setup.monitor.x_centers) > 0))))
    p("  z 是否单调递增              MATLAB %s   我们 %s（我们按图像行序，可能递减）"
      % (bool(np.all(np.diff(mz_ml) > 0)), bool(np.all(np.diff(setup.monitor.z_centers) > 0))))
    if mx_ml.size == setup.monitor.x_centers.size:
        sx = np.sort(mx_ml)
        so = np.sort(setup.monitor.x_centers)
        p("  排序后 x 最大差             %.3e" % float(np.abs(sx - so).max()))
        p("  x 步长                      MATLAB %.12e   我们 %.12e"
          % (sx[1] - sx[0], so[1] - so[0]))
    if mz_ml.size == setup.monitor.z_centers.size:
        sz = np.sort(mz_ml)
        so = np.sort(setup.monitor.z_centers)
        p("  排序后 z 最大差             %.3e" % float(np.abs(sz - so).max()))
    blk = ml["IlluminationBlock_Size"].reshape(-1)
    p("  块尺寸                      MATLAB [%.9f, %.9f]" % (blk[0], blk[1]))
    p("  我们 block_w/h              [%.9f, %.9f]" % (setup.monitor.block_w, setup.monitor.block_h))
    p("  块面积                      MATLAB %.12e   我们 %.12e"
      % (blk[0] * blk[1], setup.monitor.block_area))
    p("  NumBlocks_sim               MATLAB %s   我们 shape %s"
      % (np.array2string(ml["NumBlocks_sim"].reshape(-1).astype(int)),
         setup.monitor.shape))

    # ---- 遮挡物
    occ_corner = ml["occ_corner"]                     # (4, 3, 2)
    p("\n[遮挡物]")
    p("  occ_corner 形状             %s" % (occ_corner.shape,))
    for k in range(occ_corner.shape[2]):
        p("  多边形 %d 的 4 个角点：" % (k + 1))
        for j in range(occ_corner.shape[0]):
            p("      (x,y,z) = (%.9f, %.9f, %.9f)"
              % tuple(occ_corner[j, :, k]))
    p("  Occ_LLcorner                MATLAB %s"
      % np.array2string(ml["Occ_LLcorner"].reshape(-1), precision=9))
    p("  Occ_size                    MATLAB %s"
      % np.array2string(ml["Occ_size"].reshape(-1), precision=9))
    p("  我们 ll                     %s" % np.array2string(setup.occluder.ll, precision=9))
    p("  我们 x_range / z_range      [%.9f, %.9f] / [%.9f, %.9f]"
      % (setup.occluder.x_range[0], setup.occluder.x_range[1],
         setup.occluder.z_range[0], setup.occluder.z_range[1]))
    p("  MATLAB 角点 x 范围          [%.9f, %.9f]"
      % (occ_corner[:, 0, :].min(), occ_corner[:, 0, :].max()))
    p("  MATLAB 角点 z 范围          [%.9f, %.9f]"
      % (occ_corner[:, 2, :].min(), occ_corner[:, 2, :].max()))
    p("  MATLAB 角点 y 范围          [%.9f, %.9f]"
      % (occ_corner[:, 1, :].min(), occ_corner[:, 1, :].max()))


# ----------------------------------------------------------------- 第 3 段


def endpoint_consistent_wall(wall, x_ml, z_ml):
    """构造一个「单元中心恰好落在 MATLAB 端点上」的墙平面。

    我们的网格： x_k = x_min + (k+0.5) * (x_max-x_min)/n
    要让它等于 x0 + k*d，解出
        d = (x_max'-x_min')/n   →  x_max'-x_min' = n*d
        x_min' + d/2 = x0       →  x_min' = x0 - d/2
    这是纯粹的坐标系平移+尺度改标，用来单独检验「网格口径」这一个变量。
    """
    n = wall.n
    dx = (x_ml[-1] - x_ml[0]) / (n - 1)
    dz = (z_ml[-1] - z_ml[0]) / (n - 1)
    return dataclasses.replace(
        wall,
        x_min=x_ml[0] - dx / 2, x_max=x_ml[0] - dx / 2 + n * dx,
        z_min=z_ml[0] - dz / 2, z_max=z_ml[0] - dz / 2 + n * dz,
    )


def compare_A(setup, A_ml, out):
    p = lambda *a: print(*a, file=out)
    p("\n" + "=" * 78)
    p("第 2/3 段  A 的规模与逐值对照")
    p("=" * 78)

    p("\n[A 的基本量]")
    p("  MATLAB simA 形状            %s   （行=墙点，列=隐藏像素）" % (A_ml.shape,))
    p("  MATLAB |A|_F                %.9e" % np.linalg.norm(A_ml))
    p("  MATLAB max|A|               %.9e" % np.abs(A_ml).max())
    p("  MATLAB 非零元比例           %.6f" % float(np.mean(A_ml != 0)))

    # V1：我们的现行口径（单元中心）
    A1 = F.build_transport_matrix(setup)

    p("\n[V1  我们现行口径：墙网格取单元中心]")
    p("  形状                        %s" % (A1.shape,))
    p("  |A|_F                       %.9e" % np.linalg.norm(A1))
    p("  非零元比例                  %.6f" % float(np.mean(A1 != 0)))
    if A1.shape == A_ml.shape:
        alpha, res = best_scale(A1, A_ml)
        p("  直接相对误差 ||dA||_F/||A_ml||_F   %.6e" % rel_err(A1, A_ml))
        p("  最优单一尺度 alpha          %.9e" % alpha)
        p("  除以该尺度后的相对误差      %.6e" % res)
        p("  逐元素相关系数              %.9f"
          % float(np.corrcoef(A1.reshape(-1), A_ml.reshape(-1))[0, 1]))
    return A1


def compare_A_endpoint(setup, ml, A_ml, A1, out):
    p = lambda *a: print(*a, file=out)
    p("\n[V2  只改墙网格口径：单元中心 → MATLAB 端点]")
    wall2 = endpoint_consistent_wall(setup.wall, ml["wall_matr"][0], ml["wall_matr"][2])
    s2 = F.OpticalSetup(wall=wall2, monitor=setup.monitor,
                        occluder=setup.occluder,
                        view_angle_correction=setup.view_angle_correction)
    A2 = F.build_transport_matrix(s2)
    p("  改后墙 x 首点               %.9f   （MATLAB %.9f，差 %.3e）"
      % (wall2.x_axis[0], ml["wall_matr"][0][0], abs(wall2.x_axis[0] - ml["wall_matr"][0][0])))
    p("  改后墙 z 首点               %.9f   （MATLAB %.9f，差 %.3e）"
      % (wall2.z_axis[0], ml["wall_matr"][2][0], abs(wall2.z_axis[0] - ml["wall_matr"][2][0])))
    _report(A2, A_ml, "V2", p)
    p("  V2 对 V1 的改善：相对误差 %.6e → %.6e"
      % (rel_err(A1, A_ml), rel_err(A2, A_ml)))
    return A2


def _report(A, A_ml, tag, p):
    if A.shape != A_ml.shape:
        p("  形状不同 %s vs %s，跳过逐值" % (A.shape, A_ml.shape))
        return
    alpha, res = best_scale(A, A_ml)
    d = A - A_ml
    p("  |A|_F                       %.9e" % np.linalg.norm(A))
    p("  ||dA||_F/||A_ml||_F         %.6e" % rel_err(A, A_ml))
    p("  max|dA|                     %.9e" % np.abs(d).max())
    p("  最优单一尺度 alpha          %.9e" % alpha)
    p("  除以该尺度后相对误差        %.6e" % res)
    p("  逐元素相关系数              %.9f"
      % float(np.corrcoef(A.reshape(-1), A_ml.reshape(-1))[0, 1]))
    p("  非零元集合是否相同          %s"
      % ("是" if np.array_equal(A != 0, A_ml != 0) else
         "否（我们 %d 个非零，MATLAB %d 个，交集 %d）"
         % (int((A != 0).sum()), int((A_ml != 0).sum()),
            int(((A != 0) & (A_ml != 0)).sum()))))


# ----------------------------------------------------------------- 第 4 段


def compare_column_order(A, A_ml, out):
    p = lambda *a: print(*a, file=out)
    p("\n" + "=" * 78)
    p("第 4 段  列对应关系（列序是否一致）")
    p("=" * 78)
    if A.shape != A_ml.shape:
        p("  形状不同，跳过")
        return
    # 归一化列，算 1044x1044 相关矩阵，逐行取最大相关列的序号
    def norm_cols(M):
        M = np.asarray(M, dtype=np.float64)
        nrm = np.linalg.norm(M, axis=0)
        nrm[nrm == 0] = 1.0
        return M / nrm
    An = norm_cols(A)
    Mn = norm_cols(A_ml)
    C = Mn.T @ An                       # (N_ml, N_ours) 每列都归一化 → 即相关系数
    j = np.argmax(C, axis=0)
    c = C[j, np.arange(C.shape[1])]
    ident = int(np.sum(j == np.arange(j.size)))
    p("  每列的最佳匹配列序号与自身相同的个数： %d / %d" % (ident, j.size))
    p("  最佳匹配的相关系数：min %.6f  中位 %.9f  max %.9f"
      % (c.min(), float(np.median(c)), c.max()))
    p("  前 12 个 MATLAB 列各自的最佳匹配： %s" % np.array2string(j[:12]))
    p("  期望的恒等列序：                     %s" % np.array2string(np.arange(12)))
    bad = np.where(c < 0.99)[0]
    p("  相关系数 < 0.99 的列数： %d" % bad.size)
    if bad.size:
        p("  这些列的最佳匹配相关系数： %s" % np.array2string(c[bad][:12], precision=6))


# ----------------------------------------------------------------- 主流程


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--raw-dir", required=True)
    ap.add_argument("--tag", default="chair_va1")
    ap.add_argument("--report", default=None)
    args = ap.parse_args()

    out = open(args.report, "w", encoding="utf-8") if args.report else sys.stdout

    def p(*a):
        print(*a, file=out)

    ml = {}
    for nm in ("wall_matr", "Mon_xdiscr", "Mon_zdiscr", "IlluminationBlock_Size",
               "Mon_Offset", "NumBlocks_sim", "occ_corner", "Occ_LLcorner",
               "Occ_size", "FOV_LLCorner", "FOV_size", "simA"):
        ml[nm] = load_raw(args.raw_dir, args.tag, nm)

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    setup = F.load_setup(cfg)
    p("配置： %s" % cfg)
    p("参照量： %s  tag=%s" % (args.raw_dir, args.tag))

    compare_geometry(setup, ml, out)
    A1 = compare_A(setup, ml["simA"], out)
    A2 = compare_A_endpoint(setup, ml, ml["simA"], A1, out)
    compare_column_order(A1, ml["simA"], out)
    compare_column_order(A2, ml["simA"], out)

    if out is not sys.stdout:
        out.close()
        print("报告已写入 %s" % args.report)


if __name__ == "__main__":
    main()
