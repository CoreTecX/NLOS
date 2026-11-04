# -*- coding: utf-8 -*-
"""X8 测试向量导出：给 M2 几何算核与 M3 算子核的 C++ 主机测试用。

导出内容
--------
    x8_params.txt        纯文本参数清单（人可读，便于核对）
    x8_wall_x.bin        float32[126]   墙网格 x 单元中心（一维，只随 x 索引变）
    x8_wall_z.bin        float32[126]   墙网格 z 单元中心（一维，只随 z 索引变）
    x8_mon_x.bin         float32[1044]  隐藏场景 x 坐标
    x8_mon_z.bin         float32[1044]  隐藏场景 z 坐标
    x8_cand_ll.bin       float32[3*K]   K 个候选位置（每 3 个 = x, y深度, z）
    x8_probe_idx.bin     int32[P]       被抽查的隐藏点编号
    x8_ref_rect.bin      int32[4*K*N]   参考遮挡矩形（逐候选逐隐藏点，顺序 x_lo x_hi z_lo z_hi）
    x8_ref_A.bin         float32[K*P*M] 参考 A 的抽查列（K 个候选 x P 个隐藏点）
    x8_ref_Ad.bin        float32[K*P*Md] 参考 A_d 的抽查列

坐标约定（与 X1 接口冻结一致）
------------------------------
    墙点展平 j = z * n + x（行索引 = z、列索引 = x）
    隐藏点坐标 mx, mz 与墙点 wx 的 x 配对、wz 的 z 配对
    ll = (x 参考角, y 平面深度, z 参考角)，t = ll[1] / wall_y

为什么只抽查 A / A_d 的列
-------------------------
    A 是 15876 x 1044，单个候选就是 66 MB。全量导出既慢又占盘，而矩形这一项
    已经做了**全量**比对（K 个候选 x 全部 1044 个隐藏点）。所以矩阵只抽查列，
    用于验证权重与差分方向；矩形用于验证索引。

用法
----
    python src/hardware/scripts/export_x8_vectors.py \
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
from penumbrasight_model import io_mat  # noqa: E402
from penumbrasight_model import localization as L  # noqa: E402


def rect_from_occlusion(occ_col: np.ndarray, n: int):
    """由参考实现的被遮挡集合导出矩形索引。

    已验证该集合必为「行区间 x 列区间」的直积（见 x8a_rect_at_varying_t）。
    空集返回 (1, 0, 1, 0)，使 x_lo > x_hi 与 z_lo > z_hi 同时成立。
    """
    idx = np.nonzero(occ_col)[0]
    if idx.size == 0:
        return (1, 0, 1, 0)
    zs = idx // n
    xs = idx % n
    return (int(xs.min()), int(xs.max()), int(zs.min()), int(zs.max()))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default=os.path.join(_HW, "hwdata"))
    ap.add_argument("--n-probe", type=int, default=8)
    args = ap.parse_args()

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    setup = F.load_setup(cfg)
    n = setup.wall.n
    N = setup.monitor.n_pixels
    truth = np.asarray(setup.occluder.ll, dtype=np.float64)[:3]

    # 一维墙坐标：wx 只随 x 索引变，wz 只随 z 索引变（X1 已用契约脚本验过）
    wx, wz = setup.wall.grid()
    WX = wx.reshape(n, n)
    WZ = wz.reshape(n, n)
    wall_x = np.ascontiguousarray(WX[0, :], dtype=np.float32)
    wall_z = np.ascontiguousarray(WZ[:, 0], dtype=np.float32)
    assert np.allclose(WX, wall_x[None, :]), "墙 x 坐标不是一维可分的"
    assert np.allclose(WZ, wall_z[:, None]), "墙 z 坐标不是一维可分的"

    mon_x, mon_z = setup.monitor.flat_grid()
    mon_x = np.ascontiguousarray(mon_x, dtype=np.float32)
    mon_z = np.ascontiguousarray(mon_z, dtype=np.float32)

    # 候选：真值，以及搜索 y 的两端（矩形最大 / 最小）
    y_vals = np.asarray(io_mat.load_mat(cfg)["yhatvals"],
                        dtype=np.float64).reshape(-1)
    cands = []
    for y in (float(truth[1]), float(y_vals.max()), float(y_vals.min())):
        ll = truth.copy()
        ll[1] = y
        cands.append(ll)
    cands = np.asarray(cands, dtype=np.float64)
    K = cands.shape[0]

    # 抽查的隐藏点编号：在 1044 上均匀取，避开首尾
    probe = np.linspace(0, N - 1, args.n_probe).astype(np.int64)

    rects = np.zeros((K, N, 4), dtype=np.int32)
    ref_A = np.zeros((K, args.n_probe, n * n), dtype=np.float32)
    ref_Ad = np.zeros((K, args.n_probe, (n - 1) * n), dtype=np.float32)

    for k in range(K):
        s = setup.with_occluder_ll(cands[k])
        occ = ~F.visibility(s)                       # True = 被遮挡
        for i in range(N):
            rects[k, i] = rect_from_occlusion(occ[:, i], n)
        A = F.build_transport_matrix(s)              # (15876, 1044) float64
        Ad = L.apply_row_difference_A(A, n)
        for p, i in enumerate(probe):
            ref_A[k, p] = A[:, i].astype(np.float32)
            ref_Ad[k, p] = Ad[:, i].astype(np.float32)
        print("[info] 候选 %d/%d ll=%s  t=%.6f  A 非零占比 %.2f%%"
              % (k + 1, K, np.round(cands[k], 5).tolist(),
                 cands[k][1] / setup.wall.y,
                 100.0 * np.count_nonzero(A) / A.size))
        sys.stdout.flush()

    os.makedirs(args.out_dir, exist_ok=True)

    def w(name: str, arr: np.ndarray) -> None:
        p = os.path.join(args.out_dir, name)
        arr.tofile(p)
        print("  -> %-22s %-10s %s" % (name, str(arr.dtype), list(arr.shape)))

    w("x8_wall_x.bin", wall_x)
    w("x8_wall_z.bin", wall_z)
    w("x8_mon_x.bin", mon_x)
    w("x8_mon_z.bin", mon_z)
    w("x8_cand_ll.bin", np.ascontiguousarray(cands, dtype=np.float32))
    w("x8_probe_idx.bin", probe.astype(np.int32))
    w("x8_ref_rect.bin", rects.reshape(-1))
    w("x8_ref_A.bin", ref_A.reshape(-1))
    w("x8_ref_Ad.bin", ref_Ad.reshape(-1))

    lines = [
        "# X8 向量参数清单（由 export_x8_vectors.py 生成）",
        "wall_n            = %d" % n,
        "n_scene           = %d" % N,
        "n_cand            = %d" % K,
        "n_probe           = %d" % args.n_probe,
        "wall_y            = %.10g" % setup.wall.y,
        "monitor_y         = %.10g" % setup.monitor.y,
        "wall_x_min        = %.10g" % setup.wall.x_min,
        "wall_x_max        = %.10g" % setup.wall.x_max,
        "wall_z_min        = %.10g" % setup.wall.z_min,
        "wall_z_max        = %.10g" % setup.wall.z_max,
        "cell_x            = %.12g" % ((setup.wall.x_max - setup.wall.x_min) / n),
        "cell_z            = %.12g" % ((setup.wall.z_max - setup.wall.z_min) / n),
        "wall_x_first_ctr  = %.10g" % wall_x[0],
        "wall_z_first_ctr  = %.10g" % wall_z[0],
        "occ_size_dx       = %.10g" % setup.occluder.size[0],
        "occ_size_dz       = %.10g" % setup.occluder.size[1],
        "monitor_block_area= %.12g" % setup.monitor.block_area,
        "mu_mode           = none",
        "M_wall            = %d" % (n * n),
        "M_wall_diff       = %d" % ((n - 1) * n),
        "truth_ll          = %.10g %.10g %.10g" % tuple(truth),
    ]
    for k in range(K):
        lines.append("cand[%d]_ll        = %.10g %.10g %.10g   t=%.10g"
                     % (k, cands[k][0], cands[k][1], cands[k][2],
                        cands[k][1] / setup.wall.y))
    lines.append("probe_idx         = " + " ".join(str(int(v)) for v in probe))
    lines.append("")
    lines.append("# 文件偏移约定（读 bin 时按此计算）")
    lines.append("x8_ref_rect.bin   : int32,   索引 = ((k*n_scene) + i)*4 + {0:x_lo,1:x_hi,2:z_lo,3:z_hi}")
    lines.append("x8_ref_A.bin      : float32, 索引 = ((k*n_probe) + p)*M_wall + j")
    lines.append("x8_ref_Ad.bin     : float32, 索引 = ((k*n_probe) + p)*M_wall_diff + j")

    with io.open(os.path.join(args.out_dir, "x8_params.txt"), "w",
                 encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("  -> x8_params.txt")
    print("[done] 输出目录 %s" % args.out_dir)


if __name__ == "__main__":
    main()
