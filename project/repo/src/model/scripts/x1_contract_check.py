# -*- coding: utf-8 -*-
"""X1 接口冻结的可执行契约检查。

`01_分析报告/11_算法移植重规划/X1_接口冻结.md` 把 M1~M5 的模块边界与接口签名
写成了文字。本脚本是那份文档的**可执行副本**：任何人改了被冻结的接口，
运行本脚本就会在这里报错，而不是等下游（X8/X9/X12/X17）写完才发现。

检查分四组：

  A 组  配置与位宽（`types.py`）
  B 组  M1 预处理（`preprocess.py`）—— 与 `preprocess.hpp` 逐函数对应
  C 组  M2/M3 几何与算子（`forward.py` / `localization.py`）
  D 组  M4/M5 定位与重建入口（`localization.py` / `reconstruction.py`）

C 组中需要官方数据的部分在未给 `--data-root` 时标 SKIP，不阻塞。

用法
----
    # 只跑不需要数据的 A/B/D 组
    python src/model/scripts/x1_contract_check.py

    # 完整跑（含几何常量核对）
    python src/model/scripts/x1_contract_check.py \
        --data-root "D:\\学习\\26秋嵌入式\\论文数据集\\Ordinary-Camera-master\\Data"
"""

from __future__ import annotations

import argparse
import inspect
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import forward as F  # noqa: E402
from penumbrasight_model import localization as L  # noqa: E402
from penumbrasight_model import preprocess as P  # noqa: E402
from penumbrasight_model import reconstruction as R  # noqa: E402
from penumbrasight_model import types as T  # noqa: E402

PASS, FAIL, SKIP = [], [], []


def check(cond: bool, label: str, detail: str = "") -> bool:
    (PASS if cond else FAIL).append((label, detail))
    print(f"  [{'PASS' if cond else 'FAIL'}] {label}"
          + (f"  —— {detail}" if detail else ""), flush=True)
    return cond


def skip(label: str, why: str) -> None:
    SKIP.append((label, why))
    print(f"  [SKIP] {label}  —— {why}", flush=True)


def has(fn, *names) -> bool:
    """参数名与顺序必须与冻结签名一致（超集允许，顺序须前缀匹配）。"""
    got = list(inspect.signature(fn).parameters)
    return got[:len(names)] == list(names)


# ==================================================================== A 组

def group_a() -> None:
    print("\n=== A 组：配置与位宽（types.py） ===", flush=True)

    fields = [f for f in T.PreprocProfile.__dataclass_fields__]
    check(fields == ["name", "crop", "block", "frames"],
          "PreprocProfile 字段顺序 (name, crop, block, frames)", f"实得 {fields}")
    check(T.PreprocProfile("x", 2016, 8).frames == 1,
          "PreprocProfile.frames 默认 1")

    check(T.PROFILE_OFFICIAL.name == "official"
          and (T.PROFILE_OFFICIAL.crop, T.PROFILE_OFFICIAL.block,
               T.PROFILE_OFFICIAL.frames) == (2016, 8, 1),
          "PROFILE_OFFICIAL = (2016, 8, 1)")
    check(T.PROFILE_B.name == "b"
          and (T.PROFILE_B.crop, T.PROFILE_B.block, T.PROFILE_B.frames) == (1920, 10, 4),
          "PROFILE_B = (1920, 10, 4)")
    check(T.PROFILE_A.name == "a"
          and (T.PROFILE_A.crop, T.PROFILE_A.block, T.PROFILE_A.frames) == (1280, 10, 8),
          "PROFILE_A = (1280, 10, 8)")

    o = T.PROFILE_OFFICIAL
    check((o.plane, o.out_size, o.measurement_shape, o.diff_shape,
           o.diff_measurements)
          == (1008, 126, (126, 126, 3), (125, 126, 3), 15750),
          "官方档派生量 plane=1008 out=126 diff=(125,126) M_d=15750")
    b = T.PROFILE_B
    check((b.plane, b.out_size, b.equivalent_original_block, b.diff_measurements)
          == (960, 96, 20, 9120),
          "B 档派生量 plane=960 out=96 等效原图块=20 M_d=9120")

    for bad in (T.PreprocProfile("x", 2017, 8), T.PreprocProfile("x", 1920, 7),
                T.PreprocProfile("x", 1920, 10, 0)):
        try:
            bad.validate()
            check(False, f"validate() 应拒绝非法档位 {bad}")
        except ValueError:
            pass
    check(True, "validate() 拒绝奇数 crop / 不整除 block / frames<1")

    cf = T.CameraConfig.__dataclass_fields__
    check(list(cf) == ["bit_depth", "full_scale", "bayer_pattern", "raw_shape",
                       "dark_level"],
          "CameraConfig 字段顺序冻结")
    c = T.CameraConfig()
    check((c.bit_depth, c.full_scale, c.bayer_pattern, c.raw_shape, c.dark_level)
          == (12, 65535, "RGGB", (2016, 2016), 0.0),
          "CameraConfig 默认值 (12, 65535, RGGB, 2016, 0.0)")

    fp = T.FixedPointConfig()
    check(fp.raw_bits == 16 and fp.diff_bits == 24 and fp.coeff_bits == 16
          and fp.mac_acc_bits == 48,
          "FixedPointConfig 默认 (raw16, diff24, coeff16, macAcc48)")
    check(fp.block_sum_bits(10) == 16 + 7 == 23,
          "block_sum_bits(10) = 23", f"实得 {fp.block_sum_bits(10)}")
    check(fp.block_sum_bits(8) == 16 + 6 == 22,
          "block_sum_bits(8) = 22", f"实得 {fp.block_sum_bits(8)}")
    check(fp.frame_sum_bits(10, 4) == 23 + 2 == 25,
          "frame_sum_bits(10, 4) = 25", f"实得 {fp.frame_sum_bits(10, 4)}")
    check(fp.frame_sum_bits(8, 16) == 22 + 4 == 26,
          "frame_sum_bits(8, 16) = 26", f"实得 {fp.frame_sum_bits(8, 16)}")


# ==================================================================== B 组

def group_b() -> None:
    print("\n=== B 组：M1 预处理（preprocess.py，对应 preprocess.hpp） ===",
          flush=True)

    check(set(P.BAYER_PATTERNS) == {"RGGB", "BGGR", "GRBG", "GBRG"},
          "BAYER_PATTERNS 支持四种相位")
    check(P.BAYER_PATTERNS["RGGB"]
          == ((0, 0, "R"), (0, 1, "G1"), (1, 0, "G2"), (1, 1, "B")),
          "RGGB 相位定义 (0,0)=R (0,1)=G1 (1,0)=G2 (1,1)=B")
    check(has(P.deinterleave_bayer, "raw", "pattern"),
          "deinterleave_bayer(raw, pattern='RGGB')")

    rng = np.random.default_rng(0)
    raw = rng.integers(0, 4096, size=(16, 16)).astype(np.float64)
    r, g, b = P.deinterleave_bayer(raw, "RGGB")
    check(r.shape == g.shape == b.shape == (8, 8),
          "解交织输出边长 = 输入 / 2", f"实得 {r.shape}")
    check(np.allclose(r, raw[0::2, 0::2]) and np.allclose(b, raw[1::2, 1::2]),
          "解交织取点位置与相位表一致")
    check(np.allclose(g, (raw[0::2, 1::2] + raw[1::2, 0::2]) / 2.0),
          "双绿为 G1 与 G2 的算术平均（规则 R2）")

    # 分块不得跨行。先与「把二维索引展平后连续切块」的错误做法对照——
    # 那正是向量 v4 的 crossrow_wrong（reshape(bh, block, bw, block) 才是对的）。
    ch = np.arange(20 * 20, dtype=np.float64).reshape(20, 20)
    bm = P.block_mean(ch, 10)
    check(bm.shape == (2, 2), "block_mean 输出 (h/block, w/block)")
    correct = ch.reshape(2, 10, 2, 10).mean(axis=(1, 3)).reshape(-1)
    wrong = ch.ravel().reshape(4, 100).mean(axis=1)
    check(np.allclose(bm.reshape(-1), correct),
          "block_mean 等价于 (bh, block, bw, block) 切块后取均值")
    check(not np.allclose(correct, wrong),
          "block_mean 与「展平后连续切块」的跨行做法不同（v4 crossrow 判据）",
          f"正确首块 {correct[0]:.4f} vs 跨行首块 {wrong[0]:.4f}")

    # 行不跨块：只改第 0 行，受影响的块行必须只有第 0 块行
    ch2 = ch.copy()
    ch2[0, :] += 1000.0                      # 只动第 0 行
    affected = np.argwhere(~np.isclose(P.block_mean(ch2, 10), bm))
    check({int(a[0]) for a in affected} == {0},
          "只改第 0 行时受影响的块行只有块行 0（行不跨块）",
          f"受影响块 {affected.tolist()}")
    check(np.allclose(P.block_sum(ch, 10), P.block_mean(ch, 10) * 100.0),
          "block_sum = block_mean x block^2")

    fr = np.stack([np.full((4, 4), v) for v in (1.0, 2.0, 3.0)])
    check(np.allclose(P.accumulate(fr), 6.0),
          "accumulate 为直接求和、不除以帧数（规则 R3）")

    meas = np.zeros((3, 4, 2))
    meas[1] = 1.0
    d = P.vertical_diff(meas)
    check(d.shape == (2, 4, 2) and np.allclose(d[0], 1.0) and np.allclose(d[1], -1.0),
          "vertical_diff 为「下一行减上一行」，形状 (H-1, W, C)")

    check(has(P.preprocess_raw, "raw", "profile", "camera", "dark_level"),
          "preprocess_raw(raw, profile, camera=None, dark_level=0.0)")
    m = P.preprocess_raw(raw * 16, T.PreprocProfile("t", 16, 2), T.CameraConfig())
    check(isinstance(m, P.Measurement) and m.rgb.shape == (4, 4, 3)
          and m.diff.shape == (3, 4, 3) and m.shape == (4, 4, 3),
          "preprocess_raw 返回 Measurement(rgb, diff)，形状与档位派生量一致")

    n1, div = P.normalize(np.full((2, 2, 3), 65535.0 * 4), T.PROFILE_B)
    check(div == 65535.0 * 4 and np.allclose(n1, 1.0),
          "normalize 除数为 full_scale x frames（B 档 = 65535 x 4）",
          f"实得 divisor={div}")


# ==================================================================== C 组

def group_c(data_root: str | None) -> None:
    print("\n=== C 组：M2/M3 几何与算子（forward.py / localization.py） ===",
          flush=True)

    check(has(L.row_difference_1d, "n"), "row_difference_1d(n)")
    check(has(L.apply_row_difference_img, "y2d"), "apply_row_difference_img(y2d)")
    check(has(L.apply_row_difference_A, "A", "n"), "apply_row_difference_A(A, n)")

    D = L.row_difference_1d(5)
    check(D.shape == (4, 5) and D[0, 0] == -1.0 and D[0, 1] == 1.0,
          "row_difference_1d 为「下一个减当前」，主对角 -1、上对角 +1")

    n = 4
    rng = np.random.default_rng(1)
    y2d = rng.normal(size=(n, n))
    A = rng.normal(size=(n * n, 7))
    dy = L.apply_row_difference_img(y2d)
    dA = L.apply_row_difference_A(A, n)
    check(dy.shape == (n * (n - 1),) and dA.shape == (n * (n - 1), 7),
          "差分后形状 (n*(n-1),) 与 (n*(n-1), N)")
    # 关键一致式：对 A 逐行做同样差分，结果必须等于 D 作用在「按 n x n 展开的 A」上
    check(np.allclose(dA, (A.reshape(n, n, -1)[1:] - A.reshape(n, n, -1)[:-1]).reshape(-1, 7)),
          "A_d 与 y_d 采用同一行结构（差分口径一致，冻结）")

    check(has(L.evaluate_candidate, "setup", "y2d", "ll", "differenced", "kappa",
                "mu_mode", "mu_k"),
          "evaluate_candidate(setup, y2d, ll, differenced=True, kappa=0.75, ...)")
    ec = inspect.signature(L.evaluate_candidate).parameters
    check(ec["differenced"].default is True,
          "differenced 默认 True（T0-04 结论：定位必须在差分框架下）")
    check(ec["kappa"].default == 0.75, "kappa 默认值为 0.75",
          f"实得 {ec['kappa'].default}")

    er_fields = list(L.EnergyResult.__dataclass_fields__)
    check(er_fields == ["ll", "energy", "n_kept", "s1"],
          "EnergyResult 字段顺序 (ll, energy, n_kept, s1)", f"实得 {er_fields}")

    sr_fields = list(L.SearchRound.__dataclass_fields__)
    check(sr_fields[:6] == ["index", "kappa", "x_values", "y_values", "z_values",
                            "energies"],
          "SearchRound 字段顺序冻结（energies 为 (nx, ny, nz)）")

    check(has(L.load_search_grids, "config_path"), "load_search_grids(config_path)")
    check(has(L.coarse_to_fine, "setup", "y2d", "grids", "kappas", "max_rounds",
                "progress"),
          "coarse_to_fine(setup, y2d, grids, kappas, max_rounds, progress)")
    check(inspect.signature(L.coarse_to_fine).parameters["kappas"].default
          == (0.75, 0.5, 0.05, 0.05),
          "四轮截断阈值默认 (0.75, 0.5, 0.05, 0.05)")

    check(has(L.projection_energy_detail, "A", "y", "kappa"),
          "projection_energy_detail(A, y, kappa=0.75)")

    if not data_root:
        skip("几何常量核对（t / 平面 y / 遮挡矩形 / 墙网格）", "未给 --data-root")
        skip("build_transport_matrix 形状 15876 x 1044", "未给 --data-root")
        skip("墙网格展平约定（行索引=z、列索引=x）", "未给 --data-root")
        return

    cfg = os.path.join(data_root, "TestPosChair", "chair_test.mat")
    if not os.path.isfile(cfg):
        skip("几何常量核对", f"找不到 {cfg}")
        return

    setup = F.load_setup(cfg)
    t = setup.occluder.y / setup.wall.y
    check(abs(t - 0.576699) < 1e-5, "交点参数 t = y_occ / y_wall = 0.576699",
          f"实得 {t:.6f}")
    check(abs(setup.wall.y - 1.030) < 1e-6 and abs(setup.occluder.y - 0.594) < 1e-6
          and abs(setup.monitor.y - 0.0) < 1e-6,
          "三个平面 y 值（墙 1.030 / 遮挡物 0.594 / 显示器 0.000）",
          f"实得 wall={setup.wall.y} occ={setup.occluder.y} mon={setup.monitor.y}")
    check(setup.wall.n == 126 and setup.wall.n_points == 15876,
          "墙网格 126x126 = 15876 个测量点",
          f"实得 n={setup.wall.n} n_points={setup.wall.n_points}")
    xr, zr = setup.occluder.x_range, setup.occluder.z_range
    check(abs(xr[0] - 0.506) < 1e-6 and abs(xr[1] - 0.581) < 1e-6
          and abs(zr[0] - 0.242) < 1e-6 and abs(zr[1] - 0.317) < 1e-6,
          "遮挡物矩形 x[0.506,0.581] z[0.242,0.317]",
          f"实得 x={xr} z={zr}")

    # 展平约定：A 的第 j 行 = 墙点 j，j = z * n + x。
    # 实测：每列内 x 不变（x 只随列索引变）、每行内 z 不变（z 只随行索引变）。
    wx, wz = setup.wall.grid()
    WX, WZ = wx.reshape(setup.wall.n, setup.wall.n), wz.reshape(setup.wall.n, setup.wall.n)
    col_const_x = bool(np.allclose(WX, WX[:1, :]))
    row_const_z = bool(np.allclose(WZ, WZ[:, :1]))
    check(col_const_x and row_const_z,
          "墙网格展平：行索引 = z、列索引 = x（> row-major, j = z*n + x）",
          f"每列内 x 不变={col_const_x} 每行内 z 不变={row_const_z}")
    # 网格点是「单元中心」：首点 = FOV 下界 + 半个单元，而不是下界本身。
    # 配置里存的是 FOV 边界，S0-1 速查表里写的是中心值，两者差半个单元，不要混用。
    cell = (setup.wall.x_max - setup.wall.x_min) / setup.wall.n
    check(abs(wx[0] - (setup.wall.x_min + cell / 2)) < 1e-12
          and abs(wz[0] - (setup.wall.z_min + cell / 2)) < 1e-12,
          "墙网格取「单元中心」：首点 = FOV 下界 + 半单元",
          f"x 下界 {setup.wall.x_min} → 首点 {wx[0]:.5f}，单元 {cell:.7f}")
    check(abs(wx[-1] - 0.94241) < 1e-5 and abs(wz[-1] - 0.46341) < 1e-5,
          "中心值范围与 S0-1 速查表一致（x 0.5446~0.9424、z 0.0656~0.4634）",
          f"实得末点 x={wx[-1]:.5f} z={wz[-1]:.5f}")

    ll = np.asarray(setup.occluder.ll, dtype=np.float64)[:3]
    A = F.build_transport_matrix(setup)
    check(A.shape == (15876, 1044),
          "M2 输出 A 形状 (M, N) = (15876, 1044)", f"实得 {A.shape}")
    Ad = L.apply_row_difference_A(A, setup.wall.n)
    check(Ad.shape == (15750, 1044),
          "M3 输出 A_d 形状 (15750, 1044)", f"实得 {Ad.shape}")
    AtA = Ad.T @ Ad
    check(AtA.shape == (1044, 1044),
          "M3 输出 A_d^T A_d 形状 (1044, 1044)", f"实得 {AtA.shape}")
    check(np.allclose(AtA, AtA.T, rtol=1e-10, atol=1e-6),
          "A_d^T A_d 对称（X16 已用此性质把存储折到上三角 545,490 个权重）")
    # ll 的顺序：分量 1 必须是深度（等于遮挡物平面 y），分量 0/2 是矩形 x/z 下界。
    # 注意 ll 是**参考角**（矩形下界 + 平面 y），不是矩形中心，也不是 best_pos。
    check(abs(ll[1] - setup.occluder.y) < 1e-12
          and abs(ll[0] - xr[0]) < 1e-12 and abs(ll[2] - zr[0]) < 1e-12,
          "遮挡物参考角 ll = (x, y深度, z)：(x=矩形 x 下界, y=平面 y, z=矩形 z 下界)",
          f"实得 {np.round(ll, 5).tolist()}；期望 "
          f"[{xr[0]}, {setup.occluder.y}, {zr[0]}]")


# ==================================================================== D 组

def group_d() -> None:
    print("\n=== D 组：M4/M5 入口（localization.py / reconstruction.py） ===",
          flush=True)

    check(has(L.search_grid, "setup", "y2d", "x_values", "y_values", "z_values",
              "kappa", "differenced", "index", "progress"),
          "search_grid(setup, y2d, x_values, y_values, z_values, ...) "
          "—— 轴序 (x, y深度, z)")

    f2 = np.zeros((5, 6))
    f2[2, 3] = 1.0
    g = R.gradient(f2)
    check(g.shape == (2, 5, 6), "gradient 返回 (2, H, W)", f"实得 {g.shape}")
    div = R.divergence(g)
    check(div.shape == (5, 6), "divergence 返回 (H, W)")

    # <grad x, q> == -<x, div q>：伴随关系（Neumann 边界）
    rng = np.random.default_rng(2)
    x = rng.normal(size=(5, 6))
    q = rng.normal(size=(2, 5, 6))
    lhs = float(np.sum(R.gradient(x) * q))
    rhs = -float(np.sum(x * R.divergence(q)))
    check(abs(lhs - rhs) < 1e-9 * max(1.0, abs(lhs)),
          "divergence 是 gradient 的负伴随 <Gx,q> = -<x,Dq>",
          f"lhs={lhs:.6f} rhs={rhs:.6f}")

    check(has(R.operator_norm_sq, "A", "shape", "n_iter", "seed"),
          "operator_norm_sq(A, shape, n_iter=25, seed=0)")
    check(has(R.tv_pdhg, "A", "b", "shape", "lam", "n_iter", "nonneg", "L2",
              "record_every"),
          "tv_pdhg(A, b, shape, lam=0.01, n_iter=300, nonneg=True, L2=None, ...)")
    check(has(R.reconstruct_channel, "setup", "y2d", "ll", "lam", "n_iter",
              "mu_mode", "mu_k", "differenced", "A"),
          "reconstruct_channel(setup, y2d, ll, lam=0.01, n_iter=300, ...)")
    check(has(R.scale_matched, "a", "ref") and has(R.pearson, "a", "b"),
          "scale_matched(a, ref) / pearson(a, b)")

    print("  [note] M5 的**内部求解器**尚未冻结（X15 待 X4 修正 A 后重跑）：",
          flush=True)
    print("         X15 定案前不得据此写 HLS；本次只冻结 M5 的对外边界。",
          flush=True)


# ==================================================================== 主流程

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default=None)
    args = ap.parse_args()

    print("X1 接口冻结契约检查（可执行副本）")
    print(f"  Python {sys.version.split()[0]} / numpy {np.__version__}", flush=True)

    group_a()
    group_b()
    group_c(args.data_root)
    group_d()

    print("\n" + "=" * 62, flush=True)
    print(f"PASS {len(PASS)}   FAIL {len(FAIL)}   SKIP {len(SKIP)}", flush=True)
    if FAIL:
        print("\n失败项：", flush=True)
        for label, detail in FAIL:
            print(f"  - {label}  {detail}", flush=True)
        print("\n接口已与 X1_接口冻结.md 不一致。改接口前请先更新那份文档；",
              flush=True)
        print("若是文档先行变更，请同步更新本脚本并说明变更理由。", flush=True)
        sys.exit(1)
    print("接口与 X1_接口冻结.md 一致。", flush=True)


if __name__ == "__main__":
    main()
