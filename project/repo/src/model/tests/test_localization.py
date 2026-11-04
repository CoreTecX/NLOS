# -*- coding: utf-8 -*-
"""定位链单元测试（对应方案 §12 Task 1/3）。

覆盖 `penumbrasight_model.forward_loc` 与 `penumbrasight_model.psm` 里**定位链**
用得上的几何映射与候选网格契约：

  · B 档候选网格（9x9x3 = 243）的中心、步长、半幅；
  · B 档 LocSetup 的冻结量（墙网格 96、块数 18x24、FOV 0.44、遮挡板 75 mm）；
  · 墙网格 / 显示器整屏离散（含 z 降序）与「块 -> 显示器下标」的 x 反向；
  · A 的行号 -> 墙上 (ix, iz) 的映射（列优先 + flipud）；
  · geometry_hash 的确定性、敏感性，以及它与 tests/vectors 冻结 fixture 的一致。

这些是不需要官方数据集、也不需要跑求解器就能判的契约；真正的定位精度（P1）
不在本文件范围（判据待拍板，见 report/design_report 第五章）。

可直接运行（不依赖 pytest）：
    python src/model/tests/test_localization.py
"""

from __future__ import annotations

import copy
import json
import os
import sys
import traceback

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
_ROOT = os.path.abspath(os.path.join(_SRC_MODEL, "..", ".."))
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import forward_loc as FL          # noqa: E402
from penumbrasight_model import psm                        # noqa: E402

_GEO_B = os.path.join(_ROOT, "configs", "geometry_b.yaml")
_VEC_MANIFEST = os.path.join(_ROOT, "tests", "vectors",
                             "model_b_r8_c3.manifest.json")


# ---------------------------------------------------------------- 候选网格


def test_b_candidate_grid_shape_and_spacing():
    grid, total = FL.b_candidate_grid()
    lo = np.asarray(grid["lo"], dtype=float)
    hi = np.asarray(grid["hi"], dtype=float)
    n = tuple(grid["n"])
    assert total == 243 and n == (9, 9, 3), (total, n)
    assert int(np.prod(n)) == total
    # 步长 = 半幅 / (n-1)
    step = (hi - lo) / (np.asarray(n, dtype=float) - 1.0)
    assert np.allclose(step, [0.010, 0.010, 0.020], atol=1e-12), step
    # 中心 = (lo+hi)/2 = 标称遮挡中心 (0, 0.57, 0)
    center = 0.5 * (lo + hi)
    assert np.allclose(center, [0.0, 0.57, 0.0], atol=1e-12), center


def test_b_candidate_grid_enumerates_both_ends():
    grid, total = FL.b_candidate_grid()
    lo = np.asarray(grid["lo"], dtype=float)
    hi = np.asarray(grid["hi"], dtype=float)
    n = np.asarray(grid["n"], dtype=int)
    axes = [np.linspace(lo[i], hi[i], n[i]) for i in range(3)]
    assert axes[0][0] == lo[0] and axes[0][-1] == hi[0]
    assert axes[2][0] == lo[2] and axes[2][-1] == hi[2]
    # 升序、无重复
    for a in axes:
        assert np.all(np.diff(a) > 0.0), a
    assert axes[0].size * axes[1].size * axes[2].size == total


# ---------------------------------------------------------------- LocSetup


def test_b_loc_setup_frozen_values():
    s = FL.b_loc_setup()
    assert tuple(s.num_blocks) == (18, 24), s.num_blocks
    assert s.num_pixels_raw == 96
    assert abs(s.D - 1.03) < 1e-12, s.D
    # FOV 0.44 见方、以显示中心为原点
    assert np.allclose(s.fov_cord, [[-0.22, -0.22], [0.22, 0.22]], atol=1e-12)
    # 遮挡板 75 mm 见方（occ_size 为 [x, unused, z] 三元组）
    assert abs(s.occ_size[0] - 0.075) < 1e-12 and abs(s.occ_size[2] - 0.075) < 1e-12
    # occ_ll_true = 显示中心 - 半边长
    assert np.allclose(s.occ_ll_true, [-0.0375, 0.57, -0.0375], atol=1e-12)
    # mon_offset 为「待标定」量：只要求形状正确、有限，不锁数值
    assert s.mon_offset.shape == (2,) and np.all(np.isfinite(s.mon_offset))


def test_num_pixels_for_b_uses_blocked_side():
    s = FL.b_loc_setup()
    # B 档 num_pixels_raw 取分块后的边长 96，ds=0 得 96x96
    assert FL.num_pixels_for(s, 0) == 96
    assert FL.num_pixels_for(s, 1) == 48
    assert FL.num_pixels_for(s, 4) == 6


# ---------------------------------------------------------------- 网格映射


def test_wall_grid_endpoints_and_ascending():
    s = FL.b_loc_setup()
    wx, wz = FL.wall_grid(s, 96)
    assert wx.size == 96 and wz.size == 96
    assert wx[0] == s.fov_cord[0, 0] and wx[-1] == s.fov_cord[1, 0]
    assert wz[0] == s.fov_cord[0, 1] and wz[-1] == s.fov_cord[1, 1]
    assert np.all(np.diff(wx) > 0.0) and np.all(np.diff(wz) > 0.0)


def test_monitor_grid_x_ascending_z_descending():
    s = FL.b_loc_setup()
    N = 2
    mx, mz = FL.monitor_grid(s, N)
    nrow, ncol = s.num_blocks
    assert mx.size == N * ncol and mz.size == N * nrow
    assert np.all(np.diff(mx) > 0.0), "mon_x 应升序"
    assert np.all(np.diff(mz) < 0.0), "mon_z 应降序（SimulateForwardModelPerBlock 口径）"


def test_block_monitor_indices_x_reversed():
    s = FL.b_loc_setup()
    N = 2
    nrow, ncol = s.num_blocks
    # col0 = ncol 落在 ix 的最小端（x_reversed=True，忠实原文）
    ix_right, iz_right = FL.block_monitor_indices(s, 1, ncol, N, x_reversed=True)
    ix_left, _ = FL.block_monitor_indices(s, 1, 1, N, x_reversed=True)
    assert ix_right.min() == 0
    assert ix_left.max() == ncol * N - 1
    # iz 与 col0 无关，只随 row0
    assert list(iz_right) == list(range(0, N))
    # 关掉反向时 col0=1 落在最小端
    ix_left2, _ = FL.block_monitor_indices(s, 1, 1, N, x_reversed=False)
    assert ix_left2.min() == 0
    # 越界应报错
    for bad in ((0, 1), (1, 0), (nrow + 1, 1), (1, ncol + 1)):
        try:
            FL.block_monitor_indices(s, bad[0], bad[1], N)
        except ValueError:
            continue
        raise AssertionError("块下标越界未被拦下：%s" % (bad,))


def test_row_to_wall_index_bijection():
    s = FL.b_loc_setup()
    npx = FL.num_pixels_for(s, 4)
    ix, iz = FL.row_to_wall_index(s, downsamp_factor=4)
    assert ix.size == npx * npx and iz.size == npx * npx
    # 公式：ix = k // npx，iz = npx - 1 - (k % npx)
    k = np.arange(npx * npx)
    assert np.array_equal(ix, k // npx)
    assert np.array_equal(iz, npx - 1 - (k % npx))
    # (ix, iz) 覆盖整张 npx x npx 网格，且不重复
    pairs = set(zip(ix.tolist(), iz.tolist()))
    assert len(pairs) == npx * npx
    assert int(ix.min()) == 0 and int(ix.max()) == npx - 1
    assert int(iz.min()) == 0 and int(iz.max()) == npx - 1


# ---------------------------------------------------------------- 几何哈希


def _load_geo_b():
    try:
        import yaml                                        # 可选依赖
    except ImportError:
        return None
    if not os.path.isfile(_GEO_B):
        return None
    with open(_GEO_B, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_geometry_hash_deterministic_and_sensitive():
    geom = {
        "planes": {"wall": 1.03},
        "wall": {"n_points": 96},
        "hidden": {"rows": 18, "cols": 24},
        "occluder": {"size": [0.075, 0.075],
                     "nominal_center": {"xyz": [0.0, 0.57, 0.0]}},
        "forward_model": {"view_factor_z_pairing": "direct", "stick": True},
    }
    h0 = psm.build_geometry_hash(geom)
    assert h0 == psm.build_geometry_hash(copy.deepcopy(geom)), "同输入必须同哈希"

    g1 = copy.deepcopy(geom)
    g1["wall"]["n_points"] = 95
    assert psm.build_geometry_hash(g1) != h0, "墙网格变化未换哈希"

    g2 = copy.deepcopy(geom)
    g2["occluder"]["nominal_center"]["xyz"] = [0.01, 0.57, 0.0]
    assert psm.build_geometry_hash(g2) != h0, "遮挡中心变化未换哈希"

    g3 = copy.deepcopy(geom)
    g3["forward_model"]["view_factor_z_pairing"] = "mirrored"
    assert psm.build_geometry_hash(g3) != h0, "视角因子口径变化未换哈希"


def test_geometry_hash_matches_frozen_b_fixture():
    """configs/geometry_b.yaml 与 tests/vectors 的 fixture 必须是同一套冻结几何。"""
    geom = _load_geo_b()
    if geom is None:
        print("      (无 PyYAML 或 geometry_b.yaml，跳过与 fixture 的比对)")
        return
    with open(_VEC_MANIFEST, "r", encoding="utf-8") as f:
        man = json.load(f)
    expect = int(man["geometry_hash"])
    got = psm.build_geometry_hash(geom)
    assert got == expect, ("geometry_b.yaml 的哈希 %08X 与 fixture 的 %08X 不一致"
                           % (got, expect))


# ---------------------------------------------------------------- 运行器


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    passed, failed = 0, []
    for name, fn in tests:
        try:
            fn()
            passed += 1
            print(f"  PASS  {name}")
        except Exception as exc:  # noqa: BLE001
            failed.append((name, exc))
            print(f"  FAIL  {name}")
            traceback.print_exc()
    print()
    print("=" * 70)
    print(f"test_localization 通过 {passed} / {len(tests)}")
    if failed:
        print("失败项：")
        for name, exc in failed:
            print(f"  - {name}: {exc}")
    print("=" * 70)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
