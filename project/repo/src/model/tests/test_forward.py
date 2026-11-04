# -*- coding: utf-8 -*-
"""几何前向模型单元测试（对应方案 §12 Task 1）。

覆盖 `penumbrasight_model.forward`：墙/显示器/遮挡物的几何派生量、
可见性闭式解、光传输矩阵 A 的构造与角度响应、以及 投影能量/奇异谱 这几个
定位与重建都依赖的基础量。

测试全部用**合成的小几何**，不依赖仓库外的官方数据集，也不读任何 `.mat`。
可直接运行（不依赖 pytest）：
    python src/model/tests/test_forward.py
"""

from __future__ import annotations

import os
import sys
import traceback

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import forward as F          # noqa: E402

TOL = 1e-12


# ---------------------------------------------------------------- 合成几何


def _tiny_setup(occ_ll=(-1.0, 0.5, -1.0), occ_size=(2.0, 2.0), wall_n: int = 8):
    """构造一个尺寸很小、但与真实物理结构同构的几何配置。

    墙 y=1.0、显示器 y=0.0、遮挡物 y=0.5 -> 交点 t = 0.5。
    """
    wall = F.WallPlane(y=1.0, x_min=-0.5, x_max=0.5,
                       z_min=-0.5, z_max=0.5, n=wall_n)
    monitor = F.MonitorPlane(
        y=0.0,
        x_centers=np.array([-0.1, 0.0, 0.1]),
        z_centers=np.array([-0.05, 0.05]),
        block_w=0.01, block_h=0.01)
    occluder = F.Occluder(ll=np.asarray(occ_ll, dtype=np.float64),
                          size=np.asarray(occ_size, dtype=np.float64))
    return F.OpticalSetup(wall=wall, monitor=monitor, occluder=occluder)


# ---------------------------------------------------------------- 墙/显示器/遮挡物


def test_wall_axes_and_row_major_grid():
    w = F.WallPlane(y=1.0, x_min=0.0, x_max=1.0, z_min=0.0, z_max=1.0, n=4)
    assert np.allclose(w.x_axis, [0.125, 0.375, 0.625, 0.875], atol=TOL)
    assert np.allclose(w.z_axis, [0.125, 0.375, 0.625, 0.875], atol=TOL)
    xx, zz = w.grid()
    assert xx.shape == (16,) and zz.shape == (16,)
    # 行优先：前 n 个元素是一整行的 x（同一 z），与 meshgrid 顺序一致
    assert np.allclose(xx[:4], w.x_axis, atol=TOL)
    assert np.allclose(zz[:4], w.z_axis[0], atol=TOL)
    assert np.allclose(xx[4:8], w.x_axis, atol=TOL)


def test_wall_derived_counts():
    w = F.WallPlane(y=1.0, x_min=0.0, x_max=1.0, z_min=0.0, z_max=2.0, n=4)
    assert w.n_points == 16
    sx, sz = w.spacing
    assert abs(sx - 0.25) < TOL and abs(sz - 0.5) < TOL


def test_monitor_shape_and_block_area():
    m = F.MonitorPlane(y=0.0,
                       x_centers=np.arange(4, dtype=np.float64),
                       z_centers=np.arange(3, dtype=np.float64),
                       block_w=0.02, block_h=0.03)
    assert m.shape == (3, 4)
    assert m.n_pixels == 12
    assert abs(m.block_area - 0.0006) < 1e-15
    xx, zz = m.flat_grid()
    assert xx.shape == (12,) and zz.shape == (12,)


def test_occluder_ranges_and_center():
    o = F.Occluder(ll=np.array([-0.04, 0.594, -0.04]),
                   size=np.array([0.075, 0.075]))
    assert abs(o.y - 0.594) < TOL
    assert o.x_range[0] == -0.04 and abs(o.x_range[1] - 0.035) < TOL
    assert abs(o.center[0] - (-0.0025)) < TOL
    assert abs(o.center[1] - 0.594) < TOL


# ---------------------------------------------------------------- 可见性


def test_visibility_all_visible_when_occluder_off_scene():
    """遮挡物挪到场景外时，所有连线都不穿过它 -> V 全 True。"""
    setup = _tiny_setup(occ_ll=(100.0, 0.5, 100.0), occ_size=(1.0, 1.0))
    V = F.visibility(setup)
    assert V.shape == (setup.wall.n_points, setup.monitor.n_pixels)
    assert V.all(), "遮挡物在场景外时不应遮挡任何连线"


def test_visibility_all_blocked_when_occluder_covers():
    """遮挡物足够大时，所有交点都落在矩形内 -> V 全 False。"""
    setup = _tiny_setup(occ_ll=(-1.0, 0.5, -1.0), occ_size=(2.0, 2.0))
    V = F.visibility(setup)
    assert not V.any(), "遮挡物覆盖全部交点时应挡住所有连线"


def test_visible_fraction_shape_and_range():
    setup = _tiny_setup(occ_ll=(100.0, 0.5, 100.0), occ_size=(1.0, 1.0))
    fr = F.visible_fraction(setup)
    assert fr.shape == (setup.wall.n, setup.wall.n)
    assert fr.min() >= 0.0 and fr.max() <= 1.0


def test_crossing_points_match_closed_form():
    """交点应等于 q = (1-t) m + t w，t = y_occ / y_wall。"""
    setup = _tiny_setup(occ_ll=(100.0, 0.5, 100.0))
    qx, qz, t = F.crossing_points(setup, dtype=np.float64)
    wx, wz = setup.wall.grid()
    mx, mz = setup.monitor.flat_grid()
    exp_x = (1.0 - t) * mx[None, :] + t * wx[:, None]
    exp_z = (1.0 - t) * mz[None, :] + t * wz[:, None]
    assert abs(t - 0.5) < TOL
    assert np.allclose(qx, exp_x, atol=1e-12)
    assert np.allclose(qz, exp_z, atol=1e-12)


# ---------------------------------------------------------------- 光传输矩阵


def test_transport_matrix_positive_with_and_without_occluder():
    setup = _tiny_setup()
    A = F.build_transport_matrix(setup, with_occluder=False, dtype=np.float64)
    assert A.shape == (setup.wall.n_points, setup.monitor.n_pixels)
    assert np.all(A > 0.0), "无遮挡时权重应处处为正"
    A_occ = F.build_transport_matrix(setup, with_occluder=True, dtype=np.float64)
    assert np.all(A_occ[A_occ > 0] > 0.0)
    assert np.any(A_occ == 0.0), "全遮挡几何下应出现被挡住的零权重"


def test_transport_matrix_occluder_only_zeroes_entries():
    """加遮挡只应把部分元素置零，不应改变其余元素的数值。"""
    setup = _tiny_setup()
    A0 = F.build_transport_matrix(setup, with_occluder=False, dtype=np.float64)
    A1 = F.build_transport_matrix(setup, with_occluder=True, dtype=np.float64)
    mask = A1 != 0.0
    assert np.allclose(A1[mask], A0[mask], atol=1e-12)
    assert A1.shape == A0.shape


def test_angle_response_modes():
    cos_theta = np.array([0.2, 0.5, 1.0])
    assert np.allclose(F.angle_response(None, cos_theta, "none"), 1.0)
    assert np.allclose(F.angle_response(None, cos_theta, "cos_pow", 1.0),
                       cos_theta, atol=1e-6)
    sq = F.angle_response(None, cos_theta, "cos_pow", 2.0)
    assert np.allclose(sq, cos_theta ** 2, atol=1e-6)
    poly = F.angle_response(None, cos_theta, "poly", 0.5)
    assert np.allclose(poly, 1.0 + 0.5 * (1.0 - cos_theta), atol=1e-6)
    try:
        F.angle_response(None, cos_theta, "nope")
    except ValueError:
        return
    raise AssertionError("未知角度响应模式未报错")


def test_forward_render_shape_and_scene_check():
    setup = _tiny_setup()
    img = F.forward_render(setup)
    assert img.shape == (setup.wall.n, setup.wall.n)
    assert np.all(img >= 0.0)
    f = np.ones(setup.monitor.n_pixels)
    img2 = F.forward_render(setup, scene=f)
    assert np.allclose(img, img2, atol=1e-12)
    try:
        F.forward_render(setup, scene=np.ones(3))
    except ValueError:
        return
    raise AssertionError("场景长度不符未报错")


# ---------------------------------------------------------------- 谱与能量


def test_projection_energy_identity_operator():
    """A = I 时投影能量应恒等于 ||y||^2（kappa=0，不截断）。"""
    rng = np.random.default_rng(0)
    n = 6
    A = np.eye(n)
    y = rng.standard_normal(n)
    e = F.projection_energy(A, y, kappa=0.0)
    assert abs(e - float(y @ y)) < 1e-9, (e, float(y @ y))
    # 截断到只剩最大成分时能量应不超过全量
    assert F.projection_energy(A, y, kappa=0.75) <= e + 1e-9


def test_projection_energy_rejects_size_mismatch():
    try:
        F.projection_energy(np.eye(4), np.ones(3))
    except ValueError:
        return
    raise AssertionError("A 行数与 y 长度不符未报错")


def test_singular_spectrum_descending():
    A = np.diag([3.0, 2.0, 1.0])
    s = F.singular_spectrum(A, top=3)
    assert np.allclose(s, [3.0, 2.0, 1.0], atol=1e-9), s


def test_condition_number_matches_top_minus_one_spectrum():
    """`condition_number` 按 top = min(shape) - 1 取谱，核对它与该谱定义一致。

    这里锁的是**实现的行为口径**（s1 / s_{n-1}），不是数学上的满谱条件数；
    对该函数做定量解释时必须以这个口径为准。
    """
    A = np.diag([4.0, 2.0, 1.0])
    s = F.singular_spectrum(A, top=min(A.shape) - 1)
    want = float(s[0] / s[-1])
    assert abs(F.condition_number(A) - want) < 1e-9
    assert F.condition_number(A) >= 1.0


def test_mean_abs_column_correlation_bounds_and_duplicates():
    rng = np.random.default_rng(1)
    A = rng.standard_normal((10, 5))
    val = F.mean_abs_column_correlation(A)
    assert 0.0 <= val <= 1.0
    dup = np.tile(rng.standard_normal((10, 1)), (1, 4))   # 四列完全相同
    assert F.mean_abs_column_correlation(dup) > 0.99


def test_with_occluder_ll_preserves_geometry():
    setup = _tiny_setup()
    s2 = setup.with_occluder_ll([0.3, 0.5, 0.3])
    assert s2.wall is setup.wall and s2.monitor is setup.monitor
    assert np.allclose(s2.occluder.ll, [0.3, 0.5, 0.3])
    assert np.allclose(s2.occluder.size, setup.occluder.size)


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
    print(f"test_forward 通过 {passed} / {len(tests)}")
    if failed:
        print("失败项：")
        for name, exc in failed:
            print(f"  - {name}: {exc}")
    print("=" * 70)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
