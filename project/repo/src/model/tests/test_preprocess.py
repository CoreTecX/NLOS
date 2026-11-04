# -*- coding: utf-8 -*-
"""预处理链单元测试（T0-02 收尾）。

黄金对照证明了预处理实现在官方数据上是正确的，但黄金对照只能覆盖官方数据
恰好走到的路径。本测试用合成数据锁住边界条件与最易出错的几个点，
防止后续改动引入回归。

可直接运行（不依赖 pytest）：
    python src/model/tests/test_preprocess.py
也可由 pytest 收集：
    pytest src/model/tests/test_preprocess.py
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

from penumbrasight_model import PROFILE_A, PROFILE_B, PROFILE_OFFICIAL  # noqa: E402
from penumbrasight_model.preprocess import (  # noqa: E402
    BAYER_PATTERNS,
    accumulate,
    block_mean,
    block_sum,
    center_crop,
    deinterleave_bayer,
    horizontal_diff,
    normalize,
    preprocess_raw,
    vertical_diff,
)
from penumbrasight_model.types import CameraConfig, FixedPointConfig  # noqa: E402

TOL = 1e-12


# ---------------------------------------------------------------- Bayer


def _make_bayer(pattern: str, size: int = 8):
    """按给定相位构造四通道取值互不相同的合成 Bayer 图。"""
    values = {"R": 100.0, "G1": 200.0, "G2": 300.0, "B": 400.0}
    raw = np.zeros((size, size), dtype=np.float64)
    for r, c, ch in BAYER_PATTERNS[pattern]:
        raw[r::2, c::2] = values[ch]
    return raw


def test_bayer_rggb_two_green_averaged():
    """RGGB 解交织：绿通道应为两路 G 的算术平均。"""
    raw = _make_bayer("RGGB")
    R, G, B = deinterleave_bayer(raw, "RGGB")
    assert np.allclose(R, 100.0, atol=TOL), R.min()
    assert np.allclose(B, 400.0, atol=TOL), B.min()
    assert np.allclose(G, 250.0, atol=TOL), "绿通道应为 (200+300)/2 = 250"
    assert R.shape == (4, 4)


def test_bayer_all_phases_consistent():
    """四种相位都应把 R 放在各自声明的位置上。"""
    for pattern in ("RGGB", "BGGR", "GRBG", "GBRG"):
        raw = _make_bayer(pattern)
        R, G, B = deinterleave_bayer(raw, pattern)
        assert np.allclose(R, 100.0, atol=TOL), f"{pattern} 的 R 不正确"
        assert np.allclose(B, 400.0, atol=TOL), f"{pattern} 的 B 不正确"
        assert np.allclose(G, 250.0, atol=TOL), f"{pattern} 的 G 不正确"


def test_bayer_rejects_odd_size():
    """奇数边长应报错，而不是静默丢最后一行。"""
    for bad in (np.zeros((7, 8)), np.zeros((8, 7))):
        try:
            deinterleave_bayer(bad, "RGGB")
        except ValueError:
            continue
        raise AssertionError("奇数边长未报错")


def test_bayer_rejects_unknown_pattern():
    try:
        deinterleave_bayer(_make_bayer("RGGB"), "XYZ")
    except ValueError:
        return
    raise AssertionError("未知相位未报错")


# ---------------------------------------------------------------- 分块


def test_block_mean_does_not_cross_rows():
    """分块不得跨越行边界——这是预处理里最常见的实现错误。

    构造每行取常量、行间不同的图像：若实现把二维索引展平后分块，
    块内会混入相邻行的值，结果将偏离行常量。
    """
    ch = np.repeat(np.arange(4, dtype=np.float64)[:, None], 8, axis=1)
    out = block_mean(ch, 2)
    expected = np.array([[0.5] * 4, [2.5] * 4])
    assert out.shape == (2, 4), out.shape
    assert np.allclose(out, expected, atol=TOL), out


def test_block_mean_matches_naive_loop():
    """与逐块双重循环的朴素实现逐元素一致。"""
    rng = np.random.default_rng(7)
    ch = rng.random((12, 16))
    out = block_mean(ch, 4)
    naive = np.zeros((3, 4))
    for i in range(3):
        for j in range(4):
            naive[i, j] = ch[i * 4:(i + 1) * 4, j * 4:(j + 1) * 4].mean()
    assert np.allclose(out, naive, atol=TOL)


def test_block_sum_is_mean_times_area():
    rng = np.random.default_rng(3)
    ch = rng.random((10, 10))
    assert np.allclose(block_sum(ch, 5), block_mean(ch, 5) * 25.0, atol=1e-12)


def test_block_rejects_indivisible():
    ch = np.zeros((10, 10))
    for bad_block in (3, 4, 7):
        try:
            block_mean(ch, bad_block)
        except ValueError:
            continue
        raise AssertionError(f"block={bad_block} 不能整除却未报错")


def test_block_sum_saturation_headroom():
    """饱和像素的分块求和应等于满量程乘块面积，用于核对位宽预算。"""
    ch = np.full((10, 10), 65535.0)
    out = block_sum(ch, 10)
    assert out.shape == (1, 1)
    assert out[0, 0] == 65535.0 * 100.0


# ---------------------------------------------------------------- 裁剪与累积


def test_center_crop_offsets():
    img = np.arange(64, dtype=np.float64).reshape(8, 8)
    c = center_crop(img, 4)
    assert c.shape == (4, 4)
    assert np.array_equal(c, img[2:6, 2:6]), "居中裁剪应左右上下各取 2 像素"


def test_center_crop_identity():
    img = np.zeros((6, 6))
    assert np.array_equal(center_crop(img, 6), img)


def test_center_crop_rejects_oversize():
    try:
        center_crop(np.zeros((6, 6)), 8)
    except ValueError:
        return
    raise AssertionError("裁剪尺寸超过原图却未报错")


def test_accumulate_is_sum_not_mean():
    """累积规则为求和而非平均——已由官方数据逐元素验证，此处锁住行为。"""
    frames = np.stack([np.full((2, 2), 3.0), np.full((2, 2), 4.0)])
    out = accumulate(frames)
    assert np.allclose(out, 7.0, atol=TOL), "应为求和 3+4=7，而不是均值 3.5"


def test_accumulate_single_frame_is_copy():
    f = np.ones((3, 3)) * 2.0
    assert np.allclose(accumulate(f), f)


def test_accumulate_rejects_bad_ndim():
    try:
        accumulate(np.zeros((2, 2, 2, 2)))
    except ValueError:
        return
    raise AssertionError("四维输入未报错")


# ---------------------------------------------------------------- 差分


def test_vertical_diff_direction_and_shape():
    """垂直差分应为「下一行减上一行」，形状 (H-1, W, C)。"""
    m = np.zeros((4, 3, 1))
    m[:, :, 0] = np.arange(4, dtype=np.float64)[:, None]
    d = vertical_diff(m)
    assert d.shape == (3, 3, 1)
    assert np.allclose(d, 1.0, atol=TOL), "行间差应恒为 +1"


def test_vertical_diff_rows_not_columns():
    """差分必须沿行方向：仅在列方向变化的图像差分后应恒为 0。"""
    m = np.zeros((4, 3, 1))
    m[:, :, 0] = np.arange(3, dtype=np.float64)[None, :]
    assert np.allclose(vertical_diff(m), 0.0, atol=TOL), "沿列变化不应被差分捕捉"


def test_horizontal_diff_direction():
    m = np.zeros((4, 3, 1))
    m[:, :, 0] = np.arange(3, dtype=np.float64)[None, :]
    d = horizontal_diff(m)
    assert d.shape == (4, 2, 1)
    assert np.allclose(d, 1.0, atol=TOL)


def test_diff_rejects_2d():
    for fn in (vertical_diff, horizontal_diff):
        try:
            fn(np.zeros((4, 4)))
        except ValueError:
            continue
        raise AssertionError(f"{fn.__name__} 未拒绝二维输入")


# ---------------------------------------------------------------- 端到端与配置


def test_preprocess_shapes_all_profiles():
    """各档 profile 的输出形状必须与设计一致。"""
    cases = [
        (PROFILE_OFFICIAL, 2016, (126, 126, 3), (125, 126, 3)),
        (PROFILE_B, 2016, (96, 96, 3), (95, 96, 3)),
        (PROFILE_A, 2016, (64, 64, 3), (63, 64, 3)),
    ]
    for prof, n, want_rgb, want_diff in cases:
        raw = np.full((n, n), 1000.0)
        m = preprocess_raw(raw, prof)
        assert m.rgb.shape == want_rgb, f"{prof.name}: {m.rgb.shape}"
        assert m.diff.shape == want_diff, f"{prof.name}: {m.diff.shape}"
    # 官方配置等效 16x16 原图像素块，B 档等效 20x20
    assert PROFILE_OFFICIAL.equivalent_original_block == 16
    assert PROFILE_B.equivalent_original_block == 20


def test_preprocess_uniform_input_is_uniform():
    """均匀输入的输出应均匀，且差分恒为 0。"""
    raw = np.full((2016, 2016), 500.0)
    m = preprocess_raw(raw, PROFILE_OFFICIAL)
    assert np.allclose(m.rgb, 500.0, atol=1e-9)
    assert np.allclose(m.diff, 0.0, atol=1e-9)


def test_preprocess_dark_level_subtracted():
    raw = np.full((2016, 2016), 300.0)
    m = preprocess_raw(raw, PROFILE_OFFICIAL, dark_level=100.0)
    assert np.allclose(m.rgb, 200.0, atol=1e-9)


def test_profile_validation_catches_misconfiguration():
    from dataclasses import replace
    bad = replace(PROFILE_B, block=7)          # 960 不能被 7 整除
    try:
        bad.validate()
    except ValueError:
        return
    raise AssertionError("不整除的配置未在 validate 中报错")


def test_normalize_divisor():
    raw = np.full((2016, 2016), 100.0)
    m = preprocess_raw(raw, PROFILE_B)
    out, div = normalize(m.rgb, PROFILE_B, frames=4)
    assert div == 65535.0 * 4, div
    assert np.allclose(out, 100.0 / (65535.0 * 4), atol=1e-15)


def test_camera_config_defaults():
    cam = CameraConfig()
    assert cam.bayer_pattern == "RGGB"
    assert cam.full_scale == 65535, "官方数据满量程为 65535，非 2^12-1"


# ---------------------------------------------------------------- 定点位宽


def test_fixed_point_bit_budget():
    """位宽预算：源 16 bit 下，10x10 块和需 23 bit，8x8 需 22 bit。

    这是数据契约给出的修正结论——原技术方案按 12 bit 源设计为 20 bit，
    在官方 16 bit 容器下会溢出。
    """
    fp = FixedPointConfig()
    assert fp.raw_bits == 16
    assert fp.block_sum_bits(10) == 23, fp.block_sum_bits(10)
    assert fp.block_sum_bits(8) == 22, fp.block_sum_bits(8)
    # 12 bit 源的旧预算
    fp12 = FixedPointConfig(raw_bits=12)
    assert fp12.block_sum_bits(10) == 19, fp12.block_sum_bits(10)


def test_fixed_point_bit_budget_dual_green_container():
    """容器口径：最坏通道是双绿 G1+G2，比单通道多 1 bit。

    单通道（R 或 B）：12 bit 源 10x10 → 19，16 bit 源 10x10 → 23。
    容器（G1+G2 按和进累加，不做 0.5 舍入）：两者各 +1 → 20 / 24。
    这一条与 `src/hardware/preprocess.hpp` 的 `PS_BS_BITS` 对齐。
    """
    fp = FixedPointConfig()
    assert fp.block_sum_bits_dual_green(10) == 24, fp.block_sum_bits_dual_green(10)
    assert fp.block_sum_bits_dual_green(8) == 23, fp.block_sum_bits_dual_green(8)
    fp12 = FixedPointConfig(raw_bits=12)
    assert fp12.block_sum_bits_dual_green(10) == 20, fp12.block_sum_bits_dual_green(10)
    # 容器口径恒等于单通道 + 1
    for source_bits in (12, 16):
        cfg = FixedPointConfig(raw_bits=source_bits)
        for block in (8, 10, 16):
            assert (cfg.block_sum_bits_dual_green(block)
                    == cfg.block_sum_bits(block) + 1)


def test_fixed_point_frame_accumulation_growth():
    fp = FixedPointConfig()
    assert fp.frame_sum_bits(10, 1) == 23
    assert fp.frame_sum_bits(10, 4) == 25, fp.frame_sum_bits(10, 4)
    assert fp.frame_sum_bits(10, 35) == 29, fp.frame_sum_bits(10, 35)


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
    print(f"通过 {passed} / {len(tests)}")
    if failed:
        print("失败项：")
        for name, exc in failed:
            print(f"  - {name}: {exc}")
    print("=" * 70)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
