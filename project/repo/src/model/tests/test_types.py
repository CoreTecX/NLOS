# -*- coding: utf-8 -*-
"""配置数据类单元测试（方案 §12 Task 1）。

`src/model/tests/` 此前有 5 个单测（`test_preprocess` 等），独缺本文件；
`docs/port_completeness.md` §11.6 把它列为仅剩的文件级缺件之一。

`types.py` 是**整条链的派生量唯一来源**：几何尺寸、测量维数 M、候选数、
位宽链都从这些 frozen dataclass 字段算出。它一旦漂移，下游
（`preprocess` / `forward` / `quantize` / 硬件位宽表）会静默跟着漂移，
却不会有任何断言拦住 —— 因为派生量是算出来的，不是写死的。
本文件把「字段 -> 派生量」这层映射钉住：

  * `PreprocProfile` 的 plane / out_size / 形状 / 测量数 / 等效原图像素块；
  * `validate()` 的三条拒绝（奇数裁剪、平面不整除、帧数 < 1）；
  * frozen 语义（改字段即报错，避免流水线中途被悄悄改配置）；
  * 三个内置 profile 常量（official / B / A）的实际取值；
  * `FixedPointConfig` 的位宽链（单通道 / 双绿容器 / 帧累积）；
  * `CameraConfig` 默认值（满量程是 65535 而不是 2^12-1，这一条最易被改错）。

可直接运行（不依赖 pytest）：
    python src/model/tests/test_types.py
也可由 pytest 收集：
    pytest src/model/tests/test_types.py
"""

from __future__ import annotations

import dataclasses
import os
import sys
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import (  # noqa: E402
    PROFILE_A,
    PROFILE_B,
    PROFILE_OFFICIAL,
    CameraConfig,
    FixedPointConfig,
    PreprocProfile,
)


# ---------------------------------------------------------------- PreprocProfile 派生量


def test_official_profile_derived_values():
    """官方档：2016 -> 1008 -> 8x8 分块 -> 126x126 -> 差分 125x126。"""
    p = PROFILE_OFFICIAL
    assert (p.name, p.crop, p.block, p.frames) == ("official", 2016, 8, 1)
    assert p.plane == 1008
    assert p.out_size == 126
    assert p.measurement_shape == (126, 126, 3)
    assert p.diff_shape == (125, 126, 3)
    assert p.equivalent_original_block == 16
    assert p.diff_measurements == 125 * 126 == 15750


def test_b_profile_derived_values():
    """B 档（上板档）：1920 -> 960 -> 10x10 分块 -> 96x96 -> 差分 95x96。

    注意 M = 95*96 = 9120 就是 `psm_format.h` 的 `PS_PSM_MEAS_LEN`，
    也是 `raw_preproc`/`vertical_diff` 的墙网格口径。
    """
    p = PROFILE_B
    assert (p.name, p.crop, p.block, p.frames) == ("b", 1920, 10, 4)
    assert p.plane == 960
    assert p.out_size == 96
    assert p.measurement_shape == (96, 96, 3)
    assert p.diff_shape == (95, 96, 3)
    assert p.equivalent_original_block == 20
    assert p.diff_measurements == 95 * 96 == 9120


def test_a_profile_derived_values():
    """A 档保底：1280 -> 640 -> 10x10 分块 -> 64x64 -> 差分 63x64。"""
    p = PROFILE_A
    assert (p.name, p.crop, p.block, p.frames) == ("a", 1280, 10, 8)
    assert p.plane == 640
    assert p.out_size == 64
    assert p.diff_shape == (63, 64, 3)
    assert p.diff_measurements == 63 * 64 == 4032


def test_derived_shapes_are_self_consistent():
    """对三个内置档统一核对派生量之间的关系式（不是逐个抄数）。"""
    for p in (PROFILE_OFFICIAL, PROFILE_B, PROFILE_A):
        assert p.plane == p.crop // 2
        assert p.out_size == p.plane // p.block
        assert p.measurement_shape == (p.out_size, p.out_size, 3)
        assert p.diff_shape == (p.out_size - 1, p.out_size, 3)
        assert p.diff_measurements == p.diff_shape[0] * p.diff_shape[1]
        assert p.equivalent_original_block == p.block * 2
        p.validate()                       # 三个内置档都必须自洽


def test_custom_profile_with_identity_crop():
    """crop == plane 的退化档也应按同一套式子派生（block < plane）。"""
    p = PreprocProfile(name="tiny", crop=16, block=2, frames=1)
    assert p.plane == 8
    assert p.out_size == 4
    assert p.diff_shape == (3, 4, 3)
    assert p.diff_measurements == 12
    p.validate()


# ---------------------------------------------------------------- validate 的拒绝


def test_validate_rejects_odd_crop():
    try:
        PreprocProfile(name="x", crop=2015, block=5).validate()
    except ValueError:
        return
    raise AssertionError("奇数 crop 未被拦下")


def test_validate_rejects_indivisible_plane():
    # 1008 / 5 != 整数（1008 % 5 = 3）-> 必须报错，而不是静默取整
    try:
        PreprocProfile(name="x", crop=2016, block=5).validate()
    except ValueError:
        return
    raise AssertionError(f"平面 {2016 // 2} 不能被 block=5 整除却未报错")


def test_validate_rejects_nonpositive_frames():
    for bad in (0, -1):
        try:
            PreprocProfile(name="x", crop=16, block=2, frames=bad).validate()
        except ValueError:
            continue
        raise AssertionError(f"frames={bad} 未被拦下")


def test_validate_error_names_the_field():
    """报错信息要点出**哪个字段**不合法，便于定位配置漂移。"""
    # 注意 1008 能被 7 整除（7*144），故这里取 5 作反例
    try:
        PreprocProfile(name="x", crop=2016, block=5).validate()
    except ValueError as exc:
        msg = str(exc)
        assert "5" in msg, msg
        return
    raise AssertionError("未报错")


# ---------------------------------------------------------------- frozen 语义


def test_profile_is_frozen():
    p = PreprocProfile(name="x", crop=16, block=2, frames=1)
    try:
        p.crop = 32                       # type: ignore[misc]
    except dataclasses.FrozenInstanceError:
        return
    raise AssertionError("PreprocProfile 不是 frozen：配置可在流水线中途被改")
    # 派生量也必须只读（property，无 setter）
    try:
        p.out_size = 99                   # type: ignore[misc]
    except AttributeError:
        return
    raise AssertionError("out_size 可被赋值，派生量不再唯一")


def test_camera_config_is_frozen():
    cam = CameraConfig()
    try:
        cam.full_scale = 4095             # type: ignore[misc]
    except dataclasses.FrozenInstanceError:
        return
    raise AssertionError("CameraConfig 不是 frozen")


def test_fixed_point_config_is_frozen():
    fp = FixedPointConfig()
    try:
        fp.raw_bits = 12                  # type: ignore[misc]
    except dataclasses.FrozenInstanceError:
        return
    raise AssertionError("FixedPointConfig 不是 frozen")


# ---------------------------------------------------------------- CameraConfig


def test_camera_config_defaults():
    cam = CameraConfig()
    assert cam.bit_depth == 12
    assert cam.full_scale == 65535, "官方/本工程满量程是 16 bit 容器的 65535，不是 2^12-1"
    assert cam.bayer_pattern == "RGGB"
    assert cam.raw_shape == (2016, 2016)
    assert cam.dark_level == 0.0


def test_camera_config_override_keeps_other_defaults():
    cam = CameraConfig(dark_level=100.0)
    assert cam.dark_level == 100.0
    assert cam.full_scale == 65535 and cam.bayer_pattern == "RGGB"


# ---------------------------------------------------------------- FixedPointConfig 位宽链


def test_fixed_point_defaults():
    fp = FixedPointConfig()
    assert fp.raw_bits == 16, "源容器位宽按数据契约取 16（非 12）"
    assert fp.block_bits == 0 and fp.frame_acc_bits == 0, "0 表示自动派生"
    assert (fp.diff_bits, fp.coeff_bits, fp.mac_acc_bits) == (24, 16, 48)


def test_fixed_point_block_sum_single_channel():
    """单通道口径：源位宽 + ceil(log2(block^2))。"""
    fp = FixedPointConfig()
    assert fp.block_sum_bits(10) == 23, fp.block_sum_bits(10)
    assert fp.block_sum_bits(8) == 22
    assert fp.block_sum_bits(16) == 24
    fp12 = FixedPointConfig(raw_bits=12)
    assert fp12.block_sum_bits(10) == 19
    assert fp12.block_sum_bits(8) == 18


def test_fixed_point_block_sum_dual_green_is_plus_one():
    """容器口径 = 单通道 + 1（最坏通道是双绿 G1+G2，按和不按平均）。"""
    for source_bits in (12, 16):
        cfg = FixedPointConfig(raw_bits=source_bits)
        for block in (8, 10, 16):
            assert (cfg.block_sum_bits_dual_green(block)
                    == cfg.block_sum_bits(block) + 1)
    fp = FixedPointConfig()
    assert fp.block_sum_bits_dual_green(10) == 24
    assert fp.block_sum_bits_dual_green(8) == 23
    assert FixedPointConfig(raw_bits=12).block_sum_bits_dual_green(10) == 20


def test_fixed_point_frame_sum_growth():
    fp = FixedPointConfig()
    assert fp.frame_sum_bits(10, 1) == 23
    assert fp.frame_sum_bits(10, 4) == 25
    assert fp.frame_sum_bits(10, 8) == 26
    assert fp.frame_sum_bits(10, 35) == 29
    # frames=0 按 1 处理（max(frames,1)），不产生 log2(0)
    assert fp.frame_sum_bits(10, 0) == fp.frame_sum_bits(10, 1)


def test_fixed_point_budget_tracks_profile_block():
    """位宽链与实际档位的 block 必须对得上：B 档 10x10、官方档 8x8。"""
    fp = FixedPointConfig()
    assert fp.block_sum_bits(PROFILE_B.block) == 23
    assert fp.block_sum_bits(PROFILE_OFFICIAL.block) == 22
    # 双绿容器口径下，12 bit 源 + B 档 = 方案 §6.3 的 20 bit
    assert FixedPointConfig(raw_bits=12).block_sum_bits_dual_green(PROFILE_B.block) == 20


def test_fixed_point_monotone_in_block_and_frames():
    fp = FixedPointConfig()
    prev = 0
    for block in (2, 4, 8, 10, 16):
        cur = fp.block_sum_bits(block)
        assert cur >= prev, (block, cur, prev)
        prev = cur
    for frames in (1, 2, 4, 8, 16):
        assert fp.frame_sum_bits(10, frames) >= fp.block_sum_bits(10)


# ---------------------------------------------------------------- 包导出


def test_package_reexports_types():
    """`penumbrasight_model` 顶层应把类型与内置档一并导出（下游按名字取）。"""
    import penumbrasight_model as m

    for name in ("PreprocProfile", "PROFILE_A", "PROFILE_B", "PROFILE_OFFICIAL",
                 "CameraConfig", "FixedPointConfig"):
        assert hasattr(m, name), name
        assert name in m.__all__, name


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
    print(f"test_types 通过 {passed} / {len(tests)}")
    if failed:
        print("失败项：")
        for name, exc in failed:
            print(f"  - {name}: {exc}")
    print("=" * 70)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
