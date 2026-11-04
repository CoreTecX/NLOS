# -*- coding: utf-8 -*-
"""X6/X7 golden 测试向量：12 bit RGGB 解马赛克 + 分块求和，**双打包口径**。

为什么需要这一批向量
--------------------
相机冻结为「彩色 / 有效位深 12 bit / Bayer RGGB / 传输容器 16 bit」
（`configs/camera_b.yaml`），但 12 bit 值在 16 bit 容器里的摆法有两种：

  packed    右对齐   -> 容器值 ∈ [0, 4095]，饱和帧最大值 4095
  fullscale 左移 4 位 -> 容器值 ∈ [0, 65520]，饱和帧最大值 65520

首帧实拍前无法判定是哪一种。既有向量（`v1~v4`）走的是官方 16 bit 容器
（数值到 65535），**不能**用来区分这两种口径，所以这里单独生成一批：
同一份 12 bit 源数值，按两种口径各生成一份输入与期望值。

判据与 S0-3 一致：**全程整数、逐值相等**（不做除法、不做双绿平均）。
所有期望值由仓库既有参考实现（`penumbrasight_model.preprocess`）算出，
生成时用 `assert` 自查一遍，不依赖任何硬件工具。

产出
----
  `sim/vectors/x6_x7_bitdepth.npz`                     人类可读的 golden
  `src/hardware/hwdata/x6_x7_bitdepth__*.bin`          C++ 主机测试读的裸二进制
  `src/hardware/hwdata/manifest_x6_x7.txt`             字段清单
  `sim/results/x6_x7_bitdepth_numbers.json`            位宽/量级数字（供报告引用）

用法（在仓库根）：
    python src/model/scripts/make_x6_x7_vectors.py
"""

from __future__ import annotations

import io
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
_REPO = os.path.dirname(os.path.dirname(_SRC_MODEL))
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model.preprocess import (  # noqa: E402
    block_mean,
    block_sum,
    deinterleave_bayer,
)

VEC_DIR = os.path.join(_REPO, "sim", "vectors")
HWDATA_DIR = os.path.join(_REPO, "src", "hardware", "hwdata")
RESULT_DIR = os.path.join(_REPO, "sim", "results")

# ------------------------------------------------------------------ 规模
N = 40            # RAW 边长（偶数）
BLOCK = 10        # 分块边长（解交织后平面内）
PH = N // 2       # 平面边长 20
BH = PH // BLOCK  # 块网格 2x2
N_FRAMES = 8      # 多帧累积帧数
MARKS = (1, 4, 8)

EFFECTIVE_BITS = 12
FULLSCALE_SHIFT = 4
CODE_MAX = (1 << EFFECTIVE_BITS) - 1        # 4095
FULLSCALE_MAX = CODE_MAX << FULLSCALE_SHIFT  # 65520

PACKINGS = ("packed", "full")

LOG: list = []


def say(s: str = "") -> None:
    LOG.append(s)
    print(s, flush=True)


# ------------------------------------------------------------------ 输入构造


def case_ramp() -> np.ndarray:
    """覆盖整个 12 bit 码值的斜坡（17 与 4096 互素，400 个值互不相同）。

    用途：解马赛克里任何相位错、转置错、丢行错都会在这上面暴露。
    """
    i = np.arange(N * N, dtype=np.int64)
    return ((i * 17) % (CODE_MAX + 1)).astype(np.uint16).reshape(N, N)


def case_mix() -> np.ndarray:
    """上半幅满量程、下半幅全零。

    用途：让「相邻块行的差分」取到最大负值，是差分通路位宽的**最坏情形**。
    经过 2x2 解交织后，RAW 行 0..19 -> 平面行 0..9（满），RAW 行 20..39 -> 平面行 10..19（零），
    于是块行 0 取到最大值、块行 1 全零，行差分 = -最大值。
    """
    raw = np.zeros((N, N), dtype=np.uint16)
    raw[: N // 2, :] = CODE_MAX
    return raw


def case_sat() -> np.ndarray:
    """整幅满量程。用途：块和/累积的位宽上界。"""
    return np.full((N, N), CODE_MAX, dtype=np.uint16)


def to_fullscale(raw: np.ndarray) -> np.ndarray:
    """packed -> fullscale（左移 4 位）。"""
    return (raw.astype(np.uint32) << FULLSCALE_SHIFT).astype(np.uint16)


# ------------------------------------------------------------------ 参考链路（整数，逐值）


def planes(raw: np.ndarray):
    """解交织四路（不合并双绿）。用参考实现的分相表，避免自造相位。"""
    r, g_avg, b = deinterleave_bayer(raw.astype(np.uint64), "RGGB")
    # deinterleave_bayer 返回的 G 是 (G1+G2)/2；这里要的是**未平均**的两路，
    # 故直接按同一张相位表取。
    from penumbrasight_model.preprocess import BAYER_PATTERNS
    pos = {ch: (r_, c_) for r_, c_, ch in BAYER_PATTERNS["RGGB"]}
    g1 = np.asarray(raw[pos["G1"][0]::2, pos["G1"][1]::2], dtype=np.uint64)
    g2 = np.asarray(raw[pos["G2"][0]::2, pos["G2"][1]::2], dtype=np.uint64)
    # 自查：参考实现的 G 确实是 (G1+G2)/2
    assert np.array_equal(g_avg, (g1 + g2) / 2.0), "双绿平均口径与参考实现不一致"
    return (np.asarray(r, dtype=np.uint64), g1, g2, np.asarray(b, dtype=np.uint64))


def diff_rows(a: np.ndarray) -> np.ndarray:
    """行方向差分：下一行减上一行（与 preprocess.hpp 的 row_diff 同一口径）。"""
    return (a[1:, :].astype(np.int64) - a[:-1, :].astype(np.int64)).astype(np.int32)


def build_case(name: str, raw: np.ndarray, out: dict, log_rows: list) -> None:
    """为一个用例生成两种口径的平面/块和/差分，并逐项自查。"""
    r_p, g1_p, g2_p, b_p = planes(raw)
    sums = {
        "r": block_sum(r_p.astype(np.float64), BLOCK),
        "g1g2": block_sum((g1_p + g2_p).astype(np.float64), BLOCK),
        "b": block_sum(b_p.astype(np.float64), BLOCK),
    }
    sums = {k: v.astype(np.uint64) for k, v in sums.items()}

    raw_full = to_fullscale(raw)
    r_f, g1_f, g2_f, b_f = planes(raw_full)
    sums_f = {
        "r": block_sum(r_f.astype(np.float64), BLOCK),
        "g1g2": block_sum((g1_f + g2_f).astype(np.float64), BLOCK),
        "b": block_sum(b_f.astype(np.float64), BLOCK),
    }
    sums_f = {k: v.astype(np.uint64) for k, v in sums_f.items()}

    # ---- 自查 1：两种口径的数值只差 2^4（同一束光，两种摆法）
    for k in ("r", "g1g2", "b"):
        assert np.array_equal(sums_f[k], sums[k] << FULLSCALE_SHIFT), \
            f"{name}/{k}: fullscale 不等于 packed << 4"
    for a, b_ in ((r_p, r_f), (g1_p, g1_f), (g2_p, g2_f), (b_p, b_f)):
        assert np.array_equal(b_, a << FULLSCALE_SHIFT), f"{name}: 平面不满足 <<4 关系"

    # ---- 自查 2：块和 == 块均值 * 块面积（参考实现的另一条路径）
    # 注意：这一条**不是**逐位相等，而是只差浮点舍入——block_mean 先除以
    # BLOCK^2 再乘回来，浮点除法不精确。把这个差值量出来，正是「逐值最大
    # 相对误差」的来源（整数路径自身与自身是精确相等的）。
    mean_rel = 0.0
    for k, pl in (("r", r_p), ("b", b_p), ("g1g2", g1_p + g2_p)):
        via_mean = block_mean(pl.astype(np.float64), BLOCK) * (BLOCK * BLOCK)
        out[f"ref_float_{name}_sum_{k}"] = via_mean.astype(np.float64)
        rel = np.abs(via_mean - sums[k].astype(np.float64)) / np.maximum(
            np.abs(sums[k].astype(np.float64)), 1.0)
        mean_rel = max(mean_rel, float(np.max(rel)))
    assert mean_rel < 1e-12, f"{name}: block_mean*area 与 block_sum 偏离过大 ({mean_rel:.3e})"

    # ---- 落盘
    out[f"{name}_packed_plane_r"] = r_p.astype(np.uint16)
    out[f"{name}_packed_plane_g1"] = g1_p.astype(np.uint16)
    out[f"{name}_packed_plane_g2"] = g2_p.astype(np.uint16)
    out[f"{name}_packed_plane_b"] = b_p.astype(np.uint16)
    out[f"{name}_full_plane_r"] = r_f.astype(np.uint16)
    out[f"{name}_full_plane_g1"] = g1_f.astype(np.uint16)
    out[f"{name}_full_plane_g2"] = g2_f.astype(np.uint16)
    out[f"{name}_full_plane_b"] = b_f.astype(np.uint16)
    for k in ("r", "g1g2", "b"):
        out[f"{name}_packed_sum_{k}"] = sums[k].astype(np.uint32)
        out[f"{name}_full_sum_{k}"] = sums_f[k].astype(np.uint32)
    for ch, k in (("r", "r"), ("g", "g1g2"), ("b", "b")):
        out[f"{name}_packed_diff_{ch}"] = diff_rows(sums[k])
        out[f"{name}_full_diff_{ch}"] = diff_rows(sums_f[k])

    log_rows.append({
        "case": name,
        "panel": "packed",
        "raw_max": int(raw.max()),
        "plane_max": int(max(r_p.max(), g1_p.max(), g2_p.max(), b_p.max())),
        "sum_max": int(max(v.max() for v in sums.values())),
        "diff_absmax": int(max(np.abs(diff_rows(sums[k])).max() for k in sums)),
        "mean_vs_sum_rel_max": mean_rel,
    })
    log_rows.append({
        "case": name,
        "panel": "full",
        "raw_max": int(raw_full.max()),
        "plane_max": int(max(r_f.max(), g1_f.max(), g2_f.max(), b_f.max())),
        "sum_max": int(max(v.max() for v in sums_f.values())),
        "diff_absmax": int(max(np.abs(diff_rows(sums_f[k])).max() for k in sums_f)),
        "mean_vs_sum_rel_max": mean_rel,
    })


def build_frames(out: dict, raw: np.ndarray, log_rows: list) -> dict:
    """8 帧累积：逐帧块和 -> 累加 -> 行差分，给出 1/4/8 帧三档期望值。"""
    frames_p = np.repeat(raw[None, :, :], N_FRAMES, axis=0)
    frames_f = to_fullscale(frames_p)

    out["frames_mix_packed"] = frames_p
    out["frames_mix_full"] = frames_f

    summary = {}
    for tag, frames in (("packed", frames_p), ("full", frames_f)):
        acc = {k: np.zeros((BH, BH), dtype=np.uint64) for k in ("r", "g1g2", "b")}
        for f in range(N_FRAMES):
            r_, g1_, g2_, b_ = planes(frames[f])
            acc["r"] += block_sum(r_.astype(np.float64), BLOCK).astype(np.uint64)
            acc["g1g2"] += block_sum((g1_ + g2_).astype(np.float64), BLOCK).astype(np.uint64)
            acc["b"] += block_sum(b_.astype(np.float64), BLOCK).astype(np.uint64)
            if (f + 1) in MARKS:
                m = f + 1
                for k in ("r", "g1g2", "b"):
                    out[f"acc{m}_mix_{tag}_sum_{k}"] = acc[k].astype(np.uint64).astype(np.uint32)
                for ch, k in (("r", "r"), ("g", "g1g2"), ("b", "b")):
                    out[f"acc{m}_mix_{tag}_diff_{ch}"] = diff_rows(acc[k])
                summary[(tag, m)] = {
                    "acc_max": int(max(v.max() for v in acc.values())),
                    "diff_absmax": int(max(np.abs(diff_rows(acc[k])).max() for k in acc)),
                }
    return summary


# ------------------------------------------------------------------ 位宽数字


def bits_unsigned(x: int) -> int:
    return 0 if x <= 0 else int(math.floor(math.log2(x))) + 1


def bits_signed(x: int) -> int:
    return bits_unsigned(abs(x)) + 1


def budget(packing: str, block: int, frames: int) -> dict:
    src = EFFECTIVE_BITS if packing == "packed" else 16
    e_blk = int(math.ceil(math.log2(block * block)))
    e_acc = int(math.ceil(math.log2(max(frames, 1))))
    declared_block = 20 if packing == "packed" else 23
    diff_declared = 24
    bs_min = src + e_blk
    fa_min = bs_min + e_acc
    diff_need = fa_min + 1
    return {
        "packing": packing,
        "src_bits": src,
        "block": block,
        "frames": frames,
        "block_sum_min": bs_min,
        "block_sum_declared": declared_block,
        "frame_acc_min": fa_min,
        "frame_acc_declared": declared_block + e_acc,
        "diff_needed": diff_need,
        "diff_declared": diff_declared,
        "diff_fits": diff_declared >= diff_need,
        "headroom_bits": diff_declared - diff_need,
    }


# ------------------------------------------------------------------ 导出


def export_bins(arrays: dict, manifest_lines: list) -> int:
    total = 0
    for k, a in arrays.items():
        if a.dtype.kind == "U":
            continue  # 字符串不导二进制
        arr = np.ascontiguousarray(a)
        arr.tofile(os.path.join(HWDATA_DIR, f"x6_x7_bitdepth__{k}.bin"))
        total += arr.nbytes
        manifest_lines.append(f"  {k:38s} shape={str(arr.shape):16s} "
                              f"dtype={arr.dtype}  size={arr.size}")
        if arr.size <= 16:
            manifest_lines.append(f"       值 = {arr.ravel()}")
    return total


def main() -> int:
    os.makedirs(HWDATA_DIR, exist_ok=True)
    os.makedirs(RESULT_DIR, exist_ok=True)

    say("=" * 78)
    say("X6/X7 golden 向量生成：12 bit RGGB 解马赛克 + 分块求和，双打包口径")
    say("=" * 78)
    say(f"  RAW {N}x{N}，block={BLOCK}，平面 {PH}x{PH}，块网格 {BH}x{BH}，"
        f"差分 {BH-1}x{BH}，帧累积 {N_FRAMES} 帧（档位 {list(MARKS)}）")

    out: dict = {}
    rows: list = []
    ramp = case_ramp()
    mix = case_mix()
    sat = case_sat()

    say("\n[1/3] 单帧三用例（ramp / mix / sat）...")
    for name, raw in (("ramp", ramp), ("mix", mix), ("sat", sat)):
        out[f"raw_{name}_packed"] = raw
        out[f"raw_{name}_full"] = to_fullscale(raw)
        build_case(name, raw, out, rows)

    say("\n[2/3] 8 帧累积 ...")
    frame_summary = build_frames(out, mix, rows)

    say("\n[3/3] 自查汇总 ...")
    float_ref_max = max(r["mean_vs_sum_rel_max"] for r in rows)

    # 口径的常量类字段
    out["meta_effective_bits"] = np.array([EFFECTIVE_BITS])
    out["meta_container_bits"] = np.array([16])
    out["meta_shift"] = np.array([FULLSCALE_SHIFT])
    out["meta_code_max"] = np.array([CODE_MAX])
    out["meta_fullscale_max"] = np.array([FULLSCALE_MAX])
    out["meta_block"] = np.array([BLOCK])
    out["meta_frames"] = np.array([N_FRAMES])
    out["meta_marks"] = np.array(MARKS)
    out["meta_names"] = np.array(["ramp", "mix", "sat"])
    out["meta_note"] = np.array([
        "packed   = 12 bit 右对齐，容器最大 4095",
        "full     = packed 左移 4 位，容器最大 65520",
        "期望值全部为整数，判据为逐值相等（S0-3 原则 P1）",
        "g1g2 = G1+G2 的和，未做双绿平均（原则 P2）",
        "diff 为「下一行减上一行」，对块和图",
        "ref_float_* 仅为量相对误差的浮点对照，不作逐值判据",
    ])

    npz_path = os.path.join(VEC_DIR, "x6_x7_bitdepth.npz")
    np.savez_compressed(npz_path, **out)
    say(f"\n  golden npz -> {npz_path}（{len(out)} 个字段）")

    manifest_lines = ["=" * 78,
                      "x6_x7_bitdepth.npz（由 src/model/scripts/make_x6_x7_vectors.py 生成）",
                      "=" * 78]
    nbytes = export_bins(out, manifest_lines)
    manifest_lines.append(f"\n合计导出 {nbytes/1024:.1f} KB -> src/hardware/hwdata/")
    with io.open(os.path.join(HWDATA_DIR, "manifest_x6_x7.txt"), "w",
                 encoding="utf-8") as fh:
        fh.write("\n".join(manifest_lines))
    say(f"  裸二进制 -> src/hardware/hwdata/（{nbytes/1024:.1f} KB）")

    # ---------------------------------------------------------- 位宽数字
    numbers = {
        "effective_bits": EFFECTIVE_BITS,
        "container_bits": 16,
        "shift": FULLSCALE_SHIFT,
        "code_max": CODE_MAX,
        "fullscale_max": FULLSCALE_MAX,
        "block": BLOCK,
        "frames": N_FRAMES,
        "budgets": [budget(p, BLOCK, f) for p in ("packed", "full")
                    for f in (1, 4, 8)],
        "worst_case_magnitudes": {
            "packed": {
                "block_sum_max": CODE_MAX * BLOCK * BLOCK,
                "acc4": CODE_MAX * BLOCK * BLOCK * 4,
                "acc8": CODE_MAX * BLOCK * BLOCK * 8,
            },
            "full": {
                "block_sum_max": FULLSCALE_MAX * BLOCK * BLOCK,
                "acc4": FULLSCALE_MAX * BLOCK * BLOCK * 4,
                "acc8": FULLSCALE_MAX * BLOCK * BLOCK * 8,
            },
        },
        "measured_single_frame": rows,
        "measured_frames": {f"{k[0]}_acc{k[1]}": v for k, v in frame_summary.items()},
        "mean_vs_sum_rel_max": float_ref_max,
    }
    for tag in ("packed", "full"):
        m = numbers["worst_case_magnitudes"][tag]
        m["block_sum_bits_needed"] = bits_unsigned(m["block_sum_max"])
        m["acc4_bits_unsigned"] = bits_unsigned(m["acc4"])
        m["acc8_bits_unsigned"] = bits_unsigned(m["acc8"])
        m["diff4_bits_signed_needed"] = bits_signed(-m["acc4"])
        m["diff8_bits_signed_needed"] = bits_signed(-m["acc8"])

    json_path = os.path.join(RESULT_DIR, "x6_x7_bitdepth_numbers.json")
    with io.open(json_path, "w", encoding="utf-8") as fh:
        fh.write(json.dumps(numbers, indent=2, ensure_ascii=False))
    say(f"  数字 -> {json_path}")

    # ---------------------------------------------------------- 打印摘要
    say("\n  [口径 x 帧数] 位宽预算（块 10，冻结声明：块和 20/23、差分 24）")
    say("    口径        帧数  源   块和(最小)  块和(声明)  累积(最小)  累积(声明)  "
        "差分需  差分声明  够否")
    for b in numbers["budgets"]:
        say(f"    {b['packing']:<10s}  {b['frames']:>3d}  {b['src_bits']:>3d}  "
            f"{b['block_sum_min']:>8d}  {b['block_sum_declared']:>9d}  "
            f"{b['frame_acc_min']:>8d}  {b['frame_acc_declared']:>9d}  "
            f"{b['diff_needed']:>5d}  {b['diff_declared']:>7d}  "
            f"{'OK' if b['diff_fits'] else '差 %d bit' % (-b['headroom_bits'])}")

    say("\n  最坏情形量级（mix 用例：上半幅满量程 / 下半幅零）")
    for tag in ("packed", "full"):
        m = numbers["worst_case_magnitudes"][tag]
        say(f"    {tag:<8s} 块和最大 {m['block_sum_max']:>10d}（{m['block_sum_bits_needed']} bit）"
            f"  4 帧累积 {m['acc4']:>10d}（{m['acc4_bits_unsigned']} bit）"
            f"  8 帧累积 {m['acc8']:>10d}（{m['acc8_bits_unsigned']} bit）")
        say(f"             差分 4 帧需 {m['diff4_bits_signed_needed']} bit 有符号"
            f"，8 帧需 {m['diff8_bits_signed_needed']} bit 有符号")

    say("\n  实测（golden 自查通过）")
    say(f"    {'用例':<6s}{'口径':<10s}{'raw_max':>9s}{'plane_max':>11s}"
        f"{'sum_max':>11s}{'|diff|max':>11s}")
    for r in rows:
        say(f"    {r['case']:<6s}{r['panel']:<10s}{r['raw_max']:>9d}"
            f"{r['plane_max']:>11d}{r['sum_max']:>11d}{r['diff_absmax']:>11d}")

    say(f"\n  block_mean*面积 与精确整数块和的最大相对偏差：{float_ref_max:.3e}"
        f"（整数路径自身逐位相等，此处只反映浮点除法舍入）")

    with io.open(os.path.join(RESULT_DIR, "x6_x7_generation_log.txt"), "w",
                 encoding="utf-8") as fh:
        fh.write("\n".join(LOG))
    say("\n全部自查通过（含 fullscale == packed<<4、block_mean*area == block_sum）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
