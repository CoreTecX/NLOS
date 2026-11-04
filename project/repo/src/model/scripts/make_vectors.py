# -*- coding: utf-8 -*-
"""S0-3 测试向量生成。

产出的向量用于甲（M1 预处理 RTL）与乙（M3 矩阵组装 HLS）的逐值对照。
设计原则：

  P1. **判据只用整数**。M1 的 RTL 不做除法——它输出「块累加和」与「差分」，
      二者都是整数，因此判据可以是逐值相等（不是误差容限）。
      除法与归一化统一推到 M3 输入端，那里本来就要做定点缩放。
  P2. **双绿通道不做中间舍入**。G1 与 G2 以「和」的形式进入分块累加
      （即 G_sum = Σ(G1+G2)），最后统一除以 2*block^2。
      这样全程无 0.5 的舍入，位宽只多 1 bit。
  P3. **每档向量都自带期望值**，且期望值由本脚本用参考实现算出，
      不依赖任何硬件工具。

四档向量：

  v1_block_small.npz    单块级（20x20 RAW -> 1 个输出），含合成斜坡、饱和、全零、官方块
  v2_frame_full.npz     单帧全图（1920x1920 RAW -> 96x96 输出），官方数据
  v3_accum_frames.npz   多帧累积（16 帧 x 200x200 官方区域），含 1/2/4/8/16 帧的中间结果
  v4_edge_cases.npz     合成边界：跨行分块陷阱、暗电平

用法：
  python src/model/scripts/make_vectors.py --data-root "<官方 Data 目录>"
"""

from __future__ import annotations

import argparse
import io
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import PROFILE_B, io_mat  # noqa: E402
from penumbrasight_model.preprocess import (  # noqa: E402
    BAYER_PATTERNS,
    center_crop,
)

LOG: list = []


def say(s: str = "") -> None:
    LOG.append(s)
    print(s, flush=True)


# ---------------------------------------------------------------- 基础算子


def deinterleave_split(raw: np.ndarray, pattern: str = "RGGB"):
    """按 Bayer 相位拆成四路，不做双绿平均。

    返回 (R, G1, G2, B)，均为 uint32（便于后续整数累加不溢出）。
    """
    if pattern not in BAYER_PATTERNS:
        raise ValueError(f"未知 Bayer 相位 {pattern}")
    planes = {}
    for r, c, ch in BAYER_PATTERNS[pattern]:
        planes[ch] = np.asarray(raw[r::2, c::2], dtype=np.uint32)
    return planes["R"], planes["G1"], planes["G2"], planes["B"]


def block_sum_u32(plane: np.ndarray, block: int) -> np.ndarray:
    """分块求和，返回 uint32。按行切块（reshape 视图），禁止展平后分组。"""
    h, w = plane.shape
    if h % block or w % block:
        raise ValueError(f"平面 {plane.shape} 不能被 block={block} 整除")
    return np.asarray(
        np.asarray(plane, dtype=np.uint32).reshape(
            h // block, block, w // block, block).sum(axis=(1, 3)),
        dtype=np.uint32)


def as_u16(a: np.ndarray) -> np.ndarray:
    """安全转 uint16：要求原数组已是 [0,65535] 内的整数值。"""
    arr = np.asarray(a)
    if not np.allclose(arr, np.round(arr)):
        raise ValueError("源数据不是整数值，转 uint16 会引入舍入，破坏逐值判据")
    if arr.min() < 0 or arr.max() > 65535:
        raise ValueError(f"源数据超出 uint16 范围 [{arr.min()}, {arr.max()}]")
    return np.round(arr).astype(np.uint16)


# ---------------------------------------------------------------- v1 单块


def make_v1(out_dir: str, data_root: str) -> dict:
    """单块级向量：20x20 RAW -> 10x10 解交织平面 -> 1 个块和。"""
    block = 10
    info: dict = {"cases": []}
    arrays: dict = {}

    # --- 合成斜坡：能在解交织阶段暴露相位错误 ---
    n = 2 * block
    ramp = np.arange(n * n, dtype=np.uint16).reshape(n, n)
    # --- 饱和与全零 ---
    sat = np.full((n, n), 65535, dtype=np.uint16)
    zero = np.zeros((n, n), dtype=np.uint16)

    # --- 官方块：取自官方 Mush 序列第 1 帧的左上角对齐区域 ---
    #  注意：image_test_mushroom.mat 是 35 帧的累积图（量级 2.29e6），不是单帧 RAW。
    #  M1 的输入是单帧，故这里取序列中的单帧；累积路径由 v3 覆盖。
    tp = os.path.join(data_root, "TestPosChair")
    paths = io_mat.list_frame_series(os.path.join(tp, "Mush"))
    raw_full = io_mat.load_image(paths[0])
    off_u16 = as_u16(np.asarray(raw_full, dtype=np.float64))
    if min(off_u16.shape) < n:
        raise ValueError(f"单帧 RAW 太小：{off_u16.shape}")
    off_blk = off_u16[:n, :n].copy()
    del raw_full, off_u16

    cases = [("ramp", ramp, "合成斜坡（暴露 Bayer 相位错误）"),
             ("sat", sat, "全 65535 饱和（暴露块和位宽不足）"),
             ("zero", zero, "全零（暴露非零暗电平）"),
             ("official", off_blk, "官方 mushroom 累积图左上角 20x20")]

    for name, raw, desc in cases:
        r, g1, g2, b = deinterleave_split(raw)
        arrays[f"{name}_raw"] = raw
        arrays[f"{name}_r"] = r.astype(np.uint16)
        arrays[f"{name}_g1"] = g1.astype(np.uint16)
        arrays[f"{name}_g2"] = g2.astype(np.uint16)
        arrays[f"{name}_b"] = b.astype(np.uint16)
        arrays[f"{name}_sum_r"] = block_sum_u32(r, block)
        arrays[f"{name}_sum_g1g2"] = block_sum_u32(g1 + g2, block)
        arrays[f"{name}_sum_b"] = block_sum_u32(b, block)
        info["cases"].append({
            "name": name, "desc": desc,
            "sum_r": int(arrays[f"{name}_sum_r"].ravel()[0]),
            "sum_g1g2": int(arrays[f"{name}_sum_g1g2"].ravel()[0]),
            "sum_b": int(arrays[f"{name}_sum_b"].ravel()[0]),
        })

    arrays["meta_names"] = np.array([c[0] for c in cases])
    arrays["meta_block"] = np.array([block])
    arrays["meta_note"] = np.array([
        "sum_g1g2 = sum(G1+G2)，未做双绿平均，避免 0.5 舍入",
        "期望值均为整数，判据为逐值相等",
    ])
    path = os.path.join(out_dir, "v1_block_small.npz")
    np.savez_compressed(path, **arrays)
    info["path"] = path
    return info


# ---------------------------------------------------------------- v2 单帧全图


def make_v2(out_dir: str, data_root: str) -> dict:
    """单帧全图：1920x1920 RAW -> 96x96 块和与差分（profile_b）。"""
    block = PROFILE_B.block
    tp = os.path.join(data_root, "TestPosChair")
    paths = io_mat.list_frame_series(os.path.join(tp, "Mush"))
    src = paths[0]
    raw_full = io_mat.load_image(src)
    cropped = center_crop(np.asarray(raw_full, dtype=np.float64), PROFILE_B.crop)
    del raw_full
    raw = as_u16(cropped)
    del cropped

    r, g1, g2, b = deinterleave_split(raw)
    bs_r = block_sum_u32(r, block)
    bs_g = block_sum_u32(g1 + g2, block)
    bs_b = block_sum_u32(b, block)
    del r, g1, g2, b

    rgb = np.dstack([bs_r, bs_g, bs_b]).astype(np.uint32)
    diff = (rgb[1:, :, :].astype(np.int64) - rgb[:-1, :, :].astype(np.int64)).astype(np.int32)

    # 官方口径的块均值，作为浮点参考（不用于逐值判据）
    ref = rgb.astype(np.float64) / float(block * block)
    ref[:, :, 1] /= 2.0

    np.savez_compressed(
        os.path.join(out_dir, "v2_frame_full.npz"),
        meta_profile=np.array([PROFILE_B.name]),
        meta_block=np.array([block]),
        meta_source=np.array([os.path.basename(src)]),
        meta_note=np.array([
            "exp_blocksum_* 为块累加和（整数，逐值判据）",
            "exp_diff_* 为块和图的行方向差分（整数，逐值判据）",
            "ref_meas_float 为官方口径块均值（浮点，仅供参考，不作判据）",
        ]),
        in_raw=raw,
        exp_blocksum_r=bs_r.astype(np.uint32),
        exp_blocksum_g1g2=bs_g.astype(np.uint32),
        exp_blocksum_b=bs_b.astype(np.uint32),
        exp_diff_r=diff[:, :, 0],
        exp_diff_g=diff[:, :, 1],
        exp_diff_b=diff[:, :, 2],
        ref_meas_float=ref.astype(np.float32),
    )
    return {
        "path": os.path.join(out_dir, "v2_frame_full.npz"),
        "raw_shape": raw.shape,
        "blocksum_shape": rgb.shape,
        "raw_max": int(raw.max()),
        "bs_max": int(rgb.max()),
        "bs_bit": int(np.ceil(np.log2(max(int(rgb.max()), 1)))),
        "diff_absmax": int(np.abs(diff).max()),
        "diff_bit": int(np.ceil(np.log2(max(int(np.abs(diff).max()), 1)))) + 1,
    }


# ---------------------------------------------------------------- v3 多帧累积


def make_v3(out_dir: str, data_root: str, n_frames: int = 32,
            region: int = 200) -> dict:
    """多帧累积：官方 Mush 序列前 n_frames 帧的居中 region x region 区域。"""
    block = 10
    tp = os.path.join(data_root, "TestPosChair")
    mush_dir = os.path.join(tp, "Mush")
    paths = io_mat.list_frame_series(mush_dir)
    n_use = min(n_frames, len(paths))
    paths = paths[:n_use]

    full = io_mat.load_image(paths[0]).shape[0]
    off = (full - region) // 2
    sl = (slice(off, off + region), slice(off, off + region))

    frames = np.zeros((n_use, region, region), dtype=np.uint32)
    for i, p in enumerate(paths):
        img = np.asarray(io_mat.load_image(p), dtype=np.float64)
        frames[i] = as_u16(img[sl]).astype(np.uint32)
    del img
    if frames.max() > 65535:
        raise ValueError("单帧像素超出 uint16")

    marks = [m for m in (1, 2, 4, 8, 16, 32, 50) if m <= n_use]
    acc = np.zeros((region, region), dtype=np.uint32)
    arrays: dict = {}
    rows = []
    prev = 0
    for m in marks:
        acc += frames[prev:m].sum(axis=0).astype(np.uint32)
        prev = m
        r, g1, g2, b = deinterleave_split(acc.astype(np.uint32))
        bs_r = block_sum_u32(r, block)
        bs_g = block_sum_u32(g1 + g2, block)
        bs_b = block_sum_u32(b, block)
        arrays[f"exp_acc{m}_blocksum_r"] = bs_r
        arrays[f"exp_acc{m}_blocksum_g1g2"] = bs_g
        arrays[f"exp_acc{m}_blocksum_b"] = bs_b
        rgb = np.dstack([bs_r, bs_g, bs_b])
        df = (rgb[1:, :, :].astype(np.int64) - rgb[:-1, :, :].astype(np.int64))
        arrays[f"exp_acc{m}_diff_r"] = df[:, :, 0].astype(np.int32)
        arrays[f"exp_acc{m}_diff_g"] = df[:, :, 1].astype(np.int32)
        arrays[f"exp_acc{m}_diff_b"] = df[:, :, 2].astype(np.int32)
        rows.append({
            "frames": m,
            "acc_max": int(acc.max()),
            "acc_bit": int(np.ceil(np.log2(max(int(acc.max()), 1)))),
            "blocksum_max": int(rgb.max()),
            "blocksum_bit": int(np.ceil(np.log2(max(int(rgb.max()), 1)))),
            "diff_absmax": int(np.abs(df).max()),
        })

    np.savez_compressed(
        os.path.join(out_dir, "v3_accum_frames.npz"),
        meta_frames=np.array([n_use]),
        meta_region=np.array([region]),
        meta_offset=np.array([off]),
        meta_block=np.array([block]),
        meta_marks=np.array(marks),
        meta_note=np.array([
            f"取自官方 Mush 序列前 {n_use} 帧的居中 {region}x{region} 区域",
            "累积为直接求和，不除以帧数（规则 R3，已由黄金对照验证）",
            f"共给出 {len(marks)} 个中间累积档：{marks}",
        ]),
        in_raw_frames=frames.astype(np.uint16),
        **arrays,
    )
    return {
        "path": os.path.join(out_dir, "v3_accum_frames.npz"),
        "n_frames": n_use,
        "region": region,
        "rows": rows,
    }


# ---------------------------------------------------------------- v4 边界


def make_v4(out_dir: str) -> dict:
    """合成边界用例：跨行分块陷阱 + 暗电平。"""
    # --- 跨行分块陷阱 ---
    # 平面 15x15、block=5：行宽 15 是 block 的整数倍，但把平面展平后按 25 个
    # 元素分组会跨行。正确实现必须按 (行, 列) 二维切块。
    # 用一个行敏感图案（每行常数、逐行递增）让两种做法结果明显不同。
    w = 15
    blk = 5
    plane = np.repeat(np.arange(1, w + 1, dtype=np.uint32)[None, :], w, axis=0)
    correct = plane.reshape(w // blk, blk, w // blk, blk).sum(axis=(1, 3))
    flat = plane.reshape(-1)
    wrong_flat = flat.reshape((w * w) // (blk * blk), blk * blk).sum(axis=1)
    wrong = wrong_flat.reshape(w // blk, w // blk)

    # --- 暗电平 ---
    dark_raw = np.full((2 * blk * 2, 2 * blk * 2), 1000, dtype=np.uint16)
    dark_off = 1000
    dark_exp = np.zeros((blk * 2 // blk, blk * 2 // blk), dtype=np.uint32)

    np.savez_compressed(
        os.path.join(out_dir, "v4_edge_cases.npz"),
        meta_note=np.array([
            "crossrow_* 的输入是解交织后的平面，不是 RAW",
            "crossrow_correct 为按行切块的正确结果，crossrow_wrong 为展平分组的错误结果",
            "两者不同，实现若得到 wrong 即为跨行分块 bug",
        ]),
        crossrow_plane=plane.astype(np.uint16),
        crossrow_block=np.array([blk]),
        crossrow_correct=correct.astype(np.uint32),
        crossrow_wrong=wrong.astype(np.uint32),
        dark_raw=dark_raw,
        dark_offset=np.array([dark_off]),
        dark_exp=dark_exp,
    )
    return {
        "path": os.path.join(out_dir, "v4_edge_cases.npz"),
        "crossrow_correct": correct.tolist(),
        "crossrow_wrong": wrong.tolist(),
        "different": bool(not np.array_equal(correct, wrong)),
    }


# ---------------------------------------------------------------- README


README = """# 测试向量目录（S0-3）

本目录存放甲（M1 预处理 RTL）与乙（M3 矩阵组装 HLS）共用的激励数据。
生成脚本：`src/model/scripts/make_vectors.py`（可重复运行，覆盖本目录中的 `v*.npz`）。

## 一、两条设计原则（决定了判据的形式）

**P1 判据只用整数。** M1 的 RTL **不做除法**，输出「块累加和」与「行差分」，
二者都是整数，因此判据是**逐值相等**而不是误差容限。
除法与归一化统一推到 M3 的输入端（那里本来就要做定点缩放）。

**P2 双绿通道不做中间舍入。** G1 与 G2 以**和**的形式进入分块累加，
即 `G_sum = Σ(G1+G2)`，最后统一除以 `2*block^2`。
这样全程没有 0.5 的舍入，代价是位宽只多 1 bit。

## 二、向量清单

| 文件 | 档位 | 输入 | 期望值 | 用途 |
|---|---|---|---|---|
| `v1_block_small.npz` | 小 | 20x20 RAW（4 个用例） | 解交织四路 10x10 + 块和 | cocotb 逐值仿真，覆盖解交织/双绿/块累加 |
| `v2_frame_full.npz` | 中 | 1920x1920 RAW（官方） | 96x96 块和 + 95x96 差分 | 全图流水线与分块边界；数据量大，建议上板比对 |
| `v3_accum_frames.npz` | 完整 | 16 帧 x 200x200（官方） | 1/2/4/8/16 帧的各档块和与差分 | 帧累积与位宽增长 |
| `v4_edge_cases.npz` | 边界 | 合成平面 | 跨行分块的正确/错误对照 | 锁死分块索引逻辑 |

## 三、命名约定

- `in_*` 输入；`exp_*` 期望输出；`ref_*` 仅供参考、不作判据；`meta_*` 元信息
- `sum_r` / `sum_g1g2` / `sum_b`：三通道块累加**和**（G 通道是 G1+G2 的和）
- `blocksum_*`：块累加和；`diff_*`：块和图的行方向差分
- 所有期望值都是整数类型（uint16/uint32/int32），可直接做 `==` 比对

## 四、位宽（依据 S0-2 位宽表）

| 节点 | 类型 | 位宽 |
|---|---|---|
| RAW 像素 | uint16 | 16 |
| 解交织四路 | uint16 | 16 |
| 10x10 块和（单通道） | uint32 | 23 |
| G1+G2 的块和 | uint32 | 24 |
| 16 帧累积后的块和 | uint32 | 27 |
| 差分（块和相减） | int32 | 25 有符号 |

## 五、注意

- `profile_b_96x96.npz` 是 T0-2 阶段的产物，用 float32 存储、**不是逐值判据**，
  仅用于出图与量级核对。以本目录中的 `v*.npz` 为准。
- 官方数据集无开源许可，向量中的官方数据部分**不得再分发**。
"""


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--vec-dir", default="sim/vectors")
    ap.add_argument("--skip-v2", action="store_true",
                    help="跳过 v2（全图向量较大，生成较慢）")
    args = ap.parse_args()

    os.makedirs(args.vec_dir, exist_ok=True)
    t0 = time.time()

    say("=" * 78)
    say("S0-3 测试向量生成")
    say("=" * 78)

    say("\n[1/4] v1 单块级 ...")
    i1 = make_v1(args.vec_dir, args.data_root)
    for c in i1["cases"]:
        say(f"      {c['name']:>9s}  sum_r={c['sum_r']:>10d}  "
            f"sum_g1g2={c['sum_g1g2']:>10d}  sum_b={c['sum_b']:>10d}   {c['desc']}")

    say("\n[2/4] v2 单帧全图 ...")
    if args.skip_v2:
        i2 = {"skipped": True}
        say("      已跳过")
    else:
        i2 = make_v2(args.vec_dir, args.data_root)
        say(f"      输入 {i2['raw_shape']} max={i2['raw_max']}")
        say(f"      块和 {i2['blocksum_shape']} max={i2['bs_max']}"
            f"（{i2['bs_bit']} bit，S0-2 表定 23 bit）")
        say(f"      差分 max|·|={i2['diff_absmax']}（{i2['diff_bit']} bit 有符号）")

    say("\n[3/4] v3 多帧累积 ...")
    i3 = make_v3(args.vec_dir, args.data_root)
    say(f"      {i3['n_frames']} 帧，区域 {i3['region']}x{i3['region']}")
    say(f"      {'帧数':>4s} {'累积max':>12s} {'累积bit':>8s} "
        f"{'块和max':>12s} {'块和bit':>8s} {'差分max':>10s}")
    for r in i3["rows"]:
        say(f"      {r['frames']:>4d} {r['acc_max']:>12d} {r['acc_bit']:>8d} "
            f"{r['blocksum_max']:>12d} {r['blocksum_bit']:>8d} {r['diff_absmax']:>10d}")

    say("\n[4/4] v4 边界用例 ...")
    i4 = make_v4(args.vec_dir)
    say(f"      跨行分块：正确结果 {i4['crossrow_correct']}")
    say(f"                错误结果 {i4['crossrow_wrong']}")
    say(f"      两者不同 = {i4['different']}（必须为 True，否则陷阱无效）")

    with io.open(os.path.join(args.vec_dir, "README.md"), "w", encoding="utf-8") as fh:
        fh.write(README)
    say(f"\n[README] {os.path.join(args.vec_dir, 'README.md')}")

    with io.open(os.path.join(args.vec_dir, "generation_log.txt"), "w",
                 encoding="utf-8") as fh:
        fh.write("\n".join(LOG))
    say(f"总耗时 {time.time()-t0:.0f}s")


if __name__ == "__main__":
    main()
