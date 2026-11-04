# -*- coding: utf-8 -*-
"""x23_gain_representation.py —— 增益表示：Q16.16 小数 vs 2 的幂指数

背景
-------------------------------------------------------------------------------
定点层的 U/V 用「每候选独立 scale」（方案 §6.3），增益字段是 Q16.16。
Q16.16 的分辨率是 1.53e-5，而真实 B 档模型的 scale 只有 1e-6~1e-5 ——
`gain_q16` 会取整到 0。判据见 `src/fpga/hls/lr_operator/test_lr_operator.cpp`
的 L7 段：`gain_q16 >= 500` 才满足增益相对误差 <= 1e-3。

本脚本在**真实 B 档几何**上并排量两种表示：
    旧：scale = max|x| / 32767          （连续，但写进 Q16.16 会被舍入）
    新：scale = 2^ceil(log2(max|x|/32767))（精确，代价是码字动态范围）

不预登记结论 —— 数字说话。
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.abspath(os.path.join(_HERE, os.pardir))
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from penumbrasight_model import build_model as BM      # noqa: E402
from penumbrasight_model import forward_loc as FL      # noqa: E402
from penumbrasight_model import quantize as Q          # noqa: E402

Q16_FRAC = 16
Q16_LSB = 2.0 ** -Q16_FRAC
L7_GAIN_Q16_MIN = 500.0          # test_lr_operator.cpp L7：1e-3 判据
L7_GAIN_Q16_TIGHT = 5000.0       # L7：1e-4 判据
RANK = 8                         # 定位档用的秩（方案 §5.1 的 K=8）


def q16_gain(scale: float) -> int:
    """旧表示：scale 写进 Q16.16 后的整数增益。"""
    return int(round(scale / Q16_LSB))


def q16_rel_err_bound(gain: int) -> float:
    """Q16.16 增益取整带来的**相对**误差上界 = 0.5 LSB / gain。

    **这个误差是相对误差，不是 LSB 有界的** —— 所以不能靠加宽 MAC 解决。
    gain 为 0 时上界无意义（模型直接垮掉），返回 inf。
    """
    return float("inf") if gain <= 0 else (0.5 * Q16_LSB) / (gain * Q16_LSB)


def code_usage(codes: np.ndarray, hi: int) -> float:
    """码字动态范围使用率：max|code| / hi。越接近 1 越好。"""
    return float(np.max(np.abs(codes))) / hi if codes.size else 0.0


def main() -> int:
    st = FL.b_loc_setup()
    ds = 0
    npx = FL.num_pixels_for(st, ds)
    spec, _ = FL.b_candidate_grid()
    grid = BM._candidate_grid(spec)
    plan = FL.make_plan(st, 2, downsamp_factor=ds, mz_mirrored=False)

    print("几何     : B 档 ds=%d 墙 %dx%d 差分后 M=%d 隐藏 N=%d"
          % (ds, npx, npx, npx * (npx - 1), st.num_blocks[0] * st.num_blocks[1]))
    print("秩       : R = %d（定位档）" % RANK)
    print("判据     : gain_q16 >= %.0f 满足 1e-3；>= %.0f 满足 1e-4"
          % (L7_GAIN_Q16_MIN, L7_GAIN_Q16_TIGHT))
    print()

    rows = []
    # 取 4 个候选：中心 + 三个角，覆盖面比连号候选广
    picks = [0, len(grid) // 4, len(grid) // 2, len(grid) - 1]
    for k in picks:
        _, _, _, ll = grid[k]
        A = FL.build_A_from_plan(plan, ll)
        B = BM.apply_row_diff(A, npx, axis=0)
        U, S_norm, V, s_max = BM.truncated_svd_via_gram(B, RANK)

        row = dict(cand=k, xyz=[round(v, 4) for v in ll], s_max=float(s_max))
        for name, arr in (("U", U), ("V", V)):
            hi = (1 << 15) - 1
            # ---- 旧：连续自适应 scale -> Q16.16 ----
            fo = Q.quantize_symmetric(arr, 16, int_bits=1)
            g_old = q16_gain(fo.scale)
            e_old = q16_rel_err_bound(g_old)
            # ---- 新：2 的幂 scale ----
            fn = Q.quantize_symmetric_pow2(arr, 16, int_bits=1)
            e_new = Q.scale_to_exponent(fn.scale)
            g_new = q16_gain(fn.scale)          # 仅作对照（新方案不写这个字段）
            row[name] = dict(
                old_scale=fo.scale, new_scale=fn.scale,
                old_gain_q16=g_old, new_exp=e_new, new_gain_q16_if_old_field=g_new,
                old_rel_err=e_old, new_rel_err=0.0,
                old_usage=code_usage(fo.codes, hi), new_usage=code_usage(fn.codes, hi),
                exp_fits_int8=Q.gain_exponent_range_ok(e_new, width=8),
            )
        # S 是归一化奇异值，Q2.14，scale 恒为 2^-14（本来就是幂次）
        fs = Q.quantize_singular_values(S_norm, 16)
        row["S"] = dict(old_scale=fs.scale, new_exp=Q.scale_to_exponent(fs.scale),
                        is_pow2=abs(math.log2(fs.scale) - round(math.log2(fs.scale))) < 1e-9)
        rows.append(row)

    # ---------------------------------------------------------------- 打印
    print("%-5s %-3s %13s %13s %8s %11s %10s %10s" %
          ("候选", "量", "旧 scale", "新 scale", "新指数", "旧增益误差", "旧码用率", "新码用率"))
    for r in rows:
        for name in ("U", "V"):
            d = r[name]
            print("%-5d %-3s %13.5g %13.5g %8d %11.3g %10.4f %10.4f" %
                  (r["cand"], name, d["old_scale"], d["new_scale"], d["new_exp"],
                   d["old_rel_err"], d["old_usage"], d["new_usage"]))
    print()

    # ---------------------------------------------------------------- 判定
    old_gains = [r[n]["old_gain_q16"] for r in rows for n in ("U", "V")]
    new_exps = [r[n]["new_exp"] for r in rows for n in ("U", "V")]
    old_us = [r[n]["old_usage"] for r in rows for n in ("U", "V")]
    new_us = [r[n]["new_usage"] for r in rows for n in ("U", "V")]

    print("=== 汇总（%d 个量：%d 候选 x {U,V}）===" % (len(old_gains), len(rows)))
    print("  旧表示 gain_q16       : min=%d  max=%d" % (min(old_gains), max(old_gains)))
    print("  旧表示增益相对误差上界: min=%.3g  max=%.3g"
          % (min(r[n]["old_rel_err"] for r in rows for n in ("U", "V")),
             max(r[n]["old_rel_err"] for r in rows for n in ("U", "V"))))
    print("  取整到 0 的个数       : %d / %d" % (sum(1 for g in old_gains if g == 0), len(old_gains)))
    print("  新表示指数范围        : [%d, %d]" % (min(new_exps), max(new_exps)))
    print("  新表示指数是否装进 int8: %s" % all(r[n]["exp_fits_int8"] for r in rows for n in ("U", "V")))
    print("  新表示增益误差        : 恒为 0（尾数=1 精确）")
    print("  码字使用率代价        : 旧 min=%.4f  新 min=%.4f  （降 %.4f，最多 1 bit）"
          % (min(old_us), min(new_us), min(old_us) - min(new_us)))
    print()

    ok_old = min(old_gains) >= L7_GAIN_Q16_MIN
    ok_new = all(r[n]["exp_fits_int8"] for r in rows for n in ("U", "V"))
    print("=== L7 判据 ===")
    print("  旧表示通过？ %s（最小 gain_q16 = %d，需 >= %.0f，差 %.0f 倍）"
          % ("是" if ok_old else "**否**", min(old_gains), L7_GAIN_Q16_MIN,
             L7_GAIN_Q16_MIN / max(min(old_gains), 1e-9)))
    print("  新表示通过？ %s（增益误差恒 0，指数装得进 int8）" % ("是" if ok_new else "**否**"))
    print()

    out = os.path.join(_HERE, os.pardir, "..", "..", "sim", "results",
                       "x23_gain_representation.json")
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(dict(rank=RANK, ds=ds, npx=npx, rows=rows,
                       old_gain_min=min(old_gains), old_gain_max=max(old_gains),
                       old_all_zero=all(g == 0 for g in old_gains),
                       new_exp_min=min(new_exps), new_exp_max=max(new_exps),
                       new_exp_int8_ok=ok_new,
                       old_passes_l7=ok_old, new_passes_l7=ok_new,
                       l7_gain_q16_min=L7_GAIN_Q16_MIN), f,
                  ensure_ascii=False, indent=2)
    print("写出：%s" % out)
    return 0 if (ok_new and not ok_old) else 1


if __name__ == "__main__":
    sys.exit(main())
