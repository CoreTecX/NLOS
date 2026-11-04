# -*- coding: utf-8 -*-
"""X8 前置复验：遮挡矩形性在多个 t 上是否成立。

背景
----
S0-1 验证了「遮挡区是轴对齐矩形」这一性质，成立比例 1044/1044，但**只在真值
t = y_occluder / y_wall = 0.576699 上验证过**。

X1 接口冻结时发现一个此前没人写下来的事实：搜索变量 ll 的第二个分量就是遮挡物
平面的深度（`Occluder.y = ll[1]`），官方第 1 轮网格的 y ∈ [0.35, 0.65]，因此
**t 随候选变化**（约 0.34 ~ 0.63）。而 t 正是遮挡矩形的唯一决定参数。

于是 M2 的核心假设「每候选只需算一个矩形区间」必须在多个 t 上验过才能用。

本脚本做两件事，顺序刻意如此：

  V1 矩形性（只用参考实现，不引入本脚本的公式）
     对每个隐藏点，取 `forward.visibility` 给出的被遮挡墙点集合，检查它是否
     恰好等于「行区间 x 列区间的直积」。判据是纯组合的：
         集合元素个数 == 行跨度 x 列跨度
     若成立，则该集合必是直积（元素全在包围盒内，且个数等于包围盒容量）。

  V2 解析式能否重现（引入 S0-1 的闭式区间约束）
     把连续区间换算成墙网格的闭索引区间，组装可见性，与参考实现逐元素比对。
     这一步的差异只应来自 float32 的边界并列，因此报告的是**差异元素个数**
     而不是「通过 / 不通过」。

解析式的来源（S0-1 性质一）：
    t 与隐藏点 m、墙点 w 都无关（t = y_occ / y_wall），故交点
        q = (1-t)·m + t·w
    是仿射组合。遮挡判定 q ∈ 矩形（闭区间，见 `forward.visibility`）等价于对 w
    的两个**独立**区间约束：
        w_x ∈ [ (x0 - (1-t)m_x)/t , (x1 - (1-t)m_x)/t ]
        w_z ∈ [ (z0 - (1-t)m_z)/t , (z1 - (1-t)m_z)/t ]
    墙网格轴对齐且等距，所以每个隐藏点的遮挡区就是「一段 x 索引区间 x 一段 z
    索引区间」的直积。

用法
----
    python src/model/scripts/x8a_rect_at_varying_t.py \
        --data-root "D:\\学习\\26秋嵌入式\\论文数据集\\Ordinary-Camera-master\\Data"
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
_REPO = os.path.dirname(os.path.dirname(_SRC_MODEL))
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import forward as F  # noqa: E402
from penumbrasight_model import io_mat  # noqa: E402


# ---------------------------------------------------------------- V1：矩形性

def rect_check(V_occ: np.ndarray, n: int):
    """对每个隐藏点检查被遮挡墙点集合是否是「行区间 x 列区间」的直积。

    判据：集合元素个数 == (行跨度 x 列跨度)。
    元素必然落在自身包围盒内，故个数等于容量即等价于集合 = 直积。

    返回 (非矩形列数, 矩形面积统计, 空集列数, 逐列面积数组)。
    """
    N = V_occ.shape[1]
    area = np.zeros(N, dtype=np.int64)
    rspan = np.zeros(N, dtype=np.int64)
    cspan = np.zeros(N, dtype=np.int64)
    bad = []
    for i in range(N):
        idx = np.nonzero(V_occ[:, i])[0]
        k = idx.size
        area[i] = k
        if k == 0:
            rspan[i] = cspan[i] = 0
            continue
        r = idx // n
        c = idx % n
        rs = int(r.max()) - int(r.min()) + 1
        cs = int(c.max()) - int(c.min()) + 1
        rspan[i], cspan[i] = rs, cs
        if k != rs * cs:
            bad.append({"hidden_index": i, "n_occluded": k,
                        "row_span": rs, "col_span": cs,
                        "need": rs * cs})
    return len(bad), area, rspan, cspan, bad[:8]


# ---------------------------------------------------------------- V2：解析式

def interval_to_index(lo: np.ndarray, hi: np.ndarray, center0: float, cell: float,
                      n: int):
    """连续区间 [lo, hi] -> 单元中心网格上的闭索引区间，并**一起**裁剪到 [0, n-1]。

    关键：两个端点必须各自朝网格内部收（下端取 max(.,0)、上端取 min(.,n-1)），
    裁剪后若 lo > hi 则该区间与网格无交，遮挡集为空。

    反面做法（本次实测踩到的坑）：先把 lo、hi 各自 np.clip 到 [0, n-1]，再判断
    lo <= hi。区间完全落在网格左侧时 lo=-617、hi=-549，两侧都被夹到 0，
    于是空集被错误地变成一个 1 格宽的矩形，凭空多出遮挡。
    """
    j_lo = np.maximum(np.ceil((lo - center0) / cell - 1e-9).astype(np.int64), 0)
    j_hi = np.minimum(np.floor((hi - center0) / cell + 1e-9).astype(np.int64), n - 1)
    return j_lo, j_hi


def interval_position(lo: np.ndarray, hi: np.ndarray, center0: float, cell: float,
                      n: int):
    """统计区间相对网格的位置：完全在左 / 完全在右 / 完全在内 / 部分相交。"""
    g_lo = center0
    g_hi = center0 + (n - 1) * cell
    left = int(np.sum(hi < g_lo))
    right = int(np.sum(lo > g_hi))
    inside = int(np.sum((lo >= g_lo) & (hi <= g_hi)))
    partial = lo.size - left - right - inside
    return {"left": left, "right": right, "inside": inside, "partial": partial}


def analytic_occlusion_rect(setup: F.OpticalSetup):
    """由闭式区间约束算出被遮挡集合，返回 (V_occ, info)。"""
    n = setup.wall.n
    t = setup.occluder.y / setup.wall.y
    x0, x1 = setup.occluder.x_range
    z0, z1 = setup.occluder.z_range

    mx, mz = setup.monitor.flat_grid()
    N = mx.size

    lo_x = (x0 - (1.0 - t) * mx) / t
    hi_x = (x1 - (1.0 - t) * mx) / t
    lo_z = (z0 - (1.0 - t) * mz) / t
    hi_z = (z1 - (1.0 - t) * mz) / t

    cell = (setup.wall.x_max - setup.wall.x_min) / n
    cx0 = setup.wall.x_min + cell / 2.0
    cz0 = setup.wall.z_min + cell / 2.0

    jx_lo, jx_hi = interval_to_index(lo_x, hi_x, cx0, cell, n)
    jz_lo, jz_hi = interval_to_index(lo_z, hi_z, cz0, cell, n)

    empty = int(np.sum((jx_lo > jx_hi) | (jz_lo > jz_hi)))
    pos_x = interval_position(lo_x, hi_x, cx0, cell, n)
    pos_z = interval_position(lo_z, hi_z, cz0, cell, n)

    rows = np.arange(n)
    in_x = ((rows[:, None] >= jx_lo[None, :]) & (rows[:, None] <= jx_hi[None, :])
            & (jx_lo <= jx_hi)[None, :])
    in_z = ((rows[:, None] >= jz_lo[None, :]) & (rows[:, None] <= jz_hi[None, :])
            & (jz_lo <= jz_hi)[None, :])
    # 墙点展平 j = z * n + x，故先叠 z（行）再叠 x（列）
    occ = (in_z[:, None, :] & in_x[None, :, :]).reshape(n * n, N)

    span_x = np.where(jx_lo <= jx_hi, jx_hi - jx_lo + 1, 0)
    span_z = np.where(jz_lo <= jz_hi, jz_hi - jz_lo + 1, 0)
    info = {
        "t": float(t),
        "empty_rects": empty,
        "area_mean": float(np.mean(span_x * span_z)),
        "area_min": int(np.min(span_x * span_z)),
        "area_max": int(np.max(span_x * span_z)),
        "x_span_range": [int(span_x[span_x > 0].min()), int(span_x.max())],
        "z_span_range": [int(span_z[span_z > 0].min()), int(span_z.max())],
        "pos_x": pos_x,
        "pos_z": pos_z,
    }
    return occ, info


# ---------------------------------------------------------------- 主流程

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default=os.path.join(_REPO, "sim", "results"))
    args = ap.parse_args()

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    setup0 = F.load_setup(cfg)
    n = setup0.wall.n
    truth = np.asarray(setup0.occluder.ll, dtype=np.float64)[:3]

    print("[info] 墙网格 n=%d，隐藏场景 %d 点，遮挡物尺寸 %s m"
          % (n, setup0.monitor.n_pixels,
             np.round(np.asarray(setup0.occluder.size), 5).tolist()))
    print("[info] 真值 ll = %s，真值 t = %.6f"
          % (np.round(truth, 5).tolist(), truth[1] / setup0.wall.y))

    y_search = np.asarray(io_mat.load_mat(cfg)["yhatvals"],
                          dtype=np.float64).reshape(-1)
    y_lo, y_hi = float(y_search.min()), float(y_search.max())
    print("[info] 官方第 1 轮 y 范围 [%.4f, %.4f] -> t ∈ [%.6f, %.6f]"
          % (y_lo, y_hi, y_lo / setup0.wall.y, y_hi / setup0.wall.y), flush=True)

    cases = [
        ("y 下界（矩形最大）", y_lo),
        ("y = 0.50", 0.50),
        ("y 真值 0.594", float(truth[1])),
        ("y = 0.62", 0.62),
        ("y 上界（矩形最小）", y_hi),
    ]

    rows = []
    for label, y in cases:
        ll = truth.copy()
        ll[1] = y
        s = setup0.with_occluder_ll(ll)
        t0 = time.time()

        V_ref = F.visibility(s)
        occ_ref = ~V_ref                       # True = 被遮挡
        V_rect, info = analytic_occlusion_rect(s)

        n_bad, area, rspan, cspan, bad_detail = rect_check(occ_ref, n)
        N = occ_ref.shape[1]
        diff = int(np.count_nonzero(occ_ref != V_rect))
        col_bad = int(np.count_nonzero((occ_ref != V_rect).any(axis=0)))
        frac = area / float(n * n) * 100.0

        rows.append({
            "case": label, "y": float(y), "t": info["t"],
            "n_hidden": N, "non_rectangular_cols": n_bad,
            "rect_area_mean": float(area.mean()),
            "rect_area_min": int(area.min()), "rect_area_max": int(area.max()),
            "rect_area_max_pct": float(frac.max()),
            "empty_cols": int(np.sum(area == 0)),
            "x_span_range": [int(rspan[rspan > 0].min()), int(rspan.max())],
            "z_span_range": [int(cspan[cspan > 0].min()), int(cspan.max())],
            "analytic_vs_ref_diff_elements": diff,
            "analytic_vs_ref_diff_cols": col_bad,
            "analytic_interval_pos_x": info["pos_x"],
            "analytic_interval_pos_z": info["pos_z"],
            "mismatch_detail": bad_detail,
            "elapsed_s": time.time() - t0,
        })

        print("\n=== %s：y=%.4f  t=%.6f ===" % (label, y, info["t"]))
        print("  V1 矩形性：非矩形隐藏点 %d / %d  %s"
              % (n_bad, N, "OK" if n_bad == 0 else "BAD"))
        print("     遮挡面积 均值 %.1f，最小 %d，最大 %d（占墙点 %.2f%%）"
              % (area.mean(), area.min(), area.max(), frac.max()))
        print("     索引跨度 x %d~%d，z %d~%d；空集 %d 个"
              % (rows[-1]["x_span_range"][0], rows[-1]["x_span_range"][1],
                 rows[-1]["z_span_range"][0], rows[-1]["z_span_range"][1],
                 rows[-1]["empty_cols"]))
        print("     区间位置（x 轴）左 %d 右 %d 内 %d 部分 %d"
              % (info["pos_x"]["left"], info["pos_x"]["right"],
                 info["pos_x"]["inside"], info["pos_x"]["partial"]))
        print("  V2 解析式 vs 参考：差异元素 %d，涉及 %d 列"
              % (diff, col_bad))
        if bad_detail:
            print("     非矩形明细：%s" % json.dumps(bad_detail, ensure_ascii=False))
        sys.stdout.flush()

    all_rect = all(r["non_rectangular_cols"] == 0 for r in rows)
    worst = max(rows, key=lambda r: r["rect_area_max_pct"])
    total_diff = sum(r["analytic_vs_ref_diff_elements"] for r in rows)
    n_hidden = rows[0]["n_hidden"]
    total_elems = n * n * n_hidden * len(rows)

    print("\n" + "=" * 66)
    print("V1 结论：%s" % ("矩形性对搜索范围内的任意 t 成立"
                           "（%d 个隐藏点 x %d 个 t = %d 个组合全部通过）"
                           % (n_hidden, len(rows), n_hidden * len(rows))
                           if all_rect else "存在非矩形隐藏点，M2 假设不成立"))
    print("最坏情形：%s，单点遮挡面积最大 %d（占墙点 %.2f%%）"
          % (worst["case"], worst["rect_area_max"], worst["rect_area_max_pct"]))
    print("V2 结论：解析式与参考实现差异元素合计 %d / %d（%.2e）"
          % (total_diff, total_elems, total_diff / float(total_elems)))
    print("t 跨度：%.4f ~ %.4f" % (min(r["t"] for r in rows), max(r["t"] for r in rows)))
    print("空集最多的情形：%s，%d / %d 个隐藏点无被遮挡墙点"
          % (max(rows, key=lambda r: r["empty_cols"])["case"],
             max(r["empty_cols"] for r in rows), n_hidden))

    os.makedirs(args.out_dir, exist_ok=True)
    with io.open(os.path.join(args.out_dir, "x8a_rect_at_varying_t.json"),
                 "w", encoding="utf-8") as f:
        json.dump({"all_rectangular": all_rect,
                   "truth_ll": truth.tolist(),
                   "y_search_range": [y_lo, y_hi],
                   "t_search_range": [y_lo / setup0.wall.y, y_hi / setup0.wall.y],
                   "cases": rows}, f, ensure_ascii=False, indent=2)
    print("[done] 明细写入 %s/x8a_rect_at_varying_t.json" % args.out_dir)


if __name__ == "__main__":
    main()
