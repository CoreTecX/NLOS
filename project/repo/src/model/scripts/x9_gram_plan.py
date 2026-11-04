# -*- coding: utf-8 -*-
"""X9 前置决策：权重怎么算、Gram 存多少位、要不要分块。

X9 的任务描述是「Gram 分 2 块累加（无 UltraRAM，32 bit 需 34.88 Mb > 32.1 Mb）」，
判据「误差 ≤ 1e-3，BRAM ≤ 30%」。但「分 2 块」是从**结果大小**倒推的结论，
动手前必须把四件事分开算清：

  A 权重怎么算      —— 预存二维表（R1/R2）还是实时算
  B Gram 存多少位   —— 逐位扫，看相对 Frobenius 误差与丢元素数
  C 分几块、怎么分  —— 峰值是「累加器 + 协处理器状态」，不是「结果大小」
  D 除法怎么办      —— 1/d2^2 是唯一必须的除法，LUT 插值精度够不够

**权威矩阵是 `F.build_transport_matrix(setup)`**，即 `localization.evaluate_candidate`
实际调用、以及 M3 要复现的那一个。X16 的量化检查用的是 `paper_A_va1.npy`（论文复现的
A），两者尺度不同，故本节重新在权威矩阵上做一遍，并在报告里标明差异。

用法
----
    python src/model/scripts/x9_gram_plan.py \
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
from penumbrasight_model import localization as L  # noqa: E402

BRAM_BITS = 32.1e6        # XCZU9EG，A2 资源预算


def rel_fro(a: np.ndarray, b: np.ndarray) -> float:
    """相对 Frobenius 误差 ||a - b|| / ||b||。"""
    den = float(np.linalg.norm(b))
    return float(np.linalg.norm(np.asarray(a) - np.asarray(b))) / den if den > 0 else 0.0


def rel_l2(a: np.ndarray, b: np.ndarray) -> float:
    den = float(np.linalg.norm(b))
    return float(np.linalg.norm(a - b)) / den if den > 0 else 0.0


def make_lut(nb: int, lo: float, hi: float):
    """1/d2^2 的 LUT + 线性插值闭包。"""
    grid = np.linspace(lo, hi, nb)
    vals = 1.0 / (grid ** 2)

    def f(d2: np.ndarray) -> np.ndarray:
        pos = (d2 - lo) / (hi - lo) * (nb - 1)
        i0 = np.clip(np.floor(pos).astype(np.int64), 0, nb - 2)
        fr = pos - i0
        return vals[i0] + fr * (vals[i0 + 1] - vals[i0])
    return f


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default=os.path.join(_REPO, "sim", "results"))
    args = ap.parse_args()

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    setup = F.load_setup(cfg)
    n = setup.wall.n
    N = setup.monitor.n_pixels
    d = io_mat.load_mat(cfg)
    y2d = np.asarray(d["meas_g"], dtype=np.float64)

    y_w = setup.wall.y - setup.monitor.y
    dA = setup.monitor.block_area
    C = y_w * y_w * dA
    print("[info] n=%d N=%d  y_w=%.6f  dA=%.6g  C=y_w^2*dA=%.6g" % (n, N, y_w, dA, C))

    wx, wz = setup.wall.grid()
    wall_x = wx.reshape(n, n)[0, :]
    wall_z = wz.reshape(n, n)[:, 0]
    mx, mz = setup.monitor.flat_grid()

    # ================================================================
    # A 权重的分解方式与 d2 的取值范围
    # ================================================================
    print("\n=== A 权重怎么算 ===")
    dx2_sep = (wall_x[:, None] - mx[None, :]) ** 2      # (n_x, N)
    dz2_sep = (wall_z[:, None] - mz[None, :]) ** 2      # (n_z, N)
    d2_all = (dz2_sep[:, None, :] + dx2_sep[None, :, :]
              + y_w * y_w).reshape(n * n, N)            # [z, x, i] -> j = z*n+x
    print("  d2 范围 [%.6f, %.6f]，y_w^2 = %.6f，动态范围仅 %.3f 倍"
          % (d2_all.min(), d2_all.max(), y_w * y_w, d2_all.max() / d2_all.min()))
    print("  dx2 范围 [%.3e, %.6f]   dz2 范围 [%.3e, %.6f]"
          % (dx2_sep.min(), dx2_sep.max(), dz2_sep.min(), dz2_sep.max()))

    # 权威矩阵
    A_ref = F.build_transport_matrix(setup)
    V = F.visibility(setup)
    A_mine = (C / (d2_all ** 2)) * V
    print("  d2 分离重建核对：max|d2_sep - d2_direct| = %.3e"
          % np.abs((dz2_sep[:, None, :] + dx2_sep[None, :, :]
                    + y_w * y_w).reshape(n * n, N) - d2_all).max())
    print("  权重公式核对：max|A_mine - A_ref| / max|A_ref| = %.3e"
          % (np.abs(A_mine - A_ref).max() / np.abs(A_ref).max()))
    del A_mine
    print("  A 范围 [%.4e, %.4e]  A 非零占比 %.2f%%"
          % (A_ref.min(), A_ref.max(), 100.0 * np.count_nonzero(A_ref) / A_ref.size))

    n_elem = n * n * N
    schemes = [
        ("R1 预存 dx2/dz2 两张二维表", 2 * n * N, 0),
        ("R2 低秩 p_k/q_k（R=3）", 3 * (n * n + N), 0),
        ("S 实时算（不存表）", 0, 2 * n_elem),
    ]
    print("\n  方案                          表项数        每候选额外乘/减")
    for name, tbl, ops in schemes:
        print("  %-28s %-13s %s" % (name, f"{tbl:,}", f"{ops:,}"))
    mac_gram = (n - 1) * n * N * (N + 1) // 2
    print("  Gram 本身是 %s MAC；实时算的代价上限 %.2f%%"
          % (f"{mac_gram:,}", 100.0 * 2 * n_elem / mac_gram))

    # ================================================================
    # D 除法：LUT + 线性插值
    # ================================================================
    print("\n=== D 1/d2^2 用 LUT+线性插值 ===")
    lo, hi = float(d2_all.min()), float(d2_all.max())
    inv4_exact = 1.0 / (d2_all ** 2)
    lut_rows = []
    for nb_lut in (128, 256, 512, 1024):
        approx = make_lut(nb_lut, lo, hi)(d2_all)
        err = np.abs(approx - inv4_exact) / inv4_exact
        lut_rows.append({"n_lut": nb_lut, "rel_err_max": float(err.max()),
                         "rel_err_rms": float(np.sqrt(np.mean(err ** 2))),
                         "bram_pct": 100.0 * nb_lut * 24 / BRAM_BITS})
        print("  LUT %5d 项（24 bit）: 相对误差 max %.3e  rms %.3e  占 BRAM %.3f%%"
              % (nb_lut, err.max(), np.sqrt(np.mean(err ** 2)),
                 100.0 * nb_lut * 24 / BRAM_BITS))
    del inv4_exact

    # ================================================================
    # 参考 Gram 与 A_d^T y
    # ================================================================
    print("\n=== 构建参考量（float64，权威矩阵）===")
    t0 = time.time()
    Ad_ref = L.apply_row_difference_A(A_ref, n)
    yd = L.apply_row_difference_img(y2d)
    aty_ref = Ad_ref.T @ yd
    G_ref = Ad_ref.T @ Ad_ref
    iu = np.triu_indices(N)
    G_tri = G_ref[iu].copy()
    print("  A_d %s  G %s  %.1f s" % (Ad_ref.shape, G_ref.shape, time.time() - t0))
    print("  A_d 范围 [%.4e, %.4e]" % (Ad_ref.min(), Ad_ref.max()))
    print("  A_d^T A_d 上三角 max|.| = %.6e   A_d^T y max|.| = %.6e"
          % (np.abs(G_tri).max(), np.abs(aty_ref).max()))
    print("  上三角 %s 个" % f"{G_tri.size:,}")
    nz = np.abs(G_tri[G_tri != 0])
    print("  |G_tri| max %.6e  min(非零) %.6e  动态范围 %.3e 倍"
          % (nz.max(), nz.min(), nz.max() / nz.min()))
    print("  注意：A_d^T y 是 %.3e 量级，A_d^T A_d 是 %.3e 量级，"
          "两者相差 %.1e 倍"
          % (np.abs(aty_ref).max(), nz.max(),
             np.abs(aty_ref).max() / nz.max()))
    del nz, G_ref, Ad_ref, A_ref, V

    gmax = float(np.abs(G_tri).max())

    # ================================================================
    # B Gram 位宽扫描
    # ================================================================
    print("\n=== B Gram 位宽（上三角定标量化）===")
    bit_rows = []
    for nb in range(14, 25):
        fbits = nb - 1 - int(np.ceil(np.log2(gmax)))       # 小数位
        scale = float(2 ** fbits)
        q = np.round(G_tri * scale) / scale
        rel = rel_fro(q, G_tri)
        lost = int(np.count_nonzero((q == 0) & (G_tri != 0)))
        bit_rows.append({"bits": nb, "frac_bits": fbits, "scale": scale,
                         "rel_fro": rel, "lost": lost,
                         "bram_pct": 100.0 * G_tri.size * nb / BRAM_BITS})
        print("  %2d bit（%2d 位小数）: 相对 Frobenius %.3e  丢元素 %5d (%.4f%%)"
              "  BRAM %5.2f%%"
              % (nb, fbits, rel, lost, 100.0 * lost / G_tri.size,
                 100.0 * G_tri.size * nb / BRAM_BITS))

    # ================================================================
    # 端到端定点模型：从权重到 Gram
    # ================================================================
    print("\n=== 端到端定点模型（权重 -> A_d -> Gram 上三角）===")
    lut512 = make_lut(512, lo, hi)
    V = F.visibility(setup)
    e2e = []

    def run_scheme(tag, weight_fn, gram_bits=None):
        w = weight_fn()
        w *= V
        Ad = L.apply_row_difference_A(w, n)
        del w
        aty = Ad.T @ yd
        G = Ad.T @ Ad
        del Ad
        g = G[iu]
        if gram_bits is not None:
            fb = gram_bits - 1 - int(np.ceil(np.log2(gmax)))
            sc = float(2 ** fb)
            g = np.round(g * sc) / sc
        rG = rel_fro(g, G_tri)
        rA = rel_l2(aty, aty_ref)
        e2e.append({"scheme": tag, "gram_bits": gram_bits,
                    "rel_fro_G_triu": rG, "rel_l2_aty": rA})
        print("  %-42s Gram %.3e   A_d^T y %.3e" % (tag, rG, rA))
        sys.stdout.flush()
        del G, aty

    run_scheme("P0 全 float64（上界）", lambda: (C / (d2_all ** 2)))
    run_scheme("P1 1/d2^2 走 512 项 LUT+线性插值",
               lambda: C * lut512(d2_all))
    run_scheme("P2 P1 + Gram 20 bit 量化",
               lambda: C * lut512(d2_all), gram_bits=20)
    run_scheme("P3 P1 + Gram 16 bit 量化",
               lambda: C * lut512(d2_all), gram_bits=16)
    # d2 在 24 bit 定点下算（模拟坐标 24 位、加减 24 位）
    d2_q = np.round(d2_all * 2 ** 21) / 2 ** 21
    run_scheme("P4 d2 定点化到 24 bit（位置 + 减法舍入）",
               lambda: C / (d2_q ** 2))
    del d2_q, V

    # ================================================================
    # C 分块数与峰值
    # ================================================================
    print("\n=== C 分块数与峰值 BRAM ===")
    contrib = N - np.arange(N)
    cum = np.cumsum(contrib)
    total = int(cum[-1])
    print("  上三角总数 %s；目标峰值 ≤ %.2f Mb（BRAM 的 30%%）"
          % (f"{total:,}", 0.30 * BRAM_BITS / 1e6))

    def plan(k: int):
        if k == 1:
            return {"blocks": 1, "cuts": [], "sizes": [total], "max_size": total}
        tg = np.linspace(0, total, k + 1)[1:-1]
        cuts = [int(np.searchsorted(cum, v)) + 1 for v in tg]
        b = [0] + cuts + [N]
        sizes = [int(cum[y - 1] - (cum[x - 1] if x > 0 else 0))
                 for x, y in zip(b[:-1], b[1:])]
        return {"blocks": k, "cuts": cuts, "sizes": sizes, "max_size": max(sizes)}

    print("\n  块数  切点(隐藏点编号)   各块条目数                          最大块")
    plans = {}
    for k in (1, 2, 3, 4):
        r = plan(k)
        plans[str(k)] = r
        print("   %d   %-20s %-34s %s" % (k, str(r["cuts"]),
              ", ".join(f"{s:,}" for s in r["sizes"]), f"{r['max_size']:,}"))

    print("\n  峰值构成（Gram 20 bit、坐标 24 bit、A_d 行缓冲 24 bit、LUT 24 bit）")
    lut_bits = 512 * 24
    coord_bits = (n + N) * 24
    row_bits = N * 24
    for k in (1, 2, 3, 4):
        r = plans[str(k)]
        acc = r["max_size"] * 20
        peak = acc + coord_bits + row_bits + lut_bits
        pct = 100.0 * peak / BRAM_BITS
        print("   %d 块：累加器 %6.2f + 坐标 %.3f + 行缓冲 %.3f + LUT %.3f"
              " = %6.2f Mb（%.2f%%）%s"
              % (k, acc / 1e6, coord_bits / 1e6, row_bits / 1e6, lut_bits / 1e6,
                 peak / 1e6, pct, "  <= 30%" if pct <= 30.0 else ""))
        plans[str(k)]["peak_mb"] = peak / 1e6
        plans[str(k)]["peak_pct"] = pct

    print("\n  位宽 x 块数 的峰值 BRAM 矩阵（行=Gram 位宽，列=块数，值=峰值%%）")
    lut_bits = 512 * 24
    coord_bits = (n + N) * 24
    row_bits = N * 24
    state_bits = coord_bits + row_bits + lut_bits
    print("    位宽 |  " + "  ".join("%6d 块" % k for k in (1, 2, 4)))
    feasible = []
    for nb in (16, 17, 18, 20, 22, 24):
        cells = []
        for k in (1, 2, 4):
            peak = plans[str(k)]["max_size"] * nb + state_bits
            pct = 100.0 * peak / BRAM_BITS
            cells.append("%7.2f%%" % pct)
            if pct <= 30.0:
                feasible.append({"gram_bits": nb, "blocks": k, "peak_pct": pct,
                                 "rel_fro": next(r["rel_fro"] for r in bit_rows
                                                 if r["bits"] == nb)})
        print("    %4d |  " % nb + "  ".join(cells))
    print("\n  满足「BRAM ≤ 30%」的组合（按相对误差排序，取前 6）：")
    feasible.sort(key=lambda v: v["rel_fro"])
    for v in feasible[:6]:
        print("    %2d bit / %d 块：峰值 %.2f%%，Gram 相对 Frobenius %.3e"
              % (v["gram_bits"], v["blocks"], v["peak_pct"], v["rel_fro"]))

    m_row = (n - 1) * n
    gen_ops = 2 * m_row * N          # 一遍 A_d 行生成的操作数（权重实时算）
    print("\n  分块的代价")
    print("  关键：**累加 MAC 不随块数增长**，每行该累加的对数总量不变；")
    print("  分块只让「A_d 行生成」多跑几遍。行生成本就只占 Gram 的 %.2f%%。"
          % (100.0 * gen_ops / mac_gram))
    for k in (1, 2, 3, 4):
        total_ops = mac_gram + k * gen_ops
        print("   %d 块：累加 MAC %.3e + 行生成 %.3e x %d = %.3e"
              "（相对 1 块 +%.2f%%）"
              % (k, mac_gram, gen_ops, k, total_ops,
                 100.0 * (total_ops - (mac_gram + gen_ops)) / (mac_gram + gen_ops)))
        plans[str(k)]["accum_mac"] = mac_gram
        plans[str(k)]["gen_ops"] = k * gen_ops
        plans[str(k)]["total_ops"] = total_ops

    # ================================================================
    out = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "n": n, "n_scene": N, "y_w": y_w, "dA": dA, "C": C,
        "d2_range": [lo, hi], "d2_dynamic_range": hi / lo,
        "dx2_range": [float(dx2_sep.min()), float(dx2_sep.max())],
        "dz2_range": [float(dz2_sep.min()), float(dz2_sep.max())],
        "A_range": None, "G_triu_max": gmax, "aty_max": float(np.abs(aty_ref).max()),
        "weight_schemes": [{"name": nm, "table_entries": tb, "extra_ops": op}
                           for nm, tb, op in schemes],
        "lut_division": lut_rows,
        "gram_bits": bit_rows,
        "end_to_end": e2e,
        "triu_total": total,
        "chunk_plan": plans,
        "feasible_plans": feasible[:6],
        "mac_per_gram": mac_gram,
        "bram_bits_total": BRAM_BITS,
    }
    os.makedirs(args.out_dir, exist_ok=True)
    with io.open(os.path.join(args.out_dir, "x9_gram_plan.json"), "w",
                 encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("\n[done] 明细写入 %s/x9_gram_plan.json" % args.out_dir)


if __name__ == "__main__":
    main()
