# -*- coding: utf-8 -*-
"""阈值标定工具（对应 src/fpga/rtl/README.md 的 D1~D3）。

问题
----
`sim/results/preboard_port_status_20260930.md` §3.2 第 5 条记：「RTL 阈值未标定」。
具体指 **两个 RTL 模块的 8 个阈值输入**（RTL 里全是常量输入端口，模块内不做任何
自适应统计，见各模块的 [R8]/[R3]）：

  fast_alert.sv
    cfg_e_thresh    [47:0]   E 的告警阈值；口径 = 标定集 E 的 **99.5 百分位**（§6.6）
    cfg_min_energy  [47:0]   质心有效的最低能量；低于它**冻结**上一次质心（易错点 2）
    cfg_dir_deadband[15:0]   方向死区（Q0.8，单位 = 1/256 个块列）

  confidence_engine.sv
    snr_min_q016    [15:0]   SNR 标定阈值（Q0.16）
    margin_min_q016 [15:0]   margin 标定阈值（Q0.16）
    rho_max_q016    [15:0]   = 1.5 × 空白验证集 ρ 的 **95 百分位**（Q0.16）
    sat_max_q016    [15:0]   饱和像素比例阈值；口径 = **0.1%**（Q0.16）
    low_conf_q016   [15:0]   low_conf 判定阈值（Q0.16）

这 8 个是**唯一**的「未标定阈值」。其它候选口令（`configs/geometry_b.yaml` 的
`occluder.nominal_center.calibrated: false`、`camera_b.yaml` 的
`p999_below_ratio_of_full_scale: 0.95`）属于**几何对齐**与**相机验收**，
不是 RTL 阈值，别混为一谈。

标定口径的三条硬纪律（§6.6）
----------------------------
1. E 的百分位必须在**同一套定点算术**里算：`cfg_e_thresh` 要能直接与 RTL 的
   `e_acc` 比较，故本工具用整数 floor 除法**逐位复刻** `fast_alert.sv` 的
   `E = Σ floor((c_k-μ_k)^2 / (v_k+ε))`，不接受浮点百分位。
2. **只用调参集设定，盲测集不重新调**。本工具只吃显式传入的标定集，
   不读盲测集，也不做任何按盲测集的回退。
3. Q0.16 编码 = `round_half_even(x * 65536)`，`x` 为物理量（比例/分数），
   裁剪到 `[0, 65535]`。注意 `x = 1.0` 在本编码里不可表示（65536 溢出 16 位），
   阈值必须严格小于 1。

本工具能做什么 / 不能做什么
---------------------------
能：
  * 给定标定集（E 的逐帧样本、或 snr/margin/rho 的逐帧样本），按上面的口径算出
    这些阈值，并把结果写成 JSON；
  * `--self-test` 用合成样本把「整数 E 复刻 + 最近秩百分位 + Q0.16 编码」三条
    算术逐值验一遍（这一步是**可跑的**，与是否有真数据无关）；
  * 数据无关的那一个阈值（`sat_max_q016` = 0.1%）直接给出确定值。

不能（如实说明）：
  * 仓库内**没有**标定集。E 需要「有/无遮挡」的成对实拍帧 + 背景统计（μ/v），
    snr/margin/rho 需要真跑定位/重建链。`model_b.psm` 虽在本机（300 MB），
    但没有实拍标定帧就无从产生 E 样本。故除 `sat_max_q016` 外，本工具在缺数据时
    只输出**占位值**，并在 JSON 里用 `"placeholder": true` 明确标出。
  * 标定集到位前，任何把这些占位值写进上板镜像的做法都会让硬闸门形同虚设。

用法
----
    # 1) 只验算术（无需数据，可随时跑）
    python src/model/scripts/calibrate_thresholds.py --self-test

    # 2) 产出口径说明 + 占位值（当前仓库状态）
    python src/model/scripts/calibrate_thresholds.py --out sim/results/thresholds_b_20260930.json

    # 3) 有标定集之后（数据契约见本文件 CALIB_SET_CONTRACT）
    python src/model/scripts/calibrate_thresholds.py \
        --e-samples  <calib_e.npz>  \
        --snr-samples <snr.txt> --margin-samples <margin.txt> --rho-samples <rho.txt> \
        --out sim/results/thresholds_b_20260930.json

退出码：0 = 正常（含占位输出）；非零 = 参数/数据错误。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

CONF_W = 16
Q_ONE = float(1 << CONF_W)          # 65536

# 数据契约（写清楚，避免「看起来能跑其实喂错了」）
CALIB_SET_CONTRACT = {
    "e_samples": (
        "npz，键：c[N,K] int（Q2.16 码字）、mu[K] int、v[K] uint、eps int。"
        "每行是一次判决（一帧）的系数与背景统计；N = 标定集帧数。"
        "另可给 mom0[N]（质心矩），用于 cfg_min_energy。"),
    "snr_samples": "纯文本，每行一个 [0,1] 内的归一化 SNR（一行一帧）。",
    "margin_samples": "纯文本，每行一个 [0,1] 内的 margin。",
    "rho_samples": "纯文本，每行一个 [0,1] 内的归一化残差 ρ（**空白/无遮挡**验证集）。",
}


# ---------------------------------------------------------------- 算术（逐位复刻 RTL）


def rtl_e(c_k, mu_k, v_k, eps) -> int:
    """逐位复刻 fast_alert.sv 的 E 累加（整数 floor 除法，与 RTL 的恢复余数除法同向）。

    RTL：`d_s = $signed(c) - $signed(mu)`（17 bit signed）；`d2 = d_s * d_s`；
    `x_den = v + eps`；`q = floor(d2 / x_den)`；`e_acc += q`。
    这里用 Python 的 `//`（floor），对非负 d2 与正 x_den 与 RTL 的位串行除法结果相同。
    """
    c_k = np.asarray(c_k, dtype=np.int64)
    mu_k = np.asarray(mu_k, dtype=np.int64)
    v_k = np.asarray(v_k, dtype=np.int64)
    if c_k.shape != mu_k.shape or c_k.shape != v_k.shape:
        raise ValueError("c/mu/v 形状不一致：%s %s %s"
                         % (c_k.shape, mu_k.shape, v_k.shape))
    if eps <= 0:
        raise ValueError("eps 必须 >= 1（RTL 里 cfg_eps==0 直接 cfg_err）")
    d = c_k - mu_k
    den = v_k + int(eps)
    return int(np.sum((d * d) // den))


def nearest_rank_percentile(values, p: float) -> int:
    """最近秩（nearest-rank）百分位：排序后取第 ceil(p*n) 个。

    选最近秩而不是线性插值，是因为阈值要落在一个**真实样本**上：
    插值会给出一个标定集里根本不存在的数，评审时无法回溯到是哪一帧。
    """
    a = np.sort(np.asarray(values, dtype=np.int64).ravel())
    n = a.size
    if n == 0:
        raise ValueError("空样本集")
    if not (0.0 < p <= 1.0):
        raise ValueError("p 必须落在 (0, 1]，得到 %r" % (p,))
    rank = int(math.ceil(p * n))
    rank = min(max(rank, 1), n)
    return int(a[rank - 1])


def round_half_even(x: float) -> int:
    """向最近值舍入、半值取偶（与 quantize.round_half_even / RTL 同口径）。"""
    return int(np.rint(x))


def q016(x: float) -> int:
    """物理比例 x -> Q0.16 码字，裁剪到 [0, 65535]。

    RTL 的阈值端口是 16 位无符号，65536 装不下，所以 x 必须 < 1.0。
    """
    if not (0.0 <= x < 1.0):
        raise ValueError("Q0.16 阈值要求 0 <= x < 1，得到 %r" % (x,))
    return min(max(round_half_even(x * Q_ONE), 0), (1 << CONF_W) - 1)


def q08(x: float) -> int:
    """物理量 -> Q0.8 码字（fast_alert 的方向死区用）。"""
    if x < 0:
        raise ValueError("Q0.8 死区必须非负")
    return int(min(max(round_half_even(x * 256.0), 0), (1 << 16) - 1))


# ---------------------------------------------------------------- 标定


def calibrate(cfg):
    """按数据可得性产出阈值表。返回 (values: dict, notes: list[str])。"""
    values, notes = {}, []

    # --- 数据无关：饱和比例 0.1% (§6.6) ---
    sat_thr = q016(0.001)
    values["sat_max_q016"] = {
        "value": sat_thr, "hex": "0x%04X" % sat_thr,
        "derived": "0.1%% -> round(0.001*65536) = %d" % sat_thr,
        "placeholder": False,
    }
    notes.append(
        "sat_max_q016 与 0.1%% 的关系：round(0.001*65536) = %d = 0x%04X。"
        "confidence_engine.sv 与 rtl/README.md 的注释写「0.1%% -> Q0.16 ≈ 0x0066」，"
        "而 0x0066 = %d = %.4f%%（约 1.56 倍），两处对不上（见报告 §任务3）。"
        % (sat_thr, sat_thr, 0x0066, 0x0066 / Q_ONE * 100.0))

    # --- E 阈值 / 最小能量：需要标定集 ---
    if cfg.e_samples:
        z = np.load(cfg.e_samples)
        c = np.atleast_2d(z["c"])
        mu = np.asarray(z["mu"]).ravel()
        v = np.asarray(z["v"]).ravel()
        eps = int(z["eps"]) if "eps" in z.files else 1
        eps = cfg.eps if cfg.eps is not None else eps
        e_vals = [rtl_e(c[i], mu, v, eps) for i in range(c.shape[0])]
        ethr = nearest_rank_percentile(e_vals, cfg.e_percentile)
        values["cfg_e_thresh"] = {
            "value": ethr, "placeholder": False,
            "derived": "标定集 %d 帧；E 的 %.2f 百分位（最近秩）；整数 floor 口径"
                       % (len(e_vals), cfg.e_percentile * 100.0),
            "e_min": int(min(e_vals)), "e_max": int(max(e_vals)),
        }
        if "mom0" in z.files:
            m = np.asarray(z["mom0"]).ravel()
            me = nearest_rank_percentile(m, cfg.min_energy_percentile)
            values["cfg_min_energy"] = {
                "value": int(me), "placeholder": False,
                "derived": "标定集 mom0 的 %.2f 百分位（最近秩）"
                           % (cfg.min_energy_percentile * 100.0),
            }
        else:
            values["cfg_min_energy"] = {
                "value": 0, "placeholder": True,
                "derived": "标定集 npz 未提供 mom0[]，无法算；占位 0（=只要 mom0>0 就判有效）",
            }
        notes.append("cfg_e_thresh 已用标定集算出；注意它必须在 E_W=48 位宽内。")
    else:
        values["cfg_e_thresh"] = {
            "value": 0, "placeholder": True,
            "derived": "缺标定集；占位 0（fail-loud：任何 E 都超阈，两帧判据下 alert_event 恒 1，"
                       "一眼能看出未标定）。口径 = 标定集 E 的 99.5 百分位（整数 floor）。",
        }
        values["cfg_min_energy"] = {
            "value": 0, "placeholder": True,
            "derived": "缺标定集；占位 0（只要 mom0>0 质心即有效）。口径 = 标定集 mom0 的 1 百分位。",
        }
        notes.append("cfg_e_thresh / cfg_min_energy 为占位值：需要「有/无遮挡」成对实拍帧 + 背景 μ/v。")

    # --- 方向死区：Q0.8，规则派生 ---
    db = q08(cfg.deadband_cols)
    values["cfg_dir_deadband"] = {
        "value": db, "hex": "0x%04X" % db, "placeholder": True,
        "derived": "规则派生：%.3f 个块列 x 256 = %d（Q0.8）。标为占位是因为「几个块列」"
                   "这个倍率本身要按实测抖动量复核；RTL TB 里用的是 32（0.125 列）。"
                   % (cfg.deadband_cols, db),
    }

    # --- snr / margin：标定集百分位 ---
    if cfg.snr_samples:
        s = _read_scalars(cfg.snr_samples)
        thr = q016(float(nearest_rank_percentile(np.rint(np.asarray(s) * Q_ONE), cfg.snr_percentile)) / Q_ONE)
        values["snr_min_q016"] = {
            "value": thr, "hex": "0x%04X" % thr, "placeholder": False,
            "derived": "标定集 SNR 的 %.2f 百分位（最近秩，Q0.16）"
                       % (cfg.snr_percentile * 100.0),
        }
    else:
        values["snr_min_q016"] = {
            "value": 0x4000, "hex": "0x4000", "placeholder": True,
            "derived": "缺标定集；占位 0x4000 = 0.25（工程默认）。口径 = 标定集 SNR 的 5 百分位。",
        }
    if cfg.margin_samples:
        s = _read_scalars(cfg.margin_samples)
        thr = q016(float(nearest_rank_percentile(np.rint(np.asarray(s) * Q_ONE), cfg.margin_percentile)) / Q_ONE)
        values["margin_min_q016"] = {
            "value": thr, "hex": "0x%04X" % thr, "placeholder": False,
            "derived": "标定集 margin 的 %.2f 百分位（最近秩，Q0.16）"
                       % (cfg.margin_percentile * 100.0),
        }
    else:
        values["margin_min_q016"] = {
            "value": 0x199A, "hex": "0x199A", "placeholder": True,
            "derived": "缺标定集；占位 0x199A = 0.1（工程默认）。口径 = 标定集 margin 的 5 百分位。",
        }

    # --- rho_max：1.5 x 空白验证集 P95 ---
    if cfg.rho_samples:
        r = _read_scalars(cfg.rho_samples)
        p95 = float(nearest_rank_percentile(np.rint(np.asarray(r) * Q_ONE), cfg.rho_percentile)) / Q_ONE
        thr = q016(min(p95 * cfg.rho_factor, 0.999999))
        values["rho_max_q016"] = {
            "value": thr, "hex": "0x%04X" % thr, "placeholder": False,
            "derived": "空白验证集 ρ 的 %.0f 百分位 %.6f x %.2f = %.6f -> Q0.16"
                       % (cfg.rho_percentile * 100.0, p95, cfg.rho_factor, p95 * cfg.rho_factor),
        }
    else:
        values["rho_max_q016"] = {
            "value": 0x4CCD, "hex": "0x4CCD", "placeholder": True,
            "derived": "缺标定集；占位 0x4CCD = 0.3（工程默认）。口径 = 1.5 x 空白验证集 ρ 的 95 百分位；"
                       "1.5 刻意不写进 RTL，由本工具在 PC 端算好后传入。",
        }

    # --- low_conf：工程默认，待实测复核 ---
    lc = q016(cfg.low_conf)
    values["low_conf_q016"] = {
        "value": lc, "hex": "0x%04X" % lc, "placeholder": True,
        "derived": "工程默认 %.2f -> 0x%04X（STATUS bit3 的取数门限；标为占位是因为它依赖"
                   "融合方式 w_sel 的取值分布，需实测复核）。" % (cfg.low_conf, lc),
    }
    return values, notes


def _read_scalars(path):
    out = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            out.append(float(line))
    if not out:
        raise ValueError("没有读到数值：%s" % path)
    return out


# ---------------------------------------------------------------- 自检（算术逐值）


def self_test() -> int:
    """不依赖任何数据，把三条算术逐值验一遍。返回 0/1。"""
    ok = True

    def chk(cond, msg):
        nonlocal ok
        print(("  PASS  " if cond else "  FAIL  ") + msg)
        ok = ok and cond

    print("---- A. 整数 E 逐值复刻（对照 fast_alert_tb T1 的期望 64681）----")
    e = rtl_e([1000, 2000, -1500, 3000], [0, 500, -500, 1000],
              [100, 200, 250, 100], 1)
    chk(e == 64681, "T1 向量 E=%d（TB/xsim 实测 64681）" % e)

    print("---- B. floor 方向（负差与截断）----")
    # d=-1000, den=251 -> 1e6/251 = 3984.06 -> floor 3984
    e2 = rtl_e([-1000], [0], [250], 1)
    chk(e2 == 3984, "floor(1000000/251)=%d（应 3984，不是 3985）" % e2)
    e3 = rtl_e([1], [0], [2], 1)
    chk(e3 == 0, "floor(1/3)=%d（应 0）" % e3)

    print("---- C. 最近秩百分位 ----")
    chk(nearest_rank_percentile(range(1, 201), 0.995) == 199,
        "1..200 的 99.5 百分位 = 199（ceil(0.995*200)=199）")
    chk(nearest_rank_percentile([5], 0.995) == 5, "单元素集取该元素")
    chk(nearest_rank_percentile(range(1, 101), 0.95) == 95,
        "1..100 的 95 百分位 = 95")

    print("---- D. Q0.16 / Q0.8 编码 ----")
    chk(q016(0.001) == 66, "0.1%% -> %d（0x42）" % q016(0.001))
    chk(q016(0.001) != 0x0066, "0.1%% 不是 0x0066（0x0066=%d=%.4f%%）"
        % (0x0066, 0x0066 / Q_ONE * 100.0))
    chk(q016(0.25) == 0x4000, "0.25 -> 0x%04X" % q016(0.25))
    chk(round_half_even(0.5) == 0 and round_half_even(1.5) == 2,
        "半值取偶：0.5->0, 1.5->2")
    chk(round_half_even(2.5) == 2 and round_half_even(3.5) == 4,
        "半值取偶：2.5->2, 3.5->4")
    chk(q08(0.25) == 64, "0.25 个块列 -> Q0.8 %d" % q08(0.25))
    try:
        q016(1.0)
        chk(False, "x=1.0 应报错（16 位装不下 65536）")
    except ValueError:
        chk(True, "x=1.0 报错（16 位装不下 65536）")
    try:
        rtl_e([0], [0], [0], 0)
        chk(False, "eps=0 应报错")
    except ValueError:
        chk(True, "eps=0 报错（对应 RTL 的 cfg_err）")

    print("\n自检结果：%s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


# ---------------------------------------------------------------- CLI


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="fast_alert / confidence_engine 阈值标定工具")
    ap.add_argument("--self-test", action="store_true", help="只验算术，不需要数据")
    ap.add_argument("--out", default="sim/results/thresholds_b_20260930.json")

    ap.add_argument("--e-samples", default=None, help="标定集 npz（见 CALIB_SET_CONTRACT）")
    ap.add_argument("--snr-samples", default=None)
    ap.add_argument("--margin-samples", default=None)
    ap.add_argument("--rho-samples", default=None)
    ap.add_argument("--eps", type=int, default=None, help="覆盖 npz 里的 eps")

    ap.add_argument("--e-percentile", type=float, default=0.995)
    ap.add_argument("--min-energy-percentile", type=float, default=0.01)
    ap.add_argument("--snr-percentile", type=float, default=0.05)
    ap.add_argument("--margin-percentile", type=float, default=0.05)
    ap.add_argument("--rho-percentile", type=float, default=0.95)
    ap.add_argument("--rho-factor", type=float, default=1.5)
    ap.add_argument("--deadband-cols", type=float, default=0.25)
    ap.add_argument("--low-conf", type=float, default=0.25)
    ap.add_argument("--print-contract", action="store_true")
    args = ap.parse_args(argv)

    if args.print_contract:
        print(json.dumps(CALIB_SET_CONTRACT, ensure_ascii=False, indent=2))
        return 0

    if args.self_test:
        return self_test()

    if not args.self_test and args.e_samples is None:
        print("[note] 未提供 --e-samples：除 sat_max_q016 外均输出占位值。")

    values, notes = calibrate(args)

    print("==== 阈值标定结果（口径见文件头）====")
    print("%-20s %-10s %-8s %s" % ("端口", "值", "占位?", "来源/公式"))
    for k in ("cfg_e_thresh", "cfg_min_energy", "cfg_dir_deadband",
              "snr_min_q016", "margin_min_q016", "rho_max_q016",
              "sat_max_q016", "low_conf_q016"):
        v = values[k]
        print("%-20s %-10d %-8s %s"
              % (k, v["value"], "占位" if v["placeholder"] else "已标定", v["derived"]))
    print()
    for n in notes:
        print("[note] " + n)

    n_ph = sum(1 for v in values.values() if v["placeholder"])
    print("\n占位数：%d / %d" % (n_ph, len(values)))

    if args.out:
        payload = {
            "generated_for": "PenumbraSight B 档上板前阈值",
            "method": {
                "cfg_e_thresh": "标定集 E 的 99.5 百分位；E 用整数 floor 逐位复刻 RTL",
                "cfg_min_energy": "标定集 mom0 的 1 百分位（冻结门限）",
                "cfg_dir_deadband": "规则派生：死区 = k 个块列（Q0.8）",
                "snr_min_q016": "标定集 SNR 的 5 百分位",
                "margin_min_q016": "标定集 margin 的 5 百分位",
                "rho_max_q016": "1.5 x 空白验证集 rho 的 95 百分位",
                "sat_max_q016": "0.1% -> round(0.001*65536)",
                "low_conf_q016": "工程默认，待实测复核",
            },
            "discipline": [
                "只用调参集，盲测集不重新调阈值（§6.6）",
                "E 的百分位必须在同一套定点算术里算（不接受浮点百分位）",
                "rho 的 1.5 倍不写进 RTL，在 PC 端算好后传入",
            ],
            "blocker": (
                "仓库内没有标定集：E 需要「有/无遮挡」成对实拍帧与背景统计 μ/v；"
                "snr/margin/rho 需要真跑定位/重建链。因此除 sat_max_q016 外均为占位值。"
            ),
            "values": values,
            "notes": notes,
            "contract": {k: str(v) for k, v in CALIB_SET_CONTRACT.items()},
        }
        d = os.path.dirname(os.path.abspath(args.out))
        if d and not os.path.isdir(d):
            os.makedirs(d, exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
            f.write("\n")
        print("已写入 %s" % args.out)

    return 0


if __name__ == "__main__":
    sys.exit(main())
