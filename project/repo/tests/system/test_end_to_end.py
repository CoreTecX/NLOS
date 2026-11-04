# -*- coding: utf-8 -*-
"""tests/system/test_end_to_end.py —— 全系统端到端验收（方案 §10 / §12 Task 11）。

判据按验收指标 P1~P5（docs/requirements.md §二）。**本机没有实验台、没有相机、
没有板卡**，因此凡依赖硬件的项一律标 PENDING 并写明等什么，**不代跑、不臆造**。

口径待定（不要替项目方拍板）
---------------------------------------------------------------------------
**定位结果的口径尚未拍板，见 report/design_report/design_report.html 第五章 5.2
（表 5-2「P1 的三条出路」）。** 三条出路是：① 接受现状（单帧最好 43.8 mm）并改写
验收口径；② 改技术路线：多帧多图案；③ 降级为定位固定位置、算力全投重建。
本脚本把这条口径做成**显式参数** `--loc-policy`，默认 `undecided`（→ P1 判 PENDING）；
脚本**不预设、不代替**任何一条出路。所有与定位口径相关的判据都只在参数明确给出时判定。

各指标的判定前提（缺前提即 PENDING）
---------------------------------------------------------------------------
  P1 自定位误差     需 --loc-policy 明确 + --p1-measured-mm（实测）；否则 PENDING
  P2 端到端时延     需 --p2-measured-ms（上板实测）；否则 PENDING（另做一项可跑的算术自查）
  P3 重建质量       需 --p3-psnr-min/--p3-ssim-min 阈值 + --p3-metrics-json（实测）；否则 PENDING
  P4 资源占用       需 --p4-util-json（Vivado 实测）+ 至少一项 --p4-max-*-pct 门槛；否则 PENDING
  P5 功耗           需 --p5-measured-w（上板实测）+ --p5-max-w；否则 PENDING

本机可立即跑的三项
---------------------------------------------------------------------------
  1. psm 格式契约（Python 侧 parity_lines，与 C 端 test_psm_format 同源）；
  2. 算力预算自查：候选 243 × M 9120 × K 8 × 3 ≈ 53.2 M MAC（方案 §5.3）；
  3. 算法链冒烟：小几何 -> 前向渲染 -> 行差分 -> 候选能量 -> PDHG 重建 -> 度量，
     验证「链路能串起来、输出有限且非负」，**不构成任何精度或性能结论**。

用法
----
    python tests/system/test_end_to_end.py                      # 全 PENDING + 3 项自查
    python tests/system/test_end_to_end.py --loc-policy accept_43.8mm --p1-measured-mm 43.8
    python tests/system/test_end_to_end.py --p2-measured-ms 88.85
    python tests/system/test_end_to_end.py --json-out build/reports/tests_system_last.json
退出码非零表示存在被判定为 FAIL 的指标（PENDING 不导致失败）。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
_MODEL = os.path.join(_ROOT, "src", "model")
if _MODEL not in sys.path:
    sys.path.insert(0, _MODEL)

PASS, FAIL, PENDING, NA = "PASS", "FAIL", "PENDING", "N/A"

# 方案 §5.3 的算力口径：候选 x M x K x RGB 三通道 ≈ 53.2 M MAC。
PLAN_LOC_MAC = 53.2e6
PLAN_LOC_MAC_TOL = 0.4e6          # 只核对量级与公式，不钉死到个位


# ---------------------------------------------------------------- 小几何（冒烟用）


def _tiny_setup(wall_n: int = 8):
    from penumbrasight_model import forward as F
    wall = F.WallPlane(y=1.0, x_min=-0.5, x_max=0.5, z_min=-0.5, z_max=0.5, n=wall_n)
    monitor = F.MonitorPlane(
        y=0.0,
        x_centers=np.array([-0.1, 0.0, 0.1]),
        z_centers=np.array([-0.05, 0.05]),
        block_w=0.01, block_h=0.01)
    occluder = F.Occluder(ll=np.array([-1.0, 0.5, -1.0]),
                          size=np.array([2.0, 2.0]))
    return F.OpticalSetup(wall=wall, monitor=monitor, occluder=occluder)


# ---------------------------------------------------------------- 可立即跑的检查


def check_psm_contract():
    from penumbrasight_model import psm
    lines = psm.parity_lines()
    ok = (lines[0].startswith("b_r64 rank=64 M=9120 N=432")
          and any(ln.startswith("sizeof_header=64") for ln in lines)
          and any(("off_geom=24" in ln and "off_crc=28" in ln) for ln in lines))
    detail = "parity_lines 首行=%r" % (lines[0][:48],)
    return (PASS if ok else FAIL), detail


def check_mac_budget():
    from penumbrasight_model import forward_loc as FL
    _, total = FL.b_candidate_grid()
    meas_h, meas_w = 95, 96            # 差分后 95x96
    K = 8                              # loc_rank
    mac = total * (meas_h * meas_w) * K * 3
    ok = abs(mac - PLAN_LOC_MAC) <= PLAN_LOC_MAC_TOL
    detail = ("候选 %d x M %d x K %d x 3 = %.2f M MAC（方案口径 53.2 M）"
              % (total, meas_h * meas_w, K, mac / 1e6))
    return (PASS if ok else FAIL), detail


def check_chain_smoke():
    """小几何走一遍：前向渲染 -> 行差分 -> 候选能量 -> PDHG 重建 -> 度量。"""
    from penumbrasight_model import forward as F
    from penumbrasight_model import localization as L
    from penumbrasight_model import reconstruction as R

    setup = _tiny_setup()
    ll = np.array([-0.05, 0.5, -0.05])
    s2 = setup.with_occluder_ll(ll)
    rng = np.random.default_rng(11)
    scene = np.abs(rng.standard_normal(setup.monitor.shape)) + 0.1
    y2d = F.forward_render(s2, scene)
    if y2d.shape != (setup.wall.n, setup.wall.n):
        return FAIL, "前向渲染形状 %s 不符" % (y2d.shape,)

    # 定位：两个候选的能量都应有限
    e1 = L.evaluate_candidate(s2, y2d, ll, differenced=True).energy
    e2 = L.evaluate_candidate(s2, y2d, ll + np.array([0.1, 0.0, 0.0]),
                              differenced=True).energy
    if not (np.isfinite(e1) and np.isfinite(e2)):
        return FAIL, "候选能量非有限：%r %r" % (e1, e2)

    # 重建：小尺寸 PDHG，输出应有限且非负
    res = R.reconstruct_channel(s2, y2d, ll, lam=0.01, n_iter=60)
    img = res.image
    if img.shape != setup.monitor.shape:
        return FAIL, "重建形状 %s 不符" % (img.shape,)
    if not (np.all(np.isfinite(img)) and np.all(img >= 0.0)):
        return FAIL, "重建含非有限值或负值"
    r = R.pearson(img, scene)
    detail = ("能量 %.4g/%.4g；重建 %dx%d，非负、有限，与真值 r=%.3f（仅冒烟，不作精度结论）"
              % (e1, e2, img.shape[0], img.shape[1], r))
    return PASS, detail


# ---------------------------------------------------------------- P1~P5 判定


def judge_p1(args):
    pol = args.loc_policy
    if pol == "undecided":
        return PENDING, ("定位结果口径待定，见 design_report 第五章 5.2（表 5-2 三条出路）；"
                         "用 --loc-policy 显式选定后才判 P1")
    if pol == "fixed_position":
        return NA, ("已选择出路③：降级为定位固定位置，自定位（F2）不作为交付指标；"
                    "本项不参与 P1 判定")
    if pol == "multi_frame":
        if args.p1_measured_mm is None:
            return PENDING, ("已选多帧多图案口径，但本机无该口径的实测误差；"
                             "等采集与算力到位后补 --p1-measured-mm")
        target = args.p1_accepted_mm
        return ((PASS if args.p1_measured_mm <= target else FAIL),
                "多帧多图案口径：实测 %.2f mm，门槛 %.2f mm" % (args.p1_measured_mm, target))
    # accept_43.8mm
    if args.p1_measured_mm is None:
        return PENDING, "已选「接受现状」口径，但未给实测 --p1-measured-mm"
    target = args.p1_accepted_mm
    return ((PASS if args.p1_measured_mm <= target else FAIL),
            "接受现状口径：实测 %.2f mm，改写后的门槛 %.2f mm" % (args.p1_measured_mm, target))


def judge_p2(args):
    if args.p2_measured_ms is None:
        return PENDING, ("等上板实测：本机无板卡。给 --p2-measured-ms 才判 P2；"
                         "方案 §5.3 目标 <= %.0f ms" % args.p2_max_ms)
    return ((PASS if args.p2_measured_ms <= args.p2_max_ms else FAIL),
            "上板实测 %.2f ms（门槛 %.0f ms）" % (args.p2_measured_ms, args.p2_max_ms))


def judge_p3(args):
    if args.p3_psnr_min is None and args.p3_ssim_min is None:
        return PENDING, "P3 阈值待补（docs/requirements.md §二 P3）；用 --p3-psnr-min / --p3-ssim-min 给出"
    if not args.p3_metrics_json:
        return PENDING, "给了阈值但无实测 --p3-metrics-json（等实验台数据）"
    with open(args.p3_metrics_json, "r", encoding="utf-8") as f:
        m = json.load(f)
    psnr = float(m["psnr_db"]) if "psnr_db" in m else None
    ssim = float(m["ssim"]) if "ssim" in m else None
    ok = True
    parts = []
    if args.p3_psnr_min is not None:
        ok = ok and (psnr is not None and psnr >= args.p3_psnr_min)
        parts.append("PSNR %s >= %s" % (psnr, args.p3_psnr_min))
    if args.p3_ssim_min is not None:
        ok = ok and (ssim is not None and ssim >= args.p3_ssim_min)
        parts.append("SSIM %s >= %s" % (ssim, args.p3_ssim_min))
    return (PASS if ok else FAIL), "；".join(parts)


def judge_p4(args):
    if not args.p4_util_json:
        return PENDING, "等 Vivado 综合/实现：给 --p4-util-json（含 lut_pct/dsp_pct/bram_pct）才判 P4"
    if not any(v is not None for v in (args.p4_max_lut_pct, args.p4_max_dsp_pct,
                                       args.p4_max_bram_pct)):
        return PENDING, "有实测但 P4 门槛待补（docs/requirements.md §二 P4 目标【待补】）"
    with open(args.p4_util_json, "r", encoding="utf-8") as f:
        u = json.load(f)
    ok = True
    parts = []
    for key, cap in (("lut_pct", args.p4_max_lut_pct),
                     ("dsp_pct", args.p4_max_dsp_pct),
                     ("bram_pct", args.p4_max_bram_pct)):
        if cap is None:
            continue
        val = float(u[key])
        ok = ok and (val <= cap)
        parts.append("%s %.1f%% <= %.1f%%" % (key, val, cap))
    return (PASS if ok else FAIL), "；".join(parts)


def judge_p5(args):
    if args.p5_max_w is None:
        return PENDING, "P5 阈值待定（docs/requirements.md §二 P5）；用 --p5-max-w 给出"
    if args.p5_measured_w is None:
        return PENDING, "等上板实测功耗：给 --p5-measured-w 才判 P5"
    return ((PASS if args.p5_measured_w <= args.p5_max_w else FAIL),
            "上板实测 %.2f W（门槛 %.2f W）" % (args.p5_measured_w, args.p5_max_w))


# ---------------------------------------------------------------- 运行器


def build_parser():
    p = argparse.ArgumentParser(description="PenumbraSight 端到端验收（P1~P5）")
    p.add_argument("--loc-policy", default="undecided",
                   choices=["undecided", "accept_43.8mm", "multi_frame",
                            "fixed_position"],
                   help="定位结果口径（默认 undecided；见文件头「口径待定」）")
    p.add_argument("--p1-measured-mm", type=float, default=None)
    p.add_argument("--p1-accepted-mm", type=float, default=43.8,
                   help="选择「接受现状」口径后的改写门槛（mm）")
    p.add_argument("--p2-measured-ms", type=float, default=None)
    p.add_argument("--p2-max-ms", type=float, default=100.0)
    p.add_argument("--p3-psnr-min", type=float, default=None)
    p.add_argument("--p3-ssim-min", type=float, default=None)
    p.add_argument("--p3-metrics-json", default=None)
    p.add_argument("--p4-util-json", default=None)
    p.add_argument("--p4-max-lut-pct", type=float, default=None)
    p.add_argument("--p4-max-dsp-pct", type=float, default=None)
    p.add_argument("--p4-max-bram-pct", type=float, default=None)
    p.add_argument("--p5-measured-w", type=float, default=None)
    p.add_argument("--p5-max-w", type=float, default=None)
    p.add_argument("--json-out", default=None)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    print("==== tests/system 端到端验收（P1~P5）====")
    print("口径提示：定位结果口径待定，见 design_report 第五章 5.2（表 5-2）。")
    print()

    selfchecks = [
        ("psm 格式契约（Python 侧）", check_psm_contract),
        ("算力预算自查（53.2 M MAC）", check_mac_budget),
        ("算法链冒烟（小几何）", check_chain_smoke),
    ]
    results = {}
    print("--- 本机可立即运行 ---")
    for name, fn in selfchecks:
        try:
            status, detail = fn()
        except Exception as exc:  # noqa: BLE001
            status, detail = FAIL, "异常：%r" % (exc,)
        results[name] = {"status": status, "detail": detail}
        print("  [%-7s] %s" % (status, name))
        print("            %s" % detail)

    print()
    print("--- 验收指标 ---")
    graded = [
        ("P1 自定位误差", judge_p1(args)),
        ("P2 端到端时延", judge_p2(args)),
        ("P3 重建质量", judge_p3(args)),
        ("P4 资源占用", judge_p4(args)),
        ("P5 功耗", judge_p5(args)),
    ]
    for name, (status, detail) in graded:
        results[name] = {"status": status, "detail": detail}
        print("  [%-7s] %s" % (status, name))
        print("            %s" % detail)

    n_fail = sum(1 for r in results.values() if r["status"] == FAIL)
    n_pend = sum(1 for r in results.values() if r["status"] == PENDING)
    print()
    print("=" * 70)
    print("合计：FAIL %d，PENDING %d，其余 PASS/N/A" % (n_fail, n_pend))
    if n_fail:
        print("存在 FAIL 项：")
        for name, r in results.items():
            if r["status"] == FAIL:
                print("  - %s：%s" % (name, r["detail"]))
    print("=" * 70)

    if args.json_out:
        os.makedirs(os.path.dirname(os.path.abspath(args.json_out)) or ".", exist_ok=True)
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump({"results": results, "n_fail": n_fail, "n_pending": n_pend},
                      f, ensure_ascii=False, indent=2)
        print("结果已写入 %s" % args.json_out)

    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
