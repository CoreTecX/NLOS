# -*- coding: utf-8 -*-
"""第一版算法移植验收：一条命令跑完 8 条判据 + 双口径 + 重建出图。

判据出处：`01_分析报告/07_单人移植任务表/第一版算法移植交付定义.md` 第二节。

X21 订正（2026-09-26）：交付定义由「七条判据 + 附加项 8/9」改为
「**八条判据**（新增判据 8「M5 重建（重建出图）」）+ 附加项 **9/10**」。
本脚本的报告输出同步改名与重编号，以保证「判据表与实现一致」这条判据可机判
（校验器：`build/scripts/x21_criteria_consistency.py`）。计算逻辑未改动。

用法
----
    python build/scripts/acceptance_first_version.py
    python build/scripts/acceptance_first_version.py --quick    # 跳过耗时的定位全网格

产物
----
    sim/results/x13_acceptance.md          验收清单（判据逐条 + 读数 + 依据文件）
    sim/results/x13_reconstruction.png     重建出图（**出自移植后的 C++ 内核**）
    sim/results/x13_acceptance.json        机器可读的同一份结果

设计说明
--------
本脚本**不重新实现任何算法**，只做三件事：
  1. 编译并运行已有的主机侧测试二进制，解析其 PASS/FAIL；
  2. 读入既有实测产物（x10_verify.json 等）里的数；
  3. 补两件此前没有的：双口径的位宽对照、以及重建出图。
所以「验收」在这里是可复算的——任何一条不通过都能追到具体的二进制或文件。
"""

from __future__ import annotations

import argparse
import io
import json
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
HW = os.path.join(ROOT, "src", "hardware")
SW = os.path.join(ROOT, "src", "software")
RES = os.path.join(ROOT, "sim", "results")

CXX = os.environ.get("CXX", "g++")
CXXFLAGS = ["-O2", "-std=c++11"]

# Windows 的 CreateProcess **不接受** "./t_m1" 这种正斜杠、无扩展名的路径，
# 会直接报「找不到文件」。故一律用「绝对路径 + .exe」调用。
# 首版在 Windows 上用 "./" + 无扩展名调用，六个测试全部 PASS 0 FAIL 0 —— 不是
# 编译失败，是根本没跑起来。
EXE_SUF = ".exe" if os.name == "nt" else ""


def exe_of(base, name):
    return os.path.join(base, name + EXE_SUF)


# ------------------------------------------------------------------ 工具

def sh(cmd, cwd, timeout=1800):
    """跑一条命令，按字节收输出再按 utf-8 replace 解码。

    不能直接用 text=True：Windows 控制台是 GBK，C++ 侧的中文输出会让
    utf-8 解码抛异常并把 stdout 变成 None（X11 踩过这个坑）。
    """
    t0 = time.time()
    try:
        p = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=timeout)
        out = p.stdout.decode("utf-8", "replace")
        return p.returncode, out, time.time() - t0
    except subprocess.TimeoutExpired:
        return -9, "[timeout] " + " ".join(cmd), time.time() - t0
    except OSError as e:
        return -8, "[OSError] %s" % e, time.time() - t0


def build_and_run(src, exe, cwd, extra_incs=(), extra_defs=(), argv=()):
    """编译 + 运行一个测试，返回 (是否成功, PASS 数, FAIL 数, 摘要)。"""
    cmd = [CXX] + CXXFLAGS + list(extra_defs)
    for inc in extra_incs:
        cmd.append("-I" + inc)
    cmd += [src, "-o", exe]
    rc, out, _ = sh(cmd, cwd)
    if rc != 0:
        return False, 0, 0, "编译失败：\n" + out.strip()[:1200]
    rc, out, _ = sh([exe_of(cwd, exe)] + list(argv), cwd)
    m = re.search(r"PASS\s+(\d+)\s+FAIL\s+(\d+)", out)
    if not m:
        # test_preprocess.cpp 用的是中文汇总行，格式与后来几个测试不同。
        # 汇总格式不统一本身不算问题，但解析器必须两种都认，否则会把
        # 「跑通了」误报成「没跑起来」。
        m = re.search(u"通过\\s*(\\d+)\\s*项，失败\\s*(\\d+)\\s*项", out)
    pas = int(m.group(1)) if m else 0
    fail = int(m.group(2)) if m else 0
    ok = (rc == 0) and m is not None and fail == 0
    if not m:
        return False, 0, 0, ("编译通过但未跑出结果（rc=%d）：\n" % rc) + out.strip()[:1200]
    return ok, pas, fail, out


def load_json(path):
    try:
        with io.open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:  # noqa: BLE001
        return None


# ------------------------------------------------------------------ 各项判据

def criteria_cpp_tests(quick):
    """判据 1~4 与支撑项：编译并运行主机侧测试。"""
    rows = []

    def add(name, what, ok, pas, fail, note):
        rows.append({"name": name, "what": what, "ok": ok,
                     "pass": pas, "fail": fail, "note": note})

    ok, p, f, note = build_and_run("test_preprocess.cpp", "t_m1", HW)
    add("M1", "判据 1/2/3：块和四用例、帧累积六档、分块索引不跨行", ok, p, f, note)

    ok, p, f, note = build_and_run("test_x8_geometry.cpp", "t_m2", HW)
    add("M2", "判据 4：几何算核 vs forward.py（矩形一致 + A/A_d 误差）", ok, p, f, note)

    ok, p, f, note = build_and_run("test_x9_gram.cpp", "t_m3", HW)
    add("M3", "判据 5 支撑：Gram 累加与 A_dᵀy（含分块一致性）", ok, p, f, note)

    ok, p, f, note = build_and_run("test_x17_recon.cpp", "t_m5ops", HW)
    add("M5 算子", "支撑：TV 算子与 A_d/A_dᵀ 应用", ok, p, f, note)

    # 内核 dump 出一个相对路径，供重建出图
    ok, p, f, note = build_and_run("test_x15g_recon.cpp", "t_x15g", HW,
                                   argv=("./x13_recon_u.bin",))
    add("M5 内核", "支撑：ADMM 内核 vs Python 参考解", ok, p, f, note)

    ok, p, f, note = build_and_run("test_x12_contract.cpp", "t_x12", SW,
                                   extra_incs=("../../src/hardware",))
    add("X12", "三源同一接口 + 流契约 + Gram 搬出预算", ok, p, f, note)

    return rows


def criterion_5_6():
    """判据 5/6：定位侧的两组数，读 X10 的实测产物。

    判据 6 的口径（2026-10-02 更正）：此前只报「结果量」（972/972 位号一致 +
    Spearman 1.000000），现改报**翻转裕度 ε***（判据阈值 ε* > 2）。ε* 的定义
    （分子 m_k / 分母 e_k / 放大倍数 a_k）与不可扫描情形的处理，见
    src/model/scripts/x10_verify.py 的 eps_flip_scan 与
    src/fpga/hls/projection_engine/test_projection.cpp 的 V-1 段。
    """
    j = load_json(os.path.join(RES, "x10_verify.json"))
    if not j:
        return {"ok": False, "err": "缺少 sim/results/x10_verify.json"}
    eps = j.get("eps_star") or {}
    eps_ok = bool(eps.get("pass"))
    return {
        # 判据 5/6 的通过 = 原结果量通过 **且** 翻转裕度 ε* > 2。
        "ok": bool(j.get("all_pass")) and eps_ok,
        "all_pass": bool(j.get("all_pass")),
        "n_checked": j.get("n_checked"),
        "rel_max": j.get("rel_max"),
        "rel_median": j.get("rel_median"),
        "top1_match": j.get("top1_match"),
        "spearman": j.get("spearman"),
        "cpp_top1": j.get("cpp_top1"),
        "ref_top1": j.get("ref_top1"),
        # V-1 翻转裕度
        "eps_star": eps.get("eps_star"),
        "eps_star_ratio_min": eps.get("eps_star_ratio_min"),
        "eps_flip_k": eps.get("flip_k"),
        "eps_scannable": eps.get("scannable"),
        "eps_ok": eps_ok,
        "eps_threshold": eps.get("threshold", 2.0),
    }


def criterion_7(quick):
    """判据 7：端到端一条命令。跑 x10_localize 的一小段候选，证明链是通的。

    全网格 972 候选约 10 分钟，故默认只跑 1 个候选验证链路，精度数取 X10
    全网格的实测值。`--full` 未实现，避免验收脚本变成离线批处理。
    """
    exe = os.path.join(HW, "t_x10")
    rc, out, _ = sh([CXX] + CXXFLAGS + ["x10_localize.cpp", "-o", "t_x10"], HW)
    if rc != 0:
        return {"ok": False, "err": "编译失败：\n" + out.strip()[:1200]}

    # 单候选：官方第 1 轮网格的第 755 个（X10 的 top-1 位号）
    outname = "x13_chain_z.bin"
    outpath = os.path.join(HW, outname)
    if os.path.exists(outpath):
        os.remove(outpath)
    K0, K1 = 754, 755
    rc, out, dt = sh([exe_of(HW, "t_x10"), outname, str(K0), str(K1)], HW)

    # 输出布局：每个候选一条记录，(上三角项数 + n_scene) 个 float32。
    # x10_localize 是按 fseek **定位写**的，所以只写第 754 个候选时，
    # 文件长度是 (K1) 条记录——低 k 区留空。首版按「1 条记录」期望长度，
    # 于是 755 x 2,186,136 = 1,650,532,680 字节被误判为「不匹配」。
    tri = 1044 * 1045 // 2
    rec = tri + 1044
    expect = K1 * rec * 4
    got = os.path.getsize(outpath) if os.path.exists(outpath) else -1

    # 再确认第 K0 个候选那条记录**不是全零**——否则「长度对」可能只是稀疏占位
    nonzero = False
    if got == expect:
        with open(outpath, "rb") as f:
            f.seek(K0 * rec * 4)
            blk = f.read(rec * 4)
        import struct
        vals = struct.unpack("<%df" % rec, blk)
        nonzero = any(v != 0.0 for v in vals)

    ok = (rc == 0) and (got == expect) and nonzero
    if os.path.exists(outpath):
        os.remove(outpath)
    return {
        "ok": ok,
        "seconds": dt,
        "out_bytes": got,
        "expect_bytes": expect,
        "tri_entries": tri,
        "cand_record_nonzero": nonzero,
        "k_range": [K0, K1],
        "tail": out.strip().splitlines()[-5:] if out.strip() else [],
        "note": "跑 1 个候选证明链路贯通（判据用输出长度 + 该候选记录非全零）；"
                "精度数取 X10 全网格 972 候选的实测",
    }


def criterion_9():
    """判据 9（附加）：主机侧 C++ vs Python 基线的实测倍数。

    **口径不齐要写明**：A1 基线跑的是 profile_b（裁剪 1920、block 10），
    而 X10 跑的是官方档（裁剪 2016、block 8）。所以这个倍数是**跨口径**的，
    只能当量级参考，不能当严格加速比。
    """
    a1 = 1.0939                  # A1 基线：单候选定位（profile_b）
    cpp = 0.661                  # X10 实测：单候选（官方档）
    j10a = load_json(os.path.join(RES, "x10a_coord_descent_timing.json")) or {}
    stage = j10a.get("stage_breakdown_ms", {})
    return {
        "ok": True,
        "python_per_cand_s": a1,
        "cpp_per_cand_s": cpp,
        "ratio": a1 / cpp,
        "config_mismatch": True,
        "stage_ms": stage,
        "py_972_s": 1007.6,
        "cpp_972_s": cpp * 972,
    }


def dual_depth():
    """双口径：位宽契约在 8 / 12 / 16 bit 三个容器上都自洽。

    相机最终落在 8 bit 还是 12 bit 未定，故两套口径都要能过（X13 的原话：
    把不确定性挡在验收之外）。官方档 block=8 → 块内 64 像素 → EXTRA=6；
    B 档 block=10 → 100 像素 → EXTRA=7。帧累积 EXTRA 同规则。
    """
    # 期望值按 X29 定案的权威公式（权威定义在 src/hardware/preprocess.hpp）：
    #   bs  = PS_IN_BITS + PS_BS_EXTRA + PS_GS_EXTRA
    #   acc = bs + PS_ACC_EXTRA
    #   dif = acc + 1
    # 其中 PS_GS_EXTRA = 1（preprocess.hpp 里的默认值）。
    #
    # ⚠ 这四组期望原先是按**漏掉 PS_GS_EXTRA 的旧公式**填的（22/22/23 那一版），
    #   而 C++ 侧 bitwidth_probe.cpp 一直用的是含 GS_EXTRA 的权威公式、输出一直是
    #   (23,23,24) 这一版。于是本脚本自 X29 之后每次跑都判 FAIL —— 而且因为
    #   [1/5]~[3/5] 与 [5/5] 全绿，很容易被当成「只有位宽那一节在报噪音」而忽略过去。
    #   2026-09-30 按权威公式改正：期望改实测、formula_ok 补上 GS_EXTRA 一项。
    GS_EXTRA = 1
    cases = [
        ("官方档 16bit", 16, 8, 1, 6, 0, (23, 23, 24)),
        ("B 档 16bit", 16, 10, 4, 7, 2, (24, 26, 27)),
        ("相机 12bit", 12, 8, 1, 6, 0, (19, 19, 20)),
        ("相机 8bit", 8, 8, 1, 6, 0, (15, 15, 16)),
    ]
    out = []
    for name, ib, blk, frm, bsx, acx, exp in cases:
        defs = ["-DPS_IN_BITS=%d" % ib, "-DPS_BLOCK=%d" % blk,
                "-DPS_FRAMES=%d" % frm, "-DPS_BS_EXTRA=%d" % bsx,
                "-DPS_ACC_EXTRA=%d" % acx]
        rc, txt, _ = sh([CXX] + CXXFLAGS + defs + ["bitwidth_probe.cpp", "-o", "bw"], HW)
        if rc != 0:
            out.append({"name": name, "ok": False, "err": txt.strip()[:300]})
            continue
        rc, txt, _ = sh([exe_of(HW, "bw")], HW)
        got = {}
        for kv in txt.split():
            if "=" in kv:
                k, v = kv.split("=", 1)
                got[k] = int(v)
        bs, acc, dif = got.get("bs"), got.get("acc"), got.get("dif")
        ok = (bs, acc, dif) == exp
        # 公式自洽：bs = in + BS_EXTRA + GS_EXTRA, acc = bs + ACC_EXTRA, dif = acc + 1
        form = (bs == ib + bsx + GS_EXTRA) and (acc == bs + acx) and (dif == acc + 1)
        out.append({"name": name, "ok": ok and form, "in": ib,
                    "bs": bs, "acc": acc, "dif": dif, "gsum": got.get("gsum"),
                    "expect": list(exp), "formula_ok": form})
    return out


def render_recon():
    """重建出图：把 C++ 内核 dump 出的 u 画成图（不是画 Python 参考解）。"""
    up = os.path.join(HW, "x13_recon_u.bin")
    tp = os.path.join(HW, "hwdata", "x15g_ref_utrue.bin")
    if not os.path.exists(up):
        return None
    import numpy as np
    u = np.fromfile(up, dtype=np.float64)
    t = np.fromfile(tp, dtype=np.float64) if os.path.exists(tp) else None
    os.remove(up)

    def sm_err(a, b):
        al = float(a @ b) / float(a @ a)
        return float(np.linalg.norm(al * a - b) / np.linalg.norm(b))

    def pearson(a, b):
        a = a - a.mean(); b = b - b.mean()
        return float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))

    # rendered 标记本次运行**是否真的出了图**：数值指标（sm_err / r）只依赖 numpy，
    # 出图依赖 matplotlib。报告据此区分「已出图」与「数值达标但未出图」，
    # 不能在没出图时写「已出图」（见 write_report 的判据 8 各栏）。
    met = {"u_inf": float(np.max(np.abs(u))), "rendered": False}
    if t is not None and t.size == u.size:
        met["sm_err"] = sm_err(u, t)
        met["pearson"] = pearson(u, t)

    # 出图是给人看的附加产物，**不在任何判据里**（判据 8(b)「真实数据目视可辨
    # BU」机器不代判，只负责出图，见下面的 recon_ok）。所以 matplotlib 缺失时
    # 跳过出图、保留上面的数值指标 —— 否则一个纯绘图依赖会让整个验收脚本失败，
    # 破坏「一条命令可复现」的承诺。
    # 这条兜底是必需的：`pyproject.toml` 把 matplotlib 放在 optional-dependencies
    # （注释里写明了「避免板卡侧的算法端口被这些重依赖污染」），所以只按核心依赖
    # 安装的环境不会带上它，而 `requirements.txt` 虽然列了 matplotlib，实际装机
    # 未必照它来。
    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt
    except ImportError:
        print("  [提示] 未安装 matplotlib，跳过重建出图（数值指标不受影响）")
        return met

    SHAPE = (29, 36)
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 3.4))
    if t is not None and t.size == u.size:
        ax[0].imshow(t.reshape(SHAPE), cmap="gray")
        ax[0].set_title("真值", fontsize=11)
    ax[1].imshow(u.reshape(SHAPE), cmap="gray")
    title = "移植后 C++ ADMM 内核输出"
    if "pearson" in met:
        title += "\nsm_err=%.2e  r=%+.4f" % (met["sm_err"], met["pearson"])
    ax[1].set_title(title, fontsize=11)
    for a in ax:
        a.set_xticks([]); a.set_yticks([])
    fig.tight_layout()
    p = os.path.join(RES, "x13_reconstruction.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    met["rendered"] = True
    return met


# ------------------------------------------------------------------ 报告

def recon_ok(recon):
    """判据 8(a)：δ=0 自洽合成上，C++ ADMM 内核输出的 sm_err ≤ 1e-2。

    判据 8(b)（真实数据目视可辨 BU）**机器不代判**，只出图，由人确认。
    """
    if not recon:
        return False
    return float(recon.get("sm_err", float("inf"))) <= 1e-2


def write_report(rows, c56, c7, c9, dd, recon, elapsed):
    def mark(ok):
        return "**通过**" if ok else "**未通过**"

    L = []
    A = L.append
    A("# 第一版算法移植验收清单（X13）")
    A("")
    A("- 生成：%s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    A("- 命令：`python build/scripts/acceptance_first_version.py`（本次耗时 %.1f s）" % elapsed)
    A("- 判据出处：[第一版算法移植交付定义](../../01_分析报告/07_单人移植任务表/第一版算法移植交付定义.md) 第二节")
    A("- 说明：本清单**不重新实现算法**，只编译并运行已有的主机侧测试、读入既有实测"
      "产物、并补上双口径位宽对照与重建出图。任何一条不通过都能追到具体二进制或文件。")
    A("")

    n_ok = sum(1 for r in rows if r["ok"])
    A("## 结论")
    A("")
    c56_ok = bool(c56.get("ok"))
    c7_ok = bool(c7.get("ok"))
    dd_ok = all(d.get("ok") for d in dd)
    A("| 项 | 结果 |")
    A("|---|---|")
    A("| 主机侧 C++ 测试 | %d / %d 个二进制全过，共 %d 项断言 |" %
      (n_ok, len(rows), sum(r["pass"] for r in rows)))
    A("| 判据 5/6（定位） | %s |" % mark(c56_ok))
    A("| 判据 7（端到端链） | %s |" % mark(c7_ok))
    A("| 双口径位宽 | %s |" % mark(dd_ok))
    rendered = bool(recon and recon.get("rendered"))
    if recon_ok(recon):
        c8a_word = "**已出图且达标**" if rendered else "**数值达标（未出图）**"
    elif recon:
        c8a_word = "**已出图但超判据**" if rendered else "**数值超判据（未出图）**"
    else:
        c8a_word = "**未出图**"
    A("| 判据 8（重建出图 8a） | %s（sm_err %.4e，判据 1e-2） |" %
      (c8a_word, (recon or {}).get("sm_err", float("nan"))))
    A("| 判据 8（重建出图 8b 目视） | **需人确认**（证据 `sim/results/x14_recon_corrected*.png`）|")
    A("")

    # 八条判据（X21 订正：原七条 + 新增判据 8「M5 重建（重建出图）」）
    A("## 八条判据")
    A("")
    A("| 序 | 模块 | 判据 | 结果 | 读数与依据 |")
    A("|---|---|---|---|---|")

    def by(name):
        for r in rows:
            if r["name"] == name:
                return r
        return None

    m1 = by("M1"); m2 = by("M2"); m3 = by("M3")
    A("| 1 | M1 预处理 | 与 `v1_block_small.npz` 四用例逐值相等 | %s | "
      "来自 `test_preprocess.cpp` 的 %d 项断言 |" %
      (mark(bool(m1 and m1["ok"])), m1["pass"] if m1 else 0))
    A("| 2 | M1 帧累积与差分 | 与 `v3_accum_frames.npz` 六档帧数逐值相等 | %s | "
      "同一二进制，未单列计数（见其输出） |" % mark(bool(m1 and m1["ok"])))
    A("| 3 | M1 分块索引 | 得到 `crossrow_correct` 而非 `crossrow_wrong` | %s | 同上 |"
      % mark(bool(m1 and m1["ok"])))
    A("| 4 | M2 几何算核 | 与 `forward.py` 一致率 ≥ 99%% | %s | "
      "`test_x8_geometry.cpp` %d 项；矩形逐值一致 + A_d 相对误差 3.22e-06 |" %
      (mark(bool(m2 and m2["ok"])), m2["pass"] if m2 else 0))
    A("| 5 | 定位（`Aᵀy` 前缀和路径） | 与 S0-1 快速路径相对误差 ≤ 1e-3 | %s | "
      "**最坏 %.3e**、中位 %.3e（判据 1e-3） |" %
      (mark(c56_ok), c56.get("rel_max") or 0, c56.get("rel_median") or 0))
    def fnum(v):
        if v is None:
            return "n/a"
        try:
            return "+inf" if float(v) == float("inf") else "%.6e" % float(v)
        except (TypeError, ValueError):
            return str(v)

    A("| 6 | 定位（评分与搜索） | 官方第 1 轮网格上 top-1 与 `localization.py` 一致；"
      "**翻转裕度 ε* > 2**（口径见 `x10_verify.py` 的 eps_flip_scan） | %s | "
      "**%d/%d 候选位号一致**，Spearman **%.6f**；**ε* = %s**（首次翻转对手 k=%s，"
      "阈值 %s）；C++ 与参考 top-1 都是 idx %s |" %
      (mark(bool(c56.get("ok"))), c56.get("n_checked") or 0,
       c56.get("n_checked") or 0, c56.get("spearman") or 0,
       fnum(c56.get("eps_star")), c56.get("eps_flip_k"),
       fnum(c56.get("eps_threshold")), (c56.get("cpp_top1") or {}).get("idx")))
    c7_detail = ("耗时 %.2f s，输出 %s 字节（= 上三角 %s + Aty 1044 个 float32）" %
                 (c7["seconds"], "{:,}".format(int(c7.get("out_bytes", -1))),
                  "{:,}".format(int(c7.get("tri_entries", 0))))) if c7_ok else \
        ("输出 %s 字节，期望 %s 字节%s" %
         (c7.get("out_bytes", -1), c7.get("expect_bytes", -1),
          ("；" + str(c7.get("err"))[:300]) if c7.get("err") else ""))
    A("| 7 | 端到端 | 从 RAW 文件到位置输出，一条命令跑通 | %s | %s |" %
      (mark(c7_ok), c7_detail))
    # X21 新增的判据 8。两支分开报：8(a) 可机判，8(b) 只能目视、由人确认。
    r8 = recon_ok(recon)
    if recon and "sm_err" in recon:
        # 没出图（缺 matplotlib）时要写明：8(a) 是数值判据、照常判定，但本次未出图。
        render_note = ("" if recon.get("rendered")
                       else "；本环境未装 matplotlib、**本次未出图**（8a 以数值指标判定）")
        r8_detail = ("(a) C++ 内核 sm_err = **%.4e**（判据 1e-2）%s%s；"
                     "(b) 真实数据目视可辨 BU —— **需人确认**，"
                     "证据 `sim/results/x14_recon_corrected*.png`"
                     % (recon["sm_err"], "" if r8 else " **超判据**", render_note))
    else:
        r8_detail = "未取到内核 dump，判据 8(a) 无法计算"
    A("| 8 | M5 重建（**重建出图**） | (a) 解出自移植后的 C++ ADMM 内核且 `sm_err` ≤ 1e-2；"
      "(b) 真实数据**目视可辨认 BU**（一级判据，机器不代判） | %s | %s |" %
      ("**8(a) 通过**；8(b) 待确认" if r8 else "**8(a) 未通过**", r8_detail))
    A("")

    A("## 附加项")
    A("")
    A("| 序 | 项 | 判据 | 结果 | 读数 |")
    A("|---|---|---|---|---|")
    A("| 9 | HLS 综合 | M1 与定位核各有资源与时序报告 | **未做** | 等 Vivado（约 9/27）与"
      "环境整改（中文用户名/路径），属阶段六 |")
    A("| 10 | 加速比 | 主机侧 C++ vs Python 基线 | **部分** | Python %.4f s/候选 → "
      "主机 C++ %.3f s/候选，**%.2f 倍** |" %
      (c9["python_per_cand_s"], c9["cpp_per_cand_s"], c9["ratio"]))
    A("")
    A("附加项 10 的**口径不齐**必须写明：A1 基线跑的是 profile_b（裁剪 1920、block 10），"
      "而 X10 跑的是官方档（裁剪 2016、block 8）。所以这个倍数是跨口径的，只能当量级"
      "参考。要拿严格加速比，需在**同一档位**上各测一次。")
    A("")
    if c9.get("stage_ms"):
        st = c9["stage_ms"]
        A("X10a 的单候选分节（官方档，ms）：")
        A("")
        A("| 环节 | ms | 占比 |")
        A("|---|---|---|")
        for k in ("build", "diff", "aty", "gram", "eigh", "sum", "total"):
            if k in st:
                A("| %s | %.2f | %.1f%% |" % (k, st[k], 100.0 * st[k] / st["total"]))
        A("")
        A("`build` 一项占 **%.1f%%**（几何/可见性重建），是下一步最值得动的地方——"
          "S0-1 的前缀和路径正是为消掉它而设计的。" % (100.0 * st["build"] / st["total"]))
        A("")

    # 双口径
    A("## 双口径位宽对照")
    A("")
    A("相机最终落在 8 bit 还是 12 bit 尚未确定，故两套口径都要能过——这是把不确定性"
      "挡在验收之外的做法。")
    A("")
    A("| 档 | in | 块和 | 累积 | 差分 | 双绿和 | 期望 (bs,acc,dif) | 结果 |")
    A("|---|---|---|---|---|---|---|---|")
    for d in dd:
        if d.get("err"):
            A("| %s | — | — | — | — | — | — | **编译失败** |" % d["name"])
        else:
            A("| %s | %d | %d | %d | %d | %d | %s | %s |" %
              (d["name"], d["in"], d["bs"], d["acc"], d["dif"], d["gsum"],
               d["expect"], mark(d["ok"])))
    A("")

    # 重建出图
    A("## 重建出图")
    A("")
    if recon:
        A("本结果**出自移植后的 C++ 内核**（`recon.hpp` 的 `m5_reconstruct`，ADMM），"
          "不是 Python 参考解——判据是内核自己跑出来的数。")
        A("")
        A("- `‖u‖∞` = %.4e" % recon.get("u_inf", 0))
        if "sm_err" in recon:
            A("- 与真值的标度不变误差 `sm_err` = **%.4e**" % recon["sm_err"])
            A("- Pearson r = **%+.6f**" % recon["pearson"])
        A("")
        if recon.get("rendered"):
            A("![重建出图](x13_reconstruction.png)")
            A("")
        else:
            A("**本次未出图**：本环境未安装 matplotlib（判据 8(a) 由上面的数值指标"
              "判定，不依赖出图）；需要出图时安装 matplotlib 后重跑本脚本。")
            A("")
        A("该问题（δ=0 自洽合成）在 X15g 里的 Python 参考值是 sm_err 2.4854e-03，"
          "此处内核输出与之同量级，逐值差见 `test_x15g_recon.cpp` 的相对 L2 1.76e-06。")
    else:
        A("**未出图**：未找到内核 dump（`test_x15g_recon.cpp` 需带输出路径参数运行）。")
    A("")

    # 二进制明细
    A("## 主机侧测试明细")
    A("")
    A("| 二进制 | 覆盖 | PASS | FAIL | 结果 |")
    A("|---|---|---|---|---|")
    for r in rows:
        A("| %s | %s | %d | %d | %s |" %
          (r["name"], r["what"], r["pass"], r["fail"], mark(r["ok"])))
    A("")
    bad = [r for r in rows if not r["ok"]]
    if bad:
        A("### 未通过项的原始输出")
        A("")
        for r in bad:
            A("<details><summary>%s</summary>" % r["name"])
            A("")
            A("```")
            A(r["note"][:4000])
            A("```")
            A("</details>")
            A("")
    else:
        A("**全部通过。**")
        A("")

    A("## 本清单**没有**覆盖的")
    A("")
    A("| 项 | 原因 |")
    A("|---|---|")
    A("| 附加项 9 HLS 综合 | 等 Vivado 与环境整改（中文用户名/路径），属阶段六 |")
    A("| 附加项 10 的严格加速比 | 现有两个数跨档位，需同口径复测一次 |")
    A("| 真实相机 RAW → 位置 | 相机实物未到；X12 的三源契约已把接入点冻住，"
      "换源不改上层 |")
    A("| A 的绝对准确性 | 属 X3/X4，卡在 X2（MATLAB 导出） |")
    A("")

    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="（当前无差别，预留）")
    args = ap.parse_args()

    t0 = time.time()
    print("=" * 74)
    print("第一版算法移植验收（X13）")
    print("=" * 74)

    print("\n[1/5] 编译并运行主机侧测试 …")
    rows = criteria_cpp_tests(args.quick)
    for r in rows:
        print("  [%s] %-10s PASS %-3d FAIL %-3d  %s" %
              ("ok  " if r["ok"] else "FAIL", r["name"], r["pass"], r["fail"],
               r["what"][:44]))

    print("\n[2/5] 判据 5/6（读 X10 实测产物）…")
    c56 = criterion_5_6()
    if c56.get("err"):
        print("  [FAIL] " + c56["err"])
    else:
        print("  rel_max=%.3e  rel_median=%.3e  top1_match=%s  spearman=%.6f" %
              (c56["rel_max"], c56["rel_median"], c56["top1_match"], c56["spearman"]))

    print("\n[3/5] 判据 7（端到端链路）…")
    c7 = criterion_7(args.quick)
    if c7.get("ok"):
        print("  ok  链路贯通，%.2f s，输出 %d 字节" %
              (c7["seconds"], c7["out_bytes"]))
    else:
        print("  FAIL 输出 %d 字节，期望 %d 字节" %
              (c7.get("out_bytes", -1), c7.get("expect_bytes", -1)))
        for ln in c7.get("tail", []) or []:
            print("       | " + ln)

    print("\n[4/5] 双口径位宽 …")
    dd = dual_depth()
    for d in dd:
        if d.get("err"):
            print("  [FAIL] %-12s %s" % (d["name"], d["err"][:80]))
        elif d.get("bs") is None:
            print("  [FAIL] %-12s 探针未跑出位宽（编译通过但运行失败）" % d["name"])
        else:
            print("  [%s] %-12s in=%-2d bs=%-2d acc=%-2d dif=%-2d 期望 %s" %
                  ("ok  " if d["ok"] else "FAIL", d["name"], d["in"], d["bs"],
                   d["acc"], d["dif"], d["expect"]))

    print("\n[5/5] 重建出图（出自 C++ 内核）…")
    recon = render_recon()
    if recon:
        print("  ok  ‖u‖∞=%.4e  sm_err=%.4e  r=%+.6f%s" %
              (recon.get("u_inf", 0), recon.get("sm_err", 0), recon.get("pearson", 0),
               "" if recon.get("rendered") else "  （未出图：缺 matplotlib）"))
    else:
        print("  FAIL 未取到内核 dump")

    c9 = criterion_9()
    elapsed = time.time() - t0

    md = write_report(rows, c56, c7, c9, dd, recon, elapsed)
    os.makedirs(RES, exist_ok=True)
    p = os.path.join(RES, "x13_acceptance.md")
    with io.open(p, "w", encoding="utf-8") as f:
        f.write(md)

    # 站定约定：报告与图表在**两处都要有**——仓库 sim/results/ 与 03_任务产出/。
    # 首版漏了这条，导致 03_任务产出 一整天没动。这里做成自动镜像，不靠人记。
    ws = os.path.abspath(os.path.join(ROOT, "..", ".."))
    tdir = os.path.join(ws, "03_任务产出", "X13_第一版验收")
    mirrored = []
    # PNG 只在本次真的出了图时才镜像；否则会把历史残留 PNG 复制过去、与报告
    # “本次未出图”的措辞矛盾。
    mirror_files = ["x13_acceptance.md", "x13_acceptance.json"]
    if recon and recon.get("rendered"):
        mirror_files.append("x13_reconstruction.png")
    try:
        os.makedirs(tdir, exist_ok=True)
        for fn in mirror_files:
            s = os.path.join(RES, fn)
            if os.path.exists(s):
                shutil.copy2(s, os.path.join(tdir, fn))
                mirrored.append(fn)
        # 镜像语义：目标目录里**不在本次产物清单**中的已知产物，删除——否则历史
        # 残留（如降级前复制的 PNG）会与新报告“本次未出图”的措辞矛盾。
        for fn in ("x13_acceptance.md", "x13_acceptance.json",
                   "x13_reconstruction.png"):
            if fn not in mirror_files:
                t = os.path.join(tdir, fn)
                if os.path.exists(t):
                    try:
                        os.remove(t)
                    except OSError:
                        pass
    except OSError as e:
        print("  [warn] 镜像到 03_任务产出 失败：%s" % e)
    payload = {"generated": time.strftime("%Y-%m-%d %H:%M:%S"),
               "elapsed_s": elapsed, "tests": rows,
               "criterion_5_6": c56, "criterion_7": c7, "criterion_9": c9,
               "dual_depth": dd, "recon": recon}
    pj = os.path.join(RES, "x13_acceptance.json")
    with io.open(pj, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    # 清掉编译出的二进制，不污染仓库
    for exe in ("t_m1", "t_m2", "t_m3", "t_m5ops", "t_x15g", "t_x12", "t_x10", "bw"):
        for base in (HW, SW):
            f = exe_of(base, exe)
            if os.path.exists(f):
                try:
                    os.remove(f)
                except OSError:
                    pass

    # 本次未出图时，清掉 sim/results 里的历史 PNG —— 报告不引用它，留着会让
    # 目录状态（有 PNG）与报告措辞（本次未出图）矛盾。与上面二进制清理同类。
    if recon is not None and not recon.get("rendered"):
        stale_png = os.path.join(RES, "x13_reconstruction.png")
        if os.path.exists(stale_png):
            try:
                os.remove(stale_png)
            except OSError:
                pass

    all_ok = (all(r["ok"] for r in rows) and bool(c56.get("ok"))
              and bool(c7.get("ok")) and all(d.get("ok") for d in dd)
              and recon_ok(recon))
    print("\n已保存 %s" % p)
    print("已保存 %s" % pj)
    print("\n%s（耗时 %.1f s）" % ("验收通过。" if all_ok else "存在未通过项。", elapsed))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
