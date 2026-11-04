# -*- coding: utf-8 -*-
"""X11 四轮坐标下降调度：y 上界审计 + 收敛准则定案。

背景（三个已确立的实测事实）
----------------------------
1. A1/X10a：坐标下降 215 次评估，第 4 轮边际收益只有 0.11% —— 固定跑满四轮是浪费。
2. X10：官方第 1 轮 972 个候选的 top-1 是 idx=(13,17,1)，即 **iy=17，y 网格上界 0.65**，
   而真值 y=0.594。前三名全在 iy=17 —— 峰值被网格边界截断。
3. T0-05：能量峰值极窄、顶部平坦。

于是 X11 要回答两件事，都不是调度代码能自己决定的：
    Q1 y 上界要不要外扩？扩了以后峰值落在哪？
    Q2 收敛准则的阈值取多少才既省算力又不丢精度？

本脚本把这两件事做成可跑的实验，再给出调度规格。

调度规格（X1 接口冻结第四节 M4，X10a 已确认队友口径）
----------------------------------------------------
    第 1 轮：5 x 5 x 5 = 125 个候选，覆盖官方外框
    第 2~4 轮：每轮 30 个，按 x -> z -> y 依次一维搜索（每轴 10 点）
    合计 125 + 3 x 30 = 215 次候选评估
    每轮的截断阈值 kappa = (0.75, 0.5, 0.05, 0.05)
    每轮搜索范围缩小 4 倍

用法
----
    python src/model/scripts/x11_schedule.py \
        --data-root "D:\\学习\\26秋嵌入式\\论文数据集\\Ordinary-Camera-master\\Data" \
        --exe "c:\\...\\t_x10.exe" --scratch "c:\\...\\tmp"
"""

from __future__ import annotations

import argparse
import io
import json
import os
import subprocess
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
_REPO = os.path.dirname(os.path.dirname(_SRC_MODEL))
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from x10_verify import projection_energy_from_gram  # noqa: E402

HW = os.path.join(_REPO, "src", "hardware")
N_SCENE = 1044
T_TRI = N_SCENE * (N_SCENE + 1) // 2
PER_CAND = T_TRI + N_SCENE
IU = np.triu_indices(N_SCENE)


def stage_scratch(scratch: str) -> str:
    """准备一个"暂存区"：scratch 与 scratch/hwdata，并补齐 C++ 需要的向量。

    为什么需要这一步
    ----------------
    本机所有可写目录的路径都含中文（用户名 张三 / 工作区目录名），而 Windows 下
    C++ 的窄字符 fopen 走系统 ANSI 码页，**非 ASCII 路径必然乱码**；同时沙箱也不
    允许在盘根另建纯 ASCII 目录。解法是让 C++ 全程只用**裸文件名**：
        cwd 设为暂存区，向量从 ./hwdata/ 读，输入输出都是同目录下的裸名。
    这样传给 C++ 的每一个路径参数都是纯 ASCII，与目录名是否含中文无关。
    """
    os.makedirs(scratch, exist_ok=True)
    hw = os.path.join(scratch, "hwdata")
    os.makedirs(hw, exist_ok=True)
    needed = ("x8_params.txt", "x8_wall_x.bin", "x8_wall_z.bin", "x8_mon_x.bin",
              "x8_mon_z.bin", "x9_yd.bin", "x10_grid.bin", "x10_meta.bin")
    for name in needed:
        src = os.path.join(HW, "hwdata", name)
        dst = os.path.join(hw, name)
        if not os.path.exists(dst) or (os.path.getsize(dst)
                                       != os.path.getsize(src)):
            with open(src, "rb") as fi, open(dst, "wb") as fo:
                fo.write(fi.read())
    return scratch


class Sweeper:
    """把一批候选位置交给 C++ 定位核，拿回投影能量。"""

    def __init__(self, exe: str, scratch: str, kappa: float = 0.75):
        self.exe = exe
        self.scratch = stage_scratch(scratch)
        self.kappa = kappa
        # Python 侧用绝对路径，传给 C++ 的一律是裸文件名（见 stage_scratch）
        self.cand_bin = os.path.join(self.scratch, "x11_cand.bin")
        self.out_bin = os.path.join(self.scratch, "x11_out.bin")
        self.n_eval = 0
        self.t_total = 0.0

    def __call__(self, lls: np.ndarray):
        lls = np.asarray(lls, dtype=np.float64).reshape(-1, 3)
        K = lls.shape[0]
        np.ascontiguousarray(lls, dtype=np.float32).tofile(self.cand_bin)
        t0 = time.time()
        # C++ 的中文输出在 Windows 控制台是 GBK，不能让 subprocess 按 utf-8
        # 解码（会抛 UnicodeDecodeError 并让 stdout 变 None）。按字节取回再容错解码。
        # 候选文件写成 "./x11_cand.bin"：C++ 侧对「不含分隔符的名字」会拼上
        # HWDATA_DIR 前缀（那是给 hwdata 里的向量用的），加 "./" 明确表示这是
        # 相对当前目录的独立路径。输出文件名走的是另一条分支，不需要前缀。
        r = subprocess.run([self.exe, "x11_out.bin", "cand", "./x11_cand.bin"],
                           cwd=self.scratch, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE)
        if r.returncode != 0:
            print((r.stdout or b"")[-2000:].decode("utf-8", "replace"))
            print((r.stderr or b"")[-2000:].decode("utf-8", "replace"))
            raise RuntimeError("sweep 失败，返回码 %d" % r.returncode)
        dt = time.time() - t0
        raw = np.fromfile(self.out_bin, dtype=np.float32)
        if raw.size < K * PER_CAND:
            raise RuntimeError("输出不足：%d < %d" % (raw.size, K * PER_CAND))
        raw = raw[:K * PER_CAND].reshape(K, PER_CAND)

        E = np.empty(K)
        nk = np.empty(K, dtype=int)
        G = np.empty((N_SCENE, N_SCENE))
        for k in range(K):
            tri = raw[k, :T_TRI].astype(np.float64)
            z = raw[k, T_TRI:].astype(np.float64)
            G[IU] = tri
            G[(IU[1], IU[0])] = tri
            e, m, _ = projection_energy_from_gram(G, z, self.kappa)
            E[k] = e
            nk[k] = m
        self.n_eval += K
        self.t_total += dt
        return E, nk


# ---------------------------------------------------------------- 坐标下降

def coord_descent(sw: Sweeper, box, kappa_rounds=(0.75, 0.5, 0.05, 0.05),
                  n_first=5, n_axis=10, shrink=4.0, max_rounds=4,
                  rel_stop=0.0, log=print):
    """官方口径的四轮坐标下降。rel_stop > 0 时启用收敛停止准则。"""
    x0, x1, y0, y1, z0, z1 = box
    xv = np.linspace(x0, x1, n_first)
    yv = np.linspace(y0, y1, n_first)
    zv = np.linspace(z0, z1, n_first)
    XX, YY, ZZ = np.meshgrid(xv, yv, zv, indexing="ij")
    cand = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
    E, nk = sw(cand)
    k_best = int(np.argmax(E))
    best = cand[k_best].copy()
    e_best = float(E[k_best])
    log("  第 1 轮 5x5x5 = %d 个：best = (%.5f, %.5f, %.5f)  E = %.6e  n_kept=%d"
        % (len(cand), best[0], best[1], best[2], e_best, nk[k_best]))

    # 每轮的三轴跨度（第 k 轮 = span1 / shrink^(k-1)）
    span1 = np.array([x1 - x0, y1 - y0, z1 - z0])
    trace = [{"round": 1, "n": int(len(cand)), "best": best.tolist(),
              "E": e_best, "gain_rel": 0.0, "span": span1.tolist(),
              "kappa": 0.75}]

    stopped_at = max_rounds
    for r in range(2, max_rounds + 1):
        span = span1 / (shrink ** (r - 1))
        kap = kappa_rounds[min(r - 1, len(kappa_rounds) - 1)]
        sw.kappa = kap
        cur = best.copy()
        n_axis_eval = 0
        for ax in (0, 2, 1):                    # x -> z -> y
            lo = cur[ax] - span[ax] / 2.0
            hi = cur[ax] + span[ax] / 2.0
            vals = np.linspace(lo, hi, n_axis)
            cand = np.repeat(cur[None, :], n_axis, axis=0)
            cand[:, ax] = vals
            E, nk = sw(cand)
            j = int(np.argmax(E))
            n_axis_eval += n_axis
            if E[j] > e_best:
                e_best = float(E[j])
                cur = cand[j].copy()
        gain = (e_best - trace[-1]["E"]) / trace[-1]["E"]
        best = cur
        log("  第 %d 轮 x->z->y（kappa=%.2f，跨度缩到 %.1f%%）= %d 个："
            "best = (%.5f, %.5f, %.5f)  E = %.6e  增益 %.4f%%"
            % (r, kap, 100.0 / (shrink ** (r - 1)), n_axis_eval,
               best[0], best[1], best[2], e_best, 100.0 * gain))
        trace.append({"round": r, "n": n_axis_eval, "best": best.tolist(),
                      "E": e_best, "gain_rel": float(gain),
                      "span": span.tolist(), "kappa": kap})
        if rel_stop > 0 and gain < rel_stop:
            stopped_at = r
            log("  -> 增益 %.4f%% < 阈值 %.4f%%，第 %d 轮后停止"
                % (100.0 * gain, 100.0 * rel_stop, r))
            break
    sw.kappa = 0.75
    return {"best": best.tolist(), "E": e_best, "trace": trace,
            "n_eval": int(sum(t["n"] for t in trace)), "stopped_at": stopped_at}


# ---------------------------------------------------------------- 主流程

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--exe", required=True)
    ap.add_argument("--scratch", required=True)
    ap.add_argument("--out-dir", default=os.path.join(_REPO, "sim", "results"))
    ap.add_argument("--n-scan", type=int, default=41)
    args = ap.parse_args()

    grid = np.fromfile(os.path.join(HW, "hwdata", "x10_grid.bin"),
                       dtype=np.float32)
    meta = np.fromfile(os.path.join(HW, "hwdata", "x10_meta.bin"),
                       dtype=np.int32)
    nx, ny, nz = int(meta[0]), int(meta[1]), int(meta[2])
    xv = grid[:nx].astype(np.float64)
    yv = grid[nx:nx + ny].astype(np.float64)
    zv = grid[nx + ny:].astype(np.float64)
    box1 = (float(xv.min()), float(xv.max()), float(yv.min()), float(yv.max()),
            float(zv.min()), float(zv.max()))
    print("[info] 官方第 1 轮外框：x [%.5f, %.5f]  y [%.5f, %.5f]  z [%.5f, %.5f]"
          % box1)

    truth = np.array([0.506, 0.594, 0.242])
    top1 = np.array([xv[13], yv[17], zv[1]])
    print("[info] 真值 (0.50600, 0.59400, 0.24200)")
    print("[info] X10 穷举 top-1 = (%.5f, %.5f, %.5f)，y 落在上界"
          % tuple(top1))

    sw = Sweeper(args.exe, args.scratch)
    out = {"box1": list(box1), "truth": truth.tolist(), "top1_x10": top1.tolist()}

    # ================================================================
    # E1 y 上界审计：沿 y 扫，看峰值到底在哪
    # ================================================================
    print("\n=== E1 y 上界审计 ===")
    ys = np.linspace(0.35, 1.00, args.n_scan)
    out["e1"] = {}
    for tag, xz in (("在上界点(%s)的 (x,z)" % "0.52941,0.2", (top1[0], top1[2])),
                    ("在真值点(0.506,0.242)的 (x,z)", (truth[0], truth[2]))):
        cand = np.stack([np.full(ys.size, xz[0]), ys,
                         np.full(ys.size, xz[1])], axis=1)
        E, nk = sw(cand)
        j = int(np.argmax(E))
        print("  %s：" % tag)
        print("    扫描 y 从 %.3f 到 %.3f（%d 点）" % (ys[0], ys[-1], ys.size))
        print("    argmax y = %.5f  E = %.6e   n_kept=%d"
              % (ys[j], E[j], nk[j]))
        print("    y=0.65 处 E=%.6e；y=0.594(真值) 处 E=%.6e；y=1.00 处 E=%.6e"
              % (E[np.argmin(abs(ys - 0.65))], E[np.argmin(abs(ys - 0.594))],
                 E[-1]))
        # 注意：本扫描步长 %.5f m，而官方 y 网格步长也有 %.5f m。这个分辨率
        # 不足以断言峰的位置 —— 若 x10 的官方点在两次采样之间，峰会被漏掉。
        # E4 用 0.002 m 步长重扫，并以那里为准。
        print("    [分辨率警示] 扫描步长 %.5f m，官方 y 网格步长 %.5f m；"
              "不足以定位峰，峰位以 E4 为准"
              % (ys[1] - ys[0], 0.30 / 17.0))
        out["e1"][tag] = {"ys": ys.tolist(), "E": E.tolist(),
                          "argmax_y": float(ys[j]), "argmax_E": float(E[j]),
                          "n_kept": int(nk[j]), "step": float(ys[1] - ys[0])}

    # 沿 x、z 的一维扫描（外框内，看另外两轴是否也被切）
    out["e1_xz"] = {}
    for tag, ax, vals0 in (("x", 0, np.linspace(box1[0], box1[1], args.n_scan)),
                           ("z", 2, np.linspace(box1[4], box1[5], args.n_scan))):
        cand = np.repeat(top1[None, :], vals0.size, axis=0)
        cand[:, ax] = vals0
        E, nk = sw(cand)
        j = int(np.argmax(E))
        at_edge = (j == 0) or (j == vals0.size - 1)
        print("  沿 %s 扫（外框内 %d 点）：argmax = %.5f  E = %.6e  在边界? %s"
              % (tag, vals0.size, vals0[j], E[j], "是" if at_edge else "否"))
        out["e1_xz"][tag] = {"vals": vals0.tolist(), "E": E.tolist(),
                             "argmax": float(vals0[j]), "at_edge": bool(at_edge)}

    # ================================================================
    # E2 四轮调度：官方外框（y 上界 0.65）vs 外扩 y 上界
    # ================================================================
    print("\n=== E2 四轮坐标下降（固定跑满四轮，看逐轮增益）===")
    print("  情形 A：官方外框 y 上界 = %.5f" % box1[3])
    a = coord_descent(sw, box1, max_rounds=4, rel_stop=0.0)
    da = float(np.linalg.norm(np.array(a["best"]) - truth))
    print("    最终 (%.5f, %.5f, %.5f)  距真值 %.5f m  评估 %d 次"
          % (a["best"][0], a["best"][1], a["best"][2], da, a["n_eval"]))

    box_y = (box1[0], box1[1], box1[2], 0.90, box1[4], box1[5])
    print("\n  情形 B：y 上界外扩到 %.5f（X10 建议）" % box_y[3])
    b = coord_descent(sw, box_y, max_rounds=4, rel_stop=0.0)
    db = float(np.linalg.norm(np.array(b["best"]) - truth))
    print("    最终 (%.5f, %.5f, %.5f)  距真值 %.5f m  评估 %d 次"
          % (b["best"][0], b["best"][1], b["best"][2], db, b["n_eval"]))

    print("\n  情形 C：y 上界外扩 + 跑到 6 轮（看是否还在涨）")
    box_y6 = (box1[0], box1[1], box1[2], 0.90, box1[4], box1[5])
    c = coord_descent(sw, box_y6, max_rounds=6, rel_stop=0.0)
    dc = float(np.linalg.norm(np.array(c["best"]) - truth))
    print("    最终 (%.5f, %.5f, %.5f)  距真值 %.5f m  评估 %d 次"
          % (c["best"][0], c["best"][1], c["best"][2], dc, c["n_eval"]))

    out["e2"] = {"A_official_box": a, "B_y_extended": b, "C_y_ext_6rounds": c,
                 "dist_to_truth": {"A": da, "B": db, "C": dc}}

    def dump() -> None:
        """E1/E2 是全脚本最贵的部分（约 870 次候选评估），先落一次盘，
        后面任何一步出错都不至于把结果一起丢掉。"""
        os.makedirs(args.out_dir, exist_ok=True)
        with io.open(os.path.join(args.out_dir, "x11_schedule.json"), "w",
                     encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)

    dump()

    # ================================================================
    # E3 收敛准则：直接由 C 的 trace 推出，不额外跑
    #   每一轮的状态只依赖上一轮，所以「跑到第 r 轮就停」与「跑满 6 轮后
    #   取第 r 轮的状态」完全等价 —— 早停的轨迹就是长轨迹的前缀。
    #   于是不需要为每个阈值重跑一遍。
    # ================================================================
    print("\n=== E3 收敛准则（阈值取多少）===")
    print("  各轮增益（情形 C，6 轮）：")
    gains = [t["gain_rel"] for t in c["trace"][1:]]
    for i, g in enumerate(gains, start=2):
        # trace 的下标是 round-1，不要写成 trace[i]
        st = c["trace"][i - 1]
        print("    第 %d 轮增益 %.5f%%   best=(%.5f, %.5f, %.5f)  距真值 %.5f m"
              % (i, 100.0 * g, st["best"][0], st["best"][1], st["best"][2],
                 float(np.linalg.norm(np.array(st["best"]) - truth))))
    print("\n  由 trace 直接推出各阈值下的表现（早停轨迹 = 长轨迹的前缀）：")
    rows = []
    n_rounds = len(c["trace"])
    for eps in (1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2, 3e-2):
        r_stop = n_rounds
        for i in range(1, n_rounds):
            if c["trace"][i]["gain_rel"] < eps:
                r_stop = c["trace"][i]["round"]
                break
        st = c["trace"][r_stop - 1]
        n_eval = int(sum(t["n"] for t in c["trace"][:r_stop]))
        rows.append({"eps": eps, "stopped_at": r_stop, "n_eval": n_eval,
                     "saving_pct": 100.0 * (1 - n_eval / c["n_eval"]),
                     "E": st["E"],
                     "E_loss_rel": (c["E"] - st["E"]) / c["E"],
                     "best": st["best"],
                     "dist_to_truth": float(np.linalg.norm(
                         np.array(st["best"]) - truth))})
        print("    阈值 %7.0e：第 %d 轮停，评估 %3d 次（省 %4.1f%%），"
              "能量相对损失 %.3e，距真值 %.5f m"
              % (eps, r_stop, n_eval, rows[-1]["saving_pct"],
                 rows[-1]["E_loss_rel"], rows[-1]["dist_to_truth"]))
    out["e3"] = rows
    # 取值建议：要求省 >= 20%，且能量相对损失 <= 1e-3，且距真值不比跑满更差
    pick = [r for r in rows if r["saving_pct"] >= 20.0
            and r["E_loss_rel"] <= 1e-3
            and r["dist_to_truth"] <= c["dist_to_truth"] + 1e-6]
    if pick:
        best_pick = min(pick, key=lambda r: r["E_loss_rel"])
        print("\n  建议阈值：%.0e —— 第 %d 轮停，省 %.1f%%，"
              "能量相对损失 %.3e，距真值 %.5f m"
              % (best_pick["eps"], best_pick["stopped_at"],
                 best_pick["saving_pct"], best_pick["E_loss_rel"],
                 best_pick["dist_to_truth"]))
        out["e3_pick"] = best_pick
    else:
        print("\n  **没有阈值同时满足「省 >= 20%」与「能量损失 <= 1e-3」。**")
        print("  原因是第 3 轮的增益还有 %.2f%%，要停下就得把阈值抬到 8%% 以上，"
              % (100.0 * c["trace"][2]["gain_rel"]))
        print("  那会丢掉第 3 轮的 12% 增益。**按能量增益判停，在 4 轮预算内省不下东西。**")
        out["e3_pick"] = None

    # ================================================================
    # E3b 用「跨度」而不是「能量增益」判停
    #   能量增益在 4 轮预算内无法用来省算力（见上）。但有一条更结实的判据：
    #   模型本身的定位精度只有 ~0.05 m，把搜索跨度细化到远小于它没有意义。
    #   第 r 轮的最大轴跨度 = span1_x / 4^(r-1)。
    # ================================================================
    print("\n=== E3b 按「跨度」判停（阈值 s_min：跨度小于它就不再开新一轮）===")
    # 缩小倍数从 trace 里推，不用 coord_descent 的局部变量
    spans = [t["span"][0] for t in c["trace"]]
    shrink = (spans[-2] / spans[-1]) if (len(spans) >= 2 and spans[-1] > 0) else 4.0
    span_x = box1[1] - box1[0]
    print("  （由 trace 推出每轮跨度缩小 %.2f 倍；第 r 轮最大轴跨度 = %.4f / %.0f^(r-1)）"
          % (shrink, span_x, shrink))
    rows_b = []
    for s_min in (0.200, 0.100, 0.050, 0.020, 0.010, 0.005):
        r_run = 1
        for r in range(2, n_rounds + 1):
            if span_x / (shrink ** (r - 1)) >= s_min:
                r_run = r
            else:
                break
        st = c["trace"][r_run - 1]
        n_eval = int(sum(t["n"] for t in c["trace"][:r_run]))
        rows_b.append({"s_min": s_min, "rounds_run": r_run, "n_eval": n_eval,
                       "saving_pct": 100.0 * (1 - n_eval / c["n_eval"]),
                       "next_span": span_x / (shrink ** r_run),
                       "E_loss_rel": (c["E"] - st["E"]) / c["E"],
                       "dist_to_truth": float(np.linalg.norm(
                           np.array(st["best"]) - truth))})
        print("   s_min = %.4f m：跑 %d 轮，评估 %3d 次（省 %4.1f%%），"
              "下一轮跨度 %.5f m，能量损失 %.3e，距真值 %.5f m"
              % (s_min, r_run, n_eval, rows_b[-1]["saving_pct"],
                 rows_b[-1]["next_span"], rows_b[-1]["E_loss_rel"],
                 rows_b[-1]["dist_to_truth"]))
    out["e3b_span"] = rows_b

    # ================================================================
    # E4 y 剖面的细结构
    #   E1 的 41 点扫描步长 0.01625 m，而官方 y 网格步长 0.01765 m —— 都太粗，
    #   定位不到峰。这里在 [0.60, 0.70] 上以 0.002 m 步长细扫，并在同一批里
    #   **显式放入官方网格点 y=0.65**，用来与 E_round1[13,17,1] 做一致性核对。
    # ================================================================
    print("\n=== E4 y 剖面细结构（步长 0.002 m，含官方点 y=0.65）===")
    ys_fine = np.linspace(0.60, 0.70, 51)
    out["e4"] = {}
    for tag, xz in (("官方 top-1 的 (x,z)=(0.52941,0.2)", (0.5294118, 0.2)),
                    ("情形 A 第 3 轮的 (x,z)=(0.52813,0.20625)", (0.52813, 0.20625))):
        cand = np.stack([np.full(ys_fine.size, xz[0]), ys_fine,
                         np.full(ys_fine.size, xz[1])], axis=1)
        E, nk = sw(cand)
        j = int(np.argmax(E))
        i65 = int(np.argmin(abs(ys_fine - 0.65)))
        print("  %s：" % tag)
        print("    argmax y = %.4f  E = %.6e   n_kept=%d"
              % (ys_fine[j], E[j], nk[j]))
        print("    E(y=0.65) = %.6e   （若与官方 E_round1 对上，说明链路一致）"
              % E[i65])
        print("    峰两侧：y=%.4f E=%.4e | 峰 %.4f E=%.4e | y=%.4f E=%.4e"
              % (ys_fine[max(j - 1, 0)], E[max(j - 1, 0)], ys_fine[j], E[j],
                 ys_fine[min(j + 1, ys_fine.size - 1)],
                 E[min(j + 1, ys_fine.size - 1)]))
        # 峰的半高宽（粗略）：E 降到峰值一半的宽度
        half = E >= 0.5 * E[j]
        w = (ys_fine[half].max() - ys_fine[half].min()) if half.any() else 0.0
        print("    E >= 半峰值的 y 宽度 ≈ %.4f m（扫描步长 0.002）" % w)
        out["e4"][tag] = {"ys": ys_fine.tolist(), "E": E.tolist(),
                          "argmax_y": float(ys_fine[j]), "argmax_E": float(E[j]),
                          "E_at_0p65": float(E[i65]), "half_width": float(w)}

    dump()
    print("\n[info] 总评估 %d 次，C++ 侧累计 %.1f s" % (sw.n_eval, sw.t_total))
    print("[done] 明细写入 %s/x11_schedule.json" % args.out_dir)


if __name__ == "__main__":
    main()
