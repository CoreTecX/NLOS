# -*- coding: utf-8 -*-
"""src/fpga/sim/rtl_xsim_driver.py —— RTL 五模块 xsim 门禁的公共驱动。

用途
----
本目录四份 `test_*.py`（raw_preproc / vertical_diff / fast_alert / confidence）
共用本文件：对每个被验模块串行执行 xsim 的三步
（`xvlog -sv` -> `xelab` -> `xsim -R`），再对日志做判读。

判据（任一不成立即非零退出）
---------------------------
1. `xvlog` / `xelab` / `xsim` 三步退出码都必须为 0。
2. 日志里必须出现该 TB 自己打印的 `=== <模块>_tb : PASS ===` 标记。
3. 标记里的「比对点数」必须等于本驱动的**钉死期望值**（逐模块给出）。
   比对点少一个即 FAIL —— 这条拦住「TB 被悄悄改弱、PASS 依旧」。
4. 日志里不得出现 `FAIL` 标记，也不得出现 `[ERR` 行。

逐值断言在哪里
--------------
逐值/逐拍严格相等断言在 SV 侧 TB 内部（`err_cnt` 是错误计数、`chk_cnt` 是
比对点计数）；本驱动只做「调用 + 严格判读」，不生成激励、不替代 TB 断言。
把断言点数钉在 Python 侧，是为了让「门禁」能发现断言集被缩小。

与 run_xsim.ps1 的关系
----------------------
`run_xsim.ps1` 是「一键跑完 5 个 TB」的脚本。本驱动是「逐模块、可当门禁」的
Python 版，判据一致，差别在于：
  * **不传 `--timescale`**。五个 RTL 模块已在 2026-09-29 补 `timescale 1ns/1ps`
    （与 TB 同值），xsim 不再需要命令行补默认值；本驱动不带该开关本身即是
    「RTL 已有 timescale」这条修复的验证（见
    sim/results/xsim_first_run_20260929.md §6.1）。
  * 把「比对点数」与钉死期望值比对（见判据 3）。
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess

# 路径来源（2026-10-02 起）：**环境变量优先，默认值保留**，默认仍能在本机跑。
#   PS_VIVADO_BIN        Vivado bin 目录
#   XILINXD_LICENSE_FILE License 文件
#   PS_TEMP_DIR          xsim 中间文件目录
# 命令行参数（--vivado-bin / --license / --temp-dir）优先级最高。
DEFAULT_VIVADO_BIN = os.environ.get("PS_VIVADO_BIN", r"D:\vivado\2025.2\Vivado\bin")
DEFAULT_LICENSE = os.environ.get("XILINXD_LICENSE_FILE",
                                 r"D:\xilinx_lic\Xilinx_official_DRURY.lic")
DEFAULT_TEMP = os.environ.get("PS_TEMP_DIR", r"D:\Temp")

# 只依赖 ASCII：'(比对点 N 个)' 里中文可能被控制台按 GBK 编码后乱码，
# 故用 [^0-9]* 跳过括号内的非数字前缀，只认数字本身。
_PASS_RE = re.compile(r"===\s+(\w+_tb)\s*:\s*PASS\s*===\s*\([^0-9]*(\d+)")
_FAIL_RE = re.compile(r"===\s+\w+_tb\s*:\s*FAIL")
_ERR_RE = re.compile(r"\[ERR")


def repo_root_from_here() -> str:
    """从 src/fpga/sim 向上三级得到仓库根。"""
    here = os.path.dirname(os.path.abspath(__file__))          # src/fpga/sim
    return os.path.abspath(os.path.join(here, "..", "..", ".."))


def _child_env(vivado_bin, license_file, temp_dir):
    env = dict(os.environ)
    env["PATH"] = vivado_bin + os.pathsep + env.get("PATH", "")
    if license_file:
        env["XILINXD_LICENSE_FILE"] = license_file
    if temp_dir and os.path.isdir(temp_dir):
        env["TEMP"] = temp_dir
        env["TMP"] = temp_dir
    return env


def _decode(raw: bytes) -> str:
    for enc in ("utf-8", "gbk"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode("latin-1", errors="replace")


def _run_bat(bat, args, cwd, env):
    """经 cmd.exe 调 .bat（CreateProcess 不能直接执行批处理）。"""
    cmd = ["cmd.exe", "/c", bat] + list(args)
    p = subprocess.run(cmd, cwd=cwd, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return p.returncode, _decode(p.stdout)


def _write_log(log_path, text):
    with open(log_path, "w", encoding="utf-8", errors="replace") as f:
        f.write(text)


def run_module(module: str, expected_points: int, *, repo_root=None,
               work_root=None, vivado_bin=DEFAULT_VIVADO_BIN,
               license_file=DEFAULT_LICENSE, temp_dir=DEFAULT_TEMP,
               ext="sv", extra=None):
    """跑一个 RTL 模块的 TB。返回 (ok: bool, row: dict)。

    ext 是**核体**的文件扩展名（默认 "sv"）。新增的两个模块
    （axis_frame_sync / pl_ctrl_regs）刻意写成**纯 Verilog `.v`**：
    它们要被块设计当 module reference 引用，而 Vivado 不接受 SystemVerilog
    顶层（[filemgmt 56-195]，见 sim/results/bd_full_path_20260930.md §2.4），
    写成 .v 就不需要再配一份 pl_wrap/ 结构壳。TB 仍统一是 `.sv`。

    extra 是**同目录下需要一起编进来的附加 RTL 文件**（相对 src/fpga/rtl 的
    文件名列表，如 ["sat_ratio_div.v"]）。pl_ctrl_regs 内部实例化
    sat_ratio_div，故它的门禁必须把后者一起交给 xvlog，否则 xelab 报
    "module not found"。默认 None 时行为与以前逐字相同（四个既有门禁不受影响）。
    """
    repo_root = repo_root or repo_root_from_here()
    rtl = os.path.join(repo_root, "src", "fpga", "rtl", module + "." + ext)
    tb = os.path.join(repo_root, "src", "fpga", "rtl", "tb", module + "_tb.sv")
    tb_name = module + "_tb"
    rtl_dir = os.path.join(repo_root, "src", "fpga", "rtl")
    extra_files = [os.path.join(rtl_dir, f) for f in (extra or [])]
    snap = "sim_" + module
    work = work_root or os.path.join(repo_root, "build", "xsim_py")
    work_dir = os.path.join(work, module)
    os.makedirs(work_dir, exist_ok=True)
    log_path = os.path.join(work_dir, "xsim_run.log")

    row = {"module": module, "tb": tb_name, "expected_points": expected_points,
           "xvlog": None, "xelab": None, "xsim": None, "points": None,
           "verdict": "FAIL", "reason": ""}

    xvlog = os.path.join(vivado_bin, "xvlog.bat")
    xelab = os.path.join(vivado_bin, "xelab.bat")
    xsim = os.path.join(vivado_bin, "xsim.bat")
    for f in [rtl, tb] + extra_files:
        if not os.path.isfile(f):
            row["reason"] = "缺文件 " + f
            _write_log(log_path, row["reason"])
            return False, row
    for tool in (xvlog, xelab, xsim):
        if not os.path.isfile(tool):
            row["reason"] = "找不到工具 " + tool
            _write_log(log_path, row["reason"])
            return False, row

    env = _child_env(vivado_bin, license_file, temp_dir)
    chunks = []
    rc, out = _run_bat(xvlog, ["-sv", rtl, tb] + extra_files, work_dir, env)
    row["xvlog"] = rc
    chunks.append("### xvlog rc=%s\n%s" % (rc, out))

    if rc == 0:
        rc, out = _run_bat(xelab, [tb_name, "-s", snap], work_dir, env)
        row["xelab"] = rc
        chunks.append("### xelab rc=%s\n%s" % (rc, out))

    if row["xelab"] == 0:
        rc, out = _run_bat(xsim, [snap, "-R"], work_dir, env)
        row["xsim"] = rc
        chunks.append("### xsim rc=%s\n%s" % (rc, out))

    text = "\n".join(chunks)
    _write_log(log_path, text)
    row["log"] = log_path

    m = _PASS_RE.search(text)
    fail = _FAIL_RE.search(text)
    if m and m.group(1) == tb_name:
        row["points"] = int(m.group(2))

    ok = (row["xvlog"] == 0 and row["xelab"] == 0 and row["xsim"] == 0
          and m is not None and m.group(1) == tb_name
          and not fail and not _ERR_RE.search(text)
          and row["points"] == expected_points)
    if ok:
        row["verdict"] = "PASS"
        return True, row

    reasons = []
    if row["xvlog"] != 0:
        reasons.append("xvlog rc=%s" % row["xvlog"])
    if row["xelab"] != 0:
        reasons.append("xelab rc=%s" % row["xelab"])
    if row["xsim"] != 0:
        reasons.append("xsim rc=%s" % row["xsim"])
    if m is None:
        reasons.append("缺 PASS 标记")
    elif m.group(1) != tb_name:
        reasons.append("PASS 标记属于 %s" % m.group(1))
    if fail:
        reasons.append("出现 FAIL 标记")
    if _ERR_RE.search(text):
        reasons.append("出现 [ERR 行")
    if row["points"] is None:
        reasons.append("未读到比对点数")
    elif row["points"] != expected_points:
        reasons.append("比对点 %s != 期望 %s" % (row["points"], expected_points))
    row["reason"] = "; ".join(reasons)
    return False, row


def cli(pairs, argv=None, ext="sv", extra=None):
    """pairs = [(module, expected_points), ...]，严格串行，任一失败即返回 1。

    ext 透传给 run_module（默认 "sv"，见那里的说明；新增的 .v 核传 "v"）。
    extra 也透传给 run_module（同目录附加 RTL 文件列表；pl_ctrl_regs 需要
    ["sat_ratio_div.v"]）。
    """
    ap = argparse.ArgumentParser(
        description="RTL 模块 xsim 门禁（严格逐值判据；逐模块串行）")
    ap.add_argument("--repo-root", default=None)
    ap.add_argument("--work-dir", default=None)
    ap.add_argument("--vivado-bin", default=DEFAULT_VIVADO_BIN)
    ap.add_argument("--license", default=DEFAULT_LICENSE)
    ap.add_argument("--temp-dir", default=DEFAULT_TEMP)
    args = ap.parse_args(argv)

    ok_all = True
    for module, expected in pairs:
        ok, row = run_module(module, expected, repo_root=args.repo_root,
                             work_root=args.work_dir, vivado_bin=args.vivado_bin,
                             license_file=args.license, temp_dir=args.temp_dir,
                             ext=ext, extra=extra)
        if ok:
            print("[%s] PASS  比对点 %d  (xvlog=0 xelab=0 xsim=0)"
                  % (row["tb"], row["points"]))
        else:
            print("[%s] FAIL  %s" % (row["tb"], row["reason"]))
            ok_all = False
    return 0 if ok_all else 1
