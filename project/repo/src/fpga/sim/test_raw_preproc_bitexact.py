# -*- coding: utf-8 -*-
"""src/fpga/sim/test_raw_preproc_bitexact.py
—— raw_preproc 与 Python 定点参考链的 bit-exact 对拍（RTL README 的 B4）。

做了什么
--------
1. Python 生成一份**中间格式**的输入文件（纯文本，逐行十进制），内容 = NFRM 帧
   RAW16 光栅序码字 + 头部（帧数、暗电平）；
2. 同一份文件喂给 RTL 仿真（`raw_preproc_bitexact_tb.sv` 读它）；
3. Python 侧用 `src/model/penumbrasight_model/preprocess.block_sum` 独立算参考块和；
4. 读回 RTL 的输出文件（每个输出拍一行）与参考**逐值严格相等**比对。

为什么能 bit-exact（以及唯一一处刻意的语义差异）
----------------------------------------------
RTL `raw_preproc` 的口径（见 `src/fpga/rtl/README.md` §4 与 `raw_preproc.sv` 的
[R1]/[R2]）是：**先减暗电平并负值饱和到 0 -> 按 2x2 相位取 R/G1/G2/B ->
10x10（本小尺寸 2x2）块内**求和** -> 双绿取**和** G1+G2 -> 帧间**直接求和**（不移位）**。
全过程只有整数加法，没有除法/舍入，所以逐值严格相等是可达的判据。

`preprocess.py` 的浮点链在**两处**与 RTL 不同，这是设计上写明的差异，不是 bug：
  · 它在分块后取**平均**（`block_mean`），RTL 取块**和**（P1 原则）。
    本脚本改用 `preprocess.block_sum`，与 RTL 同口径。
  · 它的双绿取**算术平均**（`(G1+G2)/2`，数据契约 R2），RTL 取**和**（P2，多 1 bit，
    `raw_preproc` 的 [R1]）。本脚本取**和**，并在此显式记录这一处差异。
其余（相位、块不跨行、帧间求和、暗电平先减后饱和）两边一致。

规模与局限（如实写清）
--------------------
  · 尺寸：BLOCK=2 / WALL_N=4 -> 一帧 16x16，输出 16 块 x 3 通道 = 48 拍。
    **不是** 1920x1920 全尺寸（全尺寸的评估见
    `sim/results/preboard_test_gap_20260930.md` §任务6）。
  · 覆盖：块和、双绿求和、暗电平饱和、帧累积、通道顺序、tch/tlast/tuser、
    以及 `m_meas_tblk` 的索引公式。
  · 未覆盖：全尺寸地址链、`sat_pix_cnt/total_pix_cnt`、fullscale 打包口径、
    黑白单通道路径（后者在 `raw_preproc_tb` 里）。

用法
----
    python src/fpga/sim/test_raw_preproc_bitexact.py
    python src/fpga/sim/test_raw_preproc_bitexact.py --tool iverilog
退出码：全部用例逐值相等 -> 0，否则非零。
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys

import numpy as np

# 路径来源（2026-10-02 起）：**环境变量优先，默认值保留**，默认仍能在本机跑。
#   PS_VIVADO_BIN        Vivado bin 目录
#   XILINXD_LICENSE_FILE License 文件
#   PS_TEMP_DIR          xsim 中间文件目录
#   PS_IVERILOG_BIN      Icarus Verilog bin 目录
# 命令行参数（--vivado-bin / --license / --temp-dir / --iverilog-bin）优先级最高。
DEFAULT_VIVADO_BIN = os.environ.get("PS_VIVADO_BIN", r"D:\vivado\2025.2\Vivado\bin")
DEFAULT_LICENSE = os.environ.get("XILINXD_LICENSE_FILE",
                                 r"D:\xilinx_lic\Xilinx_official_DRURY.lic")
DEFAULT_TEMP = os.environ.get("PS_TEMP_DIR", r"D:\Temp")
DEFAULT_IVERILOG = os.environ.get("PS_IVERILOG_BIN", r"D:\iverilog\bin")

WALL_N = 4
BLOCK = 2
RAW_SIDE = 2 * BLOCK * WALL_N          # 16
NPIX = RAW_SIDE * RAW_SIDE             # 256
NBLK = WALL_N * WALL_N                 # 16
NBEAT = NBLK * 3                       # 48
BW = (NBLK - 1).bit_length()           # = RTL 的 $clog2(WALL_N*WALL_N) = 4

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
_RTL = os.path.join(_REPO, "src", "fpga", "rtl")
_TB = os.path.join(_RTL, "tb", "raw_preproc_bitexact_tb.sv")
_DUT = os.path.join(_RTL, "raw_preproc.sv")
_MODEL = os.path.join(_REPO, "src", "model")
if _MODEL not in sys.path:
    sys.path.insert(0, _MODEL)

from penumbrasight_model.preprocess import block_sum          # noqa: E402


# ---------------------------------------------------------------- 输入与参考

def make_frames(nfrm: int) -> np.ndarray:
    """确定性 RAW 帧：(nfrm, RAW_SIDE, RAW_SIDE) 的 int64，取值落在 packed 12 bit。"""
    f = np.arange(nfrm, dtype=np.int64)[:, None, None]
    r = np.arange(RAW_SIDE, dtype=np.int64)[None, :, None]
    c = np.arange(RAW_SIDE, dtype=np.int64)[None, None, :]
    return (r * 37 + c * 101 + f * 197) % 4093


def rtl_reference(frames: np.ndarray, dark: int) -> np.ndarray:
    """逐位复刻 raw_preproc 的块和（见文件头「语义差异」）。

    返回 (WALL_N, WALL_N, 3) 的 int64，通道序 R/G/B，G = G1 + G2（取和，不取平均）。
    """
    a = frames.astype(np.int64) - int(dark)
    a = np.clip(a, 0, None)                    # 先减后处理，负值饱和到 0
    acc = np.zeros((WALL_N, WALL_N, 3), dtype=np.int64)
    for fi in range(a.shape[0]):
        plane = a[fi]
        # 相位：RGGB，(偶,偶)=R (偶,奇)=G1 (奇,偶)=G2 (奇,奇)=B
        r_ = block_sum(plane[0::2, 0::2], BLOCK)
        g1 = block_sum(plane[0::2, 1::2], BLOCK)
        g2 = block_sum(plane[1::2, 0::2], BLOCK)
        b_ = block_sum(plane[1::2, 1::2], BLOCK)
        acc[:, :, 0] += r_
        acc[:, :, 1] += g1 + g2                # 双绿取和（RTL [R1] / 原则 P2）
        acc[:, :, 2] += b_
    return acc


def write_input(path: str, frames: np.ndarray, dark: int) -> None:
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("%d %d\n" % (frames.shape[0], int(dark)))
        for v in frames.reshape(-1):
            fh.write("%d\n" % int(v))


# ---------------------------------------------------------------- 运行仿真

def _decode(raw: bytes) -> str:
    for enc in ("utf-8", "gbk"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode("latin-1", errors="replace")


def _run(cmd, cwd, env):
    p = subprocess.run(cmd, cwd=cwd, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return p.returncode, _decode(p.stdout)


def run_xsim(work_dir, in_name, out_name, vivado_bin, license_file, temp_dir):
    for f in (os.path.join(vivado_bin, "xvlog.bat"),
              os.path.join(vivado_bin, "xelab.bat"),
              os.path.join(vivado_bin, "xsim.bat")):
        if not os.path.isfile(f):
            return False, "找不到工具 %s" % f, ""
    env = dict(os.environ)
    env["PATH"] = vivado_bin + os.pathsep + env.get("PATH", "")
    if license_file:
        env["XILINXD_LICENSE_FILE"] = license_file
    if temp_dir and os.path.isdir(temp_dir):
        env["TEMP"] = temp_dir
        env["TMP"] = temp_dir

    log = []
    rc, out = _run(["cmd.exe", "/c", os.path.join(vivado_bin, "xvlog.bat"),
                    "-sv", _DUT, _TB], work_dir, env)
    log.append("### xvlog rc=%s\n%s" % (rc, out))
    if rc != 0:
        return False, "xvlog rc=%s" % rc, "\n".join(log)
    rc, out = _run(["cmd.exe", "/c", os.path.join(vivado_bin, "xelab.bat"),
                    "raw_preproc_bitexact_tb", "-s", "sim_bitexact"], work_dir, env)
    log.append("### xelab rc=%s\n%s" % (rc, out))
    if rc != 0:
        return False, "xelab rc=%s" % rc, "\n".join(log)
    # TB 用 $value$plusargs，但默认值就是工作目录下的固定文件名，故此处不传 plusarg，
    # 依赖 CWD（跨 xsim / iverilog 都可复现，避免工具间的 plusarg 语法差异）。
    rc, out = _run(["cmd.exe", "/c", os.path.join(vivado_bin, "xsim.bat"),
                    "sim_bitexact", "-R"], work_dir, env)
    log.append("### xsim rc=%s\n%s" % (rc, out))
    if rc != 0:
        return False, "xsim rc=%s" % rc, "\n".join(log)
    return True, "", "\n".join(log)


def run_iverilog(work_dir, iverilog_bin):
    iv = os.path.join(iverilog_bin, "iverilog.exe")
    vvp = os.path.join(iverilog_bin, "vvp.exe")
    if not (os.path.isfile(iv) and os.path.isfile(vvp)):
        return False, "找不到 iverilog/vvp（%s）" % iverilog_bin, ""
    log = []
    rc, out = _run([iv, "-g2012", "-o", "bitexact.vvp", _DUT, _TB], work_dir, None)
    log.append("### iverilog rc=%s\n%s" % (rc, out))
    if rc != 0:
        return False, "iverilog rc=%s" % rc, "\n".join(log)
    rc, out = _run([vvp, "bitexact.vvp"], work_dir, None)
    log.append("### vvp rc=%s\n%s" % (rc, out))
    if rc != 0:
        return False, "vvp rc=%s" % rc, "\n".join(log)
    return True, "", "\n".join(log)


_DONE_RE = re.compile(r"===\s+raw_preproc_bitexact_tb\s*:\s*DONE\s*===\s+"
                      r"n_out=(\d+)\s+ovf=(\w)\s+proto=(\w)\s+cfg=(\w)")


def parse_out(path):
    """读回 RTL 输出：每行 5 个数 -> (tdata, tch, tlast, tuser, tblk) 的列表。"""
    rows = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            parts = line.split()
            if len(parts) != 5:
                continue
            rows.append(tuple(int(p) for p in parts))
    return rows


# ---------------------------------------------------------------- 单个用例

def run_case(name, nfrm, dark, *, tool, work_root, vivado_bin, license,
             temp_dir, iverilog_bin):
    frames = make_frames(nfrm)
    ref = rtl_reference(frames, dark)

    work = os.path.join(work_root, name)
    os.makedirs(work, exist_ok=True)
    in_path = os.path.join(work, "bitexact_in.txt")
    out_path = os.path.join(work, "bitexact_out.txt")
    write_input(in_path, frames, dark)
    if os.path.exists(out_path):
        os.remove(out_path)

    if tool == "xsim":
        ok, why, log = run_xsim(work, "bitexact_in.txt", "bitexact_out.txt",
                               vivado_bin, license, temp_dir)
    else:
        ok, why, log = run_iverilog(work, iverilog_bin)
    with open(os.path.join(work, "sim.log"), "w", encoding="utf-8") as fh:
        fh.write(log)
    if not ok:
        return False, "[%s] 仿真失败：%s（日志 %s）" % (name, why, work)

    m = _DONE_RE.search(log)
    if not m:
        return False, "[%s] 日志里没有 DONE 标记（日志 %s）" % (name, work)
    n_out, ovf, proto, cfg = int(m.group(1)), m.group(2), m.group(3), m.group(4)
    if n_out != NBEAT:
        return False, "[%s] 输出拍数 %d != 期望 %d" % (name, n_out, NBEAT)
    if ovf != "0" or proto != "0" or cfg != "0":
        return False, "[%s] 标志位异常 ovf=%s proto=%s cfg=%s" % (name, ovf, proto, cfg)

    got = parse_out(out_path)
    if len(got) != NBEAT:
        return False, "[%s] 输出文件行数 %d != 期望 %d" % (name, len(got), NBEAT)

    for b, (tdata, tch, tlast, tuser, tblk) in enumerate(got):
        blk = b // 3
        ch = b % 3
        br, bc = divmod(blk, WALL_N)
        exp = int(ref[br, bc, ch])
        if tdata != exp:
            return False, ("[%s] 第 %d 拍 tdata=%d != Python 参考 %d（blk=%d ch=%d）"
                           % (name, b, tdata, exp, blk, ch))
        if tch != ch:
            return False, "[%s] 第 %d 拍 tch=%d != %d" % (name, b, tch, ch)
        if tlast != (1 if (ch == 2 and bc == WALL_N - 1) else 0):
            return False, "[%s] 第 %d 拍 tlast=%d 位置错误" % (name, b, tlast)
        if tuser != (1 if b == 0 else 0):
            return False, "[%s] 第 %d 拍 tuser=%d 位置错误" % (name, b, tuser)
        # m_meas_tblk = 块线性索引 = br*WALL_N + bc，**等于输出循环的 out_idx**（即 blk）。
        #   [2026-09-30 修] 原 RTL 写 `BW'(out_idx)*BW'(WALL_N) + BW'(obc)`：out_idx 本身
        #   已经是线性索引，再乘一次 WALL_N 属重复计数，整个表达式又在 BW 位宽上求值被
        #   截断（本尺寸 16 个块里 12 个错、只剩 4 个不同取值）。RTL 已改为直接输出
        #   out_idx，本处随之按线性索引断言。见 sim/results/rtl_four_issues_20260930.md。
        exp_tblk = blk
        if tblk != exp_tblk:
            return False, ("[%s] 第 %d 拍 tblk=%d != 线性索引 %d（blk=%d br=%d bc=%d）"
                           % (name, b, tblk, exp_tblk, blk, br, bc))

    lo = int(ref.min())
    return True, ("[%s] nfrm=%d dark=%d：%d 拍逐值相等（块和 min=%d max=%d 全部匹配）"
                  % (name, nfrm, dark, NBEAT, lo, int(ref.max())))


# ---------------------------------------------------------------- 主入口

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="raw_preproc RTL <-> Python bit-exact 对拍")
    ap.add_argument("--tool", choices=("xsim", "iverilog"), default="xsim")
    ap.add_argument("--repo-root", default=_REPO)
    ap.add_argument("--work-dir", default=None)
    ap.add_argument("--vivado-bin", default=DEFAULT_VIVADO_BIN)
    ap.add_argument("--license", default=DEFAULT_LICENSE)
    ap.add_argument("--temp-dir", default=DEFAULT_TEMP)
    ap.add_argument("--iverilog-bin", default=DEFAULT_IVERILOG)
    args = ap.parse_args(argv)

    work_root = args.work_dir or os.path.join(args.repo_root, "build", "xsim_py")
    os.makedirs(work_root, exist_ok=True)

    cases = [
        ("bitexact_nfrm4_dark0", 4, 0),
        ("bitexact_nfrm1_dark100", 1, 100),
    ]
    print("==== raw_preproc RTL <-> Python 参考链 bit-exact 对拍 ====")
    print("TB   : %s" % _TB)
    print("工具 : %s" % args.tool)
    print("规模 : WALL_N=%d BLOCK=%d -> 一帧 %dx%d，输出 %d 拍（非全尺寸）"
          % (WALL_N, BLOCK, RAW_SIDE, RAW_SIDE, NBEAT))
    bad = 0
    for name, nfrm, dark in cases:
        ok, msg = run_case(name, nfrm, dark, tool=args.tool, work_root=work_root,
                           vivado_bin=args.vivado_bin, license=args.license,
                           temp_dir=args.temp_dir, iverilog_bin=args.iverilog_bin)
        print(("  PASS  " if ok else "  FAIL  ") + msg)
        if not ok:
            bad += 1
    print("------------------------------------------------------------")
    print("结果：%s（%d 组用例，%d 组失败）"
          % ("PASS" if bad == 0 else "FAIL", len(cases), bad))
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
