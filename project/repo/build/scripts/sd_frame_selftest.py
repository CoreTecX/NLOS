#!/usr/bin/env python3
# ============================================================================
# sd_frame_selftest.py —— 「论文 .mat -> .psf -> 板端 C 读回 -> 与期望值比对」
#                        一条命令跑完的回归入口
#
# 用法
# ---------------------------------------------------------------------------
#   python build/scripts/sd_frame_selftest.py --mat "<...>/image_test_colbar.mat"
#   python build/scripts/sd_frame_selftest.py --mat "<...>/image_test_mushroom.mat" --frame-index 3
#   python build/scripts/sd_frame_selftest.py --psf build/psf/xxx_f000.psf --block 10
#
# 跑什么（每一条都能失败）
# ---------------------------------------------------------------------------
#   1. 转换：pack_paper_frame.py 生成 .psf + _expected.json
#   2. 读回：src/software/bin/test_sd_frame（C，两条读取路径 + 四类负例）
#   3. 逐值：C dump 出的 payload 与 Python 写进文件的 payload **逐元素相等**
#            （用 numpy 比 array_equal，不是比 CRC）
#   4. 逐值（对源）：C dump 的码字 == round_half_up(源裁剪帧 * num / den)，
#            num/den 取**文件头里的值**，因此这一条同时验了「头里的系数足以还原」
#   5. 校验和：crc32 / sum / sumsq / min / max 头内值 == C 实测值 == JSON 记录值
#   6. 块和：C 的块和 CRC/总和/各通道/最大值/所需位宽 == JSON（numpy 侧）的值
#            —— 这是 RGGB 相位与双绿求和口径的跨语言一致
#
# 判据分层（重要）
# ---------------------------------------------------------------------------
#   FAIL（退出码非零）：上面 1~6 的任何一条不成立 —— 它们都是「同一份数据」的
#                       等价表述，任一不等价说明链路某处错了。
#   WARN（不影响退出码）：scale.rule_ok == 0。camera_b.yaml 的 saturation_rule
#                       两条判据（p99.9<95%FS、median 在 20%~80%）在部分论文帧上
#                       **数学上不可同时满足**（p99.9/median > 4.75 时无解）。
#                       那时工具优先不削顶，并在头里置 rule_ok=0 —— 这是**如实
#                       记录的达成情况**，不是链路故障。
#
# 边界：本脚本验的是「文件与读取层」。它不碰 SD 卡驱动、不碰 DMA、不上板；
# 论文数据是 float64/约 21 bit，不是相机的 12 bit。
# ============================================================================

import argparse
import json
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
SW_DIR = os.path.join(REPO_ROOT, "src", "software")


def run(cmd, **kw):
    """跑子进程并把输出透传。

    子进程显式要求 UTF-8 输出（PYTHONIOENCODING）：本仓库路径里有中文，
    子进程默认按 Windows ANSI 码页输出，父进程按 UTF-8 解码就会炸在
    `UnicodeDecodeError` 上 —— 而这是**环境污染**，不是链路的错，必须在这里挡掉。
    `errors="replace"` 是第二道保险：C 侧打印的路径字节即使不是 UTF-8，
    也只影响那一行的可读性，不影响键值解析（键全是 ASCII）。
    """
    print("$ " + (cmd if isinstance(cmd, str) else " ".join(cmd)))
    env = dict(kw.pop("env", None) or os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    proc = subprocess.run(cmd, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", env=env, **kw)
    if proc.stdout:
        sys.stdout.write(proc.stdout)
    if proc.returncode != 0 and proc.stderr:
        sys.stderr.write(proc.stderr)
    return proc


def run_make(target):
    """跑 make。Windows 上 make 往往是一个 .cmd 垫片，而 CreateProcess 不能直接
    执行 .cmd —— 直接以列表调用会报 WinError 2。故先按列表试，失败再走 shell。"""
    try:
        return run(["make", "-C", SW_DIR, target])
    except FileNotFoundError:
        return run('make -C "%s" %s' % (SW_DIR, target), shell=True)


class Tally:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.warned = 0

    def check(self, cond, what):
        if cond:
            self.passed += 1
            print("  PASS  %s" % what)
        else:
            self.failed += 1
            print("  FAIL  %s" % what)
        return bool(cond)

    def warn(self, what):
        self.warned += 1
        print("  WARN  %s" % what)


def parse_kv_line(line):
    out = {}
    if not line.startswith("SD_FRAME_"):
        return out
    out["_kind"] = line.split()[0]
    for m in re.finditer(r"(\w+)=(\S+)", line):
        out[m.group(1)] = m.group(2)
    return out


def find_c_binary():
    for name in ("test_sd_frame.exe", "test_sd_frame"):
        p = os.path.join(SW_DIR, "bin", name)
        if os.path.isfile(p):
            return p
    return None


def main():
    ap = argparse.ArgumentParser(description="end-to-end self-check for the SD-frame link")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--mat", help="paper .mat to pack and then read back")
    src.add_argument("--psf", help="existing .psf to read back (skips packing)")
    ap.add_argument("--frame-index", type=int, default=0)
    ap.add_argument("--block", type=int, default=10, help="B profile block side (10)")
    ap.add_argument("--work-dir", default=os.path.join(SW_DIR, "ps_tmp", "sd_frame"))
    ap.add_argument("--crop", type=int, default=1920)
    ap.add_argument("--json", help="expected json (default: next to the .psf)")
    args = ap.parse_args()

    tally = Tally()
    os.makedirs(args.work_dir, exist_ok=True)

    pack_args = [sys.executable, os.path.join(HERE, "pack_paper_frame.py")]
    if args.mat:
        pack_args += ["--mat", args.mat, "--frame-index", str(args.frame_index),
                      "--block", str(args.block), "--crop", str(args.crop),
                      "--out-dir", args.work_dir]
    if args.mat:
        proc = run(pack_args)
        if proc.returncode != 0:
            print("RESULT: FAIL (packer failed)")
            return 1
        stem = os.path.splitext(os.path.basename(args.mat))[0]
        psf_path = os.path.join(args.work_dir, "%s_f%03d.psf" % (stem, args.frame_index))
        json_path = os.path.join(args.work_dir, "%s_f%03d_expected.json" % (stem, args.frame_index))
    else:
        psf_path = args.psf
        root, _ = os.path.splitext(psf_path)
        json_path = args.json or (root + "_expected.json")

    print("\n==== C reader self-test ====")
    cbin = find_c_binary()
    if cbin is None:
        print("  building it first ...")
        proc = run_make("sd-frame-bin")
        if proc.returncode != 0:
            print("RESULT: FAIL (cannot build the C reader)")
            return 1
        cbin = find_c_binary()
    if cbin is None:
        print("RESULT: FAIL (test_sd_frame binary not found after build)")
        return 1

    dump_path = os.path.join(args.work_dir, os.path.basename(psf_path) + ".dump.u16")
    proc = run([cbin, psf_path, str(args.block), "--dump-raw", dump_path])
    if proc.returncode != 0:
        print("RESULT: FAIL (C reader reported failures)")
        return 1

    read_kv, block_kv = {}, {}
    for line in proc.stdout.splitlines():
        kv = parse_kv_line(line)
        if kv.get("_kind") == "SD_FRAME_READ":
            read_kv = kv
        elif kv.get("_kind") == "SD_FRAME_BLOCK":
            block_kv = kv
    if not read_kv:
        print("RESULT: FAIL (C reader printed no SD_FRAME_READ line)")
        return 1

    with open(json_path, "r", encoding="utf-8") as fh:
        exp = json.load(fh)
    with open(psf_path, "rb") as fh:
        raw = fh.read()
    header_bytes = exp["frame"]["header_bytes"]
    codes_file = np.frombuffer(raw[header_bytes:], dtype="<u2")
    codes_dump = np.frombuffer(open(dump_path, "rb").read(), dtype="<u2")

    print("\n==== checks ====")
    # --- 3. Python 写盘 == C 读回（逐元素） ---
    tally.check(codes_file.shape == codes_dump.shape,
                "payload length: file %d words == C dump %d words" % (codes_file.size, codes_dump.size))
    tally.check(bool(np.array_equal(codes_file, codes_dump)),
                "payload values: file == C readback (element-wise, %d words)" % codes_file.size)

    # --- 4. 头内系数足以还原：重算源 -> 码字 ---
    if args.mat:
        sys.path.insert(0, HERE)
        import pack_paper_frame as pk
        values, meta = pk.load_mat_image(args.mat)
        cropped, _off = pk.center_crop(values, args.crop)
        num = int(exp["scale"]["num"])
        den = int(exp["scale"]["den"])
        full_scale = (1 << int(exp["frame"]["valid_bits"])) - 1
        codes_re = pk.quantize_codes(cropped, num, den, full_scale)[0]
        # ravel：源是 2-D 帧、读回是 1-D 光栅序，形状不同会让 array_equal 直接为假
        # （这里踩过一次，报出来的却是「逐值不一致」，看着像量化错了）。
        codes_re = codes_re.astype(np.int64).ravel()
        dump_i = codes_dump.astype(np.int64)
        same = bool(codes_re.shape == dump_i.shape and np.array_equal(codes_re, dump_i))
        tally.check(same, "source -> codes with header (num,den) reproduces the C readback exactly")
        diag = exp["scale"]["rule_diag"]
        band = diag["median_band"]
        mcode, pcode = int(exp["scale"]["median_code"]), int(exp["scale"]["p999_code"])
        if int(exp["scale"]["rule_ok"]) == 1:
            tally.check(pcode <= diag["target_p999_code"] and band[0] <= mcode <= band[1],
                        "saturation rule: p999_code=%d <= %d and median_code=%d in [%d,%d]"
                        % (pcode, diag["target_p999_code"], mcode, band[0], band[1]))
        else:
            ratio = diag.get("p999_over_median")
            tally.warn("saturation rule not satisfiable for this frame: "
                       "p99.9/median=%s > %.2f (needs median_code=%d in [%d,%d], p999_code<=%d)"
                       % ("%.2f" % ratio if ratio is not None else "n/a",
                          diag["feasible_iff_p999_over_median_le"], mcode, band[0], band[1],
                          diag["target_p999_code"]))
            tally.check(int(exp["scale"]["rule_ok"]) == 0 and mcode < band[0],
                        "rule_ok=0 is recorded and the chosen scale is p999-anchored (no clipping priority)")

    # --- 5. 校验和：头 == C 实测 == JSON ---
    pay = exp["payload"]
    tally.check("0x%08X" % int(read_kv["crc32"], 16) == pay["crc32"],
                "crc32: C=0x%08X == json=%s" % (int(read_kv["crc32"], 16), pay["crc32"]))
    tally.check(int(read_kv["sum"]) == pay["sum"], "payload sum matches json")
    tally.check(int(read_kv["sumsq"]) == pay["sumsq"], "payload sumsq matches json")
    tally.check(int(read_kv["min"]) == pay["min"], "payload min matches json")
    tally.check(int(read_kv["max"]) == pay["max"], "payload max matches json")
    tally.check(int(read_kv["num"]) == int(exp["scale"]["num"])
                and int(read_kv["den"]) == int(exp["scale"]["den"]),
                "scale (num,den) matches json")
    tally.check(int(read_kv["rule_ok"]) == int(exp["scale"]["rule_ok"]),
                "scale_rule_ok matches json")
    tally.check(int(read_kv["packing"]) == int(exp["frame"]["value_packing_id"]),
                "value_packing recorded in header matches json")

    # --- 6. 块和：C == JSON ---
    bs = exp["block_sum"]
    cblk = int(block_kv["crc32"], 16)
    tally.check(cblk == int(bs["crc32"], 16),
                "block-sum crc32: C=0x%08X == json=%s" % (cblk, bs["crc32"]))
    tally.check(int(block_kv["total"]) == bs["total"], "block-sum total matches json")
    tally.check(int(block_kv["ch0"]) == bs["by_ch"][0]
                and int(block_kv["ch1"]) == bs["by_ch"][1]
                and int(block_kv["ch2"]) == bs["by_ch"][2],
                "block-sum per-channel totals match json")
    tally.check(int(block_kv["max"]) == bs["max"], "block-sum max matches json")
    tally.check(int(block_kv["bits"]) == bs["bits_required"], "block-sum bits_required matches json")
    # 不变式：R/G/B 三分块之和 = 全部码值之和（每个像素只进一个通道，双绿按和进 G）。
    # 它独立于 JSON，能查出「某个通道多算/漏算」这类错。
    tally.check(int(block_kv["total"]) == pay["sum"],
                "invariant: block-sum total == payload sum")

    print("\n---- summary ----")
    print("  source     : %s" % (exp["source"].get("mat") or exp["source"].get("generator")))
    print("  frame      : %dx%d valid_bits=%d packing=%s"
          % (exp["frame"]["width"], exp["frame"]["height"],
             exp["frame"]["valid_bits"], exp["frame"]["value_packing"]))
    print("  source span: %d..%d (%d bits), median=%d p99.9=%d"
          % (exp["source"]["source_min"], exp["source"]["source_max"],
             exp["source"]["bit_span_bits"], exp["source"]["source_median"],
             exp["source"]["source_p999"]))
    print("  scale      : %d/%d  step=%.4f adu  half_step=%.4f  rule_ok=%d  clipped=%d"
          % (exp["scale"]["num"], exp["scale"]["den"], exp["scale"]["quant_step_adu"],
             exp["scale"]["half_step_adu"], exp["scale"]["rule_ok"],
             exp["scale"]["clipped_pixels"]))
    print("  quant err  : max %.4f adu (unclipped), bound %.4f (idempotent=%s)"
          % (exp["scale"]["max_abs_error_adu_unclipped"], exp["scale"]["half_step_adu"],
             exp["scale"]["codec_idempotent"]))
    print("  payload    : crc32=%s sum=%d sumsq=%d min=%d max=%d"
          % (pay["crc32"], pay["sum"], pay["sumsq"], pay["min"], pay["max"]))
    print("  block sum  : block=%d wall_n=%d total=%d by_ch=%s max=%d bits=%d crc32=%s"
          % (bs["block"], bs["wall_n"], bs["total"], bs["by_ch"], bs["max"],
             bs["bits_required"], bs["crc32"]))
    print("  files      : %s" % psf_path)
    print("               %s" % json_path)
    print("RESULT: PASS %d / FAIL %d / WARN %d" % (tally.passed, tally.failed, tally.warned))
    return 0 if tally.failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
