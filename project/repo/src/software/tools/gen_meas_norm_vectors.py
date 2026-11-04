# -*- coding: utf-8 -*-
"""对拍 meas_norm 的向量与期望值：**全部由 Python（权威口径）算出**。

为什么这个脚本存在
------------------
`meas_norm` 这一级把 RTL 的原始整数差分转成定点核的 Q2.16 通道优先码字，
口径权威是 `src/model/penumbrasight_model/quantize.py` 的 `shared_exponent()`。
主机侧 C 实现必须与它**逐位相等**，所以期望值**不在 C 里写死、也不在这里抄
一份**：本脚本直接 import 那个模块、调 `shared_exponent()` 得到码字与指数，
再按通道优先重排后写成二进制。C 测试只做读取与逐位比较。

输出（写进 <outdir>，默认 ps_tmp/meas_norm/）
-------------------------------------------
  manifest.txt            每行：case rows cols ch n exponent sat raw_absmax code_absmax
  <case>.s32.bin          输入：int32 样本，流序 [row][col][ch]（ch 最内）
  <case>.w24.bin          输入：uint32 拍，低 24 bit 是 signed 差分（测符号扩展）
  <case>.exp.bin          期望：int32 码字，**通道优先** meas[ch*M + (r*cols+c)]
  clamp_probe.s32.bin     clamp 探针输入（1 维）
  clamp_probe.exp.bin     clamp 探针期望码字
  clamp_probe.exponent.txt 探针用的强制指数

四组帧输入
----------
  1) t2_seam    T2 接缝那份真实帧（vertical_diff 全 B 档导出，27360 拍）
  2) all_zero   全零退化帧
  3) tiny       极小退化帧（取值只在 {-1,0,1}）
  4) full_scale 打到 signed 24 bit 满量程（原始幅度约为核包络的 64 倍）

T2 真实帧的来源
---------------
`sim/vectors/meas_rtl_t2_seam.npz`（小向量入库）。若该 npz 不存在但仓库外仍有
T2 构建区的导出文件 `D:/psight/build_t2/seam/meas_rtl_dump.txt`，本脚本会先解析
它、落地该 npz（一次性 bootstrap），再继续。

用法
----
    python tools/gen_meas_norm_vectors.py ps_tmp/meas_norm
"""

from __future__ import annotations

import hashlib
import math
import os
import sys

import numpy as np

ROWS = 95          # WALL_N - 1
COLS = 96          # WALL_N
CH = 3
M = ROWS * COLS    # 9120
N = M * CH         # 27360

T2_NPZ = os.path.join("sim", "vectors", "meas_rtl_t2_seam.npz")
T2_DUMP_FALLBACK = r"D:\psight\build_t2\seam\meas_rtl_dump.txt"

MANTISSA_BITS = 18
FRAC_BITS = 16


def _repo_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, os.pardir, os.pardir, os.pardir))


def _load_authority():
    """import 权威量化模块；失败则直接报错，不退回自造口径。"""
    root = _repo_root()
    sys.path.insert(0, os.path.join(root, "src", "model"))
    from penumbrasight_model.quantize import (  # noqa: E402
        shared_exponent, round_half_even, saturate,
    )
    return shared_exponent, round_half_even, saturate


def _parse_t2_dump(path: str) -> np.ndarray:
    """解析 xsim 导出帧（每行 row col ch data），返回流序 int32 样本。"""
    rows = []
    with open(path, "r", encoding="ascii") as f:
        for line in f:
            p = line.split()
            if len(p) != 4:
                continue
            rows.append(tuple(int(t) for t in p))
    if len(rows) != N:
        raise SystemExit("dump 行数 %d != 期望 %d" % (len(rows), N))
    stream = np.empty(N, dtype=np.int32)
    order_ok = True
    for idx, (r, c, ch, d) in enumerate(rows):
        stream[idx] = d
        if (r * COLS + c) * CH + ch != idx:
            order_ok = False
    if not order_ok:
        raise SystemExit("dump 流序不是 [row][col][ch]")
    return stream


def _ensure_t2_npz(root: str) -> np.ndarray:
    npz_path = os.path.join(root, T2_NPZ)
    if os.path.isfile(npz_path):
        with np.load(npz_path) as z:
            stream = np.asarray(z["stream"], dtype=np.int32)
        if stream.size != N:
            raise SystemExit("T2 npz 长度 %d != 期望 %d" % (stream.size, N))
        return stream
    if not os.path.isfile(T2_DUMP_FALLBACK):
        raise SystemExit(
            "缺 T2 向量：既没有 %s，也没有构建区导出 %s" % (npz_path, T2_DUMP_FALLBACK))
    stream = _parse_t2_dump(T2_DUMP_FALLBACK)
    os.makedirs(os.path.dirname(npz_path), exist_ok=True)
    np.savez_compressed(npz_path, stream=stream)
    return stream


def _lcg_stream(seed: int, n: int, lo: int, hi: int) -> np.ndarray:
    """确定性 LCG（不依赖 numpy 版本），产生 [lo, hi] 内的 int32。"""
    out = np.empty(n, dtype=np.int32)
    x = seed & 0xFFFFFFFF
    span = hi - lo + 1
    for i in range(n):
        x = (1103515245 * x + 12345) & 0xFFFFFFFF
        out[i] = lo + int((x >> 8) % span)
    return out


def _ch_first(codes_stream: np.ndarray) -> np.ndarray:
    """流序 [m*ch + ch] -> 通道优先 [ch*M + m]。"""
    arr = codes_stream.reshape(M, CH)          # [m][ch]
    return np.ascontiguousarray(arr.T.reshape(-1))


def _write_i32(path: str, arr: np.ndarray) -> None:
    with open(path, "wb") as f:
        f.write(np.asarray(arr, dtype="<i4").tobytes())


def _write_u24(path: str, arr: np.ndarray) -> None:
    w = (np.asarray(arr, dtype=np.int64) & 0xFFFFFF).astype("<u4")
    with open(path, "wb") as f:
        f.write(w.tobytes())


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        sys.stderr.write("用法：python tools/gen_meas_norm_vectors.py <outdir>\n")
        return 2
    outdir = argv[1]
    os.makedirs(outdir, exist_ok=True)

    shared_exponent, round_half_even, saturate = _load_authority()
    root = _repo_root()

    # ------------------------------------------------ 四组帧输入
    t2 = _ensure_t2_npz(root)
    cases = {}

    all_zero = np.zeros(N, dtype=np.int32)
    cases["all_zero"] = all_zero

    tiny = np.zeros(N, dtype=np.int32)
    tiny[0] = 1
    tiny[1] = -1
    tiny[CH] = 1
    tiny[M * CH - 1] = -1
    cases["tiny"] = tiny

    # 打到 signed 24 bit 满量程：absmax = 8388608（取 -2^23），原始幅度约为
    # 核 Q2.16 包络 131071 的 64 倍。再叠一片中等幅度，避免只有两极值。
    full = _lcg_stream(0x5EED2026, N, -6000000, 6000000)
    full[0] = 8388607      # signed 24 bit 正满量程
    full[1] = -8388608     # signed 24 bit 负满量程（-2^23）
    cases["full_scale"] = full

    cases["t2_seam"] = t2

    # 顺序固定，便于人工核对 manifest
    order = ["t2_seam", "all_zero", "tiny", "full_scale"]

    manifest_lines = [
        "# case rows cols ch n_samples exponent sat_events raw_absmax code_absmax",
    ]

    print("[gen] T2 向量 sha256 = %s"
          % hashlib.sha256(t2.astype("<i4").tobytes()).hexdigest()[:16])

    for name in order:
        stream = cases[name]
        fa, frame = shared_exponent(stream, mantissa_bits=MANTISSA_BITS,
                                    frac_bits=FRAC_BITS)
        codes_stream = np.asarray(fa.codes, dtype=np.int32)
        assert codes_stream.size == N
        exp_codes = _ch_first(codes_stream)

        raw_absmax = int(np.max(np.abs(stream.astype(np.int64))))
        code_absmax = int(np.max(np.abs(exp_codes.astype(np.int64))))
        # 帧级饱和计数：用 quantize.shared_exponent 的同一个 scale/hi 算**未夹取**
        # 的码字，再数越界样本（不是数「等于边界」的样本——后者不是被夹）。
        hi = (1 << (MANTISSA_BITS - 1)) - 1
        scale_v = float(np.ldexp(1.0, frame.exponent - FRAC_BITS))
        raw_codes = round_half_even(stream.astype(np.float64) / scale_v)
        sat = int(np.sum((raw_codes > hi) | (raw_codes < -hi)))

        assert frame.exponent == (0 if raw_absmax == 0
                                  else (raw_absmax - 1).bit_length() + 1), name
        # 帧级饱和本应不可达（指数由本帧 absmax 推出，最大码字模长落在 (2^14, 2^15]）
        assert code_absmax <= (1 << 15), (name, code_absmax)

        _write_i32(os.path.join(outdir, name + ".s32.bin"), stream)
        _write_u24(os.path.join(outdir, name + ".w24.bin"), stream)
        _write_i32(os.path.join(outdir, name + ".exp.bin"), exp_codes)
        manifest_lines.append(
            "%s %d %d %d %d %d %d %d %d"
            % (name, ROWS, COLS, CH, N, frame.exponent, sat,
               raw_absmax, code_absmax))
        print("[gen] %-10s absmax=%-8d e=%-2d code_absmax=%-6d env_ratio=%.1f sat=%d"
              % (name, raw_absmax, frame.exponent, code_absmax,
                 raw_absmax / float(hi), sat))

    with open(os.path.join(outdir, "manifest.txt"), "w", encoding="ascii") as f:
        f.write("\n".join(manifest_lines) + "\n")

    # ------------------------------------------------ 指数口径表
    # 帧级用例的 absmax 只落在少数几个值上，覆盖不到「正好是 2 的幂」等边界。
    # 这里对一批精心挑的 absmax（含 2^k-1 / 2^k / 2^k+1）写下 Python 的指数，
    # 供 C 侧逐值核对 ps_meas_shared_exponent 与 quantize.py 同口径。
    exp_rows = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 15, 16, 17, 31, 32, 33,
                255, 256, 257, 1023, 1024, 1025, 4095, 4096, 4097,
                6214795, 6214796, 6214797, 8388607, 8388608]
    with open(os.path.join(outdir, "exp_table.txt"), "w", encoding="ascii") as f:
        for a in exp_rows:
            e = 0 if a == 0 else (int(math.ceil(math.log2(a))) + 1)
            assert e == (0 if a == 0 else (a - 1).bit_length() + 1)
            f.write("%d %d\n" % (a, e))

    # ------------------------------------------------ clamp 探针
    # 帧级路径够不到饱和（见上），故单开一条：把指数**外部给定**为 1，
    # 用 ±4 直接命中 2^17 = 131072 这个 18 bit 包络的边界，验证 C 的夹取
    # 与 quantize.py 同界（负侧 -131071，不是 -131072）。
    probe_e = 1
    probe = np.array([-4, -3, 3, 4, 100000, -100000, 0, 131071, 131072],
                     dtype=np.int32)
    scale = np.ldexp(1.0, probe_e - FRAC_BITS)
    hi = (1 << (MANTISSA_BITS - 1)) - 1
    probe_exp = saturate(round_half_even(probe.astype(np.float64) / scale),
                         -hi, hi).astype(np.int32)
    assert int(probe_exp.min()) == -hi, probe_exp
    assert int(probe_exp.max()) == hi, probe_exp

    _write_i32(os.path.join(outdir, "clamp_probe.s32.bin"), probe)
    _write_i32(os.path.join(outdir, "clamp_probe.exp.bin"), probe_exp)
    with open(os.path.join(outdir, "clamp_probe.exponent.txt"), "w",
              encoding="ascii") as f:
        f.write("%d\n" % probe_e)
    print("[gen] clamp_probe e=%d codes=%s" % (probe_e, probe_exp.tolist()))

    print("[gen] done -> %s" % outdir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
