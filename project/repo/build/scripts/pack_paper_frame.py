#!/usr/bin/env python3
# ============================================================================
# pack_paper_frame.py —— 论文实测帧 (.mat) -> SD 卡中间格式 (.psf)
#
# 用途：不接相机时，把论文 `image_test_*.mat` 的原始 ADU 帧转成板端能直接读的
#       定长头 + 裸 payload 文件，并把板上比对需要的期望值写成同名 JSON。
#
# 格式契约：docs/sd_frame_format.md（**唯一权威**；本文件与板端读取端
#           src/software/sd_frame/sd_frame.c 必须与该文档逐字段一致）
#
# 用法
# ---------------------------------------------------------------------------
#   1) 真数据
#      python build/scripts/pack_paper_frame.py \
#        --mat "D:\...\TestPosChair\image_test_colbar.mat" --frame-index 0
#      -> build/psf/image_test_colbar_f000.psf
#      -> build/psf/image_test_colbar_f000_expected.json
#
#   2) 仓库内小 fixture（不需要论文数据集；给自检用）
#      python build/scripts/pack_paper_frame.py --synth 8 --block 2 --out-dir tests/vectors/sd_frame
#
#   3) 只校验已有文件，不重算
#      python build/scripts/pack_paper_frame.py --check build/psf/xxx.psf
#
# 为什么这些数字不是魔数
# ---------------------------------------------------------------------------
#   * 16 bit 容器 / 12 bit 有效位 / RGGB：configs/camera_b.yaml 的 sensor 段与
#     docs/hardware_freeze.md §1 的冻结值，也是 raw_preproc.sv 的 VALID_BITS 与
#     COLOR/CH 口径。脚本从 configs/camera_b.yaml **读**这些值（已装 pyyaml 时），
#     读不到才退回同值常量，并在输出里标注来源。
#   * 缩放系数：按帧从数据本身的 (median, p99.9) 定出，判据取自
#     camera_b.yaml 的 acquisition.saturation_rule。逐帧不同是**特性**——
#     data/inventory/data_contract.md §5 量出论文各帧量纲跨约 3 个数量级，
#     按固定满量程设计必然削顶或浪费位深。
#   * 满量程 4095 是 2^valid_bits - 1，不是硬编码字面量。
#
# 边界（不要读成别的东西）
# ---------------------------------------------------------------------------
#   * 论文数据是 float64 / 约 21 bit 动态范围，**不是** MV-CS050-10GC 的 12 bit。
#     本工具把 21 bit 的 ADU 压到 12 bit 码值，验的是「板端数据通路 + 定点链」，
#     不能替代真实相机的位深与噪声特性，也不能据此宣称相机适配已验证。
#   * 论文是 2016x2016，这里**居中裁到 1920x1920**（各边丢 48 像素）。裁掉的边缘
#     与论文自身 126x126 对照路径不再同源。
#   * payload 一律按 packed（12 bit 右对齐）产出，这是 camera_b.yaml 的默认假设
#     （sensor.value_packing 仍是 todo）；所用口径写进头字段 value_packing。
# ============================================================================

import argparse
import json
import os
import struct
import sys
import zlib

import numpy as np

# --------------------------------------------------------------- 格式常量
# 与 docs/sd_frame_format.md §2 的表一一对应。改动必须同时改 sd_frame.c。
MAGIC = b"PSDF"
VERSION = 1
HEADER_BYTES = 128
OFF_PAYLOAD = HEADER_BYTES

VALID_BITS = 12
CONTAINER_BITS = 16
BAYER_PHASE_RGGB = 0
PACKING_PACKED = 0        # 12 bit 右对齐，0..4095
PACKING_FULLSCALE = 1     # 左移 4 位，0..65520
FLAG_LITTLE_ENDIAN = 1

# camera_b.yaml 的 acquisition.saturation_rule 三条数字（读不到 yaml 时的回退值）
RULE_P999_RATIO = 0.95
RULE_MEDIAN_LO = 0.20
RULE_MEDIAN_HI = 0.80

# 论文原始幅面 -> B 档 ROI（居中裁）
PAPER_SIDE = 2016
ROI_SIDE = 1920

SYNTH_OFFSET = 200
SYNTH_SPAN = 3000

# 头部字段的 struct 格式串（小端、无填充）。字段顺序**必须**与
# docs/sd_frame_format.md §2 的表以及 sd_frame.c 里的 PS_SD_OFF_* 偏移一致：
#   4s magic | H version H header_bytes | I width I height | H valid_bits H container_bits
#   | B bayer_phase B value_packing | H flags | I frame_index I payload_bytes
#   | Q scale_num Q scale_den | I payload_crc32 I clipped_pixels
#   | Q payload_sum Q payload_sumsq | H payload_min H payload_max
#   | H median_code H p999_code | B scale_rule_ok B reserved0 | H reserved1
#   | I source_median I source_p999 I reserved2
# 合计 96 字节，其后 32 字节保留区写 0，头部总长 128。
HEADER_FMT = "<4sHHIIHHBBHIIQQIIQQHHHHBBHIII"


# --------------------------------------------------------------- 配置读取
def load_camera_profile(repo_root):
    """尽量从 configs/camera_b.yaml 读冻结值；任何一步失败就回退到同值常量。

    返回 (dict, 来源说明)。**不回退到「猜」的值**：回退值与文档里冻结的数字相同，
    并且来源会打印出来，便于一眼看出这次用的是 yaml 还是常量。
    """
    prof = {
        "valid_bits": VALID_BITS,
        "container_bits": CONTAINER_BITS,
        "bayer_phase": BAYER_PHASE_RGGB,
        "bayer_pattern": "RGGB",
        "value_packing": "todo",
        "source": "builtin-constants",
    }
    path = os.path.join(repo_root, "configs", "camera_b.yaml")
    if not os.path.isfile(path):
        return prof, "builtin-constants (no configs/camera_b.yaml)"
    try:
        import yaml  # noqa: F401  (可选依赖)
        with open(path, "r", encoding="utf-8") as fh:
            doc = yaml.safe_load(fh)
        sensor = (doc or {}).get("sensor", {})
        if isinstance(sensor.get("bit_depth"), int):
            prof["valid_bits"] = int(sensor["bit_depth"])
        if isinstance(sensor.get("container_bits"), int):
            prof["container_bits"] = int(sensor["container_bits"])
        pat = str(sensor.get("bayer_pattern", "RGGB")).strip().upper()
        prof["bayer_pattern"] = pat
        phase = {"RGGB": 0, "BGGR": 1, "GRBG": 2, "GBRG": 3}.get(pat)
        if phase is None:
            raise ValueError("unknown bayer_pattern %r" % pat)
        prof["bayer_phase"] = phase
        prof["value_packing"] = str(sensor.get("value_packing", "todo"))
        prof["source"] = "configs/camera_b.yaml"
    except Exception as exc:  # 缺 pyyaml / 字段缺失 / 取值不认识
        return prof, "builtin-constants (yaml unavailable: %s)" % exc
    return prof, prof["source"]


# --------------------------------------------------------------- 源数据
def load_mat_image(mat_path):
    """读 .mat 的 `image` 变量。返回 (float64 二维数组, 元信息 dict)。

    论文数据的 `image` 是 float64 但数值恒为整数（data/inventory/data_contract.md
    §3.1 的 132/132），这里**要求**这一条成立，否则拒绝转换 —— 静默四舍五入会把
    「源不是整数」这件事藏起来。
    """
    import scipy.io as sio
    import warnings

    with warnings.catch_warnings():
        # 4 个官方配置文件有重复变量名，scipy 会警告；与我们要读的 image 无关。
        warnings.simplefilter("ignore")
        doc = sio.loadmat(mat_path)
    if "image" not in doc:
        raise SystemExit("ERROR: %s has no variable 'image'" % mat_path)
    img = np.asarray(doc["image"])
    if img.ndim != 2:
        raise SystemExit("ERROR: image is %d-D, expected 2-D" % img.ndim)
    img_f = img.astype(np.float64, copy=False)
    if not np.all(np.mod(img_f, 1.0) == 0.0):
        raise SystemExit("ERROR: image is not integer-valued; refuse to round silently")
    meta = {
        "mat": os.path.abspath(mat_path),
        "var": "image",
        "shape": [int(img.shape[0]), int(img.shape[1])],
        "dtype": str(img.dtype),
        "integer_valued": True,
    }
    return img_f, meta


def synth_image(side):
    """确定性合成帧（reproducible，不用外部数据）。

    v[i,j] = SYNTH_OFFSET + ((i*37 + j*61) mod SYNTH_SPAN)

    取模加线性组合让整幅同时含局部结构与全局梯度，块和/百分位都不会退化到常数
    （常数图会让「块和相等」这种判据失去分辨力）。
    """
    if side < 2 or side % 2:
        raise SystemExit("ERROR: --synth side must be an even integer >= 2")
    ii, jj = np.meshgrid(np.arange(side), np.arange(side), indexing="ij")
    return (SYNTH_OFFSET + (ii * 37 + jj * 61) % SYNTH_SPAN).astype(np.float64), {
        "mat": None,
        "var": "synthetic",
        "shape": [side, side],
        "dtype": "int64",
        "integer_valued": True,
        "generator": "v[i,j] = %d + ((i*37 + j*61) mod %d)" % (SYNTH_OFFSET, SYNTH_SPAN),
    }


def center_crop(img, side):
    """居中裁到 side x side。返回 (裁剪后, 每边偏移)。"""
    h, w = img.shape
    if side > h or side > w:
        raise SystemExit("ERROR: cannot crop %dx%d up to %d" % (h, w, side))
    oy = (h - side) // 2
    ox = (w - side) // 2
    return img[oy:oy + side, ox:ox + side].copy(), [int(oy), int(ox)]


# --------------------------------------------------------------- 定标
def choose_scale(median_i, p999_i, full_scale, rule):
    """按 camera_b.yaml 的 saturation_rule 定出整数比 (num, den)。

    判据：① p99.9 <= 0.95*FS  ② 0.20*FS <= median <= 0.80*FS
    可行区间里取**最大**增益（= 最小量化步长）：
        g = min( floor(0.95*FS)/p99.9 , floor(0.80*FS)/median )
    两个候选都是整数比，用交叉相乘比较，全程整数。
    返回 (num, den, rule_ok, diag)。
    """
    fs = full_scale
    t = int(np.floor(rule["p999_ratio"] * fs))       # 3890 @ 12 bit
    m_lo = int(np.ceil(rule["median_lo"] * fs))      # 819
    m_hi = int(np.floor(rule["median_hi"] * fs))     # 3276

    if p999_i <= 0:
        # 全零帧：无论怎么缩放都是全零。给 (1,1)，判据不成立（无信息量）。
        # diag 的键与非退化路径**保持同一组**，否则调用方在退化帧上会 KeyError。
        return 1, 1, 0, {
            "degenerate": "all-zero source frame",
            "target_p999_code": t,
            "median_band": [m_lo, m_hi],
            "feasible_iff_p999_over_median_le": float(rule["p999_ratio"]) / float(rule["median_lo"]),
            "p999_over_median": None,
            "chosen": "degenerate",
        }

    if median_i <= 0:
        num, den = t, p999_i
    else:
        # g_p999 = t/p999_i, g_med = m_hi/median_i；交叉相乘比大小（u64 内）
        if t * median_i <= m_hi * p999_i:
            num, den = t, p999_i
        else:
            num, den = m_hi, median_i

    # 达成判定用整数：code(x) = round_half_up(x * num / den)
    p999_code = (2 * p999_i * num + den) // (2 * den)
    median_code = (2 * median_i * num + den) // (2 * den)
    rule_ok = 1 if (p999_code <= t and m_lo <= median_code <= m_hi) else 0
    diag = {
        "target_p999_code": t,
        "median_band": [m_lo, m_hi],
        "feasible_iff_p999_over_median_le": float(rule["p999_ratio"]) / float(rule["median_lo"]),
        "p999_over_median": (float(p999_i) / float(median_i)) if median_i > 0 else None,
        "chosen": "p999-anchored" if (median_i <= 0 or t * median_i <= m_hi * p999_i) else "median-anchored",
    }
    return num, den, rule_ok, diag


def quantize_codes(values, num, den, full_scale):
    """v -> code = round_half_up(v*num/den)，夹到 [0, full_scale]。

    整数实现（无浮点）：code = floor( (2*v*num + den) / (2*den) )。
    向量是 int64；v 上限 ~3.3e6、num 上限 ~3890，乘积 ~2.6e10，远离 int64 溢出。
    """
    v = values.astype(np.int64, copy=False)
    if v.max(initial=0) * num * 2 >= (1 << 62):
        raise SystemExit("ERROR: quantization numerator would overflow int64")
    codes = (2 * v * num + den) // (2 * den)
    lo = codes < 0
    hi = codes > full_scale
    clipped = int(np.count_nonzero(lo) + np.count_nonzero(hi))
    codes = np.clip(codes, 0, full_scale)
    return codes, clipped


def decode_error(values, codes, num, den):
    """还原误差上界与实际最大值（单位 ADU）。

    解码：v_hat = code*den/num。理论界 |v-v_hat| <= step/2，step = den/num。
    """
    v = values.astype(np.float64, copy=False)
    v_hat = codes.astype(np.float64) * (float(den) / float(num))
    err = np.abs(v - v_hat)
    step = float(den) / float(num)
    return step, float(err.max(initial=0.0)), step / 2.0, v_hat


def check_codec_invariants(values, codes, num, den, full_scale):
    """两条可逆性不变式，全部用整数算术验证（不引浮点）：

      A. 幂等：reencode(decode(code)) == code
         用有理数表示 v_hat = A/B（A = code*den, B = num），则
         reencode(v_hat) = round(A*num/(B*den)) = round(code) = code。
      B. 未夹取样本的误差界：|v - code*den/num| <= den/(2*num) = step/2
    """
    A = codes.astype(np.int64) * np.int64(den)
    B = np.int64(num)
    code2 = (2 * A * np.int64(num) + B * np.int64(den)) // (2 * B * np.int64(den))
    idempotent_ok = bool(np.array_equal(code2, codes))

    v = values.astype(np.float64, copy=False)
    step = float(den) / float(num)
    err = np.abs(v - codes.astype(np.float64) * step)
    unclipped = (codes > 0) & (codes < full_scale)
    max_err_unclipped = float(err[unclipped].max(initial=0.0)) if np.any(unclipped) else 0.0
    bound_ok = bool(max_err_unclipped <= step / 2.0 + 1e-9)
    return {
        "idempotent": idempotent_ok,
        "error_bound_ok": bound_ok,
        "max_abs_error_adu_unclipped": max_err_unclipped,
        "half_step_adu": step / 2.0,
    }


# --------------------------------------------------------------- 块和
def block_sums_rggb(codes, block):
    """按 RGGB 解交织后 10x10 分块求和（与 raw_preproc.sv 的口径一致）。

    · 解交织：R=raw[2i,2j]、G1=raw[2i,2j+1]、G2=raw[2i+1,2j]、B=raw[2i+1,2j+1]
    · 双绿按**和**进累加（G = G1+G2，不平均、不右移）—— 仓库原则 P2
    · 块内只求和，不做除法（P1）
    输出 ch-major 展平：out[ch*nb*nb + br*nb + bc]，ch 0=R / 1=G / 2=B。
    """
    h, w = codes.shape
    if h != w:
        raise SystemExit("ERROR: block_sums_rggb requires a square frame")
    side = h
    if side % 2:
        raise SystemExit("ERROR: frame side must be even for RGGB de-interleave")
    plane = side // 2
    nb = plane // block
    if nb * block != plane:
        raise SystemExit("ERROR: plane side %d not divisible by block %d" % (plane, block))

    raw = codes.astype(np.int64, copy=False)
    r_pl = raw[0::2, 0::2]
    g_pl = raw[0::2, 1::2] + raw[1::2, 0::2]
    b_pl = raw[1::2, 1::2]

    out = np.empty((3, nb, nb), dtype=np.int64)
    for ch, pl in enumerate((r_pl, g_pl, b_pl)):
        out[ch] = pl.reshape(nb, block, nb, block).sum(axis=(1, 3))
    return out.reshape(-1), nb


def block_sum_stats(flat, nb):
    total = int(flat.sum())
    by_ch = [int(flat[c * nb * nb:(c + 1) * nb * nb].sum()) for c in range(3)]
    mx = int(flat.max(initial=0))
    bits = max(1, int(mx).bit_length())
    crc = zlib.crc32(flat.astype("<u4").tobytes()) & 0xFFFFFFFF
    return {
        "block": None,  # 由调用方填
        "wall_n": nb,
        "channels": 3,
        "layout": "ch-major out[ch*(wall_n^2) + br*wall_n + bc]; ch 0=R, 1=G(=G1+G2), 2=B",
        "total": total,
        "by_ch": by_ch,
        "max": mx,
        "bits_required": bits,
        "crc32": "0x%08X" % crc,
    }


# --------------------------------------------------------------- 装配
def build_header(hdr):
    """按 docs/sd_frame_format.md §2 逐字段小端打包成 128 字节。"""
    buf = bytearray(HEADER_BYTES)
    struct.pack_into(HEADER_FMT, buf, 0,
                     MAGIC,
                     VERSION,
                     HEADER_BYTES,
                     hdr["width"], hdr["height"],
                     hdr["valid_bits"], hdr["container_bits"],
                     hdr["bayer_phase"], hdr["value_packing"], hdr["flags"],
                     hdr["frame_index"], hdr["payload_bytes"],
                     hdr["scale_num"], hdr["scale_den"],
                     hdr["payload_crc32"], hdr["clipped_pixels"],
                     hdr["payload_sum"], hdr["payload_sumsq"],
                     hdr["payload_min"], hdr["payload_max"],
                     hdr["median_code"], hdr["p999_code"],
                     hdr["scale_rule_ok"], 0, 0,
                     hdr["source_median"], hdr["source_p999"], 0)
    # `struct` 的字段顺序与 docs/sd_frame_format.md §2 的表一一对应；小端、无填充，
    # 因此打包偏移就是表里的偏移（下面断言长度，防止加字段时漏改这里）。
    if struct.calcsize(HEADER_FMT) != 96:
        raise SystemExit("ERROR: header field layout is %d bytes, expected 96 before padding"
                         % struct.calcsize(HEADER_FMT))
    return bytes(buf)


def parse_header(raw):
    """读回端（Python 侧），仅用于 --check 与自检。板端实现在 sd_frame.c。"""
    if len(raw) < HEADER_BYTES:
        raise SystemExit("ERROR: file shorter than header")
    (magic, version, header_bytes, width, height, valid_bits, container_bits,
     bayer_phase, value_packing, flags, frame_index, payload_bytes,
     scale_num, scale_den, crc, clipped, psum, psumsq, pmin, pmax,
     median_code, p999_code, rule_ok, _r0, _r1,
     src_med, src_p999, _r2) = struct.unpack_from(
        HEADER_FMT, raw, 0)
    return {
        "magic": magic, "version": version, "header_bytes": header_bytes,
        "width": width, "height": height, "valid_bits": valid_bits,
        "container_bits": container_bits, "bayer_phase": bayer_phase,
        "value_packing": value_packing, "flags": flags,
        "frame_index": frame_index, "payload_bytes": payload_bytes,
        "scale_num": scale_num, "scale_den": scale_den,
        "payload_crc32": crc, "clipped_pixels": clipped,
        "payload_sum": psum, "payload_sumsq": psumsq,
        "payload_min": pmin, "payload_max": pmax,
        "median_code": median_code, "p999_code": p999_code,
        "scale_rule_ok": rule_ok,
        "source_median": src_med, "source_p999": src_p999,
    }


# --------------------------------------------------------------- 主流程
def pack(values, meta, args, prof, repo_root):
    full_scale = (1 << int(prof["valid_bits"])) - 1
    side = int(values.shape[0])
    if values.shape[1] != side:
        raise SystemExit("ERROR: only square frames are supported (got %dx%d)" % values.shape)

    src_med_f = float(np.median(values))
    src_p999_f = float(np.percentile(values, 99.9))
    src_med = int(np.rint(src_med_f))
    src_p999 = int(np.rint(src_p999_f))
    src_min = int(np.rint(values.min()))
    src_max = int(np.rint(values.max()))
    is_int = bool(np.all(np.mod(values, 1.0) == 0.0))

    rule = {"p999_ratio": RULE_P999_RATIO, "median_lo": RULE_MEDIAN_LO, "median_hi": RULE_MEDIAN_HI}
    num, den, rule_ok, diag = choose_scale(src_med, src_p999, full_scale, rule)

    codes, clipped = quantize_codes(values, num, den, full_scale)
    step, max_err, half_step, _vhat = decode_error(values, codes, num, den)
    inv = check_codec_invariants(values, codes, num, den, full_scale)
    if not inv["idempotent"]:
        raise SystemExit("ERROR: codec is not idempotent; refuse to write a non-invertible file")

    payload = codes.astype("<u2").tobytes()
    payload_crc = zlib.crc32(payload) & 0xFFFFFFFF
    payload_sum = int(codes.astype(np.int64).sum())
    payload_sumsq = int((codes.astype(np.int64) ** 2).sum())
    payload_min = int(codes.min())
    payload_max = int(codes.max())

    median_code = (2 * src_med * num + den) // (2 * den) if den else 0
    p999_code = (2 * src_p999 * num + den) // (2 * den) if den else 0

    hdr = {
        "width": side, "height": side,
        "valid_bits": int(prof["valid_bits"]),
        "container_bits": int(prof["container_bits"]),
        "bayer_phase": int(prof["bayer_phase"]),
        "value_packing": PACKING_PACKED,
        "flags": FLAG_LITTLE_ENDIAN,
        "frame_index": int(args.frame_index),
        "payload_bytes": len(payload),
        "scale_num": int(num), "scale_den": int(den),
        "payload_crc32": payload_crc, "clipped_pixels": int(clipped),
        "payload_sum": payload_sum, "payload_sumsq": payload_sumsq,
        "payload_min": payload_min, "payload_max": payload_max,
        "median_code": int(median_code), "p999_code": int(p999_code),
        "scale_rule_ok": int(rule_ok),
        "source_median": src_med, "source_p999": src_p999,
    }
    header = build_header(hdr)

    stem = args.name or ("synth%d" % side if args.synth else os.path.splitext(os.path.basename(args.mat))[0])
    out_name = "%s_f%03d" % (stem, int(args.frame_index))
    os.makedirs(args.out_dir, exist_ok=True)
    psf_path = os.path.join(args.out_dir, out_name + ".psf")
    with open(psf_path, "wb") as fh:
        fh.write(header)
        fh.write(payload)

    flat, nb = block_sums_rggb(codes, int(args.block))
    bs = block_sum_stats(flat, nb)
    bs["block"] = int(args.block)

    json_path = os.path.join(args.out_dir, out_name + "_expected.json")
    doc = {
        "schema": "psf-expected/1",
        "format_doc": "docs/sd_frame_format.md",
        "source": dict(meta, **{
            "cropped_to": [side, side],
            "crop_offset": meta.get("crop_offset", [0, 0]),
            "source_min": src_min, "source_max": src_max,
            "source_median": src_med, "source_p999": src_p999,
            "source_median_raw": src_med_f, "source_p999_raw": src_p999_f,
            "integer_valued": is_int,
            "bit_span_bits": max(1, int(src_max).bit_length()),
        }),
        "frame": {
            "file": os.path.basename(psf_path),
            "header_bytes": HEADER_BYTES,
            "payload_bytes": len(payload),
            "file_bytes": HEADER_BYTES + len(payload),
            "width": side, "height": side,
            "valid_bits": int(prof["valid_bits"]),
            "container_bits": int(prof["container_bits"]),
            "bayer_phase": int(prof["bayer_phase"]),
            "bayer_pattern": prof["bayer_pattern"],
            "value_packing": "packed",
            "value_packing_id": PACKING_PACKED,
            "value_packing_config_state": prof["value_packing"],
            "frame_index": int(args.frame_index),
            "profile_source": prof["source"],
        },
        "scale": {
            "num": int(num), "den": int(den),
            "encode": "code = floor((2*v*num + den) / (2*den)), then clip to [0, 2^valid_bits-1]",
            "decode": "v_hat = code * den / num",
            "quant_step_adu": step,
            "half_step_adu": half_step,
            "rule_ok": int(rule_ok),
            "rule_diag": diag,
            "median_code": int(median_code),
            "p999_code": int(p999_code),
            "clipped_pixels": int(clipped),
            "sat_code_count": int(np.count_nonzero(codes == full_scale)),
            "max_abs_error_adu_unclipped": inv["max_abs_error_adu_unclipped"],
            "codec_idempotent": inv["idempotent"],
            "error_bound_ok": inv["error_bound_ok"],
        },
        "payload": {
            "crc32": "0x%08X" % payload_crc,
            "sum": payload_sum,
            # 容器值是非负的（12 bit 右对齐码值），所以 sum 同时就是「绝对值和」。
            # 这里按名字重复记一份，免得读的人去找一个并不存在的字段。
            "abs_sum": payload_sum,
            "sumsq": payload_sumsq,
            "min": payload_min,
            "max": payload_max,
        },
        "block_sum": bs,
        "notes": [
            "source ADU is float64 with integer values; the per-file span ranges 16..22 bits, not a uniform 21 bit",
            "the source is NOT the 12 bit of MV-CS050-10GC; this link validates the board-side data path and the fixed-point chain only, and does not stand in for the real camera's bit depth or noise",
            "paper is 2016x2016; this tool center-crops to 1920x1920 and loses the border, so it is no longer same-source with the paper 126x126 comparison path",
            "payload is packed (12 bit right-aligned in a 16 bit container) per the camera_b.yaml default; sensor.value_packing is still 'todo'",
            "block_sum layout is ch-major and matches raw_preproc.sv semantics (G = G1+G2, sum only, no divide)",
            "scale_rule_ok=0 means the two camera_b saturation_rule clauses are mutually infeasible for this frame (p99.9/median > 4.75); no-clipping wins and the achieved codes are recorded",
        ],
    }
    with open(json_path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(doc, fh, ensure_ascii=False, indent=2, sort_keys=False)
        fh.write("\n")

    return psf_path, json_path, doc


def verify_file(path, block):
    """--check：只读回验证（Python 侧），比对头内校验值与实测值。"""
    with open(path, "rb") as fh:
        raw = fh.read()
    hdr = parse_header(raw)
    problems = []
    if hdr["magic"] != MAGIC:
        problems.append("magic")
    if hdr["version"] != VERSION:
        problems.append("version")
    if hdr["header_bytes"] != HEADER_BYTES:
        problems.append("header_bytes")
    expect_payload = hdr["width"] * hdr["height"] * hdr["container_bits"] // 8
    if hdr["payload_bytes"] != expect_payload:
        problems.append("payload_bytes vs width*height*container_bits/8")
    body = raw[hdr["header_bytes"]:]
    if len(body) != hdr["payload_bytes"]:
        problems.append("file truncated")
    if body:
        if (zlib.crc32(body) & 0xFFFFFFFF) != hdr["payload_crc32"]:
            problems.append("crc32")
        codes = np.frombuffer(body, dtype="<u2").astype(np.int64)
        if int(codes.sum()) != hdr["payload_sum"]:
            problems.append("sum")
        if int((codes ** 2).sum()) != hdr["payload_sumsq"]:
            problems.append("sumsq")
        if int(codes.min()) != hdr["payload_min"]:
            problems.append("min")
        if int(codes.max()) != hdr["payload_max"]:
            problems.append("max")
    return hdr, problems


def main(argv=None):
    ap = argparse.ArgumentParser(description="paper .mat -> PenumbraSight SD frame (.psf)")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--mat", help="paper .mat path (variable 'image')")
    src.add_argument("--synth", type=int, metavar="SIDE", help="generate a deterministic synthetic SIDE x SIDE frame")
    ap.add_argument("--check", metavar="PSF", help="verify an existing .psf and exit")
    ap.add_argument("--out-dir", default=os.path.join("build", "psf"))
    ap.add_argument("--name", help="output stem (default: mat file stem, or synth<SIDE>)")
    ap.add_argument("--frame-index", type=int, default=0)
    ap.add_argument("--block", type=int, default=10, help="block side in the de-interleaved plane (B profile: 10)")
    ap.add_argument("--crop", type=int, default=ROI_SIDE, help="center crop side (B profile: 1920)")
    ap.add_argument("--no-crop", action="store_true", help="keep the full source side (paper: 2016)")
    args = ap.parse_args(argv)

    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    if args.check:
        hdr, problems = verify_file(args.check, args.block)
        print("CHECK %s" % args.check)
        print("  magic=%s version=%u size=%ux%u valid_bits=%u container_bits=%u bayer_phase=%u packing=%u rule_ok=%u"
              % (hdr["magic"].decode("ascii", "replace"), hdr["version"], hdr["width"], hdr["height"],
                 hdr["valid_bits"], hdr["container_bits"], hdr["bayer_phase"], hdr["value_packing"],
                 hdr["scale_rule_ok"]))
        if problems:
            print("  FAIL: " + ", ".join(problems))
            return 1
        print("  PASS: header self-consistent, crc32/sum/sumsq/min/max all match")
        return 0

    if args.synth:
        values, meta = synth_image(int(args.synth))
    elif args.mat:
        values, meta = load_mat_image(args.mat)
        meta["crop_offset"] = None
    else:
        ap.error("need --mat or --synth (or --check)")

    if not args.no_crop and values.shape[0] > args.crop and values.shape[1] > args.crop:
        cropped, off = center_crop(values, int(args.crop))
        meta["crop_offset"] = off
        meta["crop_from"] = [int(values.shape[0]), int(values.shape[1])]
        values = cropped
    else:
        meta.setdefault("crop_offset", [0, 0])
        meta["crop_from"] = [int(values.shape[0]), int(values.shape[1])]

    prof, prof_src = load_camera_profile(repo_root)
    psf_path, json_path, doc = pack(values, meta, args, prof, repo_root)

    scale = doc["scale"]
    payload = doc["payload"]
    bs = doc["block_sum"]
    print("PACKED %s" % psf_path)
    print("  source      : %s  shape=%s -> %dx%d  (profile values from %s)"
          % (doc["source"]["var"], doc["source"]["shape"],
             doc["frame"]["width"], doc["frame"]["height"], prof_src))
    print("  scale       : num=%d den=%d  step=%.6f adu  rule_ok=%d  median_code=%d p999_code=%d"
          % (scale["num"], scale["den"], scale["quant_step_adu"], scale["rule_ok"],
             scale["median_code"], scale["p999_code"]))
    print("  quant       : clipped=%d sat_codes=%d  max_err_unclipped=%.4f adu (half_step=%.4f)"
          % (scale["clipped_pixels"], scale["sat_code_count"],
             scale["max_abs_error_adu_unclipped"], scale["half_step_adu"]))
    print("  payload     : crc32=%s sum=%d sumsq=%d min=%d max=%d"
          % (payload["crc32"], payload["sum"], payload["sumsq"], payload["min"], payload["max"]))
    print("  block_sum   : block=%d wall_n=%d total=%d by_ch=%s max=%d bits=%d crc32=%s"
          % (bs["block"], bs["wall_n"], bs["total"], bs["by_ch"], bs["max"],
             bs["bits_required"], bs["crc32"]))
    print("  expected    : %s" % json_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
