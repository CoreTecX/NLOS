# -*- coding: utf-8 -*-
"""生成 host_selftest 用的小 model.psm（**由 Python 侧产出**，方案 §8.1）。

为什么要这个小模型
------------------
C 侧 model_loader 的关键判据是「C 端算出的 CRC32 / 偏移 / 长度 == Python 端写下的
那些数」。用真实的 `penumbrasight_model.psm.write_model()` 落一份**小尺寸**模型，
再让 C 侧算一遍，是唯一能同时验证三件事的办法：

  1. CRC32 与 zlib.crc32 同参数（含逐候选分块累加）；
  2. psm_format.h 的 4096 对齐推导与 psm.py 的 `array_offsets()` 逐字节一致；
  3. 几何哈希不匹配时 C 侧必须拒绝出图（§6.6）。

尺寸选 4x3 -> 2x2 / rank 4 / 3 候选：单条记录 20480 B（**不是** 4096 的整数倍除以
某数那种「幸运参数」，而是需要内部补齐三次），payload 61440 B。
刻意选小是为了让自检在秒级跑完，且失败时能用肉眼看偏移表。

输出两个文件
------------
  <out>.psm            模型本体（小端、无头之外的额外包装）
  <out>.expected.txt   逐项 key=value，供 C 侧断言用；**全部由 Python 计算**
                        （尤其 payload_crc32 与 geometry_hash，C 侧不许自己编）

用法
----
    python tools/make_host_test_model.py ps_tmp/host_test_model.psm
"""

from __future__ import annotations

import os
import sys

import numpy as np

# 几何哈希：真实工程里由 configs/geometry_b.yaml 经 build_geometry_hash() 算出。
# 自检用一个固定值，目的是能构造出「期望值 != 文件里的值」的反例。
GEOMETRY_HASH = 0x5EED2026

# 小模型尺寸（与 C 侧 host_selftest 的 kTiny* 常量一一对应）
TINY = dict(meas_h=4, meas_w=3, hidden_h=2, hidden_w=2,
            candidates=3, rank_max=4, rank_default=2, loc_rank=2)


def _repo_root() -> str:
    """本脚本在 src/software/tools/ 下，仓库根在上三级。"""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, os.pardir, os.pardir, os.pardir))


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        sys.stderr.write("用法：python tools/make_host_test_model.py <输出.psm>\n")
        return 2
    out_path = argv[1]

    # 直接复用仓库里的 psm.py —— 绝不在这里抄一份布局定义，
    # 否则「两份实现同时写错、于是测试通过」这种事就会发生。
    sys.path.insert(0, os.path.join(_repo_root(), "src", "model"))
    from penumbrasight_model.psm import (  # noqa: E402
        CandidateRecord, PsmHeader, summarize, verify_model, write_model,
    )

    header = PsmHeader(
        meas_h=TINY["meas_h"], meas_w=TINY["meas_w"],
        hidden_h=TINY["hidden_h"], hidden_w=TINY["hidden_w"],
        candidates=TINY["candidates"],
        rank_max=TINY["rank_max"], rank_default=TINY["rank_default"],
        loc_rank=TINY["loc_rank"],
        geometry_hash=GEOMETRY_HASH,
    )
    header.validate()

    rng = np.random.default_rng(20260926)
    r, m, n = header.rank_max, header.meas_len, header.hidden_len
    records = []
    for i in range(TINY["candidates"]):
        records.append(CandidateRecord(
            x_mm=10 * i,
            wall_distance_mm=-20 + 10 * i,
            z_mm=20 * i,
            u_scale=1.0 / 4096.0,
            s_scale=1.0 / 16384.0,
            v_scale=1.0 / 4096.0,
            U=rng.integers(-32768, 32767, size=(r, m), dtype=np.int16),
            S=rng.integers(0, 65535, size=(r,), dtype=np.uint16),
            V=rng.integers(-32768, 32767, size=(r, n), dtype=np.int16),
            observable_mask=np.array([1, 1, 0, 1], dtype=np.uint8),
            # s_max 逐候选不同（2026-09-29 起入候选头 +20）。合成模型不关心绝对
            # 幅度，这里给一个可辨识的确定值，顺带覆盖「候选头 24 字节」这条路径。
            s_max=50.0 + i,
        ))

    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    summary = write_model(out_path, header, records, verify=True)
    chk = verify_model(out_path, expect_hash=GEOMETRY_HASH)

    offsets = header.array_offsets()
    file_bytes = os.path.getsize(out_path)
    lines = [
        "magic=PSM1",
        "version=1",
        "header_bytes=64",
        "meas_h=%d" % header.meas_h,
        "meas_w=%d" % header.meas_w,
        "hidden_h=%d" % header.hidden_h,
        "hidden_w=%d" % header.hidden_w,
        "candidates=%d" % header.candidates,
        "rank_max=%d" % header.rank_max,
        "rank_default=%d" % header.rank_default,
        "loc_rank=%d" % header.loc_rank,
        "meas_len=%d" % header.meas_len,
        "hidden_len=%d" % header.hidden_len,
        "record_bytes=%d" % header.record_bytes,
        "payload_bytes=%d" % summary["payload_bytes"],
        "file_bytes=%d" % file_bytes,
        "u_offset=%d" % offsets["U"],
        "s_offset=%d" % offsets["S"],
        "v_offset=%d" % offsets["V"],
        "mask_offset=%d" % offsets["observable_mask"],
        "geometry_hash=0x%08X" % header.geometry_hash,
        "payload_crc32=0x%08X" % summary["crc32"],
        "array_align=%d" % 4096,
    ]
    expected_path = os.path.splitext(out_path)[0] + ".expected.txt"
    with open(expected_path, "w", encoding="ascii", newline="\n") as f:
        f.write("\n".join(lines) + "\n")

    # 打印到 stdout：make 的日志里就能直接看到 Python 侧的数字，
    # 与 C 侧自检打印的数字贴在同一个输出里，人工对照成本为零。
    print("[py] %s" % summarize(out_path))
    print("[py] 校验：%s" % chk)
    print("[py] 期望表已写出：%s" % expected_path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
