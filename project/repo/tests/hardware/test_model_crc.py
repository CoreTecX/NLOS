# -*- coding: utf-8 -*-
"""tests/hardware/test_model_crc.py —— 板端模型与 PC 端逐位一致（§12 Task 10）。

验什么：板端 `model_loader` 读出的模型，其 CRC32、payload 长度、几何哈希要与生成时
写进 manifest 的值相等。三者任一不等即判 FAIL —— 几何哈希不等时板端按设计强制灰屏，
这里要测的正是「这条门禁在真板上确实会触发」。

**本脚本唯一在本机能立即运行的部分**：宿主侧一致性。用 `penumbrasight_model.psm`
的 `verify_model()` 对 `tests/vectors/` 的两个小 fixture（论文档 / B 档，各 rank 8、
3 候选）重算 CRC32 与几何哈希，并与同名 `.manifest.json` 比对。这一步验的是「文件本身
没有坏、manifest 记的值与文件自洽」，**不是**板端验证。

等什么条件
----------
- **板端 CRC**：等板卡与相机实物（阶段七）。板端路径需要 `model_loader` 在 AXU9EGB 上
  跑出 CRC 并回传，本机没有板子，故 `--board` 相关的判据一律标 ``PENDING``，**不代跑、
  不臆造**。
- 板端更上游的 DMA/DataMover 搬运与字节序处理也一并等板卡（见
  sim/results/x20_hls_reconstruction_image.md 第七节缺口 5）。

板端侧已拆出
------------
板端「回传值 vs 期望值」的比对逻辑统一在 **`tests/hardware/board_gate.py`**
（与传输无关的门禁框架，期望值同样从生成器产物读、不硬编码）。本脚本的
`--board-crc/--board-geom/--board-payload-bytes` 只是**兼容入口**：给出这些参数时
转调 `board_gate.compare`，不在此重复实现比对。

用法
----
    python tests/hardware/test_model_crc.py --self-check        # 可立即运行
    python tests/hardware/test_model_crc.py                     # 默认扫 tests/vectors
    python tests/hardware/test_model_crc.py --psm <文件> --manifest <文件>
    python tests/hardware/test_model_crc.py --board-crc 54560281 --board-geom 3552567744
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
_MODEL = os.path.join(_ROOT, "src", "model")
if _MODEL not in sys.path:
    sys.path.insert(0, _MODEL)
_VEC = os.path.join(_ROOT, "tests", "vectors")

_SCRIPT_VERSION = "1.0.0"


def _load_manifest(path):
    with io.open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def check_pair(psm_path: str, manifest_path: str) -> dict:
    """宿主侧：重算 .psm 的 CRC 与几何哈希，与 manifest 比对。"""
    import penumbrasight_model.psm as psm
    man = _load_manifest(manifest_path)
    expect_hash = int(man.get("geometry_hash", 0))
    res = psm.verify_model(psm_path, expect_hash=expect_hash or None)
    row = {
        "psm": os.path.relpath(psm_path, _ROOT).replace("\\", "/"),
        "manifest": os.path.relpath(manifest_path, _ROOT).replace("\\", "/"),
        "ok_crc32": bool(res.get("ok_crc32")),
        "ok_payload_bytes": bool(res.get("ok_payload_bytes")),
        "ok_geometry_hash": bool(res.get("ok_geometry_hash")),
        "crc32": int(res.get("crc32", 0)),
        "crc32_manifest": int(man.get("crc32", 0)),
        "geometry_hash": int(res.get("geometry_hash", 0)),
        "geometry_hash_manifest": int(man.get("geometry_hash", 0)),
        "payload_bytes": int(res.get("payload_bytes", 0)),
        "payload_bytes_manifest": int(man.get("payload_bytes", 0)),
    }
    row["match"] = (row["crc32"] == row["crc32_manifest"]
                    and row["geometry_hash"] == row["geometry_hash_manifest"]
                    and row["payload_bytes"] == row["payload_bytes_manifest"])
    row["verdict"] = "PASS" if (row["ok_crc32"] and row["match"]) else "FAIL"
    return row


def discover() -> list:
    pairs = []
    if not os.path.isdir(_VEC):
        return pairs
    for fn in sorted(os.listdir(_VEC)):
        if fn.endswith(".psm"):
            man = os.path.join(_VEC, fn[:-4] + ".manifest.json")
            if os.path.exists(man):
                pairs.append((os.path.join(_VEC, fn), man))
    return pairs


def self_check() -> int:
    pairs = discover()
    if not pairs:
        print("[self-check] tests/vectors 下没有 .psm + .manifest.json 配对，"
              "无可运行对象", file=sys.stderr)
        return 2
    ok = True
    for psm_path, man_path in pairs:
        row = check_pair(psm_path, man_path)
        print(f"[host] {row['psm']}  crc={row['crc32']:08X}  "
              f"geom={row['geometry_hash']:08X}  {row['verdict']}")
        if row["verdict"] != "PASS":
            ok = False
    print("[board] 板端 CRC / 几何哈希门禁：PENDING（等板卡与相机实物，阶段七）")
    print("[self-check] " + ("OK（宿主侧一致性）" if ok else "FAIL"))
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="模型一致性测试（宿主侧可跑；板端等硬件）")
    ap.add_argument("--self-check", action="store_true")
    ap.add_argument("--psm", default=None, help="单个 .psm；不给则扫 tests/vectors")
    ap.add_argument("--manifest", default=None)
    ap.add_argument("--board-crc", default=None, help="板端回传的 CRC32（十六进制）")
    ap.add_argument("--board-geom", default=None, help="板端回传的几何哈希（十六进制）")
    ap.add_argument("--board-payload-bytes", default=None, help="板端回传的 payload 长度")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)

    if args.psm and not args.manifest:
        ap.error("--psm 需同时给 --manifest")
    if args.self_check or (not args.psm and not args.board_crc):
        return self_check()

    rows = []
    if args.psm:
        rows.append(check_pair(args.psm, args.manifest))
        for r in rows:
            print(f"[host] {r['psm']} {r['verdict']}  crc={r['crc32']:08X} "
                  f"geom={r['geometry_hash']:08X}")

    board_verdict = "PENDING"
    if args.board_crc or args.board_geom or args.board_payload_bytes:
        if not rows:
            print("[board] 给了板端回传但没给 --psm/--manifest，无法比对",
                  file=sys.stderr)
            return 2
        # 板端比对统一走 board_gate.py（与传输无关的门禁框架）：本脚本不重复实现，
        # 期望值也仍来自 manifest / .psm（不硬编码）。
        import board_gate as bg
        exp = {
            "payload_crc32": int(rows[0]["crc32_manifest"]),
            "geometry_hash": int(rows[0]["geometry_hash_manifest"]),
            "payload_bytes": int(rows[0]["payload_bytes_manifest"]),
        }
        got = {}
        if args.board_crc:
            got["crc32"] = args.board_crc
        if args.board_geom:
            got["geometry_hash"] = args.board_geom
        if args.board_payload_bytes:
            got["payload_bytes"] = args.board_payload_bytes
        brow = bg.compare(exp, got)
        board_verdict = "PASS" if all(r["status"] != "FAIL" for r in brow) else "FAIL"
        for r in brow:
            print("[board] %-16s 回传 %-12s 期望 %-12s %s"
                  % (r["name"], bg.fmt_field(r["name"], r["got"]),
                     bg.fmt_field(r["name"], r["expected"]), r["status"]))

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with io.open(args.out, "w", encoding="utf-8") as f:
            json.dump({"rows": rows, "board_verdict": board_verdict,
                       "script": "tests/hardware/test_model_crc.py",
                       "version": _SCRIPT_VERSION}, f, ensure_ascii=False, indent=2)
        print(f"[out] {args.out}")
    bad = [r for r in rows if r["verdict"] != "PASS"]
    if board_verdict == "FAIL":
        bad.append({"board": "geom/crc mismatch"})
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
