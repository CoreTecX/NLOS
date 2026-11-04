# -*- coding: utf-8 -*-
"""tests/hardware/board_gate.py —— 板端一致性门禁框架（模型 CRC / 几何哈希 / 包长）。

本文件把「板端那一半」从 `test_model_crc.py` 里拆出来，做成一个**与传输无关**的
比对器：它只负责「拿到板端回传的值，与从生成器产物读出的期望值逐字段比对」，
不负责「怎么从板上把值读回来」。

期望值来源（**不硬编码**）
--------------------------
所有期望值都从**生成器产物**读，一处都不写死：

* 结构常量（魔数、`header_bytes`、头部各字段的偏移与字节序）取
  `penumbrasight_model.psm` 的常量与解析结果 —— 该模块是「PC 生成 / 板端读取」
  唯一的格式契约（见其文首）。**注意**：另一条并行工作正在改 psm 格式，所以这里
  刻意不复制任何字面量（魔数、头部字节数、字段偏移一律走 psm 模块）。
* 期望的 `crc32` / `geometry_hash` / `payload_bytes` 以同名 `.manifest.json`
  为准（`build_model.py` 写出的值），并与 `.psm` 头部里记的值互相印证。
* 若需要，可再调 `psm.verify_model()` 对 `.psm` 本体重算一遍 CRC 与 payload 长度
  （`--recompute`），确认基线文件自身自洽。

现在能跑 / 等实物
-----------------
**现在能跑（本机）**：
  * 基线侧：读 `.psm` + `.manifest.json`，导出期望值（`--self-check`，或正常比对时自动做）。
  * 板端侧：接受一份**已经拿到的**板端回传（命令行参数或 JSON 文件），逐字段比对，
    任一已提供字段不等即非零退出。

**等实物（阶段七，PENDING，本机不代跑、不臆造）**：
  1. 板端回传的**通路**：AXU9EGB 上 `model_loader` 算出 CRC/几何哈希/包长后经
     UART / 共享内存 / DMA 回传 PC 这条链路，本机没有板卡，故本框架只接受
     「已经拿到的回传」。板上回传文件格式定下来后，在 `--board-report` 的读取处
     接一个解析器即可，比对逻辑不用动。
  2. 板端 CRC 的**位级一致性**：板端实现的多项式/初值/字节序是否与 `zlib.crc32`
     逐位相同，必须在板上用同一份 `.psm` 复核；本机无法替板端证明这一条。
  3. 板端读到的**字节序与对齐**：板端按 `psm_format.h` 逐字节读 4096 对齐的数组，
     是否与 PC 侧写盘一致（尤其大小端），同样要等板上实测。
  4. 板端几何哈希与运行期 `configs/*.yaml` 的比对（不一致即强制灰屏）：需要真实的
     板上配置寄存器与 `psm_psm_check_header` 的返回值，等硬件在环。

用法
----
    # 基线自检：导出两个 fixture 的期望值（读生成器产物）
    python tests/hardware/board_gate.py --self-check

    # 用一份板端回传比对（给文件或给单值都可）
    python tests/hardware/board_gate.py --psm tests/vectors/model_b_r8_c3.psm \\
        --manifest tests/vectors/model_b_r8_c3.manifest.json \\
        --board-report board/outputs/model_crc.json
    python tests/hardware/board_gate.py --psm tests/vectors/model_b_r8_c3.psm \\
        --board-crc 0xE733584C --board-geom 0xD3BFE1C0 --board-payload-bytes 503808

    # 顺带对 .psm 本体重算一遍 CRC / payload 长度
    python tests/hardware/board_gate.py --psm <...> --manifest <...> --recompute

退出码：全部已提供字段一致 0；任一不一致 1；用法/IO 错误 2。
未提供（板端还没回传）的字段按 PENDING 报告，**不计为失败**。

板端回传 JSON 的字段名（大小写不敏感，值可为十进制或 0x/裸十六进制字符串）：
    {"crc32": ..., "geometry_hash": ..., "payload_bytes": ...,
     "magic": "PSM1", "version": 1, "header_bytes": 64, ...}
只有前三项是板端门禁的核心；其余字段给了就一并比对。
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

# 板端门禁的三个核心字段：板端回传名 -> (.psm/manifest 侧期望名)
_CORE_FIELDS = (
    ("crc32", "payload_crc32"),
    ("geometry_hash", "geometry_hash"),
    ("payload_bytes", "payload_bytes"),
)
# 可选字段：板端回传名 -> 期望名（给了就比）
_OPT_FIELDS = (
    ("magic", "magic"),
    ("version", "version"),
    ("header_bytes", "header_bytes"),
    ("candidates", "candidates"),
    ("rank_max", "rank_max"),
    ("rank_default", "rank_default"),
    ("loc_rank", "loc_rank"),
)


def _as_int(v):
    """接受 int / '0x..' / 裸十六进制 / 十进制字符串。"""
    if v is None:
        return None
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, int):
        return v
    s = str(v).strip()
    if s == "":
        return None
    if s.lower().startswith("0x"):
        return int(s, 16)
    try:
        return int(s, 10)
    except ValueError:
        return int(s, 16)


def read_expected(psm_path, manifest_path=None, recompute=False):
    """从生成器产物读期望值。返回 (expected: dict, notes: list)。"""
    import penumbrasight_model.psm as psm

    notes = []
    header = psm.read_model(psm_path, load_records=False)[0]
    exp = {
        "magic": psm.MAGIC.decode("ascii") if isinstance(psm.MAGIC, bytes) else str(psm.MAGIC),
        "version": header.version,
        "header_bytes": header.header_bytes,
        "meas_h": header.meas_h, "meas_w": header.meas_w,
        "hidden_h": header.hidden_h, "hidden_w": header.hidden_w,
        "candidates": header.candidates,
        "rank_max": header.rank_max, "rank_default": header.rank_default,
        "loc_rank": header.loc_rank,
        "geometry_hash": header.geometry_hash,
        "payload_crc32": header.payload_crc32,
        "payload_bytes": header.payload_bytes,
    }

    if recompute:
        chk = psm.verify_model(psm_path)          # 不一致会抛异常
        exp["payload_crc32"] = chk["crc32"]
        exp["payload_bytes"] = chk["payload_bytes"]
        notes.append("已对 .psm 本体重算 CRC32 / payload 长度（psm.verify_model）")

    if manifest_path:
        with io.open(manifest_path, "r", encoding="utf-8") as f:
            man = json.load(f)
        for key in ("crc32", "geometry_hash", "payload_bytes"):
            if key in man:
                exp_key = "payload_crc32" if key == "crc32" else key
                if exp[exp_key] != man[key]:
                    raise ValueError(
                        "基线不自洽：%s 头部 %s vs manifest %s"
                        % (exp_key, exp[exp_key], man[key]))
        notes.append("期望值已与 manifest 互相印证：%s"
                     % os.path.relpath(manifest_path, _ROOT).replace("\\", "/"))
    return exp, notes


def parse_board_report(args):
    """把命令行参数 / JSON 文件里的板端回传汇总成一个 dict。"""
    got = {}

    def _merge(d):
        for k, v in d.items():
            got[str(k).lower()] = v

    if args.board_report:
        with io.open(args.board_report, "r", encoding="utf-8") as f:
            _merge(json.load(f))
    direct = {
        "crc32": args.board_crc,
        "geometry_hash": args.board_geom,
        "payload_bytes": args.board_payload_bytes,
        "magic": args.board_magic,
        "version": args.board_version,
        "header_bytes": args.board_header_bytes,
    }
    for k, v in direct.items():
        if v is not None:
            got[k] = v
    return got


def compare(exp, got):
    """逐字段比对。返回 rows 列表；每行 {name, expected, got, status}。"""
    rows = []
    for board_name, exp_name in _CORE_FIELDS + _OPT_FIELDS:
        if board_name not in got:
            rows.append({"name": board_name, "expected": exp.get(exp_name),
                         "got": None, "status": "PENDING"})
            continue
        e = exp.get(exp_name)
        g = got[board_name]
        if board_name == "magic":
            g = g.decode("ascii") if isinstance(g, bytes) else str(g)
            ok = (g == e)
        else:
            g = _as_int(g)
            ok = (g == e)
        rows.append({"name": board_name, "expected": e, "got": g,
                     "status": "PASS" if ok else "FAIL"})
    return rows


def fmt_field(name, v):
    """字段值的打印口径：CRC32 / 几何哈希按 0x%08X，其余按原值。"""
    if v is None:
        return "-"
    if name in ("crc32", "geometry_hash"):
        return "0x%08X" % int(v)
    return str(v)


def _discover():
    pairs = []
    if not os.path.isdir(_VEC):
        return pairs
    for fn in sorted(os.listdir(_VEC)):
        if fn.endswith(".psm"):
            man = os.path.join(_VEC, fn[:-4] + ".manifest.json")
            pairs.append((os.path.join(_VEC, fn),
                          man if os.path.exists(man) else None))
    return pairs


def self_check():
    pairs = _discover()
    if not pairs:
        print("[self-check] tests/vectors 下没有 .psm，无可读基线", file=sys.stderr)
        return 2
    for psm_path, man in pairs:
        exp, notes = read_expected(psm_path, man, recompute=True)
        print("[基线] %s" % os.path.relpath(psm_path, _ROOT).replace("\\", "/"))
        print("        crc32=0x%08X  geom=0x%08X  payload_bytes=%d  (%dx%d -> %dx%d)"
              % (exp["payload_crc32"], exp["geometry_hash"], exp["payload_bytes"],
                 exp["meas_h"], exp["meas_w"], exp["hidden_h"], exp["hidden_w"]))
        for n in notes:
            print("        %s" % n)
    print("[board] 板端回传通路：PENDING（等 AXU9EGB 实物，阶段七；本机不代跑）")
    print("[self-check] OK（期望值均来自生成器产物，无硬编码）")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="板端一致性门禁（CRC32 / 几何哈希 / 包长；期望值来自生成器产物）")
    ap.add_argument("--self-check", action="store_true",
                    help="只导出基线期望值（读 tests/vectors 的 .psm + manifest）")
    ap.add_argument("--psm", default=None)
    ap.add_argument("--manifest", default=None)
    ap.add_argument("--recompute", action="store_true",
                    help="对 .psm 本体重算 CRC32 / payload 长度（psm.verify_model）")
    ap.add_argument("--board-report", default=None,
                    help="板端回传 JSON（字段名见文件头）")
    ap.add_argument("--board-crc", default=None)
    ap.add_argument("--board-geom", default=None)
    ap.add_argument("--board-payload-bytes", default=None)
    ap.add_argument("--board-magic", default=None)
    ap.add_argument("--board-version", default=None)
    ap.add_argument("--board-header-bytes", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)

    if args.self_check or not args.psm:
        if not args.self_check and not args.psm:
            ap.error("需要 --psm 或 --self-check")
        return self_check()

    try:
        exp, notes = read_expected(args.psm, args.manifest, recompute=args.recompute)
    except Exception as e:                       # noqa: BLE001
        print("[gate] 读基线失败：%s" % e, file=sys.stderr)
        return 2

    got = parse_board_report(args)
    if not got:
        print("[gate] 没给板端回传（--board-report 或 --board-*）。")
        print("[gate] 基线期望值（供板端回传对照）：")
        for board_name, exp_name in _CORE_FIELDS:
            print("        %-16s = %s" % (board_name, fmt_field(board_name, exp.get(exp_name))))
        print("[gate] 板端门禁：PENDING（等实物回传；本机不代跑、不臆造）")
        return 0

    rows = compare(exp, got)
    print("[gate] 板端回传比对（期望值来自生成器产物）")
    for r in rows:
        print("        %-16s 期望 %-12s 回传 %-12s %s"
              % (r["name"], fmt_field(r["name"], r["expected"]),
                 fmt_field(r["name"], r["got"]), r["status"]))
    bad = [r for r in rows if r["status"] == "FAIL"]
    pend = [r for r in rows if r["status"] == "PENDING"]
    if pend:
        missing = ", ".join(r["name"] for r in pend)
        print("[gate] 未提供（PENDING，不计失败）：%s" % missing)

    verdict = "PASS" if not bad else "FAIL"
    print("[gate] 结论：%s（不一致 %d 项）" % (verdict, len(bad)))

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with io.open(args.out, "w", encoding="utf-8") as f:
            json.dump({"psm": args.psm, "manifest": args.manifest,
                       "expected": exp, "rows": rows, "verdict": verdict,
                       "script": "tests/hardware/board_gate.py"}, f,
                      ensure_ascii=False, indent=2)
        print("[out] %s" % args.out)
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
