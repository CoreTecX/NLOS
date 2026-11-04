# -*- coding: utf-8 -*-
"""X21 —— 「第一版交付定义」判据表 ↔ 实现 一致性校验。

台账定义（`01_分析报告/00_分步任务与优先级.md` §四 第 18 行，**以该文件为准**）
--------------------------------------------------------------------------
| 编号 | 任务 | 依赖 | 判据 | 人日 |
|---|---|---|---|---|
| **X21** | 更新第一版交付定义：补「重建出图」判据，删「重建不做」一节 | X14 | **判据表与实现一致** | 0.2 |

「判据表与实现一致」这句话如果只靠人读，就不是一个判据。本脚本把它变成可执行的：

  1. **判据表**：解析 `01_分析报告/07_单人移植任务表/第一版算法移植交付定义.md`
     第二节的「八条判据」表与「附加项」表，逐行核对 序 / 模块 / 判据里的**阈值与
     证据文件名**是否与本脚本里的 `CRITERIA`（= 实现侧契约）一致。
  2. **删干净**：核对第三节的「重建（M5）不做」已**不在生效行**里
     （只允许带 `~~删除线~~` 的留痕行）。
  3. **生成物一致**：解析 `sim/results/x13_acceptance.md`，核对它确实输出了
     `## 八条判据`（1..8 行）与 `## 附加项`（9、10 行）——即实现侧的报告编号
     与定义侧对齐，而不是定义改了、报告还停在「七条 + 附加项 8/9」。
  4. **判据 8(a) 复算**：从 `sim/results/x13_acceptance.json` 读 C++ 内核的
     `recon.sm_err`，与判据 8(a) 的阈值 **1e-2** 比。

设计原则与仓库其它测试一致：**不依赖 Vivado / 板卡**，只用标准库 + numpy（读 json）。
定义文档不在仓库里（它在工作区），所以文档缺失时本脚本**报 SKIP 并返回 0**，
不让裸 clone 的 `make test` 变红；但文档在而内容不一致时**必须报 FAIL**。
"""

from __future__ import annotations

import argparse
import io
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
RES = os.path.join(ROOT, "sim", "results")

DEFAULT_DOC = os.path.abspath(os.path.join(
    ROOT, os.pardir, os.pardir, "01_分析报告", "07_单人移植任务表",
    "第一版算法移植交付定义.md"))

# ------------------------------------------------------------------ 实现侧契约
# 每条 = (序, 模块, 判据里必须出现的串)。这些串就是「实现」：
#   · 阈值（1e-3 / 99% / 1e-2）来自 S0-1、S0-3、X13 的实测口径；
#   · 文件名（v1_block_small.npz 等）来自 sim/vectors 与 sim/results 的实际产物；
#   · 「目视」二字是台账 §八 的一级判据口径，不许被删。
CRITERIA = [
    (1, "M1 预处理", ["v1_block_small.npz", "逐值相等"]),
    (2, "M1 帧累积与差分", ["v3_accum_frames.npz", "逐值相等"]),
    (3, "M1 分块索引", ["crossrow_correct", "crossrow_wrong"]),
    (4, "M2 几何算核", ["forward.py", "99%"]),
    (5, "定位", ["Aᵀy", "1e-3"]),
    (6, "定位", ["top-1", "localization.py"]),
    (7, "端到端", ["RAW", "一条命令"]),
    (8, "M5 重建", ["重建出图", "sm_err", "1e-2", "目视", "recon.hpp"]),
]

ADDITIONAL = [
    (9, "HLS 综合", ["资源", "时序"]),
    (10, "加速比", ["Python", "倍数"]),
]

# 第三节里**不允许**再出现的生效行关键词（只允许带删除线的留痕）
FORBIDDEN_ACTIVE = ["重建（M5）"]


# ------------------------------------------------------------------ 工具


def read_text(path):
    with io.open(path, "r", encoding="utf-8") as f:
        return f.read()


def split_tables(md):
    """把 md 里所有 markdown 表拆出来，返回 [(起始行号, [行, ...]), ...]。"""
    lines = md.splitlines()
    tables = []
    cur = []
    start = 0
    for i, ln in enumerate(lines):
        if ln.lstrip().startswith("|"):
            if not cur:
                start = i + 1
            cur.append(ln)
        else:
            if cur:
                tables.append((start, cur))
                cur = []
    if cur:
        tables.append((start, cur))
    return tables


def cells(row):
    """把 `| a | b |` 拆成 ['a', 'b']，去掉首尾空单元。"""
    parts = [c.strip() for c in row.strip().strip("|").split("|")]
    return parts


def is_sep(row):
    return set(row.replace("|", "").replace(" ", "").replace(":", "")) <= {"-"}


def rows_of(table):
    return [cells(r) for r in table if not is_sep(r)]


# ------------------------------------------------------------------ 各项检查


def check_criteria_table(doc, log):
    """在文档里定位判据表并逐行核对。"""
    tables = split_tables(doc)
    crit_rows = None
    add_rows = None
    for _, tb in tables:
        head = cells(tb[0])
        if head[:2] == ["序", "模块"] and crit_rows is None:
            crit_rows = rows_of(tb)[1:]          # 去掉表头
        elif head[:2] == ["序", "项"] and add_rows is None:
            add_rows = rows_of(tb)[1:]

    ok = True
    if crit_rows is None:
        log.append(("FAIL", "判据表：未在文档里找到表头为「| 序 | 模块 | 判据 | ... |」的表"))
        return False
    if len(crit_rows) != len(CRITERIA):
        log.append(("FAIL", "判据表：文档有 %d 行，实现侧契约为 %d 行"
                    % (len(crit_rows), len(CRITERIA))))
        ok = False

    by_no = {}
    for r in crit_rows:
        m = re.match(r"^\s*(\d+)\s*$", r[0])
        if m:
            by_no[int(m.group(1))] = r

    for no, mod, keys in CRITERIA:
        r = by_no.get(no)
        if r is None:
            log.append(("FAIL", "判据 %d：文档里没有这一行" % no))
            ok = False
            continue
        joined = " | ".join(r[1:])
        if mod not in joined:
            log.append(("FAIL", "判据 %d：模块名里没有「%s」（实为「%s」）"
                        % (no, mod, r[1])))
            ok = False
        miss = [k for k in keys if k not in joined]
        if miss:
            log.append(("FAIL", "判据 %d：判据里缺关键串 %s" % (no, miss)))
            ok = False
        else:
            log.append(("PASS", "判据 %d：%s —— 关键串齐 %s" % (no, r[1], keys)))

    if add_rows is not None:
        add_by_no = {}
        for r in add_rows:
            m = re.match(r"^\s*(\d+)\s*$", r[0])
            if m:
                add_by_no[int(m.group(1))] = r
        for no, name, keys in ADDITIONAL:
            r = add_by_no.get(no)
            if r is None:
                log.append(("FAIL", "附加项 %d：文档里没有这一行（编号未顺延？）" % no))
                ok = False
                continue
            joined = " | ".join(r[1:])
            miss = [k for k in keys if k not in joined]
            if miss or name not in joined:
                log.append(("FAIL", "附加项 %d：缺 %s" % (no, miss or [name])))
                ok = False
            else:
                log.append(("PASS", "附加项 %d：%s" % (no, joined[:40])))
    else:
        log.append(("FAIL", "附加项表：未在文档里找到表头为「| 序 | 项 | 判据 |」的表"))
        ok = False
    return ok


def check_recon_removed(doc, log):
    ok = True
    for kw in FORBIDDEN_ACTIVE:
        for tb_start, tb in split_tables(doc):
            head = cells(tb[0])
            if head[:2] != ["排除项", "原因"]:
                continue
            for r in rows_of(tb)[1:]:
                if kw not in r[0]:
                    continue
                if "~~" in r[0] or "作废" in " ".join(r[1:]):
                    log.append(("PASS", "「%s」只剩带删除线的留痕行" % kw))
                else:
                    log.append(("FAIL", "「%s」仍以生效行存在：%s" % (kw, " | ".join(r))))
                    ok = False
    if ok:
        log.append(("PASS", "第三节排除表里没有生效的「重建不做」行"))
    return ok


def check_generated_report(log):
    p = os.path.join(RES, "x13_acceptance.md")
    if not os.path.exists(p):
        log.append(("FAIL", "缺 %s：实现侧报告不存在，判据表无从对齐" % p))
        return False
    md = read_text(p)
    ok = True
    if "## 八条判据" not in md:
        log.append(("FAIL", "%s 里没有「## 八条判据」标题（实现侧仍停在七条？）" % p))
        ok = False
    else:
        log.append(("PASS", "%s 有「## 八条判据」" % os.path.basename(p)))
    # 判据表行号
    nos = []
    for _, tb in split_tables(md):
        head = cells(tb[0])
        if head[:2] == ["序", "模块"]:
            for r in rows_of(tb)[1:]:
                m = re.match(r"^\s*(\d+)\s*$", r[0])
                if m:
                    nos.append(int(m.group(1)))
            break
    if nos != [1, 2, 3, 4, 5, 6, 7, 8]:
        log.append(("FAIL", "生成报告的判据行号为 %s，期望 [1..8]" % nos))
        ok = False
    else:
        log.append(("PASS", "生成报告的判据行号 = 1..8"))
    # 附加项编号
    adds = []
    for _, tb in split_tables(md):
        head = cells(tb[0])
        if head[:2] == ["序", "项"]:
            for r in rows_of(tb)[1:]:
                m = re.match(r"^\s*(\d+)\s*$", r[0])
                if m:
                    adds.append(int(m.group(1)))
            break
    if adds and adds != [9, 10]:
        log.append(("FAIL", "生成报告的附加项编号为 %s，期望 [9, 10]" % adds))
        ok = False
    elif adds:
        log.append(("PASS", "生成报告的附加项编号 = 9, 10"))
    if "重建出图" not in md:
        log.append(("FAIL", "生成报告里没有「重建出图」一节"))
        ok = False
    else:
        log.append(("PASS", "生成报告有「重建出图」一节"))
    return ok


def check_recon_numeric(log):
    """判据 8(a)：从 X13 的机器可读产物复算 sm_err ≤ 1e-2。"""
    p = os.path.join(RES, "x13_acceptance.json")
    if not os.path.exists(p):
        log.append(("FAIL", "缺 %s" % p))
        return False
    with io.open(p, "r", encoding="utf-8") as f:
        j = json.load(f)
    r = j.get("recon") or {}
    if "sm_err" not in r:
        log.append(("FAIL", "x13_acceptance.json 的 recon 段没有 sm_err"))
        return False
    sm = float(r["sm_err"])
    ok = sm <= 1e-2
    log.append(("PASS" if ok else "FAIL",
                "判据 8(a)：C++ 内核 sm_err = %.4e ≤ 1e-2 -> %s"
                % (sm, "PASS" if ok else "FAIL")))
    log.append(("INFO", "判据 8(b)：真实数据目视可辨 BU —— **需人确认**，机器不代判；"
                        "证据 sim/results/x14_recon_corrected*.png"))
    return ok


def check_x14_evidence(log):
    """判据 8 引用了 X14 的产物，产物必须在。"""
    need = ["x14_recon_corrected.json", "x14_recon_corrected.png",
            "x14_recon_corrected_vs_baseline.png"]
    ok = True
    for fn in need:
        p = os.path.join(RES, fn)
        if os.path.exists(p):
            log.append(("PASS", "X14 产物存在：%s" % fn))
        else:
            log.append(("FAIL", "判据 8 引用的 X14 产物缺失：%s" % fn))
            ok = False
    return ok


# ------------------------------------------------------------------ 主流程


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--doc", default=DEFAULT_DOC)
    args = ap.parse_args()

    print("=" * 78)
    print("X21  第一版交付定义 · 判据表 ↔ 实现 一致性校验")
    print("=" * 78)

    if not os.path.exists(args.doc):
        print("SKIP  定义文档不在本机：%s" % args.doc)
        print("      （文档在仓库之外的工作区，裸 clone 时跳过，不算失败）")
        return 0
    doc = read_text(args.doc)
    print("文档：%s（%d 字节）" % (args.doc, len(doc.encode("utf-8"))))

    log = []
    ok = True
    ok &= check_criteria_table(doc, log)
    ok &= check_recon_removed(doc, log)
    ok &= check_generated_report(log)
    ok &= check_recon_numeric(log)
    ok &= check_x14_evidence(log)

    print()
    for tag, msg in log:
        print("  [%-4s] %s" % (tag, msg))
    n_pass = sum(1 for t, _ in log if t == "PASS")
    n_fail = sum(1 for t, _ in log if t == "FAIL")
    print()
    print("PASS %d  FAIL %d" % (n_pass, n_fail))
    if ok:
        print("判据表与实现一致。")
    else:
        print("存在不一致项，见上面的 FAIL 行。")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
