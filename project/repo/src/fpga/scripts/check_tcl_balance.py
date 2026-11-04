#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tcl / XDC 括号平衡检查 —— 本机（无 Vivado）唯一做得出的 tcl 检查。

背景：`src/fpga/scripts/README.md` 一直写着「`penumbrasight_bd.tcl` 与 `axu9eg.xdc`
在本机只做得出『花括号/方括号平衡』这一项检查」，但仓库里没有对应脚本 —— 即那句话
不可复现。本脚本把它固化下来，并挂进 `make test`。

检查三项：
  · `{}` 与 `[]` 的嵌套是否平衡
  · 每个逻辑行内的 `"` 是否闭合
  · 是否出现多余的 `}` 或 `]`

规则（够用即可，不做完整 Tcl 词法分析）：
  · 整行注释（首个非空白字符是 `#`）**整行跳过** —— Tcl 注释里允许括号不平衡；
  · 字符串内的括号不计数（含 `\\"` 转义）；
  · 行尾反斜杠续行本项目未使用，仅作防御。

退出码：0 = 全部平衡；1 = 有不平衡项。

路径从本文件位置推导（scripts -> fpga -> src -> 仓库根），因此工作区检出与
ASCII 构建区 `D:\\psight\\penumbrasight` 两个检出都能跑，见
`docs/environment_ascii_build.md`。
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         os.pardir, os.pardir, os.pardir))

# ★ 2026-09-28 改成**自动扫描**，原来是 4 条硬编码路径。
# 原因：新增 `synth_rtl_probe.tcl` 后，这个检查**静默漏过**了它 —— 跑完打印
# 「全部平衡」，但其实只看了那 4 个文件。这与 .gitignore 里记的
# 「枚举式白名单总会漏掉下一个工具」是同一类缺陷，别再留名单。
# 凡是 src/fpga/ 下的 .tcl / .xdc 一律检查。
SCAN_ROOT = os.path.join("src", "fpga")
EXTS = (".tcl", ".xdc")


def _discover():
    found = []
    for dirpath, _dirnames, filenames in os.walk(os.path.join(REPO_ROOT, SCAN_ROOT)):
        for fn in filenames:
            if fn.lower().endswith(EXTS):
                found.append(os.path.relpath(os.path.join(dirpath, fn), REPO_ROOT))
    return sorted(found)


FILES = _discover()


def check(path):
    """返回 (错误列表, 花括号余量, 方括号余量, 未闭合引号, 行数)。"""
    with open(path, "r", encoding="utf-8") as f:
        lines = f.read().split("\n")

    in_str = False
    brace = brack = 0
    errs = []
    for ln_no, line in enumerate(lines, 1):
        if not in_str and line.lstrip().startswith("#"):
            continue
        i = 0
        while i < len(line):
            c = line[i]
            if in_str:
                if c == "\\":
                    i += 2
                    continue
                if c == '"':
                    in_str = False
                i += 1
                continue
            if c == "#" and line[:i].strip() == "":
                break
            if c == '"':
                in_str = True
            elif c == "{":
                brace += 1
            elif c == "}":
                brace -= 1
                if brace < 0:
                    errs.append((ln_no, "多余的 }"))
                    brace = 0
            elif c == "[":
                brack += 1
            elif c == "]":
                brack -= 1
                if brack < 0:
                    errs.append((ln_no, "多余的 ]"))
                    brack = 0
            i += 1
    return errs, brace, brack, in_str, len(lines)


def main():
    print("==== Tcl / XDC 括号平衡检查（无 Vivado 时的唯一 tcl 检查）====")
    ok_all = True
    for rel in FILES:
        p = os.path.join(REPO_ROOT, rel)
        if not os.path.isfile(p):
            print(f"  [FAIL] 找不到 {rel}")
            ok_all = False
            continue
        errs, brace, brack, in_str, n = check(p)
        good = (not errs) and brace == 0 and brack == 0 and not in_str
        ok_all = ok_all and good
        print(f"  [{'ok  ' if good else 'FAIL'}] {rel}")
        print(f"         {n} 行  花括号余量={brace}  方括号余量={brack}  未闭合引号={in_str}")
        for ln_no, msg in errs[:10]:
            print(f"         L{ln_no}: {msg}")
    print("  全部平衡。" if ok_all else "  有不平衡项，需修。")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
