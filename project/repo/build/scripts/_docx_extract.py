# -*- coding: utf-8 -*-
"""提取 docx 正文（含表格），输出为纯文本。"""
import sys
import zipfile
import xml.etree.ElementTree as ET

W = "{http://schemas.openxmlformats.org/wordprocessingml/2006/main}"


def para_text(p):
    parts = []
    for t in p.iter(W + "t"):
        parts.append(t.text or "")
    return "".join(parts)


def extract(path):
    with zipfile.ZipFile(path) as z:
        xml = z.read("word/document.xml")
    root = ET.fromstring(xml)
    body = root.find(W + "body")
    out = []
    for el in body:
        tag = el.tag
        if tag == W + "p":
            txt = para_text(el)
            out.append(txt)
        elif tag == W + "tbl":
            for row in el.findall(W + "tr"):
                cells = []
                for tc in row.findall(W + "tc"):
                    ctxt = " ".join(para_text(p) for p in tc.findall(W + "p"))
                    cells.append(ctxt.strip())
                out.append(" | ".join(cells))
    return "\n".join(out)


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print("=" * 90)
        print("FILE:", p)
        print("=" * 90)
        print(extract(p))
        print()
