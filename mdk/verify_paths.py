#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验 .uvprojx 中所有 FilePath / IncludePath 在磁盘上真实存在。"""
import re
import os

PROJ = "STM32H743VGT6_FOC.uvprojx"
t = open(PROJ, encoding="utf-8").read()

paths = re.findall(r"<FilePath>([^<]*)</FilePath>", t)
missing = []
for p in paths:
    if not os.path.exists(p.replace("\\", "/")):
        missing.append(p)
print(f"FilePath  总数: {len(paths)}   存在: {len(paths)-len(missing)}   缺失: {len(missing)}")
for m in missing:
    print("   MISSING FILE:", m)

incs = [i for i in re.findall(r"<IncludePath>([^<]*)</IncludePath>", t) if i.strip()]
assert len(incs) == 1, f"IncludePath 段数异常: {len(incs)}"
dirs = incs[0].split(";")
bad = [d for d in dirs if not os.path.isdir(d.replace("\\", "/"))]
print(f"IncludePath 目录数: {len(dirs)}   存在: {len(dirs)-len(bad)}   缺失: {len(bad)}")
for b in bad:
    print("   MISSING DIR:", b)

sct = re.findall(r"<ScatterFile>([^<]*)</ScatterFile>", t)
print(f"ScatterFile: {sct}  存在: {[os.path.exists(s.replace(chr(92),'/')) for s in sct]}")

print()
print("OK" if not missing and not bad else "FAILED")
