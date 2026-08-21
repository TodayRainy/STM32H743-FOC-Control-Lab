#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
补齐各类构造函数中漏初始化的成员。

背景: C++ 不会自动清零非静态成员, 而这些对象全部用 new 分配,
      未初始化的成员沉底为堆内存残留值。其中危害最大的是:
        - foc_flux::flux_alpha/flux_beta  磁链观测器的积分器状态, 用 += 累积
        - foc_offline_iden 的 22 个辨识中间量
        - foc_pwm::sector / foc_abzenc::over_count 等状态量

策略: 在每个构造函数末尾(右大括号前)插入一段显式清零。
      只补真正漏掉的成员, 不动已赋值的, 因此不改变任何现有行为。

所有源文件是 GBK 编码, 必须按 GBK 读写, 否则中文注释会损坏。
"""
import re
import os
import sys

ENC = "gbk"

# 每项: 文件, 类名, [(成员, 初值)]
PATCHES = [
    ("algorithm/foc/control/foc_ctrl.cpp", "foc_spd_ctrl", [
        ("Ts", "0"),
    ]),
    ("algorithm/foc/control/foc_ctrl.cpp", "foc_pos_ctrl", [
        ("out", "0"),
    ]),
    ("algorithm/foc/modulation/foc_pwm.cpp", "foc_pwm", [
        ("sector", "0"),
        ("svpwm_val1", "0"),
        ("svpwm_val2", "0"),
    ]),
    ("algorithm/foc/observer/foc_vf.cpp", "foc_vf", [
        ("Uout", "0"),
    ]),
    ("algorithm/foc/observer/foc_hfi.cpp", "foc_hfi", [
        ("hfi_freq", "0"),
        ("hfi_time", "0"),
    ]),
    ("algorithm/foc/sensor/foc_abzenc.cpp", "foc_abzenc", [
        ("over_count", "0"),
    ]),
    ("algorithm/foc/sensor/foc_absenc.cpp", "foc_absenc", [
        ("over_count", "0"),
    ]),
    ("driver/adc_dma/adc_dma.cpp", "adc_dma", [
        ("ibus", "0"), ("vbus", "0"), ("temp", "0"),
        ("R1", "0"), ("R2", "0"), ("adc_pot", "0"),
    ]),
    ("driver/mt6701acd/mt6701acd.cpp", "mt6701acd", [
        ("angle", "0"),
    ]),
    ("driver/mt6835gt/mt6835gt.cpp", "mt6835gt", [
        ("dir_count", "0"), ("angle", "0"), ("last_angle", "0"),
        ("OverSpeed", "0"), ("WeakField", "0"), ("UnderVoltage", "0"),
    ]),
]

# foc_flux 与 foc_offline_iden 成员多, 单独处理并加说明注释
FLUX_MEMBERS = [
    "flux_alpha", "flux_beta", "err",
    "eta_x1", "eta_x2", "eta_x1_dt", "eta_x2_dt",
    "Ls_I_alpha", "Ls_I_beta",
]
IDEN_MEMBERS = [
    "step", "Is", "Is_err", "prescale_cnt", "Iqd_cnt",
    "I_start", "I_end", "I_sum", "V_pulse",
    "Vs_l", "Vs_h", "Is_l", "Is_h", "Vs_diff", "Is_diff", "Uq_dif",
    "elec_speed_ref", "elec_speed_set", "elec_speed_acc",
    "Vs_max", "Lqd_dif", "flux",
]

COMMENT = ("    /*补齐漏初始化的成员: "
           "对象由 new 分配, C++ 不会自动清零*/")
COMMENT_FLUX = ("    /*补齐漏初始化的成员。"
                "flux_alpha/flux_beta 是磁链观测器的积分"
                "器状态, 以 += 累加,\n"
                "      未清零就直接积分会让预热"
                "循环从堆内存残留值起算*/")
COMMENT_IDEN = ("    /*补齐漏初始化的辨识中间量: "
                "对象由 new 分配, C++ 不会自动清零*/")


def find_ctor_span(src, cls):
    """返回构造函数体的 (开始索引, 结束索引) — 结束索引指向匹配的右大括号。"""
    sig = f"{cls}::{cls}"
    i = src.find(sig)
    while i >= 0:
        # 跳过声明(以 ; 结尾)和析构
        b = src.find("{", i)
        semi = src.find(";", i)
        if b < 0:
            return None
        if semi >= 0 and semi < b:
            i = src.find(sig, i + 1)
            continue
        d = 0
        for j in range(b, len(src)):
            if src[j] == "{":
                d += 1
            elif src[j] == "}":
                d -= 1
                if d == 0:
                    return (b, j)
        return None
    return None


def patch(path, cls, members, comment=COMMENT):
    """在构造函数体的开头插入清零语句。

    必须插在开头而非末尾: 部分构造函数(如 foc_flux)会在体内跑预热循环,
    这些循环会读写待初始化的成员。插在末尾既没能让预热从零起算,
    还会把预热收敛出的结果冲掉。
    """
    src = open(path, encoding=ENC).read()
    span = find_ctor_span(src, cls)
    if not span:
        return False, f"找不到 {cls} 构造函数"
    b, e = span
    body = src[b:e]

    todo = [(n, v) for n, v in members
            if not re.search(rf"\bthis->{n}\b\s*=", body)
            and not re.search(rf"^\s*{n}\s*=", body, re.M)]
    if not todo:
        return False, f"{cls}: 无需改动"

    width = max(len(n) for n, _ in todo)
    lines = [comment]
    for n, v in todo:
        lines.append(f"    this->{n:<{width}} = {v};")
    ins = "\n".join(lines) + "\n"

    # b 指向构造函数体的左大括号, 紧跟其后插入
    new = src[:b + 1] + "\n" + ins + src[b + 1:]
    open(path, "w", encoding=ENC, newline="").write(new)
    return True, f"{cls}: 补 {len(todo)} 个 ({', '.join(n for n, _ in todo)})"


def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)) or ".")
    total = 0
    for path, cls, members in PATCHES:
        if not os.path.exists(path):
            print(f"  SKIP  {path} 不存在")
            continue
        ok, msg = patch(path, cls, members)
        print(("  OK    " if ok else "  --    ") + msg)
        if ok:
            total += len(members)

    ok, msg = patch("algorithm/foc/observer/foc_flux.cpp", "foc_flux",
                    [(m, "0") for m in FLUX_MEMBERS], COMMENT_FLUX)
    print(("  OK    " if ok else "  --    ") + msg)
    if ok:
        total += len(FLUX_MEMBERS)

    ok, msg = patch("algorithm/foc/ident/foc_iden.cpp", "foc_offline_iden",
                    [(m, "0") for m in IDEN_MEMBERS], COMMENT_IDEN)
    print(("  OK    " if ok else "  --    ") + msg)
    if ok:
        total += len(IDEN_MEMBERS)

    print(f"\n合计补充初始化成员: {total}")


if __name__ == "__main__":
    main()
