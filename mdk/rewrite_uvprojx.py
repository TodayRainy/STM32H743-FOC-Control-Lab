#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
重写 Keil .uvprojx 以适配重组后的目录结构。

策略:
  1. 逐条替换 <FilePath> 旧路径 -> 新路径 (使用显式映射表, 不做模糊匹配)
  2. 替换 <IncludePath>
  3. 替换 <GroupName> 使分组与新目录层次一致
  4. 替换 <OutputDirectory> 到 build/
  5. 严格校验: 每条映射必须命中且仅命中一次; 替换后不得残留 other_file

.uvprojx 的路径基准目录是工程文件所在目录 (原 MDK-ARM/, 现 mdk/),
两者都在根目录下一层, 因此 ../Core ../Drivers 这类路径无需改动。
"""
import re
import sys
import os
from collections import OrderedDict

PROJ = "STM32H743VGT6_FOC.uvprojx"

# ---- FilePath 映射: 旧 -> 新 (反斜杠形式, 与文件中一致) ----
FILE_MAP = OrderedDict([
    # dsp
    (r".\other_file\dsp_h7\arm_cortexM7lfdp_math.lib",
     r"..\third_lib\CMSIS-DSP\arm_cortexM7lfdp_math.lib"),
    # system
    (r".\other_file\system_h7\sys_m7.c", r"..\system\sys_m7.c"),
    # app
    (r".\other_file\bsp\bsp.cpp",        r"..\app\app_init.cpp"),
    (r".\other_file\foc\foc_app.cpp",    r"..\app\foc_app.cpp"),
    (r".\other_file\debug\debug.c",      r"..\app\debug\debug.c"),
    # port
    (r".\other_file\foc\foc_hw.cpp",     r"..\port\foc_hw.cpp"),
    (r".\other_file\bsp\bsp_irq.cpp",    r"..\port\irq.cpp"),
    # driver
    (r".\other_file\bsp\w25qxx\w25qxx.cpp",       r"..\driver\w25qxx\w25qxx.cpp"),
    (r".\other_file\bsp\adc_dma\adc_dma.cpp",     r"..\driver\adc_dma\adc_dma.cpp"),
    (r".\other_file\bsp\mpu\mpu.cpp",             r"..\driver\mpu\mpu.cpp"),
    (r".\other_file\bsp\can\bsp_can.cpp",         r"..\driver\can\bsp_can.cpp"),
    (r".\other_file\bsp\mt6835gt\mt6835gt.cpp",   r"..\driver\mt6835gt\mt6835gt.cpp"),
    (r".\other_file\bsp\mt6701acd\mt6701acd.cpp", r"..\driver\mt6701acd\mt6701acd.cpp"),
    # algorithm/foc/core
    (r".\other_file\foc\foc_math.cpp",   r"..\algorithm\foc\core\foc_math.cpp"),
    (r".\other_file\foc\foc_pid.cpp",    r"..\algorithm\foc\core\foc_pid.cpp"),
    (r".\other_file\foc\foc_filter.cpp", r"..\algorithm\foc\core\foc_filter.cpp"),
    # algorithm/foc/control
    (r".\other_file\foc\foc_ctrl.cpp",   r"..\algorithm\foc\control\foc_ctrl.cpp"),
    (r".\other_file\foc\foc_speed.cpp",  r"..\algorithm\foc\control\foc_speed.cpp"),
    (r".\other_file\foc\foc_time.cpp",   r"..\algorithm\foc\control\foc_time.cpp"),
    # algorithm/foc/modulation
    (r".\other_file\foc\foc_pwm.cpp",    r"..\algorithm\foc\modulation\foc_pwm.cpp"),
    # algorithm/foc/observer
    (r".\other_file\foc\foc_vf.cpp",     r"..\algorithm\foc\observer\foc_vf.cpp"),
    (r".\other_file\foc\foc_smo.cpp",    r"..\algorithm\foc\observer\foc_smo.cpp"),
    (r".\other_file\foc\foc_flux.cpp",   r"..\algorithm\foc\observer\foc_flux.cpp"),
    (r".\other_file\foc\foc_hfi.cpp",    r"..\algorithm\foc\observer\foc_hfi.cpp"),
    # algorithm/foc/sensor
    (r".\other_file\foc\foc_abzenc.cpp", r"..\algorithm\foc\sensor\foc_abzenc.cpp"),
    (r".\other_file\foc\foc_absenc.cpp", r"..\algorithm\foc\sensor\foc_absenc.cpp"),
    # algorithm/foc/ident
    (r".\other_file\foc\foc_iden.cpp",   r"..\algorithm\foc\ident\foc_iden.cpp"),
    # algorithm/foc (顶层)
    (r".\other_file\foc\foc_motor.cpp",  r"..\algorithm\foc\foc_motor.cpp"),
    (r".\other_file\foc\foc_current.cpp", r"..\algorithm\foc\foc_current.cpp"),
    # third_lib: USB 库 (整目录搬迁, 内部相对关系不变)
    (r".\other_file\usb\lib\STM32_USB_Device_Library\Core\Src\usbd_core.c",
     r"..\third_lib\STM32_USB_Device_Library\Core\Src\usbd_core.c"),
    (r".\other_file\usb\lib\STM32_USB_Device_Library\Core\Src\usbd_ctlreq.c",
     r"..\third_lib\STM32_USB_Device_Library\Core\Src\usbd_ctlreq.c"),
    (r".\other_file\usb\lib\STM32_USB_Device_Library\Core\Src\usbd_ioreq.c",
     r"..\third_lib\STM32_USB_Device_Library\Core\Src\usbd_ioreq.c"),
    (r".\other_file\usb\lib\STM32_USB_Device_Library\Class\MSC\Src\usbd_msc.c",
     r"..\third_lib\STM32_USB_Device_Library\Class\MSC\Src\usbd_msc.c"),
    (r".\other_file\usb\lib\STM32_USB_Device_Library\Class\MSC\Src\usbd_msc_bot.c",
     r"..\third_lib\STM32_USB_Device_Library\Class\MSC\Src\usbd_msc_bot.c"),
    (r".\other_file\usb\lib\STM32_USB_Device_Library\Class\MSC\Src\usbd_msc_data.c",
     r"..\third_lib\STM32_USB_Device_Library\Class\MSC\Src\usbd_msc_data.c"),
    (r".\other_file\usb\lib\STM32_USB_Device_Library\Class\MSC\Src\usbd_msc_scsi.c",
     r"..\third_lib\STM32_USB_Device_Library\Class\MSC\Src\usbd_msc_scsi.c"),
    (r".\other_file\usb\lib\STM32_USB_Device_Library\Class\CDC\Src\usbd_cdc.c",
     r"..\third_lib\STM32_USB_Device_Library\Class\CDC\Src\usbd_cdc.c"),
    (r".\other_file\usb\lib\STM32_USB_Device_Library\Class\COMPLEX\usbd_complex.c",
     r"..\third_lib\STM32_USB_Device_Library\Class\COMPLEX\usbd_complex.c"),
    # third_lib: usb_user
    (r".\other_file\usb\user\usb_device.c",          r"..\third_lib\usb_user\usb_device.c"),
    (r".\other_file\usb\user\usbd_cdc_desc.c",       r"..\third_lib\usb_user\usbd_cdc_desc.c"),
    (r".\other_file\usb\user\usbd_cdc_interface.c",  r"..\third_lib\usb_user\usbd_cdc_interface.c"),
    (r".\other_file\usb\user\usbd_msc_desc.c",       r"..\third_lib\usb_user\usbd_msc_desc.c"),
    (r".\other_file\usb\user\usbd_storage.c",        r"..\third_lib\usb_user\usbd_storage.c"),
    (r".\other_file\usb\user\usbd_conf.c",           r"..\third_lib\usb_user\usbd_conf.c"),
    (r".\other_file\usb\user\usbd_complex_desc.c",   r"..\third_lib\usb_user\usbd_complex_desc.c"),
    # readme -> resources (工程里只是挂着看的文本, 不参与编译)
    (r".\other_file\foc\readme.txt",  r"..\resources\foc_readme.txt"),
])

# ---- GroupName 映射: 使 Keil 分组树与新目录一致 ----
GROUP_MAP = OrderedDict([
    ("Application/MDK-ARM",         "mdk"),
    ("Application/User/Core",       "firmware/Core"),
    ("Drivers/STM32H7xx_HAL_Driver", "firmware/HAL"),
    ("Drivers/CMSIS",               "firmware/CMSIS"),
    ("dsp",                         "third_lib/CMSIS-DSP"),
    ("system",                      "system"),
    ("bsp",                         "driver"),
    ("usb/core",                    "third_lib/usb/core"),
    ("usb/device",                  "third_lib/usb/class"),
    ("usb/user",                    "third_lib/usb/user"),
    ("debug",                       "app"),
    ("foc",                         "algorithm/foc"),
])

OLD_INC = ("../Core/Inc;../Drivers/STM32H7xx_HAL_Driver/Inc;"
           "../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy;"
           "../Drivers/CMSIS/Device/ST/STM32H7xx/Include;../Drivers/CMSIS/Include;"
           "./other_file/system_h7;./other_file/bsp;./other_file/usb/user;"
           "./other_file/usb/lib/STM32_USB_Device_Library/Class/CDC/Inc;"
           "./other_file/usb/lib/STM32_USB_Device_Library/Class/MSC/Inc;"
           "./other_file/usb/lib/STM32_USB_Device_Library/Class/COMPLEX;"
           "./other_file/usb/lib/STM32_USB_Device_Library/Core/Inc;"
           "./other_file/foc;./other_file/dsp_h7;./other_file/debug")

# 新 IncludePath: 保持扁平 include 风格 (用户决定 ④ 不改 include 写法),
# 因此每个含头文件的目录都要出现在这里。
NEW_INC = ";".join([
    # CubeMX 固件层
    "../Core/Inc",
    "../Drivers/STM32H7xx_HAL_Driver/Inc",
    "../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy",
    "../Drivers/CMSIS/Device/ST/STM32H7xx/Include",
    "../Drivers/CMSIS/Include",
    # system
    "../system",
    # app
    "../app",
    "../app/debug",
    # port
    "../port",
    # driver (bsp.hpp 在 driver 根, 各器件在子目录; 保留 "mt6835gt/mt6835gt.hpp" 式写法)
    "../driver",
    "../driver/adc_dma",
    "../driver/can",
    "../driver/mpu",
    "../driver/mt6835gt",
    "../driver/mt6701acd",
    "../driver/w25qxx",
    # algorithm
    "../algorithm/foc",
    "../algorithm/foc/core",
    "../algorithm/foc/control",
    "../algorithm/foc/modulation",
    "../algorithm/foc/observer",
    "../algorithm/foc/sensor",
    "../algorithm/foc/ident",
    # third_lib
    "../third_lib/CMSIS-DSP",
    "../third_lib/usb_user",
    "../third_lib/STM32_USB_Device_Library/Core/Inc",
    "../third_lib/STM32_USB_Device_Library/Class/CDC/Inc",
    "../third_lib/STM32_USB_Device_Library/Class/MSC/Inc",
    "../third_lib/STM32_USB_Device_Library/Class/COMPLEX",
])


def main():
    if not os.path.exists(PROJ):
        sys.exit(f"FATAL: {PROJ} not found (cwd={os.getcwd()})")

    with open(PROJ, "r", encoding="utf-8") as f:
        src = f.read()
    text = src
    errors = []

    # ---- 1. FilePath ----
    for old, new in FILE_MAP.items():
        tag_old = f"<FilePath>{old}</FilePath>"
        n = text.count(tag_old)
        if n != 1:
            errors.append(f"FilePath 命中 {n} 次 (应为1): {old}")
            continue
        text = text.replace(tag_old, f"<FilePath>{new}</FilePath>")

    # ---- 2. IncludePath ----
    n = text.count(OLD_INC)
    if n != 1:
        errors.append(f"IncludePath 命中 {n} 次 (应为1)")
    else:
        text = text.replace(OLD_INC, NEW_INC)

    # ---- 3. GroupName ----
    for old, new in GROUP_MAP.items():
        tag_old = f"<GroupName>{old}</GroupName>"
        n = text.count(tag_old)
        if n != 1:
            errors.append(f"GroupName 命中 {n} 次 (应为1): {old}")
            continue
        text = text.replace(tag_old, f"<GroupName>{new}</GroupName>")

    # ---- 4. OutputDirectory / OutputName 保持, 输出改到 build/ ----
    old_out = "<OutputDirectory>STM32H743VGT6_FOC\\</OutputDirectory>"
    if text.count(old_out) == 1:
        text = text.replace(old_out, "<OutputDirectory>build\\</OutputDirectory>")
    else:
        errors.append(f"OutputDirectory 命中 {text.count(old_out)} 次 (应为1)")

    # ---- 5. 残留检查 ----
    if "other_file" in text:
        leftover = sorted(set(re.findall(r"[^<>]*other_file[^<>]*", text)))
        for l in leftover:
            errors.append(f"残留 other_file: {l}")

    if errors:
        print("=== 校验失败, 未写入 ===")
        for e in errors:
            print("  !", e)
        sys.exit(1)

    with open(PROJ, "w", encoding="utf-8") as f:
        f.write(text)

    print("=== .uvprojx 重写成功 ===")
    print(f"  FilePath  替换: {len(FILE_MAP)} 条")
    print(f"  GroupName 替换: {len(GROUP_MAP)} 条")
    print(f"  IncludePath 替换: 1 条 ({len(NEW_INC.split(';'))} 个目录)")
    print(f"  OutputDirectory -> build\\")
    print(f"  文件大小: {len(src)} -> {len(text)} 字节")


if __name__ == "__main__":
    main()
