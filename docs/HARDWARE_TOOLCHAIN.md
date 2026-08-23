# 硬件与工具链

更新时间：2026-08-24

本文件记录相对稳定的硬件事实、当前工作站工具路径和选择策略。设备是否当前上电、连接或仍使用某个 COM 口属于瞬时状态，不在此文件中假定。

## 硬件事实

| 项目 | 当前事实 |
|---|---|
| MCU | STM32H743VGTx |
| 片上 Flash | 1 MiB |
| 调试接口 | SWD，默认 4000 kHz |
| J-Link 探针 | 本机序列号保存在被忽略的本地状态，不写入发布文档 |
| 目标电压 | 成功连接时曾测得约 3.291 V |
| 主电源 | 24 V，电源限流设置 4.5 A |
| 电机条件 | 空载、可自由旋转 |
| 位置反馈 | 现阶段以编码器为控制和无感算法对照基础 |
| 死区 | 由栅极驱动芯片处理，当前不修改 |
| 硬件 BRK | 无比较器，不能建立比较器到定时器 BRK 的保护链 |
| 物理示波器 | 无 |
| 原理图 | 根目录 Vortex-H7原理图.pdf |

硬件型号和引脚问题应优先查原理图、CubeMX 配置和实际寄存器，不根据文件夹名或旧聊天猜测。

## 构建工具

| 工具 | 路径/版本 | 用途 |
|---|---|---|
| Keil µVision | D:\STM32\keil5\Core\UV4\UV4.exe，V5.37.0.0 | 工程构建 |
| ARMCC | D:\STM32\keil5\Core\ARM\ARMCC\bin\armcc.exe，5.06 系列 | 当前工程编译器 |
| Keil 工程 | mdk/STM32H743VGT6_FOC.uvprojx | 项目文件 |
| Keil Target | STM32H743VGT6_FOC | 默认构建目标 |
| 输出目录 | mdk/build | AXF、HEX、MAP 等 |
| 链接脚本 | mdk/flash.sct | Flash/RAM 布局 |

之前实际构建日志显示 ARMCC 5.06 update 7 build 960；Windows 文件属性只显示 5.06.0.189。需要精确报告编译器时，以新一次构建日志为准。

项目侧 .embeddedskills/config.json 已记录工程、Target 和日志目录，但没有保存 UV4.exe 的绝对路径。调用 Keil Skill 时应确认它最终选择的是上述安装，而不是系统中的其他自动发现路径。

## J-Link 版本策略

### 项目默认

独立版 SEGGER J-Link：

    C:\Program Files\SEGGER\JLink\JLink.exe

项目已选定并实际验证的版本是 V7.54d（2021）。设备参数：

- Device：STM32H743VG
- 主 IROM 区：0x08000000 起始，512 KiB；当前固件仅使用该区，第二 Flash bank 暂未加入链接布局
- Interface：SWD
- Speed：4000 kHz

### 其他本机安装

本机还存在：

    C:\Program Files (x86)\SEGGER\JLink\JLink.exe

这是 2019 年的旧版安装。除非明确进行兼容性排查，不自动回退或调用它。

STM32CubeProgrammer 位于：

    E:\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe

当前个人 J-Link Skill 配置中还声明：

- preferred_connect_flash_backend：stm32cubeprogrammer-jlink
- J-Link DLL：E:\STM32CubeProgrammer\bin\JLink_x64.dll
- DLL 版本：7.92o

这与项目指定 V7.54d 冲突。第一版文档阶段不修改个人 Skill 配置；在冲突消除前，每次 J-Link 操作必须显式使用独立版 V7.54d，禁止根据 preferred backend 自动切换到 7.92o。

### 烧录验证经验

已验证的独立 J-Link Commander 思路是：

1. 连接 STM32H743VG/SWD；
2. reset；
3. halt；
4. loadfile 当前重新构建的 HEX；
5. reset；
6. 读回向量表或关键地址；
7. go。

空白 Flash 或异常固件导致 HardFault 时，先 reset + halt 再 loadfile 比直接加载可靠。

不能只依据封装层返回的“verified”判断成功。曾出现上层报告成功、原始输出却包含 programming failed 的情况。对于 32 位内存写入，也不要把封装参数 width=32 直接翻译为未经验证的原生 w32；项目历史中实际使用过的可靠原生命令是 w4。再次修改内存前仍应根据目标和 J-Link 命令帮助核对语义。

## 串口

当前主机侧设置：

- 端口：以本机当前枚举为准；
- 波特率：115200；
- 数据格式：8N1；
- 主机文本编码配置：UTF-8。

COM 号会随 USB 重新枚举变化，因此：

- 不把具体 COM 号写入固件源代码；
- 不把它当作永久设备 ID；
- 未插拔且通信正常时不重复扫描；
- 插拔、打开失败或用户说明设备变化后再扫描并更新主机配置。

USB 虚拟串口曾出现 Windows “该设备无法启动（代码 10），系统资源不够，无法完成 API”。后来串口已经实际跑通。该历史故障不代表当前必然失败，也不能用最近成功记录推断设备当前仍插着。

运行命令和连续遥测以串口为主；软件示波器在串口不可用时可通过 J-Link 读 SRAM 作为后备。

## 工具和状态来源的优先级

遇到冲突时按以下顺序判断：

1. 用户在当前任务中的明确指示；
2. 本文件的项目选择策略；
3. .embeddedskills/config.json 的项目参数；
4. Skill 的用户级 config.json；
5. 自动发现、PATH 和默认后端；
6. .embeddedskills/state.json 的最近记录。

.embeddedskills/state.json 只用于帮助定位上一次构建、烧录或串口会话。它不能证明：

- 当前设备仍上电；
- 当前 COM 仍存在；
- 当前源码已经构建；
- 当前 Flash 中就是最近工作区代码；
- 最近一次 Skill 的解析结果一定正确。

## 源码编码

工程中部分用户层文件使用 CP936/GBK，例如：

- app/debug/debug.c
- app/debug/debug.h
- app/foc_app.cpp
- driver/mpu/mpu.cpp

编辑这些文件前应检查编码。补丁工具不能直接读取时，采用受控的机械转换：

1. 保留原文件和工作区 diff；
2. CP936/GBK 转 UTF-8；
3. 应用最小语义补丁；
4. UTF-8 转回原编码；
5. 检查 diff 不包含乱码和全文件无关变化。

是否把全工程迁移到 UTF-8 是独立的工程决策，不应夹带在控制算法任务中。

## 中文仓库路径

当前仓库位于含中文的上级目录中。Keil/ARMCC 本次全量构建能够成功，但 MinGW g++ 在接收含中文的绝对输出路径时曾出现路径乱码并导致链接失败。主机回归脚本已改用仓库内相对路径规避。

长期可以把仓库移动到短英文路径以减少旧工具兼容问题；移动不属于当前发布的必要条件。移动后需要重新挂接 IDE/Codex 本地项目，并对 Keil 做一次全量重建。

## 软件观测的能力边界

串口和内存软件示波器适合观察：

- dq 电流、参考值和误差；
- dq 电压和电压限制；
- 电角度、估计速度；
- 模式切换和控制器状态；
- 低频母线、电流峰值及控制性能指标。

它们不能直接验证：

- 栅极驱动高低侧波形；
- 开关节点上升沿、振铃和过冲；
- 实际死区；
- ADC 引脚瞬时噪声；
- 功率器件温升和 EMI。

因此高级算法可以继续开展，但功率级结论必须注明缺少物理测量。
