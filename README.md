# STM32H743 FOC Control Lab

基于 STM32H743VGTx 的 PMSM 矢量控制研究平台。项目以位置—速度—电流串级控制为基础，通过运行时命令、40 kHz 内存软件示波器和主机自动化脚本，逐步研究电机参数辨识、三环自整定、dq 解耦与前馈、电压约束、弱磁和无感控制。

> 当前发布基线：v0.3.1-stage4。阶段1电气参数辨识、阶段2三环整定、阶段3解耦/电压约束和阶段4弱磁已在当前空载边界内收尾；阶段5正在研究Active Flux与HFI全速域无感。本项目不是量产电机控制器，也不代表完成了带载、热、功率波形或汽车级安全验证。

## 项目亮点

| 能力 | 当前实现 |
|---|---|
| 实时控制 | 40 kHz FOC，4 kHz 速度环，400 Hz 位置环 |
| 控制结构 | 电流、速度、位置三种运行模式和串级控制 |
| 调制 | SVPWM/SPWM，电压矢量限幅和 PI tracking |
| 模型补偿 | OFF/CROSS/BEMF/BOTH分项dq前馈与LEGACY/IDENTIFIED模型 |
| 弱磁 | OFF/FEEDBACK/HYBRID，5 A总dq电流圆，复位默认IDENTIFIED+BOTH+HYBRID |
| 状态管理 | lifecycle/operation/rotor source/diagnostic分维度，兼容原MODE遥测 |
| 主机接口 | USB CDC ASCII 命令、36 通道二进制遥测 |
| 软件示波器 | 12 通道 × 4096 点，40/4/0.4 kHz，串口/J-Link 导出 |
| 自动化 | 一次上电连续覆盖电流、速度、前馈和位置工况 |
| 离线验证 | Clarke/Park、SVPWM、PI、轨迹、电压限制主机回归 |
| 研究路线 | 参数辨识 → 三环整定 → 解耦/电压约束 → 弱磁 → 无感控制 |

## 控制与数据架构

~~~mermaid
flowchart LR
    PC[Python 自动化与分析] -->|ASCII 命令| USB[USB CDC]
    USB --> APP[应用命令与参数]
    APP --> SUP[电机 Supervisor]
    SUP --> POS[位置环 400 Hz]
    POS --> SPD[速度环 4 kHz]
    SPD --> FW[弱磁与5 A电流圆 4 kHz]
    FW --> CUR[电流环 40 kHz]
    ADC[ADC 电流] --> CUR
    ENC[ABZ 编码器] --> ROTOR[转子来源选择]
    HFI[HFI] --> ROTOR
    AF[Active Flux] --> ROTOR
    ROTOR --> POS
    ROTOR --> SPD
    ROTOR --> CUR
    CUR --> FF[dq 前馈]
    FF --> LIM[矢量限幅]
    LIM --> PWM[SVPWM]
    PWM --> INV[三相功率级]
    INV --> PMSM[PMSM]
    CUR --> SCOPE[D2 SRAM Scope]
    SCOPE --> USB
    SCOPE --> JLINK[J-Link 后备读取]
~~~

详细实现见 [系统架构](docs/ARCHITECTURE.md)。阶段1参数、阶段2三环和阶段3 IDENTIFIED+BOTH前馈均已冻结。D-061～D-065按 [弱磁研究](docs/FIELD_WEAKENING_RESEARCH.md) 完成标准电压反馈PI、IDENTIFIED电压椭圆前馈及5 A总dq电流圆；±5000/±6500 rpm双向矩阵及固定±6500 rpm的`VOLT LIMIT 1.00→0.45→1.00`动态边界均通过。HYBRID在保持FEEDBACK电压跟踪水平的同时降低动态电流，复位默认已更新为IDENTIFIED+BOTH+HYBRID。阶段5已完成Active Flux中高速闭环和HFI首轮可观测性研究；D-074将控制调度统一到ADC时基并拆分状态所有权，为后续IPD、d轴HFI跟踪和全速域切换提供单一supervisor。MTPA延期到具备受控负载或转矩/功率测量后再研究。

## 当前验证结果

测试条件：ProDrone 3505 KV650、10 极对、24 V/4.5 A 限流电源、空载、无物理示波器。

| 项目 | 当前摘要 |
|---|---|
| Keil 增量构建 | D-074：0 error，0 warning；Flash 116,448 B、RAM 286,640 B（未烧录） |
| 主机算法回归 | 110项Python通过，C++ PASS |
| id = +0.2 A，iq = 0 | 约 125 μs 上升时间，约 23.5% 滤波后超调 |
| id = -0.2 A，iq = 0 | 约 175 μs 上升时间，约 13.35% 滤波后超调 |
| 空载速度 | D-043 候选覆盖正反 4000 rpm；默认速度 PI 的实测闭环带宽约 45–55 Hz |
| dq 前馈 | D-057中BOTH降低PI负担87.19%，CROSS重复改善动态dq误差；D-058确认受限后恢复，D-059源码默认IDENTIFIED+BOTH |
| 弱磁 | D-064稳态矩阵通过；D-065动态边界4次访问、44/44通过；默认HYBRID，空载人工边界验证至双向6500 rpm |
| 位置 | 双重轨迹已修复；稳态误差代表矩阵全部通过，默认保留p6+RAMP，约2.2°区域相关小行程过冲作为空载边界 |
| Scope ISR 开销 | 最大约 124 cycles / 0.258 μs / FOC 周期 1.03% |

完整条件、证据和限制见 [验证报告](docs/VALIDATION_REPORT.md)，阶段 0 机器可读摘要见 [stage0_20260824.json](tests/baselines/stage0_20260824.json)，当前结论索引见 [项目状态](docs/PROJECT_STATUS.md)。

## 硬件与工具链

| 项目 | 配置 |
|---|---|
| MCU | STM32H743VGTx，1 MiB Flash |
| 电机 | ProDrone 3505 KV650 |
| 位置传感器 | ABZ 编码器 |
| 电源 | 24 V，当前实验限流 4.5 A |
| PWM/FOC | TIM1 中心对齐，40 kHz 控制 |
| IDE | Keil MDK 5.37 |
| 编译器 | ARMCC 5.06 update 7 |
| 调试器 | SEGGER J-Link，项目验证版本 V7.54d |
| 主机测试 | Python 3、pyserial、MinGW g++ |

硬件原理图：[Vortex-H7 原理图](resources/Vortex-H7原理图.pdf)。

## 快速开始

### 1. 获取代码

~~~powershell
git clone https://github.com/TodayRainy/STM32H743-FOC-Control-Lab.git
cd STM32H743-FOC-Control-Lab
~~~

仓库第一次推送采用私有可见性时，clone 需要登录拥有访问权限的 GitHub 账号。

### 2. 离线算法回归

Windows PowerShell 和 PATH 中的 g++：

~~~powershell
powershell -ExecutionPolicy Bypass -File tests/run_algorithm_regression.ps1
~~~

预期结尾：

~~~text
algorithm regression: PASS
~~~

### 3. Keil 构建

打开：

~~~text
mdk/STM32H743VGT6_FOC.uvprojx
~~~

Target：

~~~text
STM32H743VGT6_FOC
~~~

构建输出位于 mdk/build，该目录不进入版本控制。

### 4. 硬件测试

硬件测试会驱动电机。必须确认：

- 电机可以自由旋转；
- 24 V 电源和限流设置正确；
- 当前固件上电不会自动启动；
- 串口没有被其他程序占用；
- 本次速度、电流和位置范围已经明确。

阶段 0 默认命令：

~~~powershell
python tests/stage0_validation.py --port COM_PORT
~~~

详细步骤见 [测试说明](tests/README.md) 和 [工作流](docs/WORKFLOW.md)。

## 软件示波器

FOC ISR 不直接发送串口，而是向 D2 SRAM 固定缓冲写入：

| 分频 | 采样率 | 4096 点时长 | 典型用途 |
|---:|---:|---:|---|
| 1 | 40 kHz | 102.4 ms | 电流环和快速瞬态 |
| 10 | 4 kHz | 1.024 s | 速度环和短位置过程 |
| 100 | 400 Hz | 10.24 s | 慢轨迹和稳态漂移 |

通道：

~~~text
ia, ib, iq, id, iq_ref, id_ref, uq, ud, elec_angle, elec_speed, speed_ref, voltage_scale
~~~

捕获结束后通过 USB CDC 导出；虚拟串口不可用时，可以根据 MAP 符号通过 J-Link 读取。

协议见 [串口与软件示波器协议](docs/SERIAL_PROTOCOL.md)。

## 仓库结构

| 路径 | 内容 |
|---|---|
| algorithm/foc | 数学、控制器、调制、传感器、辨识和观测器 |
| app | 应用参数、状态机、命令、遥测和软件示波器 |
| port、driver | STM32 外设和硬件适配 |
| Core、Drivers | STM32 启动、HAL 和 CMSIS |
| mdk | Keil 工程和链接脚本 |
| tests | 主机回归、串口自动化、Scope 采集和基线摘要 |
| docs | 架构、协议、验证、路线、决策和工作流 |
| resources | 原理图等项目资料 |

## 文档

- [项目状态](docs/PROJECT_STATUS.md)
- [系统架构](docs/ARCHITECTURE.md)
- [验证报告](docs/VALIDATION_REPORT.md)
- [串口与 Scope 协议](docs/SERIAL_PROTOCOL.md)
- [长期研究路线](docs/ROADMAP.md)
- [电流环主机监督式自整定规范](docs/CURRENT_LOOP_AUTOTUNING.md)
- [ABZ 编码器角度与速度估算评估规范](docs/ENCODER_SPEED_ESTIMATION.md)
- [速度环经验自整定规范](docs/SPEED_LOOP_AUTOTUNING.md)
- [位置环高效整定规范](docs/POSITION_LOOP_TUNING.md)
- [阶段 3：dq 解耦、前馈与电压约束研究大纲](docs/STAGE3_DECOUPLING_AND_VOLTAGE.md)
- [下一窗口交接 Prompt](docs/NEXT_SESSION_PROMPT.md)
- [硬件与工具链](docs/HARDWARE_TOOLCHAIN.md)
- [开发和实验工作流](docs/WORKFLOW.md)
- [关键决策记录](docs/DECISIONS.md)
- [测试说明](tests/README.md)
- [版本记录](CHANGELOG.md)
- [Agent 协作规则](AGENTS.md)

## 后续路线

共享主干：

~~~text
阶段 0 基线
    ↓
Rs / Ld / Lq / 磁链 / 机械参数辨识
    ↓
电流—速度—位置三环系统化设计与自整定
    ↓
解耦、前馈、电压约束
    ↓
弱磁
    ↓
全速域无感
~~~

之后根据目标分支：

- 机器人控制：低速平稳、轨迹跟踪、摩擦/齿槽补偿、扰动观测；
- 新能源电驱：快速转矩控制、弱磁、电压饱和和无感切换；MTPA待受控负载条件具备后恢复。

路线使用能力依赖和实验退出条件，不绑定僵硬工期。详情见 [ROADMAP](docs/ROADMAP.md)。

## 已知限制

- 目前只有空载电机，没有测功机或可控负载；
- 没有物理示波器，不能验证栅极、开关节点、振铃和实际死区；
- 硬件没有比较器，无法建立比较器到 TIM1 BRK 的保护链；
- 死区由栅极驱动处理，现阶段没有修改；
- 位置环稳态保持已验证，但当前空载小行程仍有约 2.2° 的机械区域相关端点过冲；
- Rs/Ld/Lq/磁链辨识、三环整定、阶段3空载解耦/人工电压约束及阶段4空载弱磁已完成当前实验边界内的闭环；自然母线基速、带载解耦/恒功率、效率、退磁边界和无感仍待继续，MTPA因缺少受控负载和转矩/功率测量而延期；
- 部分历史源码为 CP936/GBK，当前没有执行全仓编码迁移。

## 安全说明

本仓库用于学习和控制算法研究。电机和逆变器可能造成触电、烫伤、机械飞出或器件损坏。请使用限流电源、保持电机固定和旋转空间清空，并在修改控制器、功率级或保护逻辑后重新进行分级验证。

软件 STOP、电流限幅和状态机不能替代独立硬件保护。

## 许可与第三方组件

仓库当前没有统一的项目级 LICENSE。STM32 HAL、CMSIS 和其他第三方组件保留各自的版权与许可文件。公开发布或复用前，应分别核对第三方代码、原理图和项目自有代码的授权范围。
