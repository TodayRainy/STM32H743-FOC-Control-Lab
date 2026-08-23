# STM32H743 FOC Control Lab

基于 STM32H743VGTx 的 PMSM 矢量控制研究平台。项目以位置—速度—电流串级控制为基础，通过运行时命令、40 kHz 内存软件示波器和主机自动化脚本，逐步研究电机参数辨识、三环自整定、dq 解耦与前馈、MTPA/弱磁和无感控制。

> 当前版本：v0.1.0-stage0。它是空载控制与测量基线，不是量产电机控制器，也不代表完成了带载、热、功率波形或汽车级安全验证。

## 项目亮点

| 能力 | 当前实现 |
|---|---|
| 实时控制 | 40 kHz FOC，4 kHz 速度环，400 Hz 位置环 |
| 控制结构 | 电流、速度、位置三种运行模式和串级控制 |
| 调制 | SVPWM/SPWM，电压矢量限幅和 PI tracking |
| 模型补偿 | 可运行时开关的 dq 交叉耦合与反电动势前馈 |
| 状态管理 | 电流偏置校准、READY、START、功率级预充、STOP |
| 主机接口 | USB CDC ASCII 命令、36 通道二进制遥测 |
| 软件示波器 | 10 通道 × 4096 点，40/4/0.4 kHz，串口/J-Link 导出 |
| 自动化 | 一次上电连续覆盖电流、速度、前馈和位置工况 |
| 离线验证 | Clarke/Park、SVPWM、PI、轨迹、电压限制主机回归 |
| 研究路线 | 参数辨识 → 三环自整定 → MTPA/弱磁 → 无感控制 |

## 控制与数据架构

~~~mermaid
flowchart LR
    PC[Python 自动化与分析] -->|ASCII 命令| USB[USB CDC]
    USB --> APP[应用状态机]
    APP --> POS[位置环 400 Hz]
    POS --> SPD[速度环 4 kHz]
    SPD --> CUR[电流环 40 kHz]
    ADC[ADC 电流] --> CUR
    ENC[ABZ 编码器] --> POS
    ENC --> SPD
    ENC --> CUR
    CUR --> FF[dq 前馈]
    FF --> LIM[矢量限幅]
    LIM --> PWM[SVPWM]
    PWM --> INV[三相功率级]
    INV --> PMSM[PMSM]
    CUR --> SCOPE[D2 SRAM Scope]
    SCOPE --> USB
    SCOPE --> JLINK[J-Link 后备读取]
~~~

详细实现见 [系统架构](docs/ARCHITECTURE.md)。

## 当前验证结果

测试条件：ProDrone 3505 KV650、10 极对、24 V/4.5 A 限流电源、空载、无物理示波器。

| 项目 | 阶段 0 结果 |
|---|---|
| Keil 全量构建 | 0 error，0 warning |
| 主机算法回归 | PASS |
| id = +0.2 A，iq = 0 | 约 125 μs 上升时间，约 23.5% 滤波后超调 |
| id = -0.2 A，iq = 0 | 约 175 μs 上升时间，约 13.35% 滤波后超调 |
| 空载速度 | 500/1500/3000 rpm 均稳定，未观察到电压饱和 |
| dq 前馈 | 低中速差异较小，高速有轻微收益迹象 |
| 位置 | 首次 +2° 阶跃出现持续小幅摆动，后续点相对稳定，待复测 |
| Scope ISR 开销 | 最大约 124 cycles / 0.258 μs / FOC 周期 1.03% |

完整条件、证据和限制见 [阶段 0 验证报告](docs/VALIDATION_REPORT.md)，机器可读摘要见 [stage0_20260824.json](tests/baselines/stage0_20260824.json)。

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
ia, ib, iq, id, iq_ref, id_ref, uq, ud, elec_angle, elec_speed
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
- [阶段 0 验证报告](docs/VALIDATION_REPORT.md)
- [串口与 Scope 协议](docs/SERIAL_PROTOCOL.md)
- [长期研究路线](docs/ROADMAP.md)
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
MTPA / 弱磁 / 全速域无感
~~~

之后根据目标分支：

- 机器人控制：低速平稳、轨迹跟踪、摩擦/齿槽补偿、扰动观测；
- 新能源电驱：快速转矩控制、MTPA、弱磁、电压饱和和无感切换。

路线使用能力依赖和实验退出条件，不绑定僵硬工期。详情见 [ROADMAP](docs/ROADMAP.md)。

## 已知限制

- 目前只有空载电机，没有测功机或可控负载；
- 没有物理示波器，不能验证栅极、开关节点、振铃和实际死区；
- 硬件没有比较器，无法建立比较器到 TIM1 BRK 的保护链；
- 死区由栅极驱动处理，现阶段没有修改；
- 首次位置小阶跃问题尚待复核；
- 参数辨识、自整定、MTPA、弱磁和无感仍处于后续研究阶段；
- 部分历史源码为 CP936/GBK，当前没有执行全仓编码迁移。

## 安全说明

本仓库用于学习和控制算法研究。电机和逆变器可能造成触电、烫伤、机械飞出或器件损坏。请使用限流电源、保持电机固定和旋转空间清空，并在修改控制器、功率级或保护逻辑后重新进行分级验证。

软件 STOP、电流限幅和状态机不能替代独立硬件保护。

## 许可与第三方组件

仓库当前没有统一的项目级 LICENSE。STM32 HAL、CMSIS 和其他第三方组件保留各自的版权与许可文件。公开发布或复用前，应分别核对第三方代码、原理图和项目自有代码的授权范围。
