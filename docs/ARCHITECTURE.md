# 系统与软件架构

本文描述当前阶段 0 固件的运行结构、控制周期、状态机和观测链路。它反映 2026-08-24 的实现，不把尚未验证的辨识和无感模块写成已完成功能。

## 总体结构

~~~mermaid
flowchart LR
    HOST[PC 自动化脚本] -->|ASCII 命令| CDC[USB CDC]
    CDC --> APP[应用命令与状态机]
    APP --> POS[位置环 400 Hz]
    POS --> SPD[速度环 4 kHz]
    SPD --> CUR[电流环 40 kHz]

    ADC[双相电流 ADC] --> CLARKE[Clarke / Park]
    ENC[ABZ 编码器] --> ANGLE[角度与速度]
    CLARKE --> CUR
    ANGLE --> CUR
    ANGLE --> SPD
    ANGLE --> POS

    CUR --> FF[dq 解耦与反电动势前馈]
    FF --> LIMIT[电压矢量限制与 PI tracking]
    LIMIT --> SVPWM[SVPWM]
    SVPWM --> STAGE[三相功率级]
    STAGE --> MOTOR[PMSM]

    CUR --> SCOPE[D2 SRAM 软件示波器]
    SCOPE -->|捕获完成后导出| CDC
    SCOPE -->|后备读取| JLINK[J-Link]
~~~

核心思想是把实时控制、状态管理和低速通信分开：

- 40 kHz 中断只完成控制计算和内存采样；
- USB 不在 FOC ISR 中发送数据；
- 状态切换由 ASCII 命令触发；
- 主机在一次上电内组织多个安全工况；
- 原始数据离线计算上升时间、超调、稳态误差和峰值。

## 实时调度

| 周期任务 | 频率 | 入口 | 主要内容 |
|---|---:|---|---|
| 电流采样与 FOC | 40 kHz | ADC injected conversion callback → FOC_RUN | 电流刷新、状态机、电流环、坐标变换、SVPWM、保护、软件示波器采样 |
| 速度估计 | 40 kHz 调用 | TIM1 OC callback → FOC_SPEED_CALC | ABZ、绝对编码器及各观测器速度更新 |
| 速度环 | 4 kHz | FOC_SPEED_CTRL，电机内部 10 分频 | 速度轨迹和速度 PI，输出 iq 参考 |
| 位置环 | 400 Hz | FOC_POS_CTRL，电机内部 100 分频 | 位置轨迹和位置 PID，输出速度参考 |
| 命令、遥测和 scope dump | 主循环任务 | Debug_Task | USB 服务、命令解析、2 ms 遥测、捕获完成后的分块发送 |

TIM1 使用中心对齐 PWM。应用根据定时器频率和周期计算 FOC 采样周期，当前实测软件示波器基准频率为 40 kHz。

## 控制链路

### 电流环

ADC 采集两相电流，第三相由电流和重构。控制链路为：

1. Clarke 变换；
2. Park 变换；
3. q/d 电流 PI；
4. 可选 dq 交叉耦合和反电动势前馈；
5. 电压矢量幅值限制；
6. PI tracking 跟踪实际施加电压；
7. 反 Park；
8. SVPWM 和三相比较值。

SVPWM 线性区电压限制使用母线电压除以根号 3，SPWM 使用母线电压的一半。电压限制保持 q/d 矢量方向，并通过 voltage_scale 暴露饱和程度。

当前前馈模型：

~~~text
uq_ff = omega_e * (flux + Ld * id)
ud_ff = -omega_e * Lq * iq
~~~

它已经进入运行时开关和自动对比链路，但参数依赖仍需要后续辨识验证。

### 速度环

速度环输出 iq 参考，id 参考保持接近零。支持：

- 运行时修改 PI；
- 速度阶跃；
- 固定加速度线性轨迹；
- 正反向参考；
- 参考值和实际速度遥测。

当前阶段自动测试覆盖 500、1500、3000 rpm 空载速度点。

### 位置环

位置环采用位置—速度—电流串级结构，支持：

- 当前位置作为主机命令原点；
- 位置阶跃；
- 固定角速度线性轨迹；
- 最短路径角度差；
- 目标、轨迹参考和反馈同时遥测。

第一次 +2° 小阶跃曾出现持续摆动，这是当前需要复核的基线问题，不在架构文档中掩盖。

## 状态机

~~~mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> CURRENT_CALIB: request_prepare
    CURRENT_CALIB --> READY: offset valid
    CURRENT_CALIB --> CURRENT_CALIB_FAILED: offset invalid
    READY --> POWER_STAGE_CHARGE: START CURRENT/SPEED/POSITION
    POWER_STAGE_CHARGE --> CLEAR
    CLEAR --> START
    START --> ABZENC_CALIB: first run
    START --> RUN_MODE: encoder already calibrated
    ABZENC_CALIB --> RUN_MODE: calibration complete
    RUN_MODE --> READY: STOP
    CURRENT_CALIB_FAILED --> [*]: reset required
~~~

RUN_MODE 可以是：

- MODE_TORQUE_RUN；
- MODE_SPEED_TORQUE_RUN；
- MODE_POSITION_SPEED_TORQUE_RUN。

设计要点：

- FOC 对象全部构造完成后才启动 TIM1 和 ADC 触发；
- 电流偏置校准完成前 PWM 保持关闭；
- 用户发出 START 后才进入功率级预充和运行准备；
- STOP 返回 READY，关闭运行请求并清除主机目标；
- 电流偏置异常进入失败状态，保持功率输出关闭。

硬件没有比较器，因此不存在比较器到定时器 BRK 的硬件链路。当前保护主要是软件电流检查、状态机和电源限流，不能等同于汽车级功能安全。

## 软件示波器

软件示波器采用固定内存缓冲：

- 10 个 float32 通道；
- 4096 点；
- 约 160 KiB；
- D2 SRAM 0x30000000 区域；
- MPU 配置为 non-cacheable；
- 链接段为 SCOPE_DATA；
- J-Link 可以直接读取。

捕获流程：

1. 主机发送 SCOPE ARM 和分频；
2. 40 kHz ISR 按分频向 SRAM 写入；
3. 4096 点后状态切换为 DONE；
4. 主机发送 SCOPE DUMP；
5. 主循环先发送头部，再以小块发送数据；
6. 主机保存 CSV。

这种结构避免高频 ISR 被 USB 阻塞。当前最大采样开销约 124 cycles，在 480 MHz 下约 0.258 μs，占 25 μs FOC 周期约 1.03%。

## USB 与遥测

USB CDC 发送使用固定深度队列，主循环负责启动发送，发送完成回调释放槽位。运行统计包括：

- 队列深度和峰值；
- 入队、发送和丢弃帧；
- BUSY 重试；
- 未配置状态；
- 收发字节数；
- 底层错误。

普通遥测为 36 个 float32 加 JustFloat 尾部，默认每 2 ms 一帧。软件示波器 dump 期间暂停普通遥测，避免两种二进制流交错。

命令和数据格式详见 [串口与软件示波器协议](SERIAL_PROTOCOL.md)。

## 内存与链接

主要区域：

| 区域 | 用途 |
|---|---|
| Flash 0x08000000 | 程序和只读数据 |
| AXI SRAM 0x24000000 | 常规 RW/ZI |
| D2 SRAM 0x30000000 | 软件示波器缓冲，non-cacheable |
| RAMCODE | FOC 热路径和软件示波器采样代码 |

2026-08-24 全量构建结果：

- 0 error；
- 0 warning；
- Flash 99,288 B；
- RAM 204,656 B。

## 模块边界

| 目录 | 职责 |
|---|---|
| algorithm/foc/core | 数学、PI、滤波和基本类型 |
| algorithm/foc/control | 速度、位置和轨迹控制 |
| algorithm/foc/modulation | SPWM/SVPWM、电压约束 |
| algorithm/foc/sensor | ABZ/绝对编码器 |
| algorithm/foc/observer | VF、SMO、磁链、HFI |
| algorithm/foc/ident | 电阻、电感和磁链辨识基础 |
| algorithm/foc/foc_motor.* | 控制组合、状态机和调度 |
| app/foc_app.* | 电机参数、应用命令和 ISR 入口 |
| app/debug | 命令、遥测和软件示波器 |
| port | STM32 外设适配和回调 |
| tests | 主机回归与硬件自动化 |

observer 和 ident 目录中存在实现基础，不代表所有算法已经完成当前电机上的系统验证。实际成熟度以 PROJECT_STATUS 和 VALIDATION_REPORT 为准。

## 当前架构边界

- 电机参数仍是代码期配置，尚未形成可靠的自动辨识到运行参数闭环。
- 只有空载实验，没有负载转矩或效率测量。
- 无物理示波器，不能验证门极、开关节点、振铃和实际死区。
- USB 虚拟串口曾受 Windows 驱动/资源问题影响，J-Link SRAM 读取作为后备。
- 位置首次小阶跃问题尚待重复性测试。
- MTPA、弱磁和全速域无感仍属于路线图目标。
