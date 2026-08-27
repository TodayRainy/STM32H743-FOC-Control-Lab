# 系统与软件架构

本文描述当前固件的运行结构、控制周期、状态机和观测链路，更新到2026-08-28的阶段4收尾状态。算法成熟度以 PROJECT_STATUS 和 VALIDATION_REPORT 为准，不因目录中存在实验实现就视为已经完成硬件验证。

## 总体结构

~~~mermaid
flowchart LR
    HOST[PC 自动化脚本] -->|ASCII 命令| CDC[USB CDC]
    CDC --> APP[应用命令与状态机]
    APP --> POS[位置环 400 Hz]
    POS --> SPD[速度环 4 kHz]
    SPD --> FW[弱磁与5 A电流圆 4 kHz]
    FW --> CUR[电流环 40 kHz]

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
| 速度环与弱磁 | 4 kHz | FOC_SPEED_CTRL，电机内部 10 分频 | 弱磁生成id与q轴余量，速度 PI 在动态限幅内输出 iq 参考 |
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

SVPWM 线性区电压限制使用母线电压除以根号 3，SPWM 使用母线电压的一半。D-058增加的测试比例只缩放这一矢量上限，不篡改送入调制器的实测母线电压；复位比例为1。电压限制保持 q/d 矢量方向，并通过 `voltage_scale` 暴露实际/请求矢量的比例。

当前前馈模型：

~~~text
uq_ff = omega_e * (flux + Ld * id)
ud_ff = -omega_e * Lq * iq
~~~

它已拆为OFF/CROSS/BEMF/BOTH四种运行时模式，`FF ON/OFF`保留为BOTH/OFF兼容别名；LEGACY/IDENTIFIED继续只选择参数模型。前馈使用实测Iqd与滤波后的实测电角速度。D-057已确认BEMF主导反馈PI减负，CROSS在局部速度阶跃中重复改善dq跟踪；D-058确认OFF/BOTH均能从受控电压限幅恢复，D-059将复位默认更新为IDENTIFIED+BOTH。

### 速度环与弱磁

速度环输出 iq 参考；弱磁OFF时id参考为零，FEEDBACK/HYBRID时由4 kHz弱磁器生成负id参考。支持：

- 运行时修改 PI；
- 速度阶跃；
- 固定加速度线性轨迹；
- 正反向参考；
- 参考值和实际速度遥测。

当前硬件验证已覆盖正反4000 rpm空载斜坡；D-043默认速度PI的实测闭环带宽约45–55 Hz。4 kHz是速度控制器执行频率，不是速度闭环带宽。

D-064弱磁链路使用限幅前请求电压的200 Hz低通值跟踪95%实际可用dq电压。FEEDBACK由
`Kp=0.4 A/V`、`Ki=2400 A/(V*s)`电压PI生成全部负d轴电流；HYBRID先由阶段1的
IDENTIFIED稳态dq电压椭圆给出前馈工作点，再用同一PI修正残差。最终id参考限制为
[-5,0] A且变化率500 A/s。总参考满足5 A圆：弱磁先确定id，随后在速度PI运行前把q轴
输出限幅设为基础`SPEED LIMIT`与`sqrt(25-id^2)`的较小值，使速度PI自身的钳位与
anti-windup跟踪真实约束。复位默认为HYBRID；OFF和FEEDBACK保留为对照模式。

D-065允许主机启动的速度模式在运行中更新测试用`voltage_limit_ratio`。它是主循环写、FOC
ISR读的单个32位`volatile float`；弱磁目标、dq矢量限幅和PI tracking继续读取同一边界，
没有新增控制分支。该入口用于动态软件电压边界实验，不代表实现了母线扰动模型。

### 位置环

位置环采用位置—速度—电流串级结构，支持：

- 当前位置作为主机命令原点；
- 位置阶跃；
- 固定角速度线性轨迹；
- 同时限制速度和加速度的梯形/短行程三角形轨迹；
- 可选的轨迹速度前馈；
- 最短路径角度差；
- 目标、轨迹参考和反馈同时遥测。

位置—速度串级只允许外位置轨迹器负责运动轨迹整形；`START POSITION`会清除内速度轨迹。复位默认为 `Kp=376.991118, Ki=Kd=0`、`RAMP 180 deg/s`、位置速度前馈关闭；`TRAP 180/900 + 前馈开启`保留为可选快速轨迹。D-053/D-054确认稳态保持可靠，约2.2°机械区域相关端点过冲是当前空载边界，而非持续失控。

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

- 12 个 float32 通道；
- 4096 点；
- 数据体 192 KiB；
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

这种结构避免高频 ISR 被 USB 阻塞。D-058的12通道顺序在末尾追加 `voltage_scale`；链接后整个 `RW_SCOPE=196,672 B`，在256 KiB区域内剩余65,472 B。10通道硬件基线的最大采样开销约124 cycles，在480 MHz下约0.258 μs，占25 μs FOC周期约1.03%；12通道运行开销尚未专项复测。

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

最近一次D-065阶段4最终构建结果：

- 0 error；
- 0 warning；
- Flash 111,504 B；
- RAM 237,476 B。

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

位置—速度串级中只有位置轨迹器负责运动轨迹整形。`START POSITION`会把内速度轨迹器
切到NONE，防止继承速度模式的RAMP后与 `POS TRACK`重复限速；主机位置实验也显式发送
`TRACK STEP`。速度模式仍可独立使用 `TRACK RAMP`。

## 当前架构边界

- 阶段1已得到可复现的 Rs/Ld/Lq/磁链候选并用于控制设计与可切换前馈；参数仍是代码期配置，不包含持久化参数存储或板端一键自动写回。
- 只有空载实验，没有负载转矩或效率测量。
- 无物理示波器，不能验证门极、开关节点、振铃和实际死区。
- USB 虚拟串口曾受 Windows 驱动/资源问题影响，J-Link SRAM 读取作为后备。
- 位置小阶跃稳态保持已通过六区域重复性验证；剩余方向/区域相关端点过冲作为当前空载边界记录，位置参数搜索已停止。
- 弱磁已在空载人工电压边界下完成；MTPA延期，全速域无感仍属于路线图目标。
