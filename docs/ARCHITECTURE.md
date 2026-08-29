# 系统与软件架构

本文描述当前固件的运行结构、控制周期、状态机和观测链路，更新到2026-08-28的阶段5架构整理状态。算法成熟度以 PROJECT_STATUS 和 VALIDATION_REPORT 为准，不因目录中存在实验实现就视为已经完成硬件验证。

## 总体结构

~~~mermaid
flowchart LR
    HOST[PC 自动化脚本] -->|ASCII 命令| CDC[USB CDC]
    CDC --> APP[应用命令与参数]
    APP --> SUP[电机 Supervisor]
    SUP --> POS[位置环 400 Hz]
    POS --> SPD[速度环 4 kHz]
    SPD --> FW[弱磁与5 A电流圆 4 kHz]
    FW --> CUR[电流环 40 kHz]

    ADC[双相电流 ADC] --> CLARKE[Clarke / Park]
    ENC[ABZ 编码器] --> ROTOR[转子来源选择]
    HFI[HFI 低速观测] --> ROTOR
    AF[Active Flux 中高速观测] --> ROTOR
    CLARKE --> CUR
    ROTOR --> CUR
    ROTOR --> SPD
    ROTOR --> POS

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

- ADC/FOC 40 kHz中断是唯一控制调度时基，外环在同一上下文内分频；
- USB 不在 FOC ISR 中发送数据；
- 状态切换由 ASCII 命令触发；
- 主机在一次上电内组织多个安全工况；
- 原始数据离线计算上升时间、超调、稳态误差和峰值。

## 实时调度

| 周期任务 | 频率 | 入口 | 主要内容 |
|---|---:|---|---|
| 电流采样与 FOC | 40 kHz | ADC injected conversion callback → FOC_RUN | 电流刷新、supervisor、电流环、坐标变换、SVPWM、保护、软件示波器采样 |
| 速度估计 | 40 kHz 调用 | FOC_RUN → FOC_SPEED_CALC | ABZ、绝对编码器及各观测器速度更新 |
| 速度环与弱磁 | 4 kHz | FOC_RUN → FOC_SPEED_CTRL，电机内部 10 分频 | 弱磁生成id与q轴余量，速度 PI 在动态限幅内输出 iq 参考 |
| 位置环 | 400 Hz | FOC_RUN → FOC_POS_CTRL，电机内部 100 分频 | 位置轨迹和位置 PID，输出速度参考 |
| 命令、遥测和 scope dump | 主循环任务 | Debug_Task | USB 服务、命令解析、2 ms 遥测、捕获完成后的分块发送 |

TIM1 使用中心对齐 PWM，CH4继续只负责触发注入ADC；不再用独立TIM1回调调度外环。应用根据定时器频率和周期计算FOC采样周期，当前实测软件示波器基准频率为40 kHz。各环的原分频计数和执行顺序保持不变，外环更新结果由下一次电流环采样使用。

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

运行状态不再由单个不断扩张的 `MOTOR_MODE_t` 同时承担。内部拆为四个正交维度：

| 维度 | 当前取值 | 所有权 |
|---|---|---|
| 生命周期 | IDLE / CURRENT_CALIBRATION / READY / STARTING / RUNNING / CALIBRATION_FAILED | 电机supervisor |
| 运行拓扑 | IDENTIFY_RL / IDENTIFY_FLUX / OPEN_VOLTAGE / CURRENT / SPEED / 三种POSITION级联 | 电机supervisor |
| 转子来源 | VF / ABZ / ABS / Active Flux / SMO / HFI / SENSORLESS启动链 | 转子估计组合层 |
| 诊断模式 | NONE / HFI_ENCODER_SHADOW / HFI_D_AXIS_SCAN / HFI_TENSOR_SCAN | 电机supervisor，HFI数学核只执行算法 |

~~~mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> CURRENT_CALIBRATION: request_prepare
    CURRENT_CALIBRATION --> READY: offset valid
    CURRENT_CALIBRATION --> CALIBRATION_FAILED: offset invalid
    READY --> STARTING: request_start(operation)
    STARTING --> RUNNING: source ready
    RUNNING --> READY: STOP / operation complete
    RUNNING --> IDLE: software overcurrent
    CALIBRATION_FAILED --> [*]: reset required
~~~

`STARTING`内部依次执行功率级预充、控制器清零、来源选择和必要的ABZ/ABS校准；`RUNNING`再按独立的operation选择电流、速度、位置或辨识控制。旧的 `MODE_*` 数值仍由 `mode_get()`映射输出，串口协议和既有主机脚本无需改写，但它们不再是内部状态所有者。

HFI影子、轴扫描和张量扫描属于diagnostic，不会伪装成新的电机运行拓扑。正式无感启动则由supervisor依次组织轴获取/IPD、估计d轴HFI、HFI角度启动、HFI速度闭环和Active Flux来源切换。主循环只提交请求，请求在下一次40 kHz FOC边界应用；HFI真正取得控制角后，必须由STOP统一退出，避免诊断命令越过supervisor直接改变功率控制权。

设计要点：

- FOC 对象全部构造完成后才启动 TIM1 和 ADC 触发；
- 电流偏置校准完成前 PWM 保持关闭；
- 用户发出 START 后才进入功率级预充和运行准备；
- STOP 返回 READY，关闭运行请求并清除主机目标；
- 电流偏置异常进入失败状态，保持功率输出关闭。
- supervisor只发布 `DISABLED/PRECHARGE/ENABLED` 功率级请求，具体TIM通道启停由应用/port边界执行；控制状态机不再直接包含STM32 PWM宏或ISR内文本输出。

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

最近一次D-075阶段5收尾构建结果：

- 0 error；
- 0 warning；
- Flash 122,956 B；
- RAM 286,640 B。

## 模块边界

| 目录 | 职责 |
|---|---|
| algorithm/foc/core | 数学、PI、滤波和基本类型 |
| algorithm/foc/control | 速度、位置和轨迹控制 |
| algorithm/foc/modulation | SPWM/SVPWM、电压约束 |
| algorithm/foc/sensor | ABZ/绝对编码器 |
| algorithm/foc/observer | VF、SMO、磁链、HFI |
| algorithm/foc/ident | 电阻、电感和磁链辨识基础 |
| algorithm/foc/foc_motor.* | 控制组合与supervisor；生命周期、运行拓扑、转子来源和诊断所有权 |
| app/foc_app.* | 电机参数、应用命令、ADC唯一调度入口和功率级请求落地 |
| app/debug | 命令、遥测和软件示波器 |
| port | STM32 外设适配和回调 |
| tests | 主机回归与硬件自动化 |

observer 和 ident 目录中存在实现基础，不代表所有算法已经完成当前电机上的系统验证。实际成熟度以 PROJECT_STATUS 和 VALIDATION_REPORT 为准。

当前仍保留 `foc_motor`构造阶段的显式组件装配，以及旧 `foc_pwm/foc_current/foc_time/encoder`类中的部分port调用。这些是已知的渐进式收口边界，不为追求形式上的“纯算法库”一次性重写已验证热路径。后续只在相关模块继续开发时，把测量输入、算法输出和硬件落地逐个收紧；不引入通用虚基类、服务定位器或多层事件框架。

位置—速度串级中只有位置轨迹器负责运动轨迹整形。`START POSITION`会把内速度轨迹器
切到NONE，防止继承速度模式的RAMP后与 `POS TRACK`重复限速；主机位置实验也显式发送
`TRACK STEP`。速度模式仍可独立使用 `TRACK RAMP`。

## 当前架构边界

- 阶段1已得到可复现的 Rs/Ld/Lq/磁链候选并用于控制设计与可切换前馈；参数仍是代码期配置，不包含持久化参数存储或板端一键自动写回。
- 只有空载实验，没有负载转矩或效率测量。
- 无物理示波器，不能验证门极、开关节点、振铃和实际死区。
- USB 虚拟串口曾受 Windows 驱动/资源问题影响，J-Link SRAM 读取作为后备。
- 位置小阶跃稳态保持已通过六区域重复性验证；剩余方向/区域相关端点过冲作为当前空载边界记录，位置参数搜索已停止。
- 弱磁已在空载人工电压边界下完成；MTPA延期。
- 阶段5空载无感主链已闭合：控制调度统一到ADC时基，HFI诊断与运行拓扑分开；最小IPD、估计d轴HFI、专用低速速度PI及HFI/Active Flux双向切换已完成板端连续矩阵。低于250 rpm的精密非零速度伺服、全位置IPD鲁棒性和带载/温漂仍属于路线图后续项。
