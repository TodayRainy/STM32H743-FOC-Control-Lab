# 控制状态、故障诊断与主机工具工程化方案

状态：D-076冻结方案；D-077软件实现完成；D-078完成首批板端控制/无感回归和WARNING生命周期；D-079～D-081完成结构化命令、无副作用拒绝、真实GUI及统一控制域验收，受控欠压TRIP因电源不可调而延期

## 目标与边界

本阶段不继续扩展无感、三环、前馈或弱磁算法，而是把已经验证的控制能力组织成一个状态清楚、故障可解释、主机可操作的完整电控工程。

本阶段完成：

- 调整第一方FOC目录，使传感器控制、无感控制、控制器和辨识边界直观；
- 第一方非UTF-8源码迁移为UTF-8无BOM，并提供可重复检查；
- 保留生命周期、运行拓扑、转子来源和诊断模式四个正交维度；
- 增加统一故障状态、活动/锁存故障位、首次故障和定长快照；
- 将已有校准失败、软件过流和无感失败接入同一停机路径；
- 增加母线电压、超速和控制周期等现有测量能够支持的诊断；
- 扩展兼容的ASCII状态/故障命令，不改变现有36通道遥测和15通道Scope布局；
- 抽取共享Python设备客户端，并实现最小桌面上位机；
- 以主机测试、C++算法回归、静态检查和Keil构建作为本阶段证据。

本阶段不做：

- 不烧录、不连接目标、不运行电机；
- 不加入通信看门狗、自动故障重启或预测性健康评分；
- 不实现温度和母线电流故障，因为当前`adc_dma`对应换算仍为空；
- 不用重构得到的`ic=-ia-ib`宣称三相电流不平衡或缺相诊断；
- 不声称软件停机替代硬件比较器、定时器BRK或栅极驱动保护；
- 不引入RTOS、通用事件总线、服务定位器、插件框架或持久化参数系统；
- 不重新整定已经冻结的控制参数。

## 目录结构

当前`observer/hfi`只对单个算法增加一层目录，而VF、SMO、Active Flux、I/f启动和全速域接管仍在上一层，不能表达完整无感控制域。目标结构为：

~~~text
algorithm/foc/
  core/          数学、基本类型、PI和滤波
  control/       速度、位置和轨迹控制
  modulation/    PWM与电压约束
  sensor/        ABZ和绝对编码器
  sensorless/    VF、SMO、Active Flux、HFI、启动与来源切换
  ident/         电气参数辨识
  foc_motor.*    控制组合、生命周期、控制权和故障停机
~~~

`sensorless`按功能域平铺现有文件，不为每个观测算法继续增加子目录。HFI数学核、HFI速度启动辅助和HFI/Active Flux接管仍是三个独立类；它们分别负责估计、低速速度环进入和来源管理，避免重新合并成大状态机。

## 编码规则

- 第一方C/C++、Python、PowerShell、Markdown、JSON/YAML和Keil工程文本统一为UTF-8无BOM；
- 保持文件原有行尾，编码迁移不顺带全仓格式化；
- `third_lib`、构建产物、实验结果和二进制文件不参与迁移；
- Cube生成文件若已经是有效UTF-8则不重写；
- `.editorconfig`声明UTF-8，主机测试检查受管目录不存在非UTF-8文本；
- 转码前后比较非注释代码和构建结果，串口协议依赖的ASCII令牌保持逐字节不变。

中文注释保留UTF-8；ARMCC5会对UTF-8中文运行时字符串产生870-D警告，因此现有8处中文诊断文本改为ASCII，控制协议本身始终只使用ASCII。Keil/ARMCC5构建是迁移退出条件。

## 运行状态模型

四个既有正交维度继续保留：

| 维度 | 作用 |
|---|---|
| lifecycle | 功率和运行许可：IDLE、CURRENT_CALIBRATION、READY、STARTING、RUNNING、FAULT_ACTIVE、FAULT_LATCHED |
| operation | 当前控制拓扑：CURRENT、SPEED、POSITION、IDENTIFY等 |
| rotor source | VF、ABZ、ABS、SMO、Active Flux、HFI或组合无感链 |
| diagnostic | HFI影子、扫描等不拥有功率控制权的诊断 |

不把每种`operation × rotor source × fault`组合扩成一个总枚举。旧`MOTOR_MODE_t`数值继续作为串口兼容视图；内部状态才是控制权事实来源。

状态转换原则：

1. 主循环只提交请求；控制状态在40 kHz边界应用；
2. `START`只从READY接受；`STOP`只在READY/STARTING/RUNNING接受，不能把IDLE或电流偏置校准推进到READY；
3. 任意带功率状态均可由统一故障入口进入FAULT_ACTIVE；
4. 故障条件消失后进入FAULT_LATCHED，PWM仍关闭；
5. 只有显式`FAULT CLEAR`且活动故障为零时返回READY；
6. `STOP`不清除故障，故障状态也不自动重启。

电流偏置是唯一恢复特例：`CURRENT_OFFSET`确认后返回`CURRENT_CALIBRATION`重新取样，通过后才进入READY。

## 故障数据模型

采用一个定长、无动态分配的故障记录：

~~~text
active_bits       当前仍成立的故障
latched_bits      本轮发生过的故障
first_fault       首个导致停机的故障码
timestamp_ms      首个故障时间
lifecycle         故障前生命周期
operation         故障前运行拓扑
rotor_source      故障前转子来源
iq/id             故障采样的dq电流
speed/target      实际和目标电角速度
vbus              母线电压
voltage_scale     电压限幅比例
~~~

高频控制路径只更新位图、快照、状态和功率级请求，不格式化文本。主循环读取稳定快照并输出协议响应。

故障分为两类：

- `TRIP`：立即关闭PWM并锁存，需要人工确认；
- `WARNING`：只记录活动状态和计数，不改变功率控制权。

第一版不增加可配置的通用严重度表；每个故障的级别在检测代码附近固定，保持代码直接可读。

## 第一批故障

| 故障 | 级别 | 数据来源 | 软件阶段处理 |
|---|---|---|---|
| CURRENT_OFFSET | TRIP | 已有两相ADC偏置校准 | 取代独立CALIBRATION_FAILED终态 |
| SOFTWARE_OVERCURRENT | TRIP | dq电流幅值与已有持续计数 | 接入统一快照和锁存，不先改变已验证阈值 |
| BUS_UNDERVOLTAGE | TRIP/启动禁止 | ADC3母线电压 | 单独的进入/退出阈值与时间资格 |
| BUS_OVERVOLTAGE | TRIP | ADC3母线电压 | 单独的进入/退出阈值与时间资格 |
| OVERSPEED | TRIP | 当前控制转子来源速度 | 按绝对速度和持续时间判断 |
| SENSORLESS_STARTUP | TRIP | I/f启动状态 | 不再静默切回ABZ并READY |
| ROTOR_SOURCE_LOST | TRIP | HFI/Active Flux接管状态 | 保留内部失败原因并统一停机 |
| CONTROL_DEADLINE | TRIP | 现有FOC执行时间 | 连续超出25 us才锁存，阈值在软件测试中验证 |
| SPEED_TRACKING | WARNING | 速度参考与反馈 | 仅在RUNNING/SPEED且轨迹稳定后资格化 |
| POSITION_TRACKING | WARNING | 位置参考与反馈 | 默认只报告，不停机 |

母线、超速和执行周期的新阈值在代码中集中为应用配置。软件测试覆盖边界、资格时间、回差和状态限制；由于本阶段没有硬件运行，文档必须保留“默认值尚未板端故障注入验证”。

D-077应用默认值为母线18/19 V欠压进入/退出、28/27 V过压进入/退出、20 ms进入和100 ms恢复；超速为命令上限1.05倍进入、1.0倍退出；控制执行时间25/23 µs、1/5 ms资格；速度/位置跟踪WARNING为500/250 rpm和20/10°回差、0.5/0.2 s资格。这些数值只用于首次板端矩阵的可检验起点，不是硬件额定保护值。

## 协议

现有命令和36通道遥测保持兼容。新增低速ASCII控制面：

~~~text
STATUS
FAULT
FAULT CLEAR
~~~

建议响应：

~~~text
STATUS,Lifecycle=...,Operation=...,Source=...,Diagnostic=...,Mode=...,FaultActive=0x...,FaultLatched=0x...
FAULT,Active=0x...,Latched=0x...,First=...,TimeMs=...,Iq=...,Id=...,Speed=...,Target=...,Vbus=...,Scale=...
FAULT_CLEAR,OK
FAULT_CLEAR,REJECT,Active=0x...
~~~

本阶段不改变普通遥测通道顺序，避免破坏历史CSV和脚本。上位机用现有高频遥测绘图，用`STATUS/FAULT`获得结构化控制状态。

## 主机软件

先建立共享客户端，再构建GUI：

~~~text
host/
  foc_client/
    protocol.py   ASCII响应、36通道遥测和Scope头解析
    device.py     串口生命周期、命令、状态和故障接口
    scope.py      15通道Scope下载与保存
    session.py    固件/协议/测试元数据
  gui/
    main.py       PySide6入口
    main_window.py
~~~

GUI第一版包含连接、状态、START/STOP、目标、实时遥测、Scope和故障页。GUI依赖作为可选主机依赖；协议与设备客户端不依赖Qt，可由现有自动化脚本和pytest直接复用。

## 实施顺序

1. 目录与构建路径迁移；
2. 第一方编码迁移和UTF-8检查；
3. 独立纯算法故障管理类及C++回归；
4. `foc_motor`生命周期和现有故障接入；
5. 新故障检测与边界测试；
6. `STATUS/FAULT/FAULT CLEAR`应用和调试命令；
7. Python协议/设备客户端；
8. 最小GUI和无设备解析测试；
9. 全套pytest、C++回归、编码检查和Keil构建；
10. 更新PROJECT_STATUS、ARCHITECTURE、SERIAL_PROTOCOL、ROADMAP、DECISIONS和测试说明。

## 软件阶段退出条件

- 现有控制算法文件的目录迁移不改变文件内容；
- 所有第一方受管文本都是UTF-8；
- 旧命令、36通道遥测和15通道Scope协议测试继续通过；
- 每类故障至少覆盖触发、保持、条件消失、拒绝清除、成功清除和首次快照不覆盖；
- 故障触发后功率级请求为DISABLED，READY/STARTING/RUNNING中的STOP返回READY且不绕过偏置校准；
- 主机客户端可在伪串口/记录数据上完成状态、故障、遥测和Scope解析；
- GUI模块可导入并完成无设备启动冒烟；
- Python、C++算法回归和Keil均为0失败，Keil为0 error/0 warning；
- 文档明确软件验证与尚未完成的板端阈值/故障注入边界。

## 参考结构

- ST MCSDK：集中状态机，`FAULT_NOW → FAULT_OVER → acknowledge`，区分当前和已发生故障：<https://wiki.st.com/stm32mcu/wiki/STM32MotorControl:6-step_Firmware_Algorithm>
- ODrive：`requested_state/current_state`、`active_errors/disarm_reason`：<https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html>
- VESC：统一故障枚举和`mc_interface_fault_stop`入口：<https://github.com/vedderb/bldc/blob/master/datatypes.h>、<https://github.com/vedderb/bldc/blob/master/motor/mc_interface.h>
- moteus：模式/故障寄存器以及可选速度、位置跟踪误差故障：<https://github.com/mjbots/moteus/blob/main/docs/reference/configuration.md>

借鉴这些项目的状态所有权、故障语义和主机可观测性，不复制其硬件相关保护、参数规模或框架复杂度。

## D-077软件验证结果

- 第一方受管文本通过UTF-8无BOM检查，Keil工程文件和所有include/file路径通过独立pytest检查；
- 故障管理C++回归覆盖首次快照不覆盖、活动TRIP拒绝确认、WARNING不阻塞确认、资格时间和回差；
- PREPARE、START和STOP使用单槽请求在40 kHz边界生效，控制周期监测覆盖完整ADC/FOC回调并在下一采样评估；
- Python混合流回归覆盖36通道遥测、状态、故障事件、15通道Scope分片和损坏帧恢复；
- `host/gui`在隔离`.venv-host`中以Qt offscreen模式完成无设备窗口创建/关闭；
- 全量118项Python、C++算法回归和Keil 0 error/0 warning（Flash 127,724 B、RAM 286,644 B）结果见`PROJECT_STATUS.md`与`VALIDATION_REPORT.md`；未执行J-Link、串口或电机操作。

## D-078板端回归结果

首次查询发现命令已经由USB接收并清除，但主机收不到`STATUS/FAULT`。板端统计和源码交叉检查定位出两个直接原因：新控制面误用重定向到USART1的标准`printf`；500 Hz、148 B遥测在主机打开前会占满8槽USB发送队列，文本响应即使改走USB也可能在入队时被拒绝。修正保持克制：状态、故障和事件文本改用既有`usb_printf`，`STATUS/FAULT/FAULT CLEAR`只在TX队列存在一个空槽时消费；没有降低遥测频率、队列容量或增加通用消息总线。

板端连续回归完成：

- 电流`id=0/±0.15/0 A`、编码器速度±500 rpm和位置±5°基础矩阵完成，最终模式17；
- HFI/Active Flux短矩阵覆盖`+350→+600→+500→0→-350→-600→-500→0 rpm`，8/8保持点和4/4来源切换通过，峰值dq 1.027 A、最小电压缩放1；
- 以0.02 A速度限流和1000 rpm不可达命令触发`SPEED_TRACKING` WARNING，实测`active=0x100 → active=0/latched=0x100 → FAULT CLEAR → 0/0`，生命周期从RUNNING经STOP回READY；
- 基础三模式与无感矩阵中没有出现`CONTROL_DEADLINE`误触发；最终`READY/NONE/ABZ`，活动和锁存故障均为0。

最终118项Python测试和C++算法回归PASS；Keil/ARMCC 5.06u7为0 error/0 warning，Flash 127,812 B、RAM 286,644 B。HEX SHA-256为`51F654EC51853FF1630F55280E2B51EFD3B137B6C2BBC9FD6FFE3E054C80C865`；固定J-Link V7.54d原始Program & Verify O.K.，向量表读回`20008000 08000449 0800E523 0800DABD`。机器摘要见`tests/baselines/stage6_board_control_fault_d078_20260830.json`。

没有为覆盖率故意制造软件过流、过压或超速。TRIP的实际板端`FAULT_ACTIVE → FAULT_LATCHED → CLEAR`和PWM停机仍需一次受控欠压验证；这不影响本轮控制算法与WARNING结论，也不能扩写成功能安全证据。

## D-079/D-080控制面收尾

D-079只补上位机真正需要的结构化命令结果，没有增加事务队列、事件总线或自动重试层。GUI现在
以板端`STATUS`为唯一运行状态来源：READY时允许选择来源和配置，START后等待RUNNING再下发目标，
RUNNING时只开放对应目标和STOP。真实GUI完成±300 rpm、运行中配置拒绝和15通道Scope下载，最终
READY且故障位为0。

D-080随后用同一固件完成ABZ速度/位置、弱磁动态边界和两条无感主路径的连续矩阵。各脚本最终
STOP/READY，未出现控制周期或来源丢失误报。受控欠压TRIP因电源当前不可调而顺延；软件故障算法、
WARNING生命周期和无故障控制面已经验证，但实际TRIP的功率级停机与确认链仍不能宣称完成。
