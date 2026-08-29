# 串口、遥测与软件示波器协议

本文记录当前主机研究接口。协议尚未承诺长期二进制兼容；修改固件通道布局时必须同步更新 tests 中的主机脚本。

## 传输通道

- 设备：USB CDC 虚拟串口；
- 主机配置：115200、8N1；
- 命令：ASCII，以 CRLF 结束；
- 普通遥测：36 个 little-endian float32，加 4 字节 JustFloat 尾；
- 软件示波器：固定头部加连续 float32 样本；
- 后备通道：J-Link 根据 MAP 符号读取 D2 SRAM。

USB CDC 本身不依赖真实 UART 波特率，但主机统一使用 115200 便于工具配置。

COM 号属于主机当前枚举状态。没有重新插拔且通信正常时不重复扫描；重新枚举或打开失败后再发现端口。

## 命令格式

所有命令使用：

~~~text
COMMAND [arguments]\r\n
~~~

数值使用十进制 ASCII。当前解析器不支持科学计数法之外的复杂表达式，也不回显命令结果；主机应通过 mode 和参考值遥测确认命令是否生效。

## 运行控制命令

| 命令 | 作用 | 前置状态 |
|---|---|---|
| START CURRENT | 进入 q/d 电流模式 | READY |
| START SPEED | 进入速度—电流串级模式 | READY |
| START POSITION | 进入位置—速度—电流串级模式 | READY |
| START SENSORLESS rpm | 以电流控制I/f建立极性并接管到Active Flux | READY |
| START HFI | 自动执行轴获取/IPD并进入HFI—Active Flux无感速度链 | READY |
| START IDENT RL | 启动 Rs、Ld/Lq 辨识 | READY |
| START IDENT FLUX | 启动正反转多速度磁链辨识 | READY |
| IDENT RESULT | 输出最近一次辨识结果文本行 | READY |
| IDENT POINTS | 输出 16 个 Rs 辨识实际 I/U 点 | READY |
| STOP | 停止运行并返回 READY | 非校准失败状态 |

START 过程会经过功率级预充、CLEAR、START，并在首次运行时完成 ABZ 编码器校准。命令发出后不能仅按固定延时假定成功，应等待对应 mode。

`START SENSORLESS rpm`是D-068/D-069保留的中高速独立I/f启动入口，参数不得为0且范围为
±6500 rpm。`START HFI`是D-075正式全链入口：它不使用ABZ校准，自动完成凸极轴获取、
±1 A IPD和估计d轴HFI跟踪；随后用普通`SPEED rpm`给出速度目标。低于250 rpm的非零目标
使用0.5 A限流的HFI角度比例电流辅助，达到250 rpm并资格保持后切入HFI专用速度PI；
550 rpm向Active Flux切换、525 rpm反向切回。两条命令均可能立即使电机运动。

`START IDENT RL` 会执行两组相反顺序的 ±1/±2/±3 A 静止扫描、±1.5/±2.5 A 留出点，并以 500 Hz、0.25/0.35 V 两个幅值注入；`START IDENT FLUX` 会在约 ±955、±1910、±2865 rpm 旋转，其中 ±1910 rpm 不参与拟合。辨识模式设置 5 A 独立中止阈值，磁链辨识还会在实测速度超过最高目标的 1.25 倍时中止。两条命令都属于硬件运动/功率命令，执行前必须确认 24 V、电源限流和电机周边安全。辨识完成后固件关闭 PWM 并返回 READY，再查询结果。

结果行格式：

~~~text
IDENT_RL,Rs=...,Voff=...,RsRmse=...,Ld=...,Lq=...,Ls=...,Rd=...,Rq=...,IdRms=...,IqRms=...
IDENT_RS_DIAG,RsF=...,RsR=...,RsDelta=...,VoffF=...,VoffR=...,VoffDelta=...,RsHoldoutRmse=...
IDENT_LCHECK,LdHigh=...,LqHigh=...,LdDelta=...,LqDelta=...,IdHighRms=...,IqHighRms=...
IDENT_FLUX,Flux=...,FluxIntercept=...,FluxRmse=...,FluxHoldoutRmse=...,FluxHoldoutMax=...
~~~

`IDENT POINTS` 返回四行，每行四个 `序号=实际电流:实际电压` 项：

~~~text
IDENT_RS_POINTS,0=...:...,1=...:...,2=...:...,3=...:...
~~~

电阻单位为 Ω，电感单位为 H，磁链单位为 Wb，其余电压/电流字段使用 V/A。`RsDelta/LdDelta/LqDelta` 是相对变化率。负的 RMSE 或变化率表示该项尚未成功完成。`Rd/Rq` 是 500 Hz 复阻抗实部，只用于质量检查。

重要模式码：

| 数值 | 模式 |
|---:|---|
| 0 | IDLE |
| 1 | POWER_STAGE_CHARGE |
| 2 | CURRENT_CALIB |
| 5 | START |
| 6 | ABZENC_CALIB |
| 11 | TORQUE_RUN |
| 12 | SPEED_TORQUE_RUN |
| 15 | POSITION_SPEED_TORQUE_RUN |
| 16 | CURRENT_CALIB_FAILED |
| 17 | READY |

模式码来自 MOTOR_MODE_t 的当前顺序；协议版本化前若修改枚举顺序，主机脚本必须同步更新。

## 参考值命令

| 命令 | 参数 | 单位 |
|---|---|---|
| CURRENT iq id | q/d 电流参考 | A |
| CURRENT SINE D amplitude frequency | d 轴同步正弦电流参考 | A, Hz |
| CURRENT SINE Q amplitude frequency | q 轴零均值同步正弦电流参考 | A, Hz |
| CURRENT SINE amplitude frequency | 兼容旧脚本，等价于 `CURRENT SINE D` | A, Hz |
| CURRENT SINE OFF | 停止当前正弦并清零 d/q 电流参考 | - |
| SPEED rpm | 机械速度参考 | rpm |
| SPEED SINE amplitude frequency | 零均值同步机械速度参考 | rpm, Hz |
| SPEED SINE OFF | 停止速度正弦并把速度参考清零 | - |
| SPEED LIMIT iq_limit | READY 下设置速度 PI 的 q 轴电流输出限幅 | A |
| SPEED ID id | 速度模式下的 d 轴实验电流 | A，限于 ±1.5 A |
| FW MODE OFF | READY 下关闭弱磁，d 轴弱磁参考为 0 | - |
| FW MODE FEEDBACK | READY 下选择定子电压反馈 PI 弱磁 | - |
| FW MODE HYBRID | READY 下选择 IDENTIFIED 模型前馈 + 电压反馈弱磁 | - |
| SPEED EST BASE | READY 下选择现役 12.5 ms 固定窗口测速 | - |
| SPEED EST WIN | READY 下选择 1 ms 短窗 + 150 Hz 一阶低通 | - |
| SPEED EST PLL | READY 下选择 200 Hz 临界阻尼编码器 PLL | - |
| POSITION offset_deg | 相对进入位置模式时原点的角度偏移 | degree |

示例：

~~~text
CURRENT 0 0.2
SPEED 1500
POSITION -2
~~~

CURRENT 参数顺序是 iq:id。阶段 0 的小信号测试保持 iq = 0，对 id 做 ±0.2 A 阶跃。

`CURRENT SINE D/Q amplitude frequency` 只在 CURRENT 运行模式接受，幅值范围为 0–0.5 A，频率范围为 50–5000 Hz。正弦由 40 kHz FOC 中断内的递推振荡器生成，分别出现在现有 `id_ref` 或 `iq_ref` scope 通道，不改变二进制通道布局。D 轴用于静止阶跃/频响；Q 轴是零均值转矩电流激励，在自由转子上可能产生小幅双向摆动，但不应形成持续旋转。发送普通 `CURRENT iq id`、`CURRENT SINE OFF` 或 `STOP` 都会终止正弦。

`SPEED rpm` 的当前机械速度参考范围为 ±6500 rpm。`SPEED SINE amplitude frequency` 只在主机启动的速度模式接受，幅值范围为 0–500 rpm，频率范围为 1–190 Hz。正弦以命令发出前最近一次 `SPEED rpm` 为偏置，且 `|偏置|+幅值` 不得超过电机速度参考上限；原脚本先发送 `SPEED 0`，所以仍得到零均值激励。参考由 40 kHz 递推振荡器生成、由 4 kHz 速度环采样；发送普通 `SPEED rpm`、`SPEED SINE OFF`、重新 START 或 STOP 都会终止正弦并清除偏置。`SPEED LIMIT` 只在 READY 接受，范围为 0–7 A；阶段 2 固定发送 4 A。该值是速度PI的基础q轴输出限幅；弱磁活动时还会按5 A总dq电流圆进一步动态收紧，控制器自身的饱和回算始终看到最终限幅。

`SPEED ID` 只在主机启动的速度模式且弱磁模式为OFF时接受，用于保持速度环接管 q 轴转矩的同时施加小幅 d 轴模型验证激励；STOP、重新 START 或控制器 clear 会恢复为 0。FEEDBACK/HYBRID活动时弱磁器是唯一d轴参考源，因此命令被拒绝。它是硬件运动实验命令，不能在未确认 24 V 和转子安全时发送。

`FW MODE OFF/FEEDBACK/HYBRID` 只在READY接受，不写入Flash。`FEEDBACK`使用限幅前请求电压的200 Hz低通值，由4 kHz电压PI跟踪95%可用dq电压；`HYBRID`用IDENTIFIED `Rs/Ld/Lq/flux`电压椭圆给出前馈d轴工作点，再由同一PI修正。两种活动模式都把`id_ref`限制在[-5,0] A、斜率限制为500 A/s，并在速度PI运行前按`sqrt(25-id_ref^2)`更新q轴余量。复位默认HYBRID；正常`VOLT LIMIT 1`且6500 rpm以内时未触及模型电压边界，弱磁保持不动作。

`SPEED EST BASE/WIN/PLL` 只在 READY 接受，用于测速器对比。三种估算器在每个 40 kHz 采样点并行更新，切换只改变送入速度环和遥测的输出，不清空编码器角度或修改 FOC 换相角。运行时选择不会写入 Flash，复位后 ABZ 路径恢复 D-037 选定的 WIN；WIN 固定为 40 点（1 ms）位置窗口加 150 Hz 一阶低通，PLL 固定为 200 Hz 自然频率、临界阻尼。BASE 仍保留作回归对照，但其 12.5 ms 固定窗在 80 Hz 的陷波已由硬件确认，不再作为正常默认。

## HFI诊断命令

| 命令 | 作用 |
|---|---|
| HFI FIXED voltage offset_deg | ABZ控制下启动估计d轴固定扫描 |
| HFI TENSOR voltage offset_deg | ABZ控制下启动四轴张量扫描 |
| HFI SHADOW voltage offset_deg | ABZ真值下启动张量影子跟踪 |
| HFI DTRACK | 将已建立的影子跟踪切到估计d轴载波 |
| HFI CONTROL | 让已就绪的HFI承担控制角/速度控制 |
| HFI OFF | 停止仅作影子或扫描的HFI诊断 |

诊断注入电压范围0.1～1.0 V，初始电角偏置范围±80°。这些命令用于分步研究，不是正式
启动所必需；`START HFI`已由supervisor自动组织正式链路。HFI请求在下一40 kHz FOC周期
边界生效。HFI已取得控制权时`HFI OFF`会被拒绝，必须发送`STOP`统一退出。

## PI 命令

| 命令 | 作用 |
|---|---|
| PI SPEED kp ki | 修改速度 PI |
| PI POSITION kp ki kd | 修改位置 PID |
| PI IQ kp ki | 修改 q 轴电流 PI |
| PI ID kp ki | 修改 d 轴电流 PI |
| PI CLEAR | 清除电流、速度和位置控制器状态 |

运行时 PI 命令用于实验和自整定研究。当前不会把参数持久化到 Flash，复位后恢复代码中的默认值。

## 轨迹命令

| 命令 | 作用 |
|---|---|
| TRACK STEP | 速度参考直接阶跃 |
| TRACK RAMP accel_rpm_per_sec | 速度固定加速度轨迹 |
| POS TRACK STEP | 位置参考直接阶跃 |
| POS TRACK RAMP velocity_deg_per_sec | 位置固定角速度轨迹 |
| POS TRACK TRAP velocity_deg_per_sec acceleration_deg_per_sec2 | 位置梯形/短行程三角形轨迹 |
| POS SPEED LIMIT rpm | READY 下限制位置环输出的机械速度参考 |
| POS FF ON/OFF | READY 下开关位置轨迹速度前馈 |

设置轨迹模式时会清除对应轨迹器状态，使新轨迹从当前反馈附近开始。
`START POSITION`会自动把内层速度轨迹器切换为STEP/NONE，避免上一次速度模式的
`TRACK RAMP`与 `POS TRACK`重复整形；进入位置模式后运动轨迹只由 `POS TRACK`决定。
`POS TRACK TRAP` 的两个参数分别是机械角速度上限和机械角加速度上限，单位为deg/s和
deg/s²；两者必须大于0。长行程形成梯形速度轨迹，短行程自动退化为三角形。
`POS FF ON` 把轨迹机械角速度乘以极对数后叠加到位置P输出，单位与速度环电角速度参考
一致；`POS FF OFF` 只保留位置误差反馈。STEP模式的轨迹速度为零，因此位置阶跃不会
产生前馈冲击。该开关不写入Flash，复位默认关闭。

## dq 前馈

| 命令 | 作用 |
|---|---|
| FF ON | 开启交叉耦合与反电动势前馈 |
| FF OFF | 关闭前馈，仅保留 PI |
| FF MODE OFF | 关闭全部电流前馈 |
| FF MODE CROSS | 只开启dq交叉耦合项 |
| FF MODE BEMF | 只开启永磁反电动势项 |
| FF MODE BOTH | 开启交叉耦合与反电动势两项 |
| FF MODEL LEGACY | READY 下选择原固件 Ld/Lq/flux |
| FF MODEL IDENTIFIED | READY 下选择阶段 1 辨识候选 Ld/Lq/flux |
| VOLT LIMIT ratio | READY或主机启动的速度模式下设置测试电压矢量上限比例 |

模型选择只修改 dq 解耦/反电动势前馈使用的 Ld、Lq 和 flux，不修改当前 PI、速度环或观测器参数。LEGACY 为 34.986/53.177 µH、0.904510 mWb；IDENTIFIED 为 26.044/40.344 µH、0.907950 mWb。模型命令只在 READY 接受，实验脚本应在 START 前选择；D-059后复位默认为IDENTIFIED。模式值为 `OFF=0/BOTH=1/CROSS=2/BEMF=3`，其中0/1保留旧遥测兼容；`FF ON/OFF`分别等价于 `FF MODE BOTH/OFF`，D-059后复位默认为BOTH。模式命令使用单次32位写入，仍允许旧脚本在运行中A/B，但正式连续实验应优先在STOP/READY后切换。

`VOLT LIMIT ratio`在READY或由主机命令启动的速度模式接受，范围为 `0 < ratio <= 1`，不写入Flash，复位默认1；电流、位置和非主机速度模式运行中仍拒绝。它把SVPWM/SPWM的dq矢量上限分别设为 `ratio*Vbus/sqrt(3)` 与 `ratio*Vbus/2`，但SVPWM仍使用实测Vbus计算占空比；它不是虚拟母线。弱磁器读取同一个实际dq上限计算95%目标，因此人工比例和真实母线变化沿同一控制链生效。D-065只用运行时写入补齐动态边界实验，未改变调制或PI tracking。实验退出应STOP、确认READY后发送 `VOLT LIMIT 1`。

## 普通遥测帧

帧格式：

~~~text
36 × float32 little-endian
00 00 80 7F
~~~

默认每 2 ms 发送一次。通道：

| 索引 | 名称 | 说明 |
|---:|---|---|
| 0 | ua | A 相调制电压 |
| 1 | ub | B 相调制电压 |
| 2 | uc | C 相调制电压 |
| 3 | ia_shifted | A 相电流 + 12 |
| 4 | ib_shifted | B 相电流 + 12 |
| 5 | ic_shifted | C 相电流 + 12 |
| 6 | iq | q 轴电流 |
| 7 | id | d 轴电流 |
| 8 | vf_angle | VF 电角度 |
| 9 | hfi_angle | HFI 电角度 |
| 10 | smo_angle | SMO 电角度 |
| 11 | flux_angle | 磁链观测器电角度 |
| 12 | abz_angle | ABZ 编码器电角度 |
| 13 | abs_angle | 绝对编码器电角度 |
| 14 | vf_speed | VF 电角速度 |
| 15 | hfi_speed | HFI 电角速度 |
| 16 | smo_speed | SMO 电角速度 |
| 17 | flux_speed | 磁链观测器电角速度 |
| 18 | abz_speed | ABZ 电角速度 |
| 19 | abs_speed | 绝对编码器电角速度 |
| 20 | uq | q 轴施加电压 |
| 21 | ud | d 轴施加电压 |
| 22 | iq_ref | q 轴电流参考 |
| 23 | id_ref | d 轴电流参考 |
| 24 | target_elec_speed | 速度环外部电角速度目标 |
| 25 | elec_speed | 当前控制电角速度 |
| 26 | voltage_scale | 电压限制缩放比例 |
| 27 | motor_mode | MOTOR_MODE_t 数值 |
| 28 | speed_ref | 轨迹后的电角速度参考 |
| 29 | vbus | ADC 测得母线电压 |
| 30 | target_position | 外部位置目标，rad |
| 31 | position_ref | 轨迹后位置参考，rad |
| 32 | position_feedback | 多圈位置反馈，rad |
| 33 | uq_feedforward | q 轴前馈电压 |
| 34 | ud_feedforward | d 轴前馈电压 |
| 35 | feedforward_mode | OFF=0、BOTH=1、CROSS=2、BEMF=3 |

相电流通道 3–5 为兼容原有 VOFA 观察增加了 +12 偏置；主机脚本在分析时减去 12。

## 软件示波器命令

| 命令 | 作用 |
|---|---|
| SCOPE ARM 1 | 40 kHz 采样，102.4 ms |
| SCOPE ARM 10 | 4 kHz 采样，1.024 s |
| SCOPE ARM 100 | 400 Hz 采样，10.24 s |
| SCOPE DUMP | 捕获完成后导出 |

ARM 只接受 1、10、100 三档分频。重新 ARM 会清零样本计数和最大周期统计；正在 dump 时拒绝新的 ARM。

## 软件示波器头

当前协议版本为 1，主机按 little-endian 解析：

| 字段 | 类型 | 说明 |
|---|---|---|
| magic | uint32 | SCP1 |
| version | uint16 | 1 |
| channel_count | uint16 | 12 |
| sample_count | uint32 | 4096 |
| divider | uint32 | 1/10/100 |
| base_sample_hz | uint32 | 40000 |
| sample_size | uint32 | 48 bytes |
| data_bytes | uint32 | sample_count × sample_size |
| max_capture_cycles | uint32 | ISR 单次记录最大 cycles |
| cpu_hz | uint32 | SystemCoreClock |
| reserved | uint32 | 0 |

数据部分每个样本固定12个float32；D-058只在旧11通道末尾追加一项，因此前11项顺序不变：

~~~text
ia, ib, iq, id, iq_ref, id_ref, uq, ud, elec_angle, elec_speed, speed_ref, voltage_scale
~~~

普通遥测在 scope dump 期间暂停，dump 完成后恢复。

持续过流保护触发时，40 kHz控制路径会先直接关闭六路PWM并进入IDLE，前台随后输出一次：

~~~text
FAULT_OVERCURRENT,Iq=...,Id=...
~~~

故障文本不再从控制中断内阻塞打印。该状态不是READY，主机仍应按异常路径停止实验并人工确认原因。

## 主机脚本

| 脚本 | 用途 |
|---|---|
| tests/telemetry_capture.py | 普通遥测抓取 |
| tests/motor_baseline.py | 连续速度基线 |
| tests/scope_capture.py | 串口软件示波器 |
| tests/scope_current_step.py | 小电流阶跃 |
| tests/scope_jlink_capture.py | J-Link 后备读取 |
| tests/stage0_validation.py | 电流、速度、位置和前馈连续验证 |

使用示例见 [测试说明](../tests/README.md)。

## 兼容性与修改规则

修改协议时必须同时检查：

1. app/debug/debug.c 和 debug.h；
2. FOC_DEBUG_t；
3. tests/motor_baseline.py 的 36 通道解析；
4. tests/scope_capture.py 的12通道布局；
5. tests/scope_jlink_capture.py 的 MAP 符号和结构布局；
6. 本文件和 tests/README.md。

当前协议没有 CRC、序号或正式握手，适合单机实验，不适合作为产品通信协议。后续若迁移 CAN 或设计正式上位机，应单独版本化，不在现有调试协议上无限叠加字段。
