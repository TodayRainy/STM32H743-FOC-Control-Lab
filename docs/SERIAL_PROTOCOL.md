# 串口、遥测与软件示波器协议

本文记录阶段 0 主机接口。协议当前用于研究和自动化测试，尚未承诺长期二进制兼容；修改固件通道布局时必须同步更新 tests 中的主机脚本。

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
| STOP | 停止运行并返回 READY | 非校准失败状态 |

START 过程会经过功率级预充、CLEAR、START，并在首次运行时完成 ABZ 编码器校准。命令发出后不能仅按固定延时假定成功，应等待对应 mode。

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
| SPEED rpm | 机械速度参考 | rpm |
| POSITION offset_deg | 相对进入位置模式时原点的角度偏移 | degree |

示例：

~~~text
CURRENT 0 0.2
SPEED 1500
POSITION -2
~~~

CURRENT 参数顺序是 iq:id。阶段 0 的小信号测试保持 iq = 0，对 id 做 ±0.2 A 阶跃。

## PI 命令

| 命令 | 作用 |
|---|---|
| PI SPEED kp ki | 修改速度 PI |
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

设置轨迹模式时会清除对应轨迹器状态，使新轨迹从当前反馈附近开始。

## dq 前馈

| 命令 | 作用 |
|---|---|
| FF ON | 开启交叉耦合与反电动势前馈 |
| FF OFF | 关闭前馈，仅保留 PI |

前馈开关不会改变电机参数。Ld、Lq 和 flux 来自当前固件配置，后续需要参数辨识对它们进行校正。

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
| 35 | feedforward_enabled | 0/1 |

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
| channel_count | uint16 | 10 |
| sample_count | uint32 | 4096 |
| divider | uint32 | 1/10/100 |
| base_sample_hz | uint32 | 40000 |
| sample_size | uint32 | 40 bytes |
| data_bytes | uint32 | sample_count × sample_size |
| max_capture_cycles | uint32 | ISR 单次记录最大 cycles |
| cpu_hz | uint32 | SystemCoreClock |
| reserved | uint32 | 0 |

数据部分每个样本固定 10 个 float32：

~~~text
ia, ib, iq, id, iq_ref, id_ref, uq, ud, elec_angle, elec_speed
~~~

普通遥测在 scope dump 期间暂停，dump 完成后恢复。

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
4. tests/scope_capture.py 的 10 通道布局；
5. tests/scope_jlink_capture.py 的 MAP 符号和结构布局；
6. 本文件和 tests/README.md。

当前协议没有 CRC、序号或正式握手，适合单机实验，不适合作为产品通信协议。后续若迁移 CAN 或设计正式上位机，应单独版本化，不在现有调试协议上无限叠加字段。
