# 阶段 4 弱磁控制研究

更新时间：2026-08-28

状态：D-061～D-065 已完成；复位默认冻结为 IDENTIFIED+BOTH 电流前馈与 HYBRID 弱磁

## 1. 目标与结论边界

阶段 4 只研究弱磁，不把 MTPA 与首版实现捆绑。当前只有空载电机、24 V 限流电源和软件
观测，不能验证“同转矩最小电流”、效率或带载恒功率；但可以直接验证电压利用率、负 d 轴
电流、电流圆协调、正反转进入/退出以及速度扩展。因此 MTPA 延期到具备可控负载或转矩/
功率测量后再评估。

最终结论是：标准定子电压反馈 PI 可作为稳健工业基线；IDENTIFIED 稳态 dq 电压椭圆作为
前馈、同一电压 PI 校正残差的 HYBRID，在本项目人工电压边界下保持同等电压跟踪，同时
显著降低动态电流峰值，因而成为复位默认。该结论只覆盖空载、±6500 rpm 以内和 5 A 总
dq 电流圆，不代表自然母线基速、带载恒功率、效率、热稳定或退磁安全边界已经验证。

## 2. 控制链与坐标

电流环为 40 kHz，速度环和弱磁器为 4 kHz。阶段 3 的 IDENTIFIED+BOTH 继续按实测电流和
实测电角速度计算：

~~~text
ud_ff = -omega_e * Lq * iq
uq_ff =  omega_e * (flux + Ld * id)
~~~

电流 PI 与前馈相加得到限幅前请求电压，SVPWM 使用
`voltage_limit_ratio * Vbus / sqrt(3)` 等比例限制 dq 矢量；限幅后的实际 dq 电压减去前馈，
再由 `track_output()` 回写两轴 PI。弱磁只改变电流参考和速度 PI 的 q 轴输出限幅，不修改
这条 applied-voltage tracking 链。

弱磁反馈使用限幅前请求幅值，而不是等待 `voltage_scale < 1`：

~~~text
Umax    = voltage_limit_ratio * Vbus / sqrt(3)
Utarget = 0.95 * Umax
Ureq    = hypot(ud_request, uq_request)
eu      = lowpass_200Hz(Ureq) - Utarget
~~~

`eu > 0` 时令 d 轴电流更负，`eu < 0` 时退出弱磁。现有 Scope 的 `ud/uq` 和
`voltage_scale` 可由 `hypot(ud,uq)/voltage_scale` 重建 `Ureq`，因此未增加新通道。

## 3. 冻结算法

板端模式为：

- `OFF`：`id_fw=0`，保留 IDENTIFIED+BOTH 电流前馈；
- `FEEDBACK`：4 kHz 电压 PI 从零生成负 d 轴电流；
- `HYBRID`：IDENTIFIED 电压椭圆给出前馈工作点，电压 PI 修正模型残差。

电压 PI 参数冻结为 `Kp=0.4 A/V`、`Ki=2400 A/(V*s)`，输入使用 200 Hz 一阶低通，4 kHz
离散系数为 0.2695973。该组参数由约 925 Hz 已验证电流环、约 0.125 V/A 的局部电压灵敏度
和 50 Hz 弱磁交越推导，再由板端进入/退出验证，不重新搜索三环 PI。

HYBRID 使用阶段 1 参数：

~~~text
Rs   = 0.144818365 ohm
Ld   = 26.043582 uH
Lq   = 40.343765 uH
flux = 0.907950340 mWb

ud = Rs*id - omega_e*Lq*iq
uq = Rs*iq + omega_e*(Ld*id + flux)
ud^2 + uq^2 = Utarget^2
~~~

把最后一式展开为 `a*id^2+b*id+c=0`，选择满足 `id<=0` 且最接近零的可行根。求解使用带
符号的 `omega_e` 和 `iq`，离线回归覆盖零速、正反转、电压圆内外、根选择及方向对称性。
纯 MODEL 只用于离线对照，不增加第四种板端模式。

总 dq 参考采用 5 A 圆：

~~~text
-5 A <= id_ref <= 0
abs(iq_ref) <= sqrt(5^2 - id_ref^2)
~~~

弱磁器先生成 `id_ref`，再在速度 PI 运行前把其输出限幅改为上述 q 轴余量，因此速度 PI
自身的钳位和 anti-windup 看见真实动态约束。最终 `id_ref` 变化率限制为 500 A/s；早期
1000 A/s 候选在扩展工况暴露单采样电流峰值，减半后主矩阵和扩展矩阵均通过。电压 PI
对电流圆和斜率限制采用 unity tracking，避免约束期间累积积分偏置。

## 4. 离线设计与可行域

D-058 的 4 kHz 请求电压噪声在 8 份 Scope 中约为 0.08 V RMS，支持 200 Hz 输入滤波。
24 V 和当前磁链对应的理想自然空载基速约 14,573 rpm，明显高于已验证机械范围，因此
板端使用人工电压上限，而不是直接追逐自然基速。

D-061 最终选择两档：

| 阶段 | 速度脉冲 | 电压比例 | 95%目标电压（约） |
|---|---:|---:|---:|
| 主矩阵 | ±4500 → ±5000 rpm | 0.35 | 4.61 V |
| 扩展矩阵 | ±6000 → ±6500 rpm | 0.45 | 5.92 V |

在 5000 rpm，模型对 `iq=0/0.5/1/2 A` 给出的 `id` 约为
`-1.10/-1.71/-2.39/-4.01 A`；在 6500 rpm 对应约为
`-1.47/-1.94/-2.45/-3.64 A`。`iq=3 A` 在两档的 5 A 电流圆内均不可行，这也是必须在
速度 PI 内部应用动态 q 轴余量而不是事后裁剪参考的原因。

## 5. 连续硬件验证

D-063/D-064 固定 IDENTIFIED+BOTH，每档执行两轮反序：第一轮
`OFF→FEEDBACK→HYBRID`、第二轮 `HYBRID→FEEDBACK→OFF`，每轮均覆盖正反方向，共
12 份 4 kHz、1.024 s Scope。主矩阵和扩展矩阵的 64/64 资格检查均通过，最终均 STOP/
READY。聚合结果如下：

| 工况 | 模式 | 电压误差均值 / RMS (V) | 速度误差 RMS (rpm) | 稳态 `id_ref` (A) | 实际 dq 峰值 (A) |
|---|---|---:|---:|---:|---:|
| 5000 rpm | OFF | +0.235 / 0.243 | 40.65 | 0 | 3.029 |
| 5000 rpm | FEEDBACK | -0.004 / 0.150 | 63.00 | -1.370 | 4.713 |
| 5000 rpm | HYBRID | +0.037 / 0.156 | 59.17 | -1.107 | 3.984 |
| 6500 rpm | OFF | +0.451 / 0.458 | 116.04 | 0 | 3.213 |
| 6500 rpm | FEEDBACK | -0.014 / 0.204 | 65.47 | -1.698 | 5.416 |
| 6500 rpm | HYBRID | +0.014 / 0.198 | 63.58 | -1.581 | 4.131 |

HYBRID 相对 FEEDBACK 的电压 RMS 误差相当，5000/6500 rpm 的速度 RMS 略低；更重要的是
实际 dq 峰值分别降低约 15.5% 和 23.7%，最大参考电流也分别从 4.209/4.824 A 降到
3.742/3.452 A。正反方向差仍小于资格余量。两档全局最小 `voltage_scale` 分别为
0.963833 和 0.916495，没有失去调制控制。

复位默认冒烟不发送 `FW MODE`，在 +5000 rpm、0.35 人工电压比例下测得
`id_ref=-1.295 A`、`Ureq=4.624 V`、`Utarget=4.606 V`、误差 +17.8 mV、实际 dq 峰值
3.186 A、最小缩放 0.984581，并确认电流前馈模式仍为 BOTH。测试最终恢复
`HYBRID + VOLT LIMIT 1 + IDENTIFIED+BOTH + SPEED LIMIT 7` 并进入 READY。

原始结果：

- `tests/results/stage4_field_weakening_main_20260828_010821.json`
- `tests/results/stage4_field_weakening_extension_20260828_010949.json`
- `tests/results/stage4_default_field_weakening_smoke_20260828_011405.json`
- 机器摘要：`tests/baselines/stage4_field_weakening_d064_20260828.json`

## 6. 动态电压边界补充

D-065不再增加速度或模式网格，而是在固定±6500 rpm下把同一段4 kHz Scope内的
`VOLT LIMIT`按`1.00→0.45→1.00`切换，分别覆盖FEEDBACK/HYBRID和正反方向，共4次
访问。为使测试命令能表达真实母线变化，固件只把`VOLT LIMIT`的接受范围从READY扩展到
主机启动的速度模式；比例仍由一个32位`volatile float`在主循环写、FOC ISR读，调制、
电压PI和tracking路径均未改变。

| 模式 | 受限段`id_ref` (A) | 电压误差均值/RMS (V) | 速度误差RMS (rpm) | 最长进入/退出 (ms) | 实际dq峰值 (A) |
|---|---:|---:|---:|---:|---:|
| FEEDBACK | -1.831 | -0.009/0.210 | 66.85 | 3.50/10.25 | 3.912 |
| HYBRID | -1.732 | +0.017/0.201 | 64.46 | 4.25/9.00 | 3.788 |

两种模式在边界下降前和恢复后的`id_ref`均回到0，电压边界变化后的首个10 ms均值窗已经
进入±0.25 V，最小`voltage_scale=0.981924`，最大参考/实际dq电流为2.902/3.912 A。
初次在线报告错误地要求连续10 ms内每个4 kHz原始电压样点都落在±0.25 V，因正常纹波把
4个settling项判为失败；保持原始Scope不变、按预先使用的10 ms观测尺度重算后44/44检查
通过。机器摘要为`tests/baselines/stage4_voltage_boundary_d065_20260828.json`，原始结果为
`tests/results/stage4_voltage_boundary_20260828_014559.json`。

## 7. 构建、默认与阶段退出

最终源码默认是 `IDENTIFIED+BOTH + HYBRID`，正常 `voltage_limit_ratio=1` 下，6500 rpm
仍低于模型自然电压边界，弱磁保持不动作，不会平白注入负 d 轴电流。运行时可在 READY
使用 `FW MODE OFF|FEEDBACK|HYBRID`；弱磁活动时禁止手工 `SPEED ID`，避免两个 d 轴参考
源相互覆盖。

最终 106 项 Python 测试通过，C++ 算法回归 PASS；Keil/ARMCC 5.06u7 为 0 error、
0 warning，Flash 111,504 B、RAM 237,476 B。固定 J-Link V7.54d 对 STM32H743VG、SWD
4000 kHz 的原始 Program & Verify 为 O.K.，向量表读回
`20008000 08000449 0800E3BF 0800D959`。

阶段 4 至此在当前空载与人工电压边界内完成。不继续为尚未出现的问题增加 Ld 调度、MTPV、
过调制、q 轴电压回退或预测控制。将来若具备受控负载、母线变化或更高电流数据，应先用
新的主导误差决定是否加入 Ld/磁链调度；当前不能宣称已验证带载转矩、效率、恒功率、自然
最高转速或退磁边界。
