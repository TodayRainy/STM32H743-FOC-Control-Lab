# 测试、自动化与软件示波器

tests 目录包含主机侧算法回归、串口遥测、连续电机测试和内存软件示波器工具。硬件脚本会控制电机，不应在未确认 24 V、电机和周边安全状态时运行。

## 脚本概览

| 文件 | 作用 | 硬件 |
|---|---|---|
| foc_algorithm_regression.cpp | 坐标变换、SVPWM、PI、轨迹和电压限制的主机回归 | 不需要 |
| run_algorithm_regression.ps1 | 使用 PATH 中的 g++ 构建并运行主机回归 | 不需要 |
| motor_baseline.py | 连续速度点、运行参数和遥测基线 | 需要 |
| telemetry_capture.py | 串口遥测帧采集 | 需要 |
| scope_capture.py | 通过串口 ARM/DUMP 软件示波器 | 需要 |
| scope_current_step.py | 软件示波器电流阶跃与指标摘要 | 需要 |
| scope_jlink_capture.py | 通过 J-Link 和 MAP 符号读取软件示波器 | 需要 |
| stage0_validation.py | 电流、速度、前馈和位置的连续阶段 0 验证 | 需要 |
| stage1_identification.py | 连续重复 Rs/Ld/Lq/磁链辨识并保存 CSV/JSON | 需要 |
| stage1_model_validation.py | 前馈关闭/旧模型/辨识模型的连续 A/B 验证 | 需要 |
| stage2_current_loop_design.py | d/q PI 带宽计算和 R-L/延迟离线扫掠 | 不需要 |
| stage2_current_autotune.py | 不依赖现役 PI 的安全初值、实测对象设计和 6–8 组候选生成 | 不需要 |
| stage2_current_autotune_validation.py | 新 PI 的双轴、双顺序连续硬件筛选 | 需要 |
| stage2_current_autotune_analysis.py | 重建 PWM-off dq 噪声并按指定 D-028/D-029 标准离线复算 | 不需要 |
| test_stage2_current_autotune.py | 自整定频域设计与当前数据契约主机回归 | 不需要 |
| stage2_current_loop_validation.py | 现役/候选 PI 的连续 d 轴阶跃 A/B | 需要 |
| stage2_current_loop_analysis.py | 阶跃网格的量化感知离线复核 | 不需要 |
| stage2_current_frequency_response.py | 同步正弦 d 轴闭环频响与稳定裕量 | 需要 |
| stage2_current_frf_analysis.py | 合并阶跃与频响门槛的离线摘要 | 不需要 |
| stage2_current_combination_validation.py | 现役/2 kHz PI 与辨识前馈的正反旋转组合验证 | 需要 |
| stage2_current_combination_analysis.py | 旋转组合结果的双顺序汇总与写回决策 | 不需要 |
| stage2_q_current_pulse.py | 自由转子 q 轴短脉冲边界验证 | 需要 |
| stage3_feedforward_component_validation.py | OFF/CROSS/BEMF/BOTH两轮反序分项验证 | 需要 |
| test_stage3_feedforward_component_validation.py | 阶段3矩阵、公式与因子对比主机回归 | 不需要 |
| stage3_voltage_saturation_validation.py | OFF/BOTH受控电压限幅进入与退出恢复矩阵 | 需要 |
| test_stage3_voltage_saturation_validation.py | 电压矩阵、Scope恢复窗口和前馈坐标回归 | 不需要 |
| stage3_default_feedforward_smoke.py | 不发送FF选择命令的IDENTIFIED+BOTH复位默认冒烟 | 需要 |
| test_stage3_default_feedforward_smoke.py | 复位默认冒烟计划与运动边界回归 | 不需要 |
| stubs/ | 主机回归需要的嵌入式接口桩 | 不需要 |
| results/ | CSV、JSON 和分阶段软件示波器数据 | 数据 |

主机回归入口：

~~~powershell
powershell -ExecutionPolicy Bypass -File tests/run_algorithm_regression.ps1
~~~

默认使用 PATH 中的 g++，临时可执行文件写入被忽略的 tmp 目录。2026-08-24 已使用 MinGW g++ 15.2.0 验证 PASS。

阶段 1 增加的主机回归覆盖：

- 双极性多点 Rs 与逆变器等效压降联合拟合；
- Rs 和磁链留出点预测误差；
- 基波 DFT 复阻抗到等效 R/L 的换算；
- 正负多速度点磁链和电压截距回归。

这些测试使用合成数据，只验证计算实现，不代替硬件可观测性和精度验证。

## 阶段 1 辨识命令

阶段 1 暂不增加庞大的独立工具框架，连续实验入口为：

~~~powershell
python tests/stage1_identification.py --port COM_PORT
~~~

默认执行两轮 RL 和两轮磁链辨识，监控 5 A、3500 rpm 和 18–28 V 边界，在异常和正常退出时都发送 STOP。5 A 是包含校准退出瞬态的主机中止阈值，RL 稳态目标仍为最大 3 A。首次安全验证可以先只运行 RL：

~~~powershell
python tests/stage1_identification.py --port COM_PORT --rl-repeats 1 --skip-flux
~~~

完成 RL 后可单独重复磁链辨识，避免再次施加静止电流：

~~~powershell
python tests/stage1_identification.py --port COM_PORT --skip-rl --flux-repeats 1
~~~

底层复用的串口命令为：

~~~text
START IDENT RL
IDENT RESULT
IDENT POINTS
START IDENT FLUX
IDENT RESULT
IDENT POINTS
STOP
~~~

每条 START 前必须处于 READY。RL 辨识会施加电流，FLUX 辨识会正反转电机；两者都要求用户当次确认 24 V 已上电且电机周边安全。`IDENT POINTS` 只查询最近一次 Rs 实际 I/U 点，不会驱动电机。脚本在失败时也会保存已采集的 CSV/JSON，再抛出异常。

当前 RL 在一次运行内完成正序/反序扫描和留出点，因此首次检查只需一轮 RL 与一轮 FLUX：

~~~powershell
python tests/stage1_identification.py --port COM_PORT --rl-repeats 1 --flux-repeats 1
~~~

报告中的 `ModelKvRpmPerVdc` 使用 `60/(2π√3·pole_pairs·Flux)`，对应正弦反电势、相峰值磁链及 SVPWM 线性母线利用假设；`TorqueConstantNmPerPeakA` 使用 `1.5·pole_pairs·Flux`。两者是坐标约定检查，不是新的独立测量。

第二批首轮筛选线在实验前固定为：

- `RsDelta ≤ 2%`，`VoffDelta ≤ 5 mV`；
- Rs 拟合 RMSE 和留出 RMSE 均不超过 10 mV；
- `LdDelta/LqDelta ≤ 5%`，否则记录为幅值相关而不采用单一常数结论；
- 磁链拟合和留出 RMSE 均不超过 10 mV，留出最大误差不超过 20 mV；
- 模型 KV 与 650 rpm/V 铭牌差异不超过 10%，仅作为量纲/约定检查。

任一项超限时先查看 16 个点及时间顺序，不自动增大电流，也不写回控制参数。

自由转子上的 Rs 注入必须使用编码器转子 d 轴，且电压调节在收敛、稳定和平均期间持续运行。历史诊断版固定定子轴注入会产生转矩并使实际电流偏离目标；提高电流不能修复这种采样误差。

2026-08-24 已在 24 V、4.5 A 限流、空载条件下完成修正版两次 RL/L 重复和一次磁链留出验证。两次 Rs/L 全部筛选线通过，磁链拟合/留出也通过。机器可读摘要为 `tests/baselines/stage1_20260824.json`；当前候选参数的本地原始证据为：

~~~text
results/stage1_identification_rl_daxis_regulated_20260824.csv/.json
results/stage1_identification_rl_daxis_regulated_repeat_20260824.csv/.json
results/stage1_identification_flux_post_rl_20260824.csv/.json
~~~

`stage1_identification_validation_20260824*` 和 `stage1_identification_rl_validation_repeat_20260824*` 保留了旧采样方法的失败证据，用于解释为何必须采用 d 轴持续调压。

## 阶段 1 模型 A/B 验证

辨识候选参数不直接覆盖原 PI。首轮控制验证入口为：

~~~powershell
python tests/stage1_model_validation.py --port COM_PORT
~~~

一次运行连续比较：

1. 前馈关闭；
2. 原固件 Ld/Lq/flux 前馈；
3. 阶段 1 辨识 Ld/Lq/flux 前馈。

默认速度点为 ±1500、±3000 rpm，每个速度下依次施加 0、+1、-1 A 的 d 轴实验电流。速度斜坡为 1500 rpm/s，主机边界为 3 A、3500 rpm 和 18–28 V。脚本记录 dq 电流误差、PI 补偿电压、前馈电压、公式复算残差和电压限幅比例，并在正常或异常退出时发送 `SPEED ID 0`、`SPEED 0` 和 STOP。

该实验会使电机正反转，执行前必须重新确认 24 V、电机可自由旋转和周边安全。首轮只使用现有 500 Hz 遥测；如果旧/新模型差异接近噪声，再对少数工况增加软件示波器，不默认导出大量高速波形。

`--profiles` 可指定子集和顺序，例如反顺序复核：

~~~powershell
python tests/stage1_model_validation.py --port COM_PORT `
  --profiles ff_identified,ff_legacy --speeds 3000,-3000 --id-points 1,-1
~~~

2026-08-24 的首轮和反顺序硬件结果表明，前馈相对关闭将平均 PI 电压负担降低约 86%；LEGACY/IDENTIFIED 的高速合并均值只差约 3.1%，小于重复波动，当前不能可靠区分。原始数据为：

~~~text
results/stage1_model_validation_20260824.csv/.json
results/stage1_model_validation_reverse_20260824.csv/.json
~~~

## 阶段 2 电流环设计与验证

### 新电机冷启动自整定离线入口

当前主线使用不依赖现役 PI 的主机监督式设计：

~~~powershell
python tests/stage2_current_autotune.py `
  --stage1 tests/baselines/stage1_20260824.json `
  --frf tests/results/stage2_current_frf_20260824_145248.json `
  --output tests/baselines/stage2_current_autotune_design_20260824.json

python tests/test_stage2_current_autotune.py
~~~

脚本只读取阶段 1 参数和实际对象频响，不读取现役 PI 增益，不连接串口，也不写固件。当前历史频响中显式排除 `active_legacy`，只使用由阶段 1 参数生成过的四组控制器得到的 `I/U` 对象数据；新电机流程将改用工具自己生成的 R-L 安全初值采集同样的数据。

40 kHz、1–2 周期延迟和当前电压/电流边界得到 `f_upper=1666.7 Hz`、`f_seed=800 Hz`。固定网格原始 8 点中，1500 Hz/65° 因 3 A 离散阶跃预测峰值 4.160 A 超过 4 A 预留门槛而被拒绝；工具没有放宽门槛，而是以同频率 70° 的相邻安全点替换，最终形成 8 组硬件粗筛候选。机器可读基线为 `baselines/stage2_current_autotune_design_20260824.json`。

闭环 -3 dB 带宽统一按 `T=I/Iref` 相对低频平台下降 3.0103 dB 计算。R-L 锚点用它检查模型目标；实测对象候选用 0 dB 环路交越和相位裕量检查设计目标，-3 dB 只表达实际跟踪速度。当前 q 轴对象由实测 d 轴公共采样/逆变器路径和辨识 Lq 推导，因此离线输出最多支持 `d_axis_qualified_q_axis_model_based`，不能冒充 q 轴硬件验证。

### 新电机冷启动自整定首轮硬件粗筛（历史基线）

先离线检查完整实验计划，不连接串口，也不需要 24 V：

~~~powershell
python tests/stage2_current_autotune_validation.py `
  --design tests/baselines/stage2_current_autotune_design_20260824.json `
  --plan-only
~~~

默认计划不把现役 PI 放入候选集。8 组候选第一轮按设计顺序运行，第二轮反序；每次访问包含 1 个零电流点、`±0.75/±1.5/±3 A` 六个 d 轴阶跃、d/q 两轴各 10 个 `100–4000 Hz` 正弦频点。另在启动 PWM 前采一次噪声基线，总计 433 份 4096 点/40 kHz scope。q 轴在 300 Hz 以上使用 0.3 A 零均值正弦，补足既有旋转实验只有约 0.12 A iq 激励的证据缺口；100/150 Hz 为限制单半周转矩冲量，分别降为 0.10/0.15 A。它可能使自由转子原地小幅双向摆动，但不要求持续旋转或提高稳态转速。

硬件入口为：

~~~powershell
python tests/stage2_current_autotune_validation.py `
  --design tests/baselines/stage2_current_autotune_design_20260824.json `
  --port COM_PORT
~~~

脚本逐点硬中止条件为 dq 峰值电流 4 A、转速 500 rpm、母线 18–28 V 和 98% 电压利用率；30% 超调或 2 ms 噪声受限整定时间只记录性能警告，不跳过其余候选。正常或异常退出都会停止 d/q 正弦、清零电流、恢复不依赖旧 PI 的 `rl_seed` 并 STOP，最终必须确认 READY。候选选择严格复用规范中的 10% 超调、整定时间不超过 `1/f_bw`、60° 相位裕量、`Ms≤1.7`、目标频率一致性、噪声归一化、正负对称和两轮重复性等固定门槛，不因结果不理想而放宽。

该实验必须在当次确认 24 V 已上电、转子可自由双向旋转且周边安全后运行。

2026-08-24 已完整执行 433 点硬件粗筛：96 个阶跃、16 个 PWM-on 零点、320 个 d/q 频响点和 1 个 PWM-off 基线全部完成，最终 READY。原始结果为 `results/stage2_current_autotune_hw_20260824_174926.json`。PWM-off 时固件 `iq/id` 调试字段保持为零，因此离线分析必须从 `ia/ib/elec_angle` 按固件 Clarke/Park 约定重建 dq 噪声；复现入口为：

~~~powershell
python tests/stage2_current_autotune_analysis.py `
  tests/results/stage2_current_autotune_hw_20260824_174926.json `
  --output tests/baselines/stage2_current_autotune_hw_20260824.json
~~~

修正后 8 组的零点噪声、相位裕量、Ms、目标频率、电流/电压安全和重复性均得到有效评价，但没有候选通过全部固定门槛，状态为 `requirement_not_met`。`rl_seed` 只剩稳态 RMS 失败；`rl_high` 和 `measured_fc1000_pm75` 还差整定时间。稳态平均偏差仅约 0.06–0.21 mA，而 37–39 mA RMS 几乎全部来自 10–19 kHz 的共同采样/开关纹波，当前不能通过事后放宽门槛把任一候选改写成推荐。

D-028 在上述结果冻结后把指标简化为：PM/Ms 稳定性、超调、`t_settle*f_bw`、带宽内跟踪 RMS，以及一个合并的方向/重复一致性门槛。带宽外采样纹波只做异常约束，不用于 PI 排名。旧数据按新版本复算的独立基线为 `baselines/stage2_current_autotune_hw_d028_20260824.json`；只有 `rl_seed` 通过，但它只是粗筛领先者。D-028 的 7 组、519 点方案保留为历史计划，不再是默认入口。

当前 D-029 先生成覆盖 800–2000 Hz 的 8 组设计并检查 593 点计划：

~~~powershell
python tests/stage2_current_autotune.py `
  --stage1 tests/baselines/stage1_20260824.json `
  --frf tests/results/stage2_current_frf_20260824_145248.json `
  --exclude-profile active_legacy --policy d029 `
  --output tests/baselines/stage2_current_autotune_d029_design_20260824.json
python tests/stage2_current_autotune_validation.py --plan-only
~~~

候选为 R-L 800/1250/1500/1750/2000 Hz 和实测对象 1000/1250/1500 Hz、75°。每次访问包含一个零点、`±0.75/±1.5/±2.25/±3 A` 八个阶跃，以及 d/q 各 14 个 `100–4000 Hz` 频点；两轮正反顺序共 16 次访问、593 份 scope。真正执行仍使用 `--port COM_PORT`，需要 24 V 上电和自由安全转子，并以当次确认状态为准。

D-029 只把归一化整定硬门槛改为 `t_settle*f_bw<=1.25`，`<=1.0` 保留为优秀目标。旧 433 点数据的复现入口为：

~~~powershell
python tests/stage2_current_autotune_analysis.py `
  tests/results/stage2_current_autotune_hw_20260824_174926.json `
  --criteria-version d029 `
  --output tests/baselines/stage2_current_autotune_hw_d029_reanalysis_20260824.json
~~~

2026-08-24 已执行 D-029 默认 593 点实验，原始结果为 `results/stage2_current_autotune_hw_20260824_221550.json`，离线摘要为 `baselines/stage2_current_autotune_hw_d029_20260824.json`。`measured_fc1000_pm75` 和 `rl_seed` 全部通过；前者以 1112.5 Hz 鲁棒带宽排名第一，后者为 612.0 Hz。安全峰值为 3.580 A、39.85 rpm 和 1.126 V，最终 READY。当前领先者仍需独立留出验证，不直接修改默认 PI。

D-030 独立留出复用同一脚本，只选择冻结领先者并自动改用未见的 6 个阶跃和 d/q 各 14 个频点：

~~~powershell
python tests/stage2_current_autotune_validation.py `
  --experiment holdout --plan-only
# 当次重新确认 24 V 和机械安全后：
python tests/stage2_current_autotune_validation.py `
  --experiment holdout --port COM_PORT
~~~

默认两轮共 71 份 scope。留出通过后才进入现役 PI 对照；失败时不修改冻结门槛或自动选择其他候选。

D-030 已完整执行并通过。原始结果为 `results/stage2_current_autotune_hw_20260824_222819.json`，正式摘要为 `baselines/stage2_current_autotune_holdout_d030_20260824.json`。鲁棒带宽 1120.6 Hz，与 D-029 筛选值相差约 0.7%；最大超调 5.20%、最坏归一化整定 1.006、最小 PM 73.41°、最大 Ms 1.144、方向/重复差 12.39%/0.57%。安全峰值为 3.229 A、37.51 rpm、0.570 V，最终 READY。

D-031 用同一脚本比较推荐参数和现役 PI，不改变工具推荐：

~~~powershell
python tests/stage2_current_autotune_validation.py `
  --experiment comparison --plan-only
# 当次重新确认 24 V 和机械安全后：
python tests/stage2_current_autotune_validation.py `
  --experiment comparison --port COM_PORT
~~~

默认两轮交换配置顺序，共 165 份 scope；频响扩展到板端允许的 5 kHz。推荐参数必须继续通过 D-029，现役 PI 只作逐项参照。脚本退出时恢复现役 PI 并 STOP。初版曾误列 6/7 kHz，首次运行在 6 kHz 参考有效性检查处安全中止并最终 READY；部分数据不进入对照，当前入口已修正为从头运行。

修正后的 D-031 已完整执行，原始结果为 `results/stage2_current_autotune_hw_20260824_223934.json`，正式摘要为 `baselines/stage2_current_pi_comparison_d031_20260824.json`。推荐参数只有方向一致性失败，最坏方向差 17.40%；其余 D-029 门槛继续通过。它在平均上升、平均整定、总采样稳态 RMS、零点 id 噪声和峰值控制电压上均优于现役，但依照预先声明规则不自动写回。安全峰值为 3.291 A、37.51 rpm、1.621 V，恢复现役 PI 后最终 READY。

D-032 是最后一次有边界的局部鲁棒性迭代。离线生成 850/925/1000 Hz × 80°/85° 六组高相位裕度候选并查看 421 点计划不需要目标或 24 V：

~~~powershell
python tests/stage2_current_autotune.py `
  --stage1 tests/baselines/stage1_20260824.json `
  --frf tests/results/stage2_current_frf_20260824_145248.json `
  --exclude-profile active_legacy --policy d032 `
  --output tests/baselines/stage2_current_autotune_d032_design_20260824.json
python tests/stage2_current_autotune_validation.py `
  --experiment robustness --plan-only
# 当次重新确认 24 V 和机械安全后：
python tests/stage2_current_autotune_validation.py `
  --experiment robustness --port COM_PORT
~~~

该轮完整复用 D-029 门槛。421 点实验已完成，原始结果为 `results/stage2_current_autotune_hw_20260824_225433.json`，正式摘要为 `baselines/stage2_current_autotune_robustness_d032_20260824.json`。三组 PM80 候选通过；三组 PM85 候选只失败目标频率一致性。冻结领先者为 `measured_fc0925_pm80`：968.6 Hz 鲁棒带宽、78.08° PM、Ms 1.135、10.97%/1.70% 方向/重复差。

唯一一次 75 点独立留出使用原 D-029 点集：

~~~powershell
python tests/stage2_current_autotune_validation.py `
  --experiment robustness_holdout --plan-only
# 当次重新确认 24 V 和机械安全后：
python tests/stage2_current_autotune_validation.py `
  --experiment robustness_holdout --port COM_PORT
~~~

留出已完整执行并通过。原始结果为 `results/stage2_current_autotune_hw_20260824_230223.json`，正式摘要为 `baselines/stage2_current_autotune_robustness_holdout_d032_20260824.json`。鲁棒带宽 969.5 Hz、PM 78.81°、Ms 1.137、超调 5.12%、归一化整定 1.016、方向/重复差 11.90%/2.13%；安全峰值 3.520 A、38.68 rpm、0.597 V，最终 READY。该结果支持将 d/q 分轴增益写为默认值，电流环候选搜索结束。

默认值写回后已完成 Keil 构建、J-Link V7.54d Program & Verify、向量表读回和不发送运行时 PI 命令的 ±1 A 回归。结果摘要为 `baselines/stage2_current_pi_default_smoke_d032_20260824.json`，波形为 `results/stage2_d032_default_step_pos_20260824.csv` 与 `results/stage2_d032_default_step_neg_20260824.csv`；两方向超调、归一化整定和 14.49% 方向差均满足 D-029，最终 READY。

上述入口是新方案。下面的 `stage2_current_loop_design.py`、既有网格和现役 A/B 说明保留为历史证据，不再作为新电机自整定必须具有的默认基线。

离线计算不需要连接目标或 24 V：

~~~powershell
python tests/stage2_current_loop_design.py `
  --output tests/baselines/stage2_current_loop_design_20260824.json
~~~

脚本按当前并联 PI 实现使用 `Kp=2πfcL`、`Ki=2πfcRs`。默认保持原 1/1.5/2 kHz 离线基线，也可用 `--bandwidths` 生成任意候选；仿真同时覆盖旧/辨识两组 R-L 参数和 1/2 个 25 µs 采样周期延迟，只用于筛选，不代替硬件阶跃。

先离线查看扩展实验的精确顺序、增益和规模，不需要连接目标或 24 V：

~~~powershell
python tests/stage2_current_loop_validation.py --plan-only
~~~

当前扩展硬件入口为：

~~~powershell
python tests/stage2_current_loop_validation.py --port COM_PORT
~~~

一次运行比较现役 PI 和 0.75/1.0/1.25/1.5/1.75/2.0/2.25 kHz 候选。第一轮按带宽升序，第二轮降序；每次配置先记录零电流噪声，再执行 ±0.5/±1/±1.5/±2/±2.5/±3 A 共 12 个 d 轴阶跃。完整计划为 16 次配置访问、208 份 4096 点/40 kHz 波形，所有 CSV 逐点保存，JSON 在实验失败时也保留已完成记录。

最终选优只评价 |id|≥1 A 点，门槛为：滤波超调 ≤15%、噪声受限整定时间 ≤0.75 ms、稳态 RMS ≤0.08 A、正负上升时间差 ≤20%、两轮上升时间差 ≤15%、超调差 ≤5 个百分点；峰值 dq 电流必须 <4 A、无电压饱和，零电流噪声和控制电压 RMS 不高于现役 20%。这里的整定时间使用 5 点中值滤波，并把 2% 带宽扩展到至少三倍阶跃前滤波噪声 RMS，同时要求连续 0.5 ms 在带内，避免把小电流噪声误认为振荡。

运行中安全门槛仍为 4 A、500 rpm、18–28 V 母线和 98% 电压利用率。|id|≥1 A 时若超调 >30% 或噪声受限整定时间 >2 ms，脚本先保存失败点，再淘汰该配置并跳过更高带宽；最终 15% 选优门槛不会直接因单点噪声中止全场。已知编码器校准瞬态单独使用 5 A 中止阈值。finally 中清零电流、恢复现役 PI、清 PI 状态并 STOP，最终必须确认 READY。

该实验需要当次确认 24 V 已上电；电机可能在编码器校准时短暂转动，但正式阶跃沿转子 d 轴，正常情况下不产生持续转矩。自由转子不适合直接 q 轴阶跃。

可以缩减候选、轮数或电流点进行诊断，例如：

~~~powershell
python tests/stage2_current_loop_validation.py --port COM_PORT `
  --bandwidths 1250 1500 1750 --rounds 1 --steps 0.5 -0.5 1 -1
~~~

2026-08-24 离线包络结果为：现役/1 kHz/1.5 kHz/2 kHz 最坏预测超调约 60.6%/14.8%/21.9%/34.0%，因此硬件从 1 kHz 开始并继续比较 1.5 kHz。

两轮正/反顺序 d 轴硬件结果共 12 点/配置：

| 配置 | ±1/±2 A 平均上升时间 | 平均超调 | 最大超调 | 峰值 dq 电流 |
|---|---:|---:|---:|---:|
| 现役 | 312.5 µs | 9.12% | 13.82% | 2.350 A |
| 1 kHz | 359.4 µs | 7.77% | 12.22% | 2.433 A |
| 1.5 kHz | 215.6 µs | 8.27% | 13.13% | 2.388 A |

1.5 kHz 是当前静止 d 轴首选候选，但尚未写回默认 PI。原始摘要：

~~~text
results/stage2_current_loop_validation_20260824_134831.json
results/stage2_current_loop_validation_20260824_134936.json
baselines/stage2_current_loop_20260824.json
~~~

`stage2_q_current_pulse.py` 的首个 +0.5 A、目标 5 ms q 轴脉冲记录到约 930.7 rpm，触发 500 rpm 边界并在后续脉冲前 STOP。因此在当前低惯量自由转子上不再直接运行该脚本；它保留为实验边界实现，不是推荐的常规入口。后续改用速度环接管 q 轴转矩：三组 PI 在 ±1500/±3000 rpm 均稳定、无电压饱和，但 1.5 kHz 的高速 iq 误差 RMS 0.127 A 与现役 0.122 A 基本相当，id RMS 0.245 A 高于现役 0.195 A。扩展网格和频响完成后，2.0 kHz 已进入 dq 前馈组合验证队列，仍未直接写回。

2026-08-24 扩展网格已完整执行，原始结果为 `results/stage2_current_loop_grid_20260824_143145.json`。预定义逐点最大相对差因 25 µs 采样量化和现役零点基线首末变化而没有选出全通过者；不要删除或改写该结论。可复现的二级分析入口为：

~~~powershell
python tests/stage2_current_loop_analysis.py `
  tests/results/stage2_current_loop_grid_20260824_143145.json `
  --output tests/baselines/stage2_current_loop_grid_20260824.json
~~~

二级分析对交越时间做线性插值，并以每轮正负均值、两轮均值和两轮平均零点噪声处理高速量化。1.25–2.0 kHz 通过汇总筛选，2.0 kHz 是最高通过者；2.25 kHz 的聚合正负差 23.4%、平均 id 噪声相对现役 1.220，作为频响边界点。该结果只决定下一批频响候选，不授权写回 PI。

### d/q 轴闭环频响

离线查看频响计划不需要目标或 24 V：

~~~powershell
python tests/stage2_current_frequency_response.py --plan-only
~~~

默认比较现役、1.5/1.75/2.0/2.25 kHz，频点为 100–4000 Hz 共 16 点，正弦幅值 0.5 A，总计 80 份 40 kHz scope。硬件入口为：

~~~powershell
python tests/stage2_current_frequency_response.py --port COM_PORT
~~~

固件命令 `CURRENT SINE D/Q amplitude frequency` 在 FOC 中断内用递推振荡器生成同步 d/q 轴参考，旧两参数命令保持为 d 轴兼容入口，scope 二进制布局不变。主机按所选轴对每个通道做带常数项的最小二乘正弦/余弦拟合，计算闭环 `T=I/Iref`、环路 `L=T/(1-T)`、相对低频平台的 -3 dB 闭环带宽、环路交越、相位裕量和实测灵敏度；只有扫频内实际观察到 -180° 相位交越时才报告增益裕量，不能用缺失值冒充通过。

脚本限制参考幅值不超过 0.5 A，逐点检查记录参考幅值、2 A 峰值电流和 500 rpm；每点先停止正弦再导出 scope，异常退出时停止正弦、清零、恢复现役 PI 并 STOP。该实验需要当次确认 24 V 和转子自由，但正常 d 轴正弦不应持续旋转。

2026-08-24 的 80 点硬件频响全部完成，原始结果为 `results/stage2_current_frf_20260824_145248.json`。可复现的阶跃+频响联合摘要入口为：

~~~powershell
python tests/stage2_current_frf_analysis.py `
  tests/results/stage2_current_frf_20260824_145248.json `
  tests/baselines/stage2_current_loop_grid_20260824.json `
  --output tests/baselines/stage2_current_frf_20260824.json
~~~

1.5/1.75/2.0/2.25 kHz 的实测闭环带宽约 1.54/1.95/2.41/2.91 kHz，相位裕量约 84.5°/82.2°/79.6°/77.4°。联合规则选择 2.0 kHz 进入旋转前馈验证；2.25 kHz 保留频域边界。4 kHz 内未观察到 -180° 相位交越，因此增益裕量仍是未测得，而不是通过。

### 旋转组合验证

离线查看最终阶段 2 旋转计划不需要目标或 24 V：

~~~powershell
python tests/stage2_current_combination_validation.py --plan-only
~~~

硬件入口为：

~~~powershell
python tests/stage2_current_combination_validation.py --port COM_PORT
~~~

实验保持最小三组对照：现役 PI/前馈关闭、现役 PI/IDENTIFIED 前馈、2 kHz PI/IDENTIFIED 前馈。第一轮按上述顺序，第二轮反序；速度轨迹分别覆盖 `0 → +3000 → +1500 → -1500 → -3000 → 0 rpm` 及其反向，斜坡默认 3000 rpm/s。每次配置访问在 ±1500 rpm 换向起始处记录一份 4096 点/40 kHz scope，用 500 Hz 遥测覆盖完整斜坡和稳态，因此既能分离前馈收益，也能在相同前馈下比较两组 PI。

运行边界为 3 A dq 矢量、3500 rpm、18–28 V 和 0.90 最低电压缩放；最终评价另要求电压缩放不低于 0.98。2 kHz 候选相对“现役 PI/IDENTIFIED 前馈”的速度、iq、id 误差允许 10%–15% 相对差，并设置 0.01–0.02 A/10–20 rpm 的绝对分辨率余量；每次 scope 的峰值 iq 参考至少应达到 0.2 A，否则判为激励不足而不是通过。正常和异常退出都会 STOP，并在 READY 下恢复现役 PI、LEGACY 模型和前馈关闭状态。

该实验会让电机多次加减速和正反转到 3000 rpm。运行前必须由用户当次确认 24 V 已上电、转子可自由双向旋转且周边安全；不能沿用旧消息中的上电状态。

2026-08-24 的硬件实验完成六次配置访问、30 个速度转换和六份 40 kHz scope，最终恢复现役 PI、关闭前馈并确认 READY。离线决策可复现为：

~~~powershell
python tests/stage2_current_combination_analysis.py `
  tests/results/stage2_current_combination_20260824_151007.json `
  --output tests/baselines/stage2_current_combination_20260824.json
~~~

相同 IDENTIFIED 前馈下，2 kHz 相对现役 PI 的斜坡 iq 误差只变化 +0.31%，速度误差变化 +3.28%，没有形成动态优势；斜坡 id、稳态 id 和 scope id 误差分别增加 11.1%、29.5% 和 22.1%，且两轮方向一致。预定义 scope 峰值 iq 参考仅约 0.122–0.126 A，低于 0.2 A 激励门槛，因此不能宣称 q 轴全局优劣；但“没有一致收益且旋转 id 误差重复增加”已经不支持自动写回 2 kHz，现役 PI 保留为默认值。

现役 PI 下启用 IDENTIFIED 前馈把平均 PI 电压从 1.872 V 降到 0.216 V，降低 88.5%；斜坡/scope id 误差也分别下降约 8.8%/5.7%。这再次确认前馈功能有价值，但 LEGACY/IDENTIFIED 模型在阶段 1 中仍不可可靠区分，默认模型和默认启用策略留给后续独立决策。

## Python 环境

硬件脚本使用 Python 3 和 pyserial。所有命令从仓库根目录执行，输出默认写入 tests/results。

先查看帮助：

~~~powershell
python tests/stage0_validation.py --help
python tests/scope_capture.py --help
python tests/scope_jlink_capture.py --help
~~~

查看帮助不访问硬件。实际运行命令前必须确认电源状态。

## 阶段 0 连续验证

默认命令形式：

~~~powershell
python tests/stage0_validation.py --port COM_PORT
~~~

COM 号是本机当前枚举值。重新插拔或打开失败后应先确定当前端口。

默认序列：

1. 等待 READY，并检查母线电压；
2. 前馈关闭，进入电流模式；
3. 按 iq:id 格式测试 0:0、0:+0.2 A、0:-0.2 A、0:0，即 iq 保持 0、id 做 ±0.2 A 小阶跃；
4. 停止并确认 READY；
5. 进入速度模式；
6. 在前馈 OFF 和 ON 下分别测试 500、1500、3000 rpm，每点后回零；
7. 停止并确认 READY；
8. 进入位置模式；
9. 以当前位置为原点执行 +2°、-2°、+5°、0° 偏移；
10. finally 中发送 STOP，并要求最终遥测确认 READY。

默认保护阈值：

- 最大电流 3 A；
- 最大速度 4000 rpm；
- 母线电压 18–28 V；
- 串口 115200。

这些阈值是脚本当前默认值，不是硬件绝对额定值，也不授权自动扩大测试范围。修改点位、速度或阈值前应与用户讨论。

新版本脚本写出：

- 主遥测 CSV；
- 同名 JSON 指标报告；
- 同名 _scope 目录中的各阶段高速波形。

2026-08-24 01:05:38 的历史运行发生在 JSON 报告写入修复之前，因此主要证据是 CSV 和 _scope 目录。

## 速度环经验自整定准备

速度环 v1.0 的固定规则见 `docs/SPEED_LOOP_AUTOTUNING.md`，机器可读基线为 `baselines/stage2_speed_autotune_spec_v1_20260824.json`。`stage2_speed_autotune.py` 已能离线生成 P 扫描，并可在提供 `Kp_base/tau` 后预览六组 PI：

~~~powershell
python tests/stage2_speed_autotune.py
python tests/stage2_speed_autotune.py --kp-base 0.002 --tau-63 0.04
~~~

默认 P 扫描得到 `Kp0=0.00095493`，六组预测初始电流为 0.5–3.797 A；速度环专项主机回归现为 6 项通过。当前没有新速度硬件结论。第一批连续硬件入口已实现：

~~~powershell
python tests/stage2_speed_autotune_validation.py --plan-only
python tests/stage2_speed_autotune_validation.py --port COM_PORT
~~~

该脚本一次 START 完成零速噪声、最多 12 份 P 扫描 scope 和 48 份 PI 筛选 scope，候选间只切换易失 PI 并清状态；安全或异常退出都会发送 STOP 并确认 READY。第一批将完成：

1. `Ki=0` 的最多六组 1.5 倍几何 P 扫描；
2. 由 `Kp_base` 和 63.2% 响应时间生成六组 PI；
3. `±500/±1500 rpm` 两轮筛选；
4. finally STOP、READY 确认和完整 CSV/JSON 保存。

合格者正反 3000/4000 rpm 斜坡、高速局部阶跃、领先者频响护栏和唯一留出将在第一批数据确认指标提取可靠后接入同一监督器，不提前把全部阶段塞入首次运动实验。固件已提供参与反积分饱和的 `SPEED LIMIT`、同步 `SPEED SINE`、`speed_ref` scope 通道和完整 `PI CLEAR`。烧录及运动实验仍必须在当次明确确认供电和机械安全后执行。

固定实验边界：`iq_ref` 限幅 4 A，实际 dq 电流 5 A、机械转速 4800 rpm、电压利用率 98% 时中止。性能标准只有超调、整定/稳态误差、方向/重复一致性三项；安全阈值和最终带宽护栏不作为加权评分。

## 软件示波器

当前协议：

- 11 个 float32 通道；
- 4096 个样本；
- 基准采样率 40 kHz；
- 分频只允许 1、10、100；
- 先写 D2 SRAM，捕获完成后再传输。

通道顺序必须与固件一致：

1. ia
2. ib
3. iq
4. id
5. iq_ref
6. id_ref
7. uq
8. ud
9. elec_angle
10. elec_speed
11. speed_ref

串口单次采集：

~~~powershell
python tests/scope_capture.py --port COM_PORT --divider 1
~~~

分频选择：

| divider | 采样率 | 记录时长 | 适合观察 |
|---:|---:|---:|---|
| 1 | 40 kHz | 102.4 ms | 电流环和快速瞬态 |
| 10 | 4 kHz | 1.024 s | 速度环和短位置过程 |
| 100 | 400 Hz | 10.24 s | 慢速轨迹和稳态漂移 |

不要在 FOC ISR 中直接打印或持续发送串口。软件示波器的设计目的就是把实时采样和低速导出分离。

## J-Link 后备读取

串口不可用时：

~~~powershell
python tests/scope_jlink_capture.py --divider 1 --probe-serial PROBE_SERIAL
~~~

脚本默认使用：

    C:\Program Files\SEGGER\JLink\JLink.exe

即项目选定的 V7.54d，并通过 mdk/build/STM32H743VGT6_FOC.map 查找 foc_scope_control 和 foc_scope_buffer。

使用前必须：

- 重新构建并确认 MAP 与当前固件匹配；
- 确认目标板已供电；
- 告知用户将连接 J-Link；
- 避免同时运行另一个 J-Link 会话。

J-Link 是后备路径。串口已经成功导出时，没有必要为了形式完整再读一次相同波形。

## 结果和证据

当前主要基线：

- results/stage0_validation_20260824_010538.csv
- results/stage0_validation_20260824_010538_scope/
- results/scope_current_step_20260824_010145.csv
- results/position_validation_20260824_001353.csv

阶段 0 结果摘要见 docs/PROJECT_STATUS.md。分析时应引用原始文件，不只复制聊天中的数值。

结果命名使用时间戳。同一次批量测试的主 CSV、JSON 和 _scope 目录应共享同一 stem，便于追溯。

## 运行前检查

1. 当前源码已经重新构建并烧录。
2. 固件与脚本的命令、遥测帧和软件示波器协议匹配。
3. 用户已确认 24 V 上电和电机可自由转动。
4. 串口未被其他程序占用。
5. 电源限流、最大测试电流和最大速度符合本次计划。
6. 输出目录可写，磁盘空间足够。

未重新插拔且串口正常时，不重复扫描端口。

## 运行后检查

1. 脚本最终确认 READY；若没有确认，立即通知用户并判断是否断电。
2. 保存控制台异常、主 CSV、JSON 和 scope 数据。
3. 检查是否有缺样、通道布局不匹配或保护阈值触发。
4. 用同一指标比较基线和新算法。
5. 只有形成能影响后续工作的结论时，才更新 docs/PROJECT_STATUS.md 或 docs/DECISIONS.md。

## 当前待完善项

- `tests/results`继续保存本地原始证据，`tests/baselines`保存精选机器摘要；外部归档方案确定前不批量删除文档引用的数据。
- 为实验报告增加固件构建标识，避免把同名脚本的不同固件结果混在一起。
- 阶段3分项先复用11通道Scope和36通道遥测；D-058因限幅恢复存在明确可观测缺口，只在末尾追加 `voltage_scale`形成12通道，24份板端Scope已验证该布局。
- 阶段3不再追加饱和网格；下一次板端动作只应验证LEGACY+BOTH复位默认写回，之后按 [阶段3研究大纲](../docs/STAGE3_DECOUPLING_AND_VOLTAGE.md) 的退出条件转入阶段4。

## ABZ 角度与速度估算离线回归

角度计数归一化的 C++ 回归已经并入统一算法测试：

~~~powershell
./tests/run_algorithm_regression.ps1
~~~

它覆盖 TIM3 `0..4095` 的全部角度步长和回绕，确认换算分母为 `ARR+1`。速度估算专项回归为：

~~~powershell
python -m unittest tests/test_encoder_speed_estimation.py -v
~~~

单份既有 scope 可离线重放当前 12.5 ms BASE、短窗+IIR和编码器 PLL：

~~~powershell
python tests/encoder_speed_estimation.py `
  tests/results/stage2_speed_frf_20260825_004259_scope/speed_sine_20hz.csv `
  --win-window-ms 1 --win-cutoff-hz 100 --pll-bandwidth-hz 100
~~~

输出中的 `recorded_reproduction_rmse_rad_s` 用于检查 BASE 是否复现现役固件速度，不能用于宣称新估算器更准确。既有 CSV 没有原始边沿时间，M/T 必须等待最小边沿观测设计；闭环记录也由 BASE 参与控制，只能用于离线初筛，不能替代上板 A/B。

统一合成扫掠和机器可读基线可用以下命令重建：

~~~powershell
python tests/encoder_speed_estimation_sweep.py `
  --output tests/baselines/encoder_speed_estimation_synthetic_20260825.json
~~~

当前首批硬件候选固定为 BASE、1 ms+150 Hz WIN、200 Hz PLL。无需目标即可检查完整实验计划：

~~~powershell
python tests/encoder_speed_estimation_validation.py --plan-only
~~~

实际实验入口为：

~~~powershell
python tests/encoder_speed_estimation_validation.py --port COM_PORT
~~~

该命令会让烧录后的电机在速度闭环下自动正反转，覆盖 `±50/±500/±1500/±3000 rpm` 和 100 rpm 幅值的 20/40/80 Hz 正弦，两轮反序共 72 份 scope。运行前必须当次确认 24 V 已上电、转子可自由双向旋转且周围安全；脚本正常或异常退出都尝试 STOP、确认 READY，并在 D-037 后恢复当前默认 WIN。

D-035 原始数据和机器可读聚合入口为：

~~~powershell
python tests/encoder_speed_estimation_analysis.py `
  tests/results/encoder_speed_ab_20260825_154216.json `
  --output tests/baselines/encoder_speed_estimation_hw_20260825.json
~~~

该批次确认 BASE 的 80 Hz 陷波，但零均值机械摆动存在顺序依赖。D-036 持续旋转偏置留出可先离线展开：

~~~powershell
python tests/encoder_speed_estimation_biased_holdout.py --plan-only
~~~

实际入口为：

~~~powershell
python tests/encoder_speed_estimation_biased_holdout.py --port COM_PORT
~~~

它在 `+1500/-1500 rpm` 上叠加 100 rpm、20/40/80 Hz 小信号，BASE/WIN/PLL 两轮反序共 36 份 scope，最高参考 1600 rpm。它仍是自动运动实验，必须在当次确认 24 V、自由转子和周围安全后运行。

D-036 已完成：原始结果为 `tests/results/encoder_speed_bias_20260825_155849.json`，聚合入口为：

~~~powershell
python tests/encoder_speed_estimation_biased_analysis.py `
  tests/results/encoder_speed_bias_20260825_155849.json `
  --output tests/baselines/encoder_speed_estimation_biased_holdout_20260825.json
~~~

WIN 的两轮最大幅值/相位差为 `0.61 dB/3.11°`，正反方向最大差为 `0.33 dB/1.63°`，且平均 `iq_ref` RMS 不高于 PLL，因此 D-037 已将 ABZ 复位默认冻结为 WIN。默认值上板冒烟入口为：

~~~powershell
python tests/encoder_speed_default_smoke.py --port COM_PORT
~~~

该脚本故意不发送 `SPEED EST`，只执行 +1500 rpm 偏置、100 rpm/80 Hz 单点并以 BASE 陷波作辨别门限。2026-08-25 结果为 `tests/results/encoder_speed_default_smoke_20260825_160736.json` 与同名 CSV，版本化摘要为 `tests/baselines/encoder_speed_default_smoke_20260825.json`：幅值 `-19.73 dB`、峰值 dq 电流 `0.596 A`、最终 READY。它只验证复位默认值生效，不新增速度环性能验收标准。

D-038 在冻结 WIN 后测现役速度闭环基线：

~~~powershell
python tests/stage2_speed_baseline.py --plan-only
python tests/stage2_speed_baseline.py --port COM_PORT
~~~

脚本固定当前默认速度 PI，在 `±1500 rpm` 上叠加 100 rpm、`1/2/4/6/8/10/15/20/30/40/60 Hz` 正弦，共 22 份 scope。带宽按相对单位直流增益的绝对 -3.0103 dB 交点计算，并报告闭环峰值、方向幅相差和电流代价；它不调整 WIN 参数、不筛选 PI。实际入口是自动旋转实验，执行前必须当次确认 24 V、自由转子和周围安全。

D-038 正式结果为 `tests/results/stage2_speed_baseline_20260825_162540.json`，22 份 CSV 位于同名 `_scope` 目录，版本化摘要为 `tests/baselines/stage2_speed_baseline_d038_20260825.json`。正反带宽为 `10.29/10.12 Hz`，4 Hz 峰值 `+1.58/+1.61 dB`，最大方向差 `0.31 dB/2.82°`；峰值 dq 电流 `0.813 A`、峰值 `iq_ref=0.339 A`，最终 READY。WIN 模型在 10 Hz 只有约 `-0.021 dB/-5.52°`，所以当前约 10.2 Hz 带宽不是测速滤波器造成的。

D-039 从上述闭环和已知现役 PI 反算实测对象，并生成闭环带宽目标
`15/20/25/30/35/40 Hz`、65°相位裕度的六组候选：

~~~powershell
python tests/stage2_speed_pi_design.py
python -m unittest tests/test_stage2_speed_pi_design.py -v
~~~

机器结果为 `tests/baselines/stage2_speed_pi_design_d039_20260825.json`。这是离线计算，
不需要目标或 24 V。D-040 的连续硬件频率资格验证可先离线查看：

~~~powershell
python tests/stage2_speed_pi_frequency_validation.py --plan-only
~~~

实际入口为：

~~~powershell
python tests/stage2_speed_pi_frequency_validation.py --port COM_PORT
~~~

它固定 WIN、前馈关闭和 4 A 速度输出限幅，在 `+1500/-1500 rpm` 偏置上对六组候选
扫描 100 rpm 幅值的 `4/8/10/15/20/25/30/35/40/50/60 Hz`，共 132 份 scope。
验收只含四项：双方向带宽预测误差各 `<=20%`、闭环峰值 `<=2 dB`、方向带宽差
`<=15%`、`iq_ref` 不接近限幅。脚本退出时 STOP/READY并恢复源码默认速度 PI。它
不会烧录、不会写入默认参数；实际运行仍必须当次确认 24 V、自由转子和周围安全。

D-040 正式结果为
`tests/results/stage2_speed_pi_frequency_validation_20260825_165849.json`，132 份 CSV 位于
同名 `_scope` 目录，版本化摘要为
`tests/baselines/stage2_speed_pi_frequency_validation_d040_20260825.json`。15 Hz 候选
实测 `14.26/13.95 Hz`，20 Hz 候选实测 `17.57/17.27 Hz`，两组通过；25–40 Hz候选
没有达到各自目标，且 40 Hz 方向带宽差为 `17.25%`。全批峰值 `iq_ref=0.981 A`、
实际 dq `1.415 A`，排除 4 A 限幅影响。第一次 104 份 CSV 的写超时批次不参与正式
结论；正式批次最终 READY，并已在 READY 下单独确认恢复 WIN 和源码默认速度 PI。

D-041 不直接试 100/150 Hz 高带宽 PI，而是先用 D-040 已保持稳定的 20/40 Hz目标 PI
测量 40–150 Hz 对象边界。离线查看计划和专项回归不需要目标或 24 V：

~~~powershell
python tests/stage2_speed_high_bandwidth_probe.py --plan-only
python -m unittest tests/test_stage2_speed_high_bandwidth_probe.py -v
~~~

实际入口为：

~~~powershell
python tests/stage2_speed_high_bandwidth_probe.py --port COM_PORT
~~~

脚本在 `±1500/±3000 rpm` 持续旋转偏置上叠加 `100/300 rpm`，扫描
`40/50/60/70/80/100/120/150 Hz`；两组 PI、16 次访问共 128 份 scope。它检查参考幅值
误差 `<=2%`、速度正弦 SNR `>=6 dB`，并要求对象在控制器、幅值、速度和方向之间的
成对差异不超过 `2 dB/10°`，随后只按对象相位把普通 PI可行区分为绿/黄/红。本批不
排名速度 PI、不写默认、不烧录；实际运行会旋转电机，必须当次确认 24 V、自由转子和
周围安全。

每采完一份 scope 都会更新结果 JSON。若 USB/串口中断且已经确认电机停止，可用原结果
续跑未完成点：

~~~powershell
python tests/stage2_speed_high_bandwidth_probe.py --port COM_PORT --resume tests/results/stage2_speed_high_bandwidth_probe_YYYYMMDD_HHMMSS.json
~~~

正常或异常退出均尝试 STOP/READY；确认 READY 后重新打开串口恢复 WIN 和源码默认速度
PI。只有完整 128 点且默认恢复成功时脚本才返回成功。

2026-08-25 硬件结果为
`tests/results/stage2_speed_high_bandwidth_probe_20260825_180920.json`，128 份 scope 完整，
最终 READY且默认 PI恢复。两次 Windows CDC `ClearCommError` 后均从检查点续跑；峰值
`iq_ref=2.787 A`、实际 dq `3.494 A`、转速 `3281.6 rpm`。

严格全组没有合格频率：每个频率的 16 点都含 SNR `<6 dB` 的弱测量组合，门槛没有
后验修改。离线诊断入口为：

~~~powershell
python tests/stage2_speed_high_bandwidth_probe_analysis.py
python -m unittest tests/test_stage2_speed_high_bandwidth_probe_analysis.py -v
~~~

版本化结果为
`tests/baselines/stage2_speed_high_bandwidth_probe_d041_20260825.json`。固定但仅作探索的
`bw40_pm65 + 300 rpm` 子集在 40–100 Hz四个速度偏置全部满足 SNR；其相位中值为
`-105/-118/-126/-136/-146/-164°`。40 Hz绿色且跨速度重复通过，50/60 Hz黄色，其中
60 Hz重复通过；70 Hz以上为红色或一致性失败。这形成了下一步候选的相位边界；D-042
随后否决了直接闭环带宽命名方案，不测试 100/150 Hz普通 PI。

D-042 离线设计入口：

~~~powershell
python tests/stage2_speed_high_bandwidth_pi_design.py
python -m unittest tests/test_stage2_speed_high_bandwidth_pi_design.py -v
~~~

最初的 `40/50/60 Hz闭环目标 × 55°/60°` 六组因预测峰值 `2.69–3.56 dB` 或 PM不足
全部被离线预筛拒绝。正式设计改为 `22/24/26/28/30/32 Hz` 开环交越、70°名义 PM；
机器结果为 `baselines/stage2_speed_high_bandwidth_pi_design_d042_20260825.json`，六组预测
最小 PM `62.41–69.68°`、最大峰值 `1.79–2.00 dB`、200 rpm最大电流幅值
`1.58–2.31 A`，全部通过预筛。闭环带宽不由离线模型命名。

D-042 硬件计划可离线查看：

~~~powershell
python tests/stage2_speed_high_bandwidth_pi_validation.py --plan-only
python -m unittest tests/test_stage2_speed_high_bandwidth_pi_validation.py -v
~~~

实际入口为：

~~~powershell
python tests/stage2_speed_high_bandwidth_pi_validation.py --port COM_PORT
~~~

计划包含 `bw20_pm65/bw40_pm65` 两个只报告锚点和六组候选，在
`±1500/±3000 rpm`、200 rpm幅值下扫描14个 `4–100 Hz` 频点，共448份scope。四条资格
标准为交越误差 `<=20%`、PM `>=60°`、峰值 `<=2 dB`、四速度带宽范围 `<=15%`；参考
幅值、交越两侧 SNR和电流限幅属于有效性/安全。脚本支持逐scope检查点和 `--resume`，
退出 STOP/READY并恢复默认；不烧录、不写默认。实际运行前仍须当次确认24 V、自由转子
和周围安全。

2026-08-25 硬件结果为
`tests/results/stage2_speed_high_bandwidth_pi_validation_20260825_184830.json`，448份scope
完整，最终 READY且源码默认速度 PI恢复；版本化摘要为
`baselines/stage2_speed_high_bandwidth_pi_validation_d042_20260825.json`。安全峰值
`iq_ref/实际dq/转速` 为 `2.816 A/3.377 A/3352.8 rpm`。

冻结资格列表为空。数据完整的 `fc24_pm70` 通过交越目标、PM和峰值三项，但四速度
闭环带宽 `53.15/47.59/55.28/44.58 Hz` 的相对范围为 `21.34%`，超过15%门槛。
`fc26` 有一处带宽交点 SNR不足且原始峰值/一致性略超线；`fc28/fc30` 在 +3000 rpm
降到约54° PM；`fc32` 的 +3000 rpm -3 dB点超出100 Hz扫频上限。不要用本结果直接
写默认 PI，也不要通过修改冻结门槛制造通过者。

D-043 收尾 A/B只比较源码默认 PI与 `fc24_pm70`。离线查看和专项回归不需要硬件：

~~~powershell
python tests/stage2_speed_final_ab_validation.py --plan-only
python -m unittest tests/test_stage2_speed_final_ab_validation.py -v
~~~

实际入口为：

~~~powershell
python tests/stage2_speed_final_ab_validation.py --port COM_PORT
~~~

脚本执行两组参数各两次反序访问。每次先用3000 rpm/s斜坡连续覆盖正反
500/1500/3000/4000 rpm，再采集 `0↔±1500 rpm` 大阶跃和 `±3000 rpm`附近300 rpm
局部阶跃；总计32份scope，并保存四份全范围串口遥测 CSV。大阶跃专门报告4 A贴限时长
及退出限幅后的超调/恢复，用于验证现有积分限幅和饱和回算是否够用。实际运行会高速
双向旋转，必须当次确认24 V、自由转子和周围安全；脚本不烧录、不写默认，退出时
STOP/READY并恢复源码默认速度 PI。

首批结果为 `tests/results/stage2_speed_final_ab_20260825_193405.json`。32份阶跃全部有效，
`fc24/default` 最大噪声修正超调 `7.44%/25.02%`、最坏整定
`86.75/220.25 ms`、最坏90%时间 `33.16/48.96 ms`。`fc24` 的峰值
`iq_ref/实际dq=4.000/4.207 A`，贴限最长3.25 ms后恢复，电压缩放始终为1；最终
READY且默认 PI恢复。
版本化阶跃摘要为 `baselines/stage2_speed_final_ab_d043_steps_20260825.json`。

首批四份斜坡CSV不作性能证据：多个阶段的遥测外部速度目标没有与脚本请求同步，阶段
标签与实际轨迹错位。脚本现会在每条 `SPEED` 后确认遥测通道24，未匹配即中止。补测
只运行斜坡，不重复有效阶跃：

~~~powershell
python tests/stage2_speed_final_ab_validation.py --port COM_PORT --ramps-only
~~~

修正版补测结果为 `tests/results/stage2_speed_final_ab_20260825_195257.json`，4/4次访问完整，
最终READY。`fc24/default` 平均斜坡RMS误差为 `23.90/30.08 rpm`，最坏为
`47.09/54.62 rpm`，峰值实际dq为 `1.272/1.064 A`，无电压饱和。最终速度环摘要为
`tests/baselines/stage2_speed_final_ab_d043_20260825.json`，源码默认已改为fc24。

位置环 D-044 入口为：

~~~powershell
python tests/stage2_position_tuning.py --port COM_PORT
~~~

固件提供运行时 `PI POSITION kp ki kd` 和READY下的 `POS SPEED LIMIT rpm`。首轮历史
默认与4/6/8/10 Hz四组P候选共120个阶跃已经完成；后续固定机械位置p4/p6/p8留出均
未通过10%超调和200 ms整定门槛。`--p4-diagnostic` 用于已完成的低增益同位置复核，
`--trace-diagnostic` 保存p6小阶跃的JSON汇总和原始CSV时序。所有入口正常或异常退出
均STOP并确认READY。结论和证据索引见
`baselines/stage2_position_p_grid_d044_20260825.json`；当前不再扩大纯P候选网格。

D-045轨迹基线与前馈A/B入口：

~~~powershell
python tests/stage2_position_tuning.py --port COM_PORT --trajectory-validation
python tests/stage2_position_tuning.py --port COM_PORT --trajectory-feedforward-ab
~~~

矩阵固定为180/720 deg/s下 `±10°/±30°/±180°`及回零。A/B在同一固件内通过READY下
`POS FF OFF/ON`切换，关闭组按180→720、开启组按720→180执行；退出恢复OFF并确认
READY。两组分别保存 `_ff_off.csv` 与 `_ff_on.csv`，不能用尚未包含 `POS FF` 命令的
旧固件执行A/B。

D-045实测结果为 `tests/results/stage2_position_tuning_20260825_204729.json`：OFF/ON各24段
完整、最终READY并恢复OFF。前馈使最大等效跟踪滞后30.45→12.58 ms，但最大端点过冲
3.71→7.00°，因此不写默认ON。版本化摘要为
`baselines/stage2_position_velocity_feedforward_d045_20260825.json`。后续不在当前速度
不连续的RAMP上继续扫前馈比例；若采用梯形轨迹，应建立新的D-046入口和独立结果文件。

D-046梯形/三角形轨迹离线计划：

~~~powershell
python tests/stage2_position_tuning.py --plan-only
python -m unittest tests.test_stage2_position_tuning -v
~~~

硬件连续A/B入口为：

~~~powershell
python tests/stage2_position_tuning.py --port COM_PORT --trajectory-trapezoid-ab
~~~

固件命令 `POS TRACK TRAP velocity acceleration` 同时限制机械角速度和机械角加速度。
脚本固定100 ms加速时间，即180/720 deg/s分别使用1800/7200 deg/s²；仍覆盖
`±10°/±30°/±180°`及回零。前馈OFF按180→720、ON按720→180执行，共48段；每段保存
轨迹模式与加速度，输出 `_trapezoid_ff_off.csv` 和 `_trapezoid_ff_on.csv`。退出恢复
`POS FF OFF`、`POS TRACK RAMP 180`并确认READY。执行前必须使用包含D-046命令的新固件，
并当次确认24 V、自由转子和周围安全。

D-046实测结果为 `tests/results/stage2_position_tuning_20260825_211631.json`，前馈OFF/ON
各24段完整并最终READY。相对D-045 RAMP，TRAP OFF合格段6→8但最大过冲3.71→3.80°；
TRAP ON合格段4→9、最大过冲7.00→5.38°。720 deg/s ON子集最大滞后5.59→2.94 ms，
峰值实际dq1.136→0.556 A。100 ms加速方案未通过0.5°端点门槛，不写默认；机器摘要为
`baselines/stage2_position_trapezoid_d046_20260825.json`。

D-047只把加速时间改为200 ms，运行同一48段矩阵：

~~~powershell
python tests/stage2_position_tuning.py --plan-only --trapezoid-acceleration-time-sec 0.20
python tests/stage2_position_tuning.py --port COM_PORT --trajectory-trapezoid-ab --trapezoid-acceleration-time-sec 0.20
~~~

对应加速度为900/3600 deg/s²。脚本只接受冻结的0.10/0.20 s两档；D-047完成后不追加
第三个加速时间。使用现有D-046固件即可，不需要重烧；实际运行仍须当次确认24 V、自由
转子和周围安全，退出恢复前馈OFF、RAMP 180并确认READY。

D-047实测结果为 `tests/results/stage2_position_tuning_20260827_191645.json`，48/48段完成并
最终READY。200 ms下FF OFF/ON分别通过9/24和12/24，最大过冲3.54°/3.97°，最大滞后
25.06/10.21 ms。180 deg/s FF ON为当前最佳候选（8/12通过、最大过冲3.01°、最坏到位
188 ms），但仍未过0.5°全矩阵门槛，故不自动写默认，也不追加第三个加速时间。机器摘要
为 `baselines/stage2_position_trapezoid_d047_20260827.json`。

D-048零速位置伺服诊断入口：

~~~powershell
python tests/stage2_zero_speed_position_servo.py --plan-only
python -m unittest tests.test_stage2_zero_speed_position_servo -v
python tests/stage2_zero_speed_position_servo.py --port COM_PORT
~~~

连续硬件入口先运行 `±5/±10/±20/±50 rpm` 两轮反序低速跟踪，再以10°间隔覆盖一圈
36个位置，从递增/递减方向各接近两轮，共144个保持点。脚本输出原始CSV与JSON摘要，
把正反平均解释为位置相关等效分量、正反差解释为方向效应；第一批不拟合或下发齿槽
补偿。现有固件接口足够，不烧录；运行需要当次确认24 V、自由转子和周围安全，正常或
异常退出均STOP/READY并恢复WIN、现役PI、`POS FF OFF`和 `POS TRACK RAMP 180`。

D-048正式结果为 `tests/results/stage2_zero_speed_servo_20260827_193356.json` 和同名CSV：
20个低速段、144个位置保持点完整，最终READY并恢复默认。低速均值双向正确到5 rpm，
但最坏速度纹波在5/10/20/50 rpm分别为8.69/10.57/13.39/21.20 rpm RMS；保持位置
绝对平均误差中位数6.88°，WIN与位置差分都确认真实往复摆动。正反两方向的同向重复
相关系数仅0.384/0.068，因此停止齿槽图谱，不把本批角度曲线用于补偿。机器摘要为
`baselines/stage2_zero_speed_position_servo_d048_20260827.json`。

D-049低速速度伺服交叉诊断入口：

~~~powershell
python tests/stage2_low_speed_servo_ab.py --plan-only
python -m unittest tests.test_stage2_low_speed_servo_ab -v
python tests/stage2_low_speed_servo_ab.py --port COM_PORT
~~~

脚本完整比较 `BASE/WIN/PLL × fc24/bw20` 六组。每组执行
`+5/+10/+20/0/-5/-10/-20/0 rpm` 和 `+10/0/-10/0°` 长保持，总计48个低速段、24个
位置段。CSV额外保存ABZ电角度；摘要使用20 ms角度差分评价真实机械运动，并同时报告
待测测速器输出，避免平滑滤波造成自证偏差。现有固件足够且不烧录；硬件运行前需重新
确认24 V、自由转子和周围安全，退出STOP/READY并恢复WIN、现役PI和默认位置配置。

D-049首次预运行文件 `tests/results/stage2_low_speed_servo_ab_20260827_194912.json` 在第一组
开始后即因旧WIN状态触发250 rpm保护，0个有效记录，不作性能证据。READY下WIN冻结为
-229.7 rpm，而ABZ原始角度3 s差分为0 rpm。运行完整矩阵前必须使用包含D-050的固件；
该固件在MODE_CLEAR清编码器测速状态。脚本的异常STOP确认也已改为不检查READY冻结
速度，专项现为4项通过。

D-050最终版的START/STOP/START冒烟已通过；D-049正式结果为
`tests/results/stage2_low_speed_servo_ab_20260827_200222.json`和同名CSV，48个速度段、24个
位置段完整，最终READY并恢复默认。三种测速器使用bw20时纯速度纹波均明显差于fc24；
fc24下WIN/PLL纯速度相当，因此默认保持WIN和D-043 fc24。D-051后来确认位置段继承了
`TRACK RAMP 100`并与位置轨迹重复整形，位置列不再用于比较。机器摘要为
`baselines/stage2_low_speed_servo_ab_d049_20260827.json`。

D-051位置端点重复性入口：

~~~powershell
python tests/stage2_position_endpoint_repeatability.py --plan-only
python -m unittest tests.test_stage2_position_endpoint_repeatability -v
python tests/stage2_position_endpoint_repeatability.py --port COM_PORT
~~~

脚本固定WIN、D-043 fc24、p6和 `TRAP 180/900 + FF ON`，在六个相对机械区域各执行两轮
`±10°`及回零，共48段、每段2.5 s。位置配置必须显式发送 `TRACK STEP`；D-052固件还会
在 `START POSITION`自动完成同一动作。正式STEP结果为
`tests/results/stage2_position_endpoint_repeatability_20260827_202232.json`和同名CSV：
32/48通过，稳态误差48/48通过，最大过冲1.78°、最大绝对稳态误差0.036°、最坏保持
速度纹波0.87 rpm RMS，最终READY。机器摘要为
`baselines/stage2_position_endpoint_repeatability_d051_20260827.json`。

D-053位置默认配置代表区域A/B入口：

~~~powershell
python tests/stage2_position_default_ab.py --plan-only
python -m unittest tests.test_stage2_position_default_ab -v
python tests/stage2_position_default_ab.py --port COM_PORT
~~~

脚本固定WIN、D-043 fc24、p6和内速度STEP/NONE，只在0°动态制动、120°近零速纠偏、
240°良好对照三区域比较 `RAMP 180 + FF OFF` 与 `TRAP 180/900 + FF ON`。每组24段，
总计48段；仍只使用0.5°过冲、200 ms整定和0.2°稳态误差三项验收。端点瞬间的速度
命令、选中/ABZ速度和iq只用于区分动态制动滞后与静态克服转矩，不增加硬门槛。脚本
不会烧录；运行需要当次确认24 V、自由转子和周围安全，退出STOP/READY并恢复默认。

D-053正式结果为 `tests/results/stage2_position_default_ab_20260827_204844.json`：默认
RAMP+FF OFF与候选TRAP+FF ON均通过8/24；候选整定分项13→18、峰值dq/速度降低，但
最坏过冲2.070→2.217°，因此不升级默认。随后D-054用
`tests/stage2_position_trap_ff_ab.py`固定TRAP只比较FF OFF/ON，结果文件为
`tests/results/stage2_position_trap_ff_ab_20260827_205326.json`：合格9→10、整定14→20、
最坏过冲同为2.217°，最终READY并恢复默认。机器摘要为
`baselines/stage2_position_default_ab_d053_d054_20260827.json`。

## 阶段3分项前馈

离线计划和专项回归不需要目标板或24 V：

~~~powershell
python tests/stage3_feedforward_component_validation.py --plan-only
python -m unittest tests.test_stage3_feedforward_component_validation -v
~~~

计划固定IDENTIFIED模型，第一轮按 `OFF/CROSS/BEMF/BOTH`、第二轮严格反序；每次模式访问
覆盖 `±1500/±3000 rpm`，各速度依次施加 `id=0/±1 A`，两轮同时反转速度和d轴顺序。
每次访问在首方向的±3000 rpm记录一份300 rpm局部阶跃Scope，总计8次访问、96个稳态点、
8份4096点/40 kHz Scope。普通遥测通道35必须确认对应模式码
`OFF=0/BOTH=1/CROSS=2/BEMF=3`；Scope布局保持11通道不变，主机用模式和模型离线重算
分项及PI电压。

真正运行入口为：

~~~powershell
python tests/stage3_feedforward_component_validation.py --port COM_PORT
~~~

该命令需要先烧录本次新固件，并会让电机空载自由双向旋转，范围为±1500/±3000 rpm、
±1 A d轴保持和±3000→±3300 rpm局部阶跃。执行前必须当次确认24 V、电源限流、自由
转子及周边安全；脚本边界为4 A速度q轴参考、5 A实际dq、3500 rpm、18–28 V和0.90
最低电压缩放，正常或异常退出均STOP/READY，并恢复LEGACY、FF OFF、WIN和7 A默认限幅。
Scope只在捕获完成且该模式已回零、STOP/READY后导出，避免USB传输期间无遥测监督地高速
保持。这批只支持空载分项、跟踪与电压裕量结论，不支持带载解耦、效率或恒功率声明。

D-057结果为 `results/stage3_feedforward_component_20260827_223940.json`，机器摘要为
`baselines/stage3_feedforward_component_d057_20260827.json`。96个稳态点和8份Scope全部完成，
最终READY并恢复LEGACY、FF OFF。BOTH/OFF两轮PI电压降幅87.17%/87.20%；BEMF稳态q轴
实测/理论斜率1.00075、相关0.99997，CROSS q轴斜率1.0996、相关0.9667。局部阶跃中
CROSS跨两方向把id误差RMS降低约0.040 A、iq误差RMS降低0.010–0.014 A。全程
`voltage_scale=1`，最大请求利用率25.35%，因此该结果不是饱和验证。

首次硬件执行还暴露了主机相位边界问题：每个hold后重写CSV期间会积压旧遥测。脚本现在
在新的速度、hold和Scope相位发命令前清空主机输入积压；速度稳态判据对阶段3显式使用
±90 rpm以覆盖3000 rpm处WIN测速纹波，阶段2默认判据仍保持±60 rpm。

## 阶段3受控电压限幅

D-058的离线计划和专项回归不需要目标板或24 V：

~~~powershell
python tests/stage3_voltage_saturation_validation.py --plan-only
python -m unittest tests.test_stage3_voltage_saturation_validation -v
~~~

固件只新增READY命令 `VOLT LIMIT ratio`，将既有dq矢量上限乘以比例；复位默认1，调制仍
使用实测母线。Scope在旧11通道末尾追加 `voltage_scale`，形成12通道、48 B样本。旧CSV
按字段名读取的离线入口继续兼容，新的板端固件和ARM/DUMP主机必须使用同一12通道布局。

硬件矩阵固定IDENTIFIED模型，只比较OFF与BOTH。两轮使用反序，在±3000 rpm基点执行
±3300 rpm短脉冲，总计8次访问和8份Scope。测试比例固定0.22，按D-057最大请求利用率
25.35%外推最小 `voltage_scale≈0.868`。前两批40 kHz短窗已经负责验证限幅进入；当前脚本
使用分频10的4 kHz、1.024 s记录，在100 ms进入脉冲、140 ms返回基点，以末尾20 ms和
脉冲前20 ms的速度、dq误差及PI电压直接比较恢复，不用基点偶发的 `scale<1` 定义恢复时间。

真正运行入口为：

~~~powershell
python tests/stage3_voltage_saturation_validation.py --port COM_PORT
~~~

该命令需要目标上已有D-058固件，并需当次确认24 V、电源限流、自由转子和周边安全；电机会
双向运行，最高命令±3300 rpm。运行边界为4 A速度q轴参考、5 A实际dq、3500 rpm、18–28 V
和0.70最低缩放。正常或异常退出均STOP并确认READY，再恢复 `VOLT LIMIT 1`、IDENTIFIED、
FF BOTH、WIN和默认速度限幅。

D-058最终结果为 `results/stage3_voltage_saturation_20260827_232614.json`，进入限幅的两个
40 kHz诊断批次为 `..._231622.json`和 `..._232038.json`，机器摘要为
`baselines/stage3_voltage_saturation_d058_20260827.json`。三批共24份Scope全部完成，
全局最小缩放0.863987。长窗中OFF/BOTH的脉冲前→末尾速度误差RMS为
39.69→39.39 / 40.34→39.77 rpm，iq误差为0.161→0.179 / 0.166→0.188 A，PI电压偏移
均值0.0306/0.0166 V；两种模式均正确恢复。最终READY并恢复测试上限和原复位默认。
离线门槛仍为95项Python测试、C++算法回归PASS，Keil/ARMCC 5.06u7构建0 error/0 warning，
Flash 110,008 B、RAM 237,476 B；固定J-Link V7.54d Program & Verify及向量读回通过。

D-059后，`stage3_feedforward_component_validation.py`和
`stage3_voltage_saturation_validation.py`的清理路径改为恢复IDENTIFIED+BOTH。实验访问中的
OFF/CROSS/BEMF/BOTH设置不变，只更新“恢复默认”的含义。固件初始化同样改为
IDENTIFIED+BOTH；专项6项、全套96项Python测试和C++回归PASS，Keil构建0 error/0 warning，
Flash 110,016 B、RAM 237,476 B。

复位默认冒烟入口为：

~~~powershell
python tests/stage3_default_feedforward_smoke.py --port COM_PORT
~~~

该脚本不发送FF模型或模式选择命令，只检查初始BOTH=1和±1500 rpm的前馈符号，最后STOP并
恢复IDENTIFIED+BOTH。D-059结果为
`results/stage3_default_feedforward_smoke_20260827_235240.json`，机器摘要为
`baselines/stage3_default_feedforward_smoke_d059_20260827.json`：正反速度1499.44/−1500.49 rpm，
`uq_ff=+1.419/−1.422 V`，运行峰值dq 0.627 A、最小缩放1，最终READY。

## 阶段4弱磁控制

离线设计与回归入口不需要目标板或24 V：

~~~powershell
python tests/stage4_field_weakening_design.py
python -m unittest tests.test_stage4_field_weakening_design -v
powershell -ExecutionPolicy Bypass -File tests/run_algorithm_regression.ps1
~~~

D-061固定IDENTIFIED模型、95%目标电压、200 Hz请求电压低通、`Kp=0.4 A/V`、
`Ki=2400 A/(V*s)`和5 A总dq电流圆；离线覆盖电压椭圆根选择、正反对称性、FEEDBACK/
HYBRID进入退出以及D-058历史数据重放。设计摘要为
`baselines/stage4_field_weakening_design_d061_20260828.json`。

连续硬件入口：

~~~powershell
python tests/stage4_field_weakening_validation.py --plan-only --phase main
python tests/stage4_field_weakening_validation.py --port COM_PORT --phase main
python tests/stage4_field_weakening_validation.py --port COM_PORT --phase extension
python tests/stage4_default_field_weakening_smoke.py --port COM_PORT
~~~

`main`使用0.35人工电压比例和±4500→±5000 rpm，`extension`使用0.45比例和
±6000→±6500 rpm。每档固定IDENTIFIED+BOTH，第一轮OFF/FEEDBACK/HYBRID、第二轮反序，
方向顺序同时反转，共12份4 kHz、1.024 s Scope。速度PI基础限幅5 A，弱磁按5 A总dq圆
动态收紧q轴余量；硬件资格还检查5.5 A实际dq、速度、18–28 V母线和最低缩放。脚本正常或
异常退出均STOP/READY，并恢复HYBRID、`VOLT LIMIT 1`、IDENTIFIED+BOTH、WIN和7 A基础限幅。

D-064最终结果：

- `results/stage4_field_weakening_main_20260828_010821.json`：12/12 Scope、64/64资格检查通过，
  全局实际dq峰值4.712893 A、最小缩放0.963833；
- `results/stage4_field_weakening_extension_20260828_010949.json`：12/12 Scope、64/64资格检查
  通过，全局实际dq峰值5.415867 A、最小缩放0.916495；
- HYBRID相对FEEDBACK在5000/6500 rpm把实际dq峰值从4.713/5.416 A降至3.984/4.131 A，
  电压误差RMS保持0.156/0.150 V和0.198/0.204 V的相当水平；
- `results/stage4_default_field_weakening_smoke_20260828_011405.json`未发送`FW MODE`，确认复位
  默认HYBRID，+5000 rpm时`id_ref=-1.295 A`、请求/目标电压4.624/4.606 V，最终READY。

版本化摘要为`baselines/stage4_field_weakening_d064_20260828.json`。本批只支持空载人工电压
边界，不能外推为自然基速、带载恒功率、效率、退磁或机械超速验证。

D-065动态边界补充入口：

~~~powershell
python tests/stage4_voltage_boundary_validation.py --plan-only
python tests/stage4_voltage_boundary_validation.py --port COM_PORT
~~~

脚本固定IDENTIFIED+BOTH和±6500 rpm，在每次4 kHz、1.024 s Scope内于0.200 s把
`VOLT LIMIT`从1降到0.45、0.650 s恢复为1；访问顺序为FEEDBACK正向、HYBRID反向、
HYBRID正向、FEEDBACK反向。固件只允许该命令在READY或主机启动的速度模式中生效，
正常或异常退出均STOP/READY并恢复HYBRID、`VOLT LIMIT 1`、IDENTIFIED+BOTH和默认限幅。

原始结果`results/stage4_voltage_boundary_20260828_014559.json`及4份Scope表明：FEEDBACK/
HYBRID受限段电压误差RMS为0.210/0.201 V、速度误差RMS为66.85/64.46 rpm，最长进入/
退出为4.25/10.25 ms，全局实际dq峰值3.912 A、最小缩放0.981924，最终模式17。初次报告
仅因把±0.25 V收敛带逐点施加到含正常纹波的4 kHz样点而误报4项失败；分析改为首个
10 ms均值窗后44/44通过，回归覆盖带纹波样例。版本化摘要为
`baselines/stage4_voltage_boundary_d065_20260828.json`。

## 阶段5无感与HFI

中高速无感的三个入口依次为：

~~~powershell
python tests/stage5_sensorless_shadow_validation.py --plan-only
python tests/stage5_sensorless_handoff_validation.py --plan-only
python tests/stage5_sensorless_closed_loop_validation.py --plan-only
~~~

D-067的ABZ影子矩阵覆盖双向500～6500 rpm，Active Flux 14/14点通过并冻结为主观测器；
D-068完成双向电流控制I/f到Active Flux接管；D-069完成独立启动后的500～6500 rpm闭环。
版本化摘要分别为`baselines/stage5_sensorless_shadow_d067_20260828.json`、
`baselines/stage5_sensorless_handoff_d068_20260828.json`和
`baselines/stage5_sensorless_closed_loop_d069_20260828.json`。

HFI从诊断到正式控制的实验入口为：

~~~powershell
python tests/stage5_hfi_axis_characterization.py --help
python tests/stage5_hfi_tensor_capture.py --help
python tests/stage5_hfi_shadow_validation.py --help
python tests/stage5_hfi_polarity_characterization.py --help
python tests/stage5_hfi_ipd_validation.py --help
python tests/stage5_hfi_d_axis_tracking_validation.py --help
python tests/stage5_hfi_speed_feedback_validation.py --help
python tests/stage5_hfi_speed_pi_design.py --help
python tests/stage5_hfi_speed_loop_validation.py --help
python tests/stage5_hfi_flux_handoff_validation.py --help
~~~

早期结果证明0.5 V、10 kHz注入在本电机/PWM/ADC链上可观测，固定角张量两次拟合的
特征值比约1.96～2.04；`results/stage5_hfi_shadow_20260828_134045.json`覆盖0和
±100～±750 rpm。D-072直接张量控制只完成零速保持，这组失败没有删除，而是推动D-075
把正式控制载波改为估计d轴双极性注入，并为HFI单独辨识速度反馈和设计PI。

D-075的分步证据为：

- `results/stage5_hfi_ipd_validation_20260829_203049.json`：±1 A IPD空间8点无180°极性错误，
  7/8满足20°最终角误差P95；
- `results/stage5_hfi_d_axis_tracking_20260829_202752.json`：ABZ控制下0、±100、±250、
  ±500、±750 rpm的估计d轴跟踪11/11运行点通过；
- `results/stage5_hfi_speed_feedback_20260829_234412.json`及同批补点：在±250 rpm、
  1～20 Hz下得到HFI反馈幅相，用于证明D-043编码器PI不能直接照搬；
- `results/stage5_hfi_speed_loop_20260830_002612.json`：HFI角度、HFI速度反馈和专用PI下
  `0/±350/±500 rpm`共8/8保持点通过；
- `results/stage5_hfi_flux_handoff_20260830_003608.json`：一次连续运行完成0、±350、±500、
  ±600、±750 rpm的14/14保持点和4/4 HFI/Active Flux来源切换，最终READY。

当前冻结参数为：HFI速度PI `Kp=0.00160960136, Ki=0.012`；HFI角度启动限流0.5 A、
250 rpm/s、250 rpm进入PI；HFI→Active Flux 550 rpm、Active Flux→HFI 525 rpm，
角度/速度门槛30°/100 rpm，资格和融合各20 ms。完整机器摘要为
`baselines/stage5_hfi_full_sensorless_d075_20260830.json`。

命令兼容语义仍保留：`HFI FIXED`是d轴扫描，`HFI TENSOR`是四轴张量扫描，`HFI SHADOW`
是编码器真值下的张量跟踪，`HFI DTRACK`切到估计d轴跟踪，`HFI CONTROL`启用控制权。
正式无感启动不需要主机按顺序拼这些诊断命令；状态均在下一FOC周期边界生效。HFI已接管
控制角时必须用`STOP`退出，硬件脚本正常或异常结束均STOP、确认READY并恢复默认。
