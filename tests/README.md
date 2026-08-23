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
| stubs/ | 主机回归需要的嵌入式接口桩 | 不需要 |
| results/ | CSV、JSON 和分阶段软件示波器数据 | 数据 |

主机回归入口：

~~~powershell
powershell -ExecutionPolicy Bypass -File tests/run_algorithm_regression.ps1
~~~

默认使用 PATH 中的 g++，临时可执行文件写入被忽略的 tmp 目录。2026-08-24 已使用 MinGW g++ 15.2.0 验证 PASS。

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

## 软件示波器

当前协议：

- 10 个 float32 通道；
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

- 决定 tests/results 的版本控制策略：精选基线、外部归档或全部保留。
- 为实验报告增加固件构建标识，避免把同名脚本的不同固件结果混在一起。
- 在位置极限环可重复后，增加专门的首次进入位置模式测试。
- 参数辨识开始时，按真实需要扩展软件示波器通道和元数据，不预先搭建庞大通用框架。
