# 下一窗口交接 Prompt

更新时间：2026-08-30

下面代码块可直接复制到新窗口。它只保存稳定事实和下一阶段入口，不保存当前上电、COM号或连接状态。

~~~text
请继续推进工作区 STM32H743VGT6_FOC 的FOC控制研究。不要依赖旧聊天上下文，先以仓库文档、源码、git状态和实验数据为事实来源。

开始前完整阅读：

1. AGENTS.md
2. README.md
3. docs/PROJECT_STATUS.md
4. docs/ENGINEERING_CONTROL_AND_FAULTS.md
5. docs/NEXT_CONTROL_RESEARCH.md
6. docs/ROADMAP.md
7. docs/WORKFLOW.md
8. docs/DECISIONS.md
9. docs/HARDWARE_TOOLCHAIN.md
10. docs/ARCHITECTURE.md
11. docs/SERIAL_PROTOCOL.md
12. docs/VALIDATION_REPORT.md
13. tests/README.md
14. CHANGELOG.md

随后只读检查git status、当前分支和最近提交。当前发布基线为main分支的v0.4.0-stage5；其上存在D-076～D-081尚未提交的工程化、控制域验收和研究工作。不得重置、覆盖或为了得到干净基线删除有效修改和原始结果。

当前稳定事实：

- 阶段1电气参数辨识候选为Rs=144.818 mΩ、Ld=26.044 µH、Lq=40.344 µH、磁链0.907950 mWb、10极对。
- 阶段2默认电流PI为d轴0.155978918/1086.465606、q轴0.251296400/1151.639999；ABZ默认WIN（1 ms窗口+150 Hz IIR）；速度PI为0.00536533785/0.258523708，实测闭环带宽约45–55 Hz。位置默认Kp=376.991118、RAMP 180 deg/s、前馈OFF。
- 阶段3默认IDENTIFIED+BOTH；BEMF主导PI电压减负，CROSS在局部阶跃重复改善dq误差，人工电压限幅后的PI tracking恢复通过。
- 阶段4默认HYBRID弱磁，5 A总dq电流圆；人工电压边界下双向5000/6500 rpm及固定6500 rpm动态边界通过。MTPA延期到具备受控负载或转矩/功率测量后。
- 阶段5空载无感链为最小IPD → 估计d轴HFI → HFI专用低速速度PI → 550/525 rpm滞回切换IDENTIFIED Active Flux。连续矩阵覆盖0、±350、±500、±600、±750 rpm并通过；低于250 rpm仍是HFI角度下限流电流辅助，不是精密非零速度伺服。1 A IPD空间8点无180°错误，但只有7/8满足20°角误差P95。
- D-076/D-077将VF、SMO、Active Flux、HFI、无感启动和来源接管统一到algorithm/foc/sensorless；第一方文本统一UTF-8无BOM，历史实验数据和第三方代码不迁移。D-078完成这一重组后的首批板端控制与无感回归，D-079/D-080完成结构化命令、状态驱动上位机和统一控制域验收，D-081冻结REJECT无副作用语义。
- 生命周期为IDLE/CURRENT_CALIBRATION/READY/STARTING/RUNNING/FAULT_ACTIVE/FAULT_LATCHED；operation、rotor source和diagnostic保持正交。STOP不清故障，活动TRIP消失后仍需FAULT CLEAR；电流偏置故障确认后重新校准。
- 首批统一诊断为电流偏置、软件过流、母线欠/过压、超速、无感启动/来源丢失、控制周期，以及速度/位置跟踪WARNING。D-078已确认25/23 µs周期阈值在基础三模式和HFI短矩阵中不误触发，并完成速度跟踪WARNING活动/恢复/锁存/清除；D-080在双向6500 rpm、位置、动态弱磁和两条无感矩阵中继续保持零活动/锁存故障。受控欠压因电源暂不能调压而延期，不主动制造过流、过压或超速。
- 新增STATUS、FAULT、FAULT CLEAR、FAULT_EVENT和关键命令的结构化COMMAND结果；36通道遥测不变。当前Scope为15通道×4096点，在D-058的12通道末尾追加hfi_phase_error/hfi_d_response/hfi_q_response；RW_SCOPE=245,824 B，位于0x30000000 D2 SRAM。
- host/foc_client提供共享混合流解析和串口设备接口；host/gui提供ABZ、HFI/Active Flux、I/f/Active Flux三来源，状态驱动START/STOP/目标、实时遥测、Scope、故障和运行配置。目标在RUNNING确认后才下发，运行态非法配置显示明确拒绝原因。
- D-081当前板端基线为119项Python测试通过、1项默认Python缺PySide6跳过，C++算法回归PASS、专用虚拟环境GUI冒烟PASS、Keil 0 error/0 warning，Flash 129,512 B、RAM 286,648 B；固定J-Link V7.54d Program & Verify和向量读回通过。HEX SHA256为9FCBB9BBA23E09309AE2F95B9B3268D7BE8E2186E8BEAD7FD856D484BD3C51F0。重复START拒绝证据见tests/baselines/stage6_control_plane_d081_20260830.json。
- D-080速度/位置矩阵为32/36：0及双向250～6500 rpm、RAMP位置13/13、TRAP+前馈位置5/5通过；±50/±100 rpm平均误差绝对值≤0.696 rpm，但严格RMS为17.10～22.36 rpm。动态弱磁4/4、HFI/Active Flux 14/14和4次切换、I/f/Active Flux双向至6500 rpm、观测器影子矩阵完成。机器摘要见tests/baselines/stage6_control_domains_d080_20260830.json。

下一步回到控制算法主线。第一优先级是ABZ低速计数/周期融合：先只读审计是否存在可复用的真实边沿时间戳，不能用1 ms计数反推伪周期；若硬件可观测，保留WIN为中高速主支路，只增加一个低速周期支路并在交叠区融合，不重调D-043速度PI。第二优先级是保留现有三环、只替换参考发生器的单轴jerk受限S曲线，与TRAP做位置域对照。DOB/ESO、齿槽补偿、在线参数辨识、MTPA/MTPV、过调制和热/母线功率管理都等真实可测需求或硬件条件；详见docs/NEXT_CONTROL_RESEARCH.md。

硬件规则：不要从本Prompt推断当前24 V、USB、COM或板上固件状态。离线阅读、编辑、主机测试和Keil构建不需要上电。连接目标、烧录、串口观测或发送运动命令前，明确告知是否需要24 V、电机如何运动和测试范围，并等待用户当次确认。J-Link固定使用C:\Program Files\SEGGER\JLink\JLink.exe V7.54d，目标STM32H743VG、SWD 4000 kHz；核对原始Program & Verify及必要读回。测试退出必须STOP或保持FAULT功率关闭，并确认最终状态。

协作取舍：不修改死区和BRK，不重新搜索已冻结三环PI，不扩大阶段3/4网格，不删除文档引用的原始结果。把重心放在控制算法和必要的性能实验；状态与故障基础已建立，不继续增加通信看门狗、自动重启、插件、持久化或预测性防御框架，除非新的真实需求出现。

当前第一小步是只读审计TIM3 ABZ边沿时间戳可观测性，不需要24 V。若随后实现并上板验证，建议覆盖0、±25、±50、±100、±250、±500 rpm及过零；任何新的运动或受控欠压实验仍需用户当次确认电源、USB和电机状态。
~~~
