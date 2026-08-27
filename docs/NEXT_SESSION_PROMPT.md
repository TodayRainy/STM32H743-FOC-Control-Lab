# 下一窗口交接 Prompt

更新时间：2026-08-28

下面代码块可直接复制到新窗口。它只保存稳定事实和下一阶段入口，不保存当前上电、COM号或连接状态。

~~~text
请继续推进工作区 STM32H743VGT6_FOC 的 FOC 控制研究。不要依赖旧聊天上下文，先以仓库文档、源码、git状态和实验数据为事实来源。

开始前完整阅读：

1. AGENTS.md
2. README.md
3. docs/PROJECT_STATUS.md
4. docs/FIELD_WEAKENING_RESEARCH.md
5. docs/ROADMAP.md
6. docs/WORKFLOW.md
7. docs/DECISIONS.md
8. docs/HARDWARE_TOOLCHAIN.md
9. docs/ARCHITECTURE.md
10. docs/SERIAL_PROTOCOL.md
11. docs/VALIDATION_REPORT.md
12. tests/README.md
13. CHANGELOG.md

随后只读检查git status、当前分支和最近提交。当前阶段性Git基线应为main分支的v0.3.0-stage4；不得重置、覆盖或为了得到干净基线删除工作区内已有有效修改和原始结果。

当前阶段事实：

- 阶段1已完成当前空载边界内的Rs/Ld/Lq/磁链辨识与留出验证；候选为Rs=144.818 mΩ、Ld=26.044 µH、Lq=40.344 µH、磁链0.907950 mWb、10极对。
- 阶段2三环整定已收尾。默认电流PI为d轴Kp=0.155978918、Ki=1086.465606，q轴Kp=0.251296400、Ki=1151.639999；ABZ测速默认WIN（1 ms窗口+150 Hz IIR）；速度PI为Kp=0.00536533785、Ki=0.258523708，实测闭环带宽约45–55 Hz。
- 位置默认是Kp=376.991118、Ki=Kd=0，RAMP 180 deg/s、位置速度前馈OFF；TRAP 180 deg/s、900 deg/s²、前馈ON是可选快速轨迹。约2.2°机械区域相关小行程过冲作为当前空载限制保留。
- 阶段3完成OFF/CROSS/BEMF/BOTH分项、受控电压限幅恢复和默认写回。电流前馈复位默认是IDENTIFIED+BOTH；BOTH相对OFF降低PI电压约87.19%，CROSS在局部阶跃中重复改善dq误差。LEGACY/IDENTIFIED的旧A/B不足以排序，但IDENTIFIED有阶段1辨识/留出和阶段3直接验证依据。
- 当前Scope为12通道×4096点：ia、ib、iq、id、iq_ref、id_ref、uq、ud、elec_angle、elec_speed、speed_ref、voltage_scale；放在0x30000000 D2 SRAM，普通遥测仍为36通道。
- 阶段4只实现弱磁，MTPA延期到具备受控负载或转矩/功率测量后。控制结构是4 kHz定子请求电压反馈PI + IDENTIFIED稳态dq电压椭圆前馈HYBRID，速度PI给iq、弱磁给负id，总dq参考限制为5 A圆。
- 弱磁模式为OFF/FEEDBACK/HYBRID。目标电压为实际可用dq上限的95%，请求电压用200 Hz一阶低通；PI为Kp=0.4 A/V、Ki=2400 A/(V*s)，最终id_ref斜率500 A/s。弱磁先生成id，再在速度PI运行前以sqrt(25-id²)更新q轴输出余量，使其内部anti-windup跟踪真实约束。
- D-064主矩阵使用0.35人工电压比例和±4500→±5000 rpm，扩展矩阵使用0.45比例和±6000→±6500 rpm；每档OFF/FEEDBACK/HYBRID两轮反序、双向各一次，共12/12 Scope和64/64资格检查通过。
- HYBRID/FEEDBACK在5000 rpm的电压误差RMS为0.156/0.150 V、速度误差RMS为59.17/63.00 rpm、实际dq峰值3.984/4.713 A；6500 rpm分别为0.198/0.204 V、63.58/65.47 rpm、4.131/5.416 A。HYBRID在相当电压调节下明显减少动态电流，收益超过方向/轮次散布。
- 固件复位默认已冻结为IDENTIFIED+BOTH+HYBRID。未发送FW MODE的+5000 rpm冒烟测得id_ref=-1.295 A、请求/目标电压4.624/4.606 V、实际dq峰值3.186 A、最小voltage_scale=0.984581，最终READY。
- 最终103项Python测试通过，C++算法回归PASS；Keil/ARMCC 5.06u7构建0 error/0 warning，Flash 111,492 B、RAM 237,476 B。固定J-Link V7.54d、STM32H743VG、SWD 4000 kHz的Program & Verify O.K.，向量读回20008000 08000449 0800E3BF 0800D959。
- 阶段4机器摘要为tests/baselines/stage4_field_weakening_d064_20260828.json，引用三份tests/results原始结果。不要删除文档引用的原始结果。

阶段4在当前空载、人工电压边界和±6500 rpm以内已经收尾。它不证明自然24 V基速、带载恒功率、效率、热稳定、退磁或机械超速安全；当前也没有证据要求追加Ld/磁链调度、MTPV、过调制、q轴电压回退或MPC。

下一步先和用户讨论阶段5无感控制研究方向，再冻结最小方案。重点应是：以ABZ编码器为真值，先做中高速反电动势/磁链类观测器的角度与速度误差评价；明确观测器与现有IDENTIFIED模型、BOTH前馈和HYBRID弱磁如何共享参数但避免自证；确定开环观测、闭环接管和失锁边界的分阶段证据。低速HFI和全速域切换不要与第一批实现捆绑，除非离线与现有observer源码审计证明必须一起处理。

硬件规则：不要从本Prompt推断当前24 V、USB、COM或板上固件状态。离线阅读、编辑、主机测试和Keil构建不需要上电。连接目标、烧录、串口观测或发送运动命令前，明确告知是否需要24 V、电机如何运动和测试范围，并等待用户当次确认。J-Link固定优先使用C:\Program Files\SEGGER\JLink\JLink.exe V7.54d，目标STM32H743VG、SWD 4000 kHz；核对原始Program & Verify及必要读回。测试退出必须STOP并确认READY，再说明是否可断电。

协作取舍：不修改死区和BRK，不重新搜索已冻结的三环PI，不重新扩大阶段3/4网格，不做全仓编码迁移，不删除文档引用的原始结果。把重心放在控制/观测算法、必要的性能实验和量化证据上；避免无关验证和预见性防御性框架。

当前第一小步是阶段5离线方案讨论和现有observer/angle链审计，不需要24 V。
~~~
