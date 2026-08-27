# 下一窗口交接 Prompt

更新时间：2026-08-27

下面代码块可直接复制到新窗口。它只保存稳定事实和下一阶段入口，不保存当前上电、COM号或连接状态。

~~~text
请继续推进工作区 STM32H743VGT6_FOC 的 FOC 控制研究。不要依赖旧聊天上下文，先以仓库文档、源码、git状态和实验数据为事实来源。

开始前完整阅读：

1. AGENTS.md
2. README.md
3. docs/PROJECT_STATUS.md
4. docs/STAGE3_DECOUPLING_AND_VOLTAGE.md
5. docs/ROADMAP.md
6. docs/WORKFLOW.md
7. docs/DECISIONS.md
8. docs/HARDWARE_TOOLCHAIN.md
9. docs/ARCHITECTURE.md
10. docs/SERIAL_PROTOCOL.md
11. docs/VALIDATION_REPORT.md
12. tests/README.md
13. CHANGELOG.md

随后只读检查 git status、当前分支和最近提交。当前Git基线仍是main分支的v0.1.0-stage0，工作区中存在尚未提交的阶段1/2大量有效修改和新增文件；不得重置、覆盖或为了得到干净基线删除它们。

当前阶段事实：

- 阶段1已完成当前空载边界内的Rs/Ld/Lq/磁链辨识与留出验证；候选为Rs=144.818 mΩ、Ld=26.044 µH、Lq=40.344 µH、磁链0.907950 mWb。
- 阶段2三环整定已收尾。复位默认电流PI为d轴Kp=0.155978918、Ki=1086.465606，q轴Kp=0.251296400、Ki=1151.639999；ABZ默认测速为WIN（1 ms窗口+150 Hz IIR）；速度PI为Kp=0.00536533785、Ki=0.258523708，实测闭环带宽约45–55 Hz。
- 位置默认是Kp=376.991118、Ki=Kd=0，RAMP 180 deg/s、位置速度前馈OFF；TRAP 180 deg/s、900 deg/s²、前馈ON是可选快速轨迹。START POSITION会自动清除内速度轨迹。
- D-053/D-054后位置环在当前空载边界下闭合：稳态误差代表矩阵全部通过，约2.2°机械区域相关小行程过冲保留为限制，不继续扩大位置参数、速度PI或齿槽补偿网格。
- 阶段3 A/B离线阶段已完成。电流前馈现支持OFF/CROSS/BEMF/BOTH，取值0/1/2/3且旧FF OFF/ON继续映射0/1；公式仍为uq_cross=omega_e*Ld*id、ud_cross=-omega_e*Lq*iq、uq_bemf=omega_e*flux。可切换LEGACY/IDENTIFIED模型；D-059源码复位默认已更新为IDENTIFIED+BOTH，三环参数不变。
- D-057已完成IDENTIFIED模型下OFF/CROSS/BEMF/BOTH两轮反序矩阵：96个稳态点和8份40 kHz Scope均有效，最终READY并恢复默认。BOTH/OFF两轮PI电压降幅87.17%/87.20%；BEMF q轴理论/实测斜率1.00075，CROSS q轴斜率1.0996，局部阶跃中CROSS跨方向重复降低dq误差。LEGACY与IDENTIFIED仍未排序。
- D-057全程voltage_scale=1，最大请求利用率25.35%。D-058已增加复位为1、READY下可设的测试电压上限比例；它只缩放既有dq矢量上限，调制仍使用实测母线，限幅后的PI tracking链路不变。
- 当前Scope为12通道×4096点，在旧11通道末尾追加voltage_scale，放在0x30000000 D2 SRAM；数据体196,608 B，整个RW_SCOPE 196,672 B，普通遥测仍为36通道。
- D-058已完成三批各8份Scope。前两批40 kHz短窗的最小缩放0.86842/0.86399，证明受控限幅进入，但102.4 ms不足以观察完全回稳；第三批只改为4 kHz、1.024 s长窗，在100–140 ms施加同一±3000→±3300 rpm脉冲。
- 长窗OFF/BOTH的脉冲前→末尾速度误差RMS为39.69→39.39 / 40.34→39.77 rpm，iq误差为0.161→0.179 / 0.166→0.188 A，PI电压偏移均值0.0306/0.0166 V；两种模式均无持续积分残留。基点偶发触碰0.22人工上限，不用最后一个scale<1样本定义恢复时间。
- D-058离线门槛为95项Python测试和C++算法回归PASS；Keil/ARMCC增量构建0 error/0 warning，Flash 110,008 B、RAM 237,476 B。固定J-Link V7.54d最终Program & Verify和向量读回通过；三批最终均READY并恢复VOLT LIMIT 1、LEGACY、FF OFF。机器摘要为 `tests/baselines/stage3_voltage_saturation_d058_20260827.json`。不要据此推断当前板上固件、连接或上电状态。
- D-059仅修改复位初始化和当前阶段3脚本的恢复默认语义：前馈模型IDENTIFIED、模式BOTH。专项6项、全套96项Python测试和C++回归PASS；Keil/ARMCC 5.06u7构建0 error/0 warning，Flash 110,016 B、RAM 237,476 B。固定J-Link V7.54d烧录/读回通过；不发送FF选择命令的±1500 rpm复位冒烟确认初始BOTH=1、`uq_ff=+1.419/−1.422 V`，最终READY并恢复IDENTIFIED+BOTH。机器摘要为 `tests/baselines/stage3_default_feedforward_smoke_d059_20260827.json`。不要据此推断当前板上连接或上电状态。
- 同批修复：失败的START POSITION不再改变内速度轨迹；持续过流在40 kHz路径先立即关PWM，故障文本由前台延迟输出。

阶段3已经结束。下一主线进入阶段4：先定义实际电压利用率、电流圆与负d轴弱磁生成器，再做进入/退出连续性和人工电压上限验证；MTPA先离线验证轨迹与适用性，没有测功设备时不宣称效率收益。不要重跑阶段3模型、四模式或饱和比例网格。

研究应保留探索自由：具体速度、电流、频点和阈值由离线可观测性与电压裕量决定；建议从已经验证的±1500/±3000 rpm、有限d轴激励和速度环间接q轴激励出发，但这些不是永久限制。先证明分项和坐标，再安全接近电压边界。没有负载/测功设备时不宣称带载解耦、效率或恒功率性能。

硬件规则：不要从本Prompt推断当前24 V、USB、COM或固件状态。离线阅读、编辑、主机测试和Keil构建不需要上电。连接目标、烧录、串口在线观测或发送运动命令前，明确告知是否需要24 V、电机可能如何运动和测试范围，并等待用户当次确认。J-Link固定优先使用C:\Program Files\SEGGER\JLink\JLink.exe V7.54d，目标STM32H743VG、SWD 4000 kHz；核对原始Program & Verify及必要读回。测试退出必须STOP并确认READY，再说明是否可断电。

协作取舍：不修改死区和BRK，不重新搜索已冻结的三环PI，不做全仓编码迁移，不删除文档引用的原始结果。优先控制方法、实验设计和量化证据；形成阶段性结论后更新PROJECT_STATUS、DECISIONS、VALIDATION_REPORT、ROADMAP或tests/README。

当前第一小步是离线默认策略讨论与一行源码写回，不需要24 V。若进入烧录或板端冒烟，必须重新说明是否需要24 V、电机如何运动和范围；得到用户当次确认前，不连接目标、不烧录、不打开串口。测试后必须STOP并确认READY，再汇报是否可断电。
~~~
