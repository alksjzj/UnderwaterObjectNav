# 第 7 天：从期望轨迹到六个推进器

日期：2026-10-26（Asia/Shanghai），学习进行中。

## 目标与入口

依据当前《学习方法.md》第七天，理解期望路径和实际里程计如何进入控制器，经广义力与六推进器分配，作用于仿真并获得新的反馈。第六天记录确认其核心目标及主线验收已完成。

本日三个核心文件：

- `src/aquaflow_stonefish/launch/tracking.launch`
- `src/aquaflow_stonefish/scripts/planar_pid_tracker.py`
- `src/aquaflow_stonefish/scripts/brics6_thruster_allocator.py`

数据链：nominal_reference_publisher → /aquaflow/nominal_path（Path），与 /bricsbot/odometry（Odometry）一起进入 planar_pid_tracker → /controller/generalized_force（WrenchStamped）→ brics6_thruster_allocator → /bricsbot/setpoint/pwm（Float64MultiArray，六个元素）→ Stonefish → 新的 odometry。

## 当前小段与待回答预测

先读 tracking.launch 中 enable 启动参数到 BricsBot 分配器 enabled 私有参数的传递（第 8、49–54 行）。区分 launch 配置与节点参数，简短对照 ROS 2 DeclareLaunchArgument / LaunchConfiguration / Node(parameters=...)。

待学习者预测：使用 enable:=false、正常收到期望路径和里程计时，控制器的广义力是否一定为零？分配器发布的六个 PWM 值预计怎样？本轮不提前给出两个问题的实验答案。

源码已重新核对：tracker.update 仍将 nearest 固定为 0，默认 lookahead_index=2；当前 tracking.launch 直接连接完整 nominal_path，没有启动 reference_processor。后续须按实际选点讲解，不将此链描述为已实现沿完整路径逐点推进。尚未修改实现。

本轮只核对计划、学习记录和源码，没有查询当前 ROS 运行状态，也未启动仿真、控制器或实验。实际实验前须重新核对容器、进程、Master 端口和发布者。没有创建临时探针；只新增本学习记录。第七天实验与验收均未完成。

## 禁用分配器预测与实际实验

学习者先正确预测控制器输出“不一定”为零；询问六个 PWM 值是否就是电机。已解释：数组的六个元素分别是给 T1–T6 的归一化驱动命令，当前接口范围 [-1,1]，并非电机本体、牛顿值或直接填写的转速。随后学习者正确预测禁用分配器时为“全零命令”。

实验前重新检查宿主机：无运行容器或相关 ROS 进程，11311/11371/11381 未监听，Noetic 镜像和显示授权可用。沙箱内不能访问 Docker/socket，转到宿主机权限检查成功。

使用临时容器 `underwater-objectnav-day7-disabled`，工程只读挂载，独立 Master 11381，原始启动入口：

```bash
roslaunch -p 11381 aquaflow_stonefish tracking.launch enable:=false graphics_quality:=low
```

容器设置 ROS_MASTER_URI=http://localhost:11381，使用现有 GPU 与 Xauthority 文件，没有更改显示访问权限。

首次启动未收到 PWM。ROS 图无分配器节点；roslaunch 日志明确报 `cannot launch node of type [aquaflow_st onefish/brics6_thruster_allocator.py]`。核对发现 tracking.launch 第 50 行包名新增了一个空格，git diff 也仅显示该处拼写变化。Agent 已说明并将 `aquaflow_st onefish` 修正为 `aquaflow_stonefish`，停止失败容器后重新启动；未修改控制算法。此修正后的 launch 与 Git 原版本一致。

第二次启动成功，确认 `/brics6_thruster_allocator/enabled=false`。等待五类消息及非空 Path 就绪后，以约 5 秒墙钟窗口采样；到达频率由窗口内首末消息的单调时钟计算。

| 话题 | 发布者 | 消息数 | 到达频率 |
|---|---|---:|---:|
| /aquaflow/nominal_path | /nominal_reference_publisher | 10 | 2.000 Hz |
| /bricsbot/odometry | /stonefish_simulator | 501 | 99.940 Hz |
| /controller/generalized_force | /planar_pid_tracker | 101 | 20.000 Hz |
| /bricsbot/setpoint/pwm | /brics6_thruster_allocator | 100 | 20.000 Hz |
| /aquaflow/actual_path | /actual_path_recorder | 50 | 10.000 Hz |

100 条 PWM 全部恰为六个零；101 条 WrenchStamped 全部至少有一个非零分量。第一条控制消息 force=(4.0,-0.03163,-1.24266) N，torque=(0,0,-0.03876) N·m，说明控制器可以提出非零需求，而禁用的分配器仍发布零驱动命令。

期望 Path 为 world_ned、41 个点，首点 (-5,0,0)、下标 2 为 (-4.5,0,0)、末点 (5,0,0) m。实际 odometry 在采样窗口内由约 (-4.999323,-0.001820,0.278035) 变为 (-4.998428,-0.003764,0.322798) m。零 PWM 不保证机器人位置绝对不变；本次未分离测量各被动力来源。

解释当前小段：`update()` 每次先令 `out=[0.0]*6`；enabled=false 跳过计算非零命令的分支，最后仍执行 publish。下一小段进入 tracker 的订阅回调如何保存 odometry/Path，以及 Timer 如何触发 update。暂未启用推进器、验证受控跟踪或完成第七天验收。

观察脚本通过标准输入运行，无临时脚本或大日志写入工程。实验容器已停止并自动删除，11381 已释放，未发现残留相关 ROS 进程。tracking.launch 相对 Git 无差异，git diff --check 通过；保留本学习记录。

## 控制器的输入保存与定时更新

学习者要求继续，重新核对 planar_pid_tracker.py 第 22–23、59–70 行。讲解两路订阅回调分别以最新完整消息替换 self.odom/self.path，不累计历史点；path_cb 对非 world_ned 的非空 frame_id 仅告警，仍保存消息，并未进行坐标变换或拒收。Timer 按 ~rate 参数（默认 20 Hz，即 0.05 s）调用 update，消息回调保存数据与定时计算是两种触发，二者不自动同步两路消息时间戳。简短对照 ROS 2 的 create_subscription/create_timer。此轮仅源码讲解，无新增实验或源码修改；下一小段读取 update 开头的缺失/空路径/超时检查。

## 输入有效性与超时检查

学习者确认已理解上一小段，并说明有 STM32 嵌入式小车 PID 调试经验，要求后续用此经验类比。后续可类比周期控制任务、反馈变量、目标值、数据有效标志与通信超时；须说明 rospy.Timer/订阅回调不是 MCU 硬件中断，也不保证硬实时。

本段读取 update 第 202–212 行：缺失里程计、缺失路径或路径点列表为空时 publish_zero 后 return；里程计消息年龄超过默认 0.25 s 或路径消息年龄超过默认 0.50 s 时同样输出零广义力并结束本轮。年龄使用 ROS 当前时间减消息 header.stamp，并非直接测量本地接收间隔。publish_zero 发布默认零 force/torque 的 WrenchStamped；不是直接发布六路 PWM，且这些分支没有清空积分状态。此轮仅源码讲解，无新实验。

## 控制周期、当前位姿和目标点

学习者要求进入下一步。读取 update 第 214–220 行：首次通过输入检查时 dt=0.05 s，之后根据 now-last_time 计算并限制到 [0.001,0.2] s，再更新 last_time；该限制不改变 Timer 周期，也不保证实际执行频率。以 STM32 PID 控制任务中用于积分的 dt 类比。读取 odometry 中位置 p 和四元数 q，通过 yaw_from_quat 得到弧度制偏航角。目标由 path.poses[min(len-1, nearest+lookahead)] 选择；当前 nearest=0、默认 lookahead=2，因此完整默认路径选下标 2，即第三点 (-4.5,0,0) m，而非随运动自动推进。讲解代码注释依赖 reference_processor，但当前 launch 未启动该处理节点。此轮仅源码讲解，无新实验或算法修改；下一段解释世界系位置误差旋转到机体系。

## 水平位置误差的坐标转换

学习者指出上一段语言重复；后续减少重复总结，每轮集中解释一个代码作用。当前讲解第 221–223 行：dx/dy 为世界坐标中的目标减当前位置，经过偏航角旋转得到机体系水平误差 ex/ey，单位均为米；本工程机体系约定 x 前、y 右。使用小车地图中的固定方向与车头方向区别来类比，说明这里只按 yaw 做平面转换，并非包含 roll/pitch 的完整三维转换。以 dx=1、dy=0、yaw=0 示范 ex=1、ey=0；让学习者预测 yaw=+90°（车头朝世界 +Y）时目标在前/后/左/右哪侧。尚未给出该预测题答案或执行计算实验。

学习者回答“左侧”，方向判断正确。Agent 从当前源码 AST 提取 c/s、ex/ey 两条赋值，用合成输入 dx=1、dy=0、yaw=pi/2 在内存执行，得到 ex≈6.12e-17、ey=-1，支持前后误差近零、目标在左侧一米。没有启动 ROS 或仿真、没有创建临时文件。随后讲解当前第 227–228 行的目标航向减实际航向与 wrap；同样从源码提取 wrap 函数，用当前 +179°、目标 -179° 的合成例子核对原始误差 -358° 被转换为约 +2°。说明代码内部使用弧度，wrap 处理角度跨越 ±pi 的边界，不是角速度限幅。

## 积分累积与限幅

学习者继续进入分配器。本段核对 brics6_thruster_allocator.py 第 114–118、143–144、183–189 行：订阅 WrenchStamped 后 wrench_cb 将最新完整消息保存到 last_wrench，Timer 每 0.05 s 调用 update；只有 enabled、已收消息且消息年龄满足 0<=age<=0.25 s（默认）才进入分配。wrench 元组顺序是 Fx,Fy,Nz,Fz，与消息字段的自然罗列顺序不同，必须与分配矩阵顺序一致。用 STM32 差速小车把整体前进/转向需求交给左右轮混控作职责类比，但这里输入是力/力矩而非速度。此次只读源码，无新实验；下一段讲 pinv 将四维需求变为六个推进器期望推力。

学习者确认理解控制公式后，继续输出段（当前第 248–255 行）：四个结果 fx/fy/fz 为力（N），nz 为绕 Z 轴力矩（N·m）；构造 WrenchStamped，填 ROS 当前时间与 base_link 标签，各分量按 max_fx/max_fy/max_fz/max_nz 限幅，最后发布到 /controller/generalized_force。默认限制分别为 4/4/20 N、1.4 N·m。以 fx=7 被夹到 4 N 类比 STM32 最终控制量限幅，但此处不是 PWM 限幅。frame_id 只是标注，不会自动执行坐标转换；未赋值的 torque.x/y 默认零。此轮只核对源码，无新运行实验；后续从分配器 wrench_cb 保存消息进入六推进器分配。

后续学习者询问是否在计算 I，已区分 int_x 是积分状态、ki_xy*int_x 才是输出中的 I 项。学习者要求继续，转入 fx=kp_xy*ex-kd_xy*vx+ki_xy*int_x：类比小车位置外环，输出这里是期望力（N）；D 部分是负速度反馈，不能一般性等同于旋转机体系误差的完整导数。仅在固定目标、一维固定轴等条件下，de/dt=-v 的推导成立。给出默认 Kp=15、Kd=3、Ki=0，ex=0.2 m、vx=0.1 m/s 时限幅前 fx=2.7 N 的教学算例；说明实际最终输出还要经过后续力限幅。未运行新实验。

学习者确认理解航向误差并继续。本段聚焦当前第 238–243 行：以 self.int_x 为例，用 STM32 中 integral += error*dt 后 clamp 的等价写法解释。积分状态跨控制周期保存，x/y/z 的积分单位为 m·s，航向积分为 rad·s；x/y/yaw 积分状态限于 [-1,1]，z 使用 z_integral_limit。区分积分状态限幅与最终力输出限幅；此处仅做积分夹紧，不能据此认定已实现依据输出饱和进行的条件积分或回算。用旧值 0.99、ex=0.5 m、dt=0.05 s 演示累加 1.015 后夹到 1.0（教学算例，未新增运行实验）。下一段读力/力矩计算式，补充前面的航向死区与速度反馈。

## 分配矩阵的输入输出

重新核对 brics6_thruster_allocator.py 第 37–55、123–141、189–193 行。四个整体需求按 Fx、Fy、Nz、Fz 排列，pinv 为 6×4，乘积得到 T1–T6 各自期望推力（牛顿）；这一步尚未转换成 PWM。矩阵从六个推进器的安装方向和位置建立；本轮只解释四到六的对应与物理意义，不展开伪逆数学。学习者待预测仅请求 Fx 时竖直的 T5/T6 是否需要出力，然后用纯计算最小例子核对。未运行 ROS、未创建临时文件。

学习者预测 T5/T6 需要出力，理由是浮力。纠正边界：浮力是仿真物理施加在机器人上的外力，不自动成为分配器输入；本题显式给定 Fz=0，仅请求 Fx。Agent 从当前源码 AST 提取安装方向、位置、invert4 与 _build_pseudoinverse，在内存中计算 wrench=(4,0,0,0)，得到 T1/T2≈+1.414427 N、T3/T4≈-1.414427 N、T5/T6=0 N，竖直合力=0 N。该纯计算验证了当前分配公式，不是 ROS 或 Stonefish 运行；实际闭环里若控制器为维持深度发出非零 Fz，则 T5/T6 可出力，需另看控制器输出与物理响应。无临时文件。

学习者追问分配矩阵是否自行调写。核对当前代码：directions 与 positions 六组安装参数直接写在类中，_build_pseudoinverse 根据每组方向 d 和位置 r 生成 4×6 的力/力矩矩阵，再计算 6×4 伪逆，初始化时只计算一次；并未手填矩阵 24 个元素，也无自动在线辨识或自适应调参。源码注释称参数来自测量安装方向与力臂，但本轮只核对代码与 SCN 中 T1–T6 静态定义，未验证实体测量表精度。区别于 PID 增益调节。若修改这些源码常量，Python 节点需重启才会重建矩阵。

## 推力到归一化命令

学习者要求下一步，核对分配器 force_to_pwm 第 162–181 行及 positive_curve/negative_curve、SCN 的 normalized_setpoint 和插值推力模型。此函数输入单个推进器的期望推力（N），绝对值 <=0.05 N 返回零；正负推力分别用不同曲线做反查与插值，输出限制 [-1,1] 的归一化带符号命令。名称中的 pwm 是工程接口名，ROS 话题为 Float64MultiArray，不能当作真实电路中的脉宽微秒值；Stonefish 的 thruster 模型再将其换算为推力。学习者待预测等幅正负推力是否会得到绝对值相同的命令，再做内存最小例子。未运行新实验。

学习者预测“不会”，并认为查命令符号。已纠正为期望推力 force 的符号选择正/负标定曲线，|force| 在对应曲线上反查命令幅值，最后恢复命令符号。Agent 从当前源码 AST 提取两条曲线、_inverse_curve 和 force_to_pwm，在内存中计算 +2 N→+0.106300853、-2 N→-0.185248222；绝对值不同，符合两条曲线不对称。仅验证当前 Python 算式，无 ROS/Stonefish 运行，无临时文件。下一段连接六路 out 数组、Float64MultiArray 发布、SCN 订阅与新 odometry 反馈。

## 六路命令进入仿真

核对分配器第 192–204 行和 publisher 定义：按 T1–T6 顺序把 forces 转为 out 数组，再以 Float64MultiArray 发布到 /bricsbot/setpoint/pwm。bricsbot_reference.scn 依次定义 T1–T6，并将 ros_subscriber thrusters 属性连接此话题。ROSScenarioParser.cpp 第 317–318 行创建 Float64MultiArray 订阅；ROSSimulationManager.cpp 第 836–845 行回调先核对数组长度，再按下标存入 thrusterSetpoints；第 480–489 行在执行器更新中依次调用 Thruster::setSetpoint。该源代码链表明正常情况下命令如何送到 Stonefish 执行器；本日先前 enable=false 实验只验证零命令的话题链，尚未启用推进器验证受控运动。待学习者预测 enable=true 后输出与路径效果，再做独立运行实验。

## 启用推进器的独立运行实验

学习者预测启用后“不一定”沿 41 点到终点，符合当前 nearest 固定为 0 的源码风险。实验前检查宿主机无相关运行容器/ROS 进程，11311/11381 无监听；检查发现 planar_pid_tracker.py 有学习者当前注释修改，保留未触碰。tracking.launch 与分配器源码无 diff。使用现有 Noetic 镜像、只读工程挂载、当前 Xauthority 和 GPU，在独立 Master 11381 启动：

```bash
roslaunch -p 11381 aquaflow_stonefish tracking.launch enable:=true graphics_quality:=low
```

容器名 underwater-objectnav-day7-enabled。等待五类话题就绪后，以约 20.03 秒墙钟窗口采样。频率按窗口内首末消息的单调时钟计算；本次窗口从话题就绪后开始，机器人已离开初始 x≈-5 m，不能据本窗口重建最初起步阶段。

| 话题 | 发布者 | 窗口消息数 | 约到达频率 |
|---|---|---:|---:|
| /bricsbot/odometry | /stonefish_simulator | 2003 | 100.015 Hz |
| /aquaflow/nominal_path | /nominal_reference_publisher | 40 | 2.000 Hz |
| /controller/generalized_force | /planar_pid_tracker | 400 | 20.000 Hz |
| /bricsbot/setpoint/pwm | /brics6_thruster_allocator | 400 | 20.000 Hz |
| /aquaflow/tracking_error/xy_norm_m | /planar_pid_tracker | 400 | 20.000 Hz |

确认 /brics6_thruster_allocator/enabled=true，400 条 PWM 均有六个值且至少一个非零，最大绝对值约 0.15817。参考 Path 始终 41 点，下标 2 的目标 (-4.5,0,0) m，终点 (5,0,0) m。窗口首尾 odometry 位置约 (-4.61661,-0.00119,0.20879) → (-4.49712,-0.00013,0.04646) m；x 最小 -4.61661、最大 -4.48055 m。x 在 -4.5 附近略有越过并回调，未沿完整 Path 接近 x=5 m。水平误差范数首尾约 0.11574→0.00291 m，窗口中最大 0.11574 m、最小 0.00139 m；误差并非严格单调。本结果是短时受控仿真表现，不证明长时稳定、所有初始状态收敛或实体机器人性能。

观察代码经标准输入运行，无临时脚本或大日志写入工程。实验容器已停止并自动删除，11381 已释放，未发现其他相关 ROS 进程；git diff --check 通过。保留学习者对 planar_pid_tracker.py 的注释改动和本日记录，不更改控制算法。下一步可进行第七天闭环复述与 3 个理解题、1 个找代码题；未完成独立验收前不标记整日通过。

## 第七天结束题首轮评价

学习者回答：1）期望路径和实际位姿来自 nominal 与 actualpath，控制器输出力与力矩、分配器输出 PWM；2）enable=false 时分配器“不工作”，控制器正常；3）第三个目标点导致停在 -4.5 附近；4）不知道目标选择代码。

评价：第 1 题后半段职责正确，需纠正实际位姿：控制器直接订阅 Stonefish 发出的 /bricsbot/odometry（nav_msgs/Odometry）；actual_path_recorder 也订阅该话题，另将累计历史位姿发布成 /aquaflow/actual_path（nav_msgs/Path），tracking 控制器没有订阅 actual_path。第 2 题控制器正常正确，分配器“不工作”不精确：它仍运行并按 20 Hz 发布六个零；仅跳过非零力分配分支。第 3 题核心判断正确，需能指出当前 nearest=0 和默认 lookahead=2，以及运行中 x≈-4.497 m 与 Path 下标 2 的 -4.5 m 对应；不把 20 秒短时结果说成长期稳定。第 4 题尚未通过：planar_pid_tracker.py update 中 target=self.path.poses[min(len(self.path.poses)-1,nearest+self.lookahead)].pose；min 防止下标超过 len-1。Agent 已按当前源码复核并讲解，待学习者用自己的话复述第 1/2/4 题再评价；本轮未运行实验或改控制代码。

## 纠正复述与综合关卡

学习者随后准确复述：控制器的实际位姿直接来自 Stonefish 发布的 odometry；禁用时分配器仍发布六个零；`min()` 防止目标点数组下标越界。第 1/2/4 题的上述误解已纠正，结合第 3 题，第七天 3 个理解题与 1 个找代码题通过。第七天“期望路径→控制器→广义力→分配器→六路命令→仿真里程计”主线已由源码和 enable=false/true 两组短时运行证据支持。

计划中最终关卡还要求学习者向师兄连贯解释场景、SCN/URDF 与运动闭环，并给文件依据；该完整口述尚未由学习者独立完成。本轮继续邀请简短综合复述后再评价，不把短时 x≈-4.5 m 表现扩大为完整 41 点路径跟踪或长期稳定。此轮仅更新学习记录，无新实验与控制源码修改。

## 综合口述评价与第七天收尾

学习者独立口述了：aquaflow_pool.launch 根据场景选 SCN；SCN 用于 Stonefish 的仿真环境与机器人，URDF 主要供 RViz 显示且应与 SCN 对应；nominal_reference_publisher.py 提供目标路径，经 planar_pid_tracker.py 和 brics6_thruster_allocator.py 控制，反馈回里程计。语音转写中 SCN、empty_pool、random_pillars 及文件名有音近词，按其表达意图评价，不当作概念错误。

对照当前源码，口述的三条主线均正确，第七天学习主线和综合关卡记为完成。Agent 补充精确文件与缺口：aquaflow_pool.launch 的 scene 分支选 empty_pool/single_obstacle/random_pillars 对应 SCN，tracking.launch 默认 random_pillars+bricsbot；visualization.launch 将 bricsbot.urdf 加入 robot_description 供 RViz RobotModel 使用，SCN 与 URDF 不自动同步；完整闭环还需明确分配器发 /bricsbot/setpoint/pwm 六路命令，经 SCN 的 ros_subscriber 进入 Stonefish，仿真物理更新后发布 /bricsbot/odometry 再供控制器使用。上述补充由 Agent 给出，未要求用户再次默写；不将其记录为用户独立说出的每个细节。

验收范围是源码阅读、两组短时运行和学习者口述；仍未证明 41 点完整路径跟踪、长时稳定或实体机器人表现。无新实验，保留本学习记录及用户已有源码注释，已清理先前实验容器与临时观察进程。
