# 第 9 天：键盘到动作执行器

日期：2026-10-26（Asia/Shanghai），按学习者要求收尾；结束题跳过。

## 目标与入口

学习者基于此前 STM32 小车控制经验，明确跳过第八天。按当前《学习方法.md》第九天，理解动作请求如何生成目标位姿，再交给串级控制器和推进器分配器执行。

核心文件：`src/robot_control/scripts/keyboard_reference_publisher.py`、`scripts/action_executor.py`、`scripts/cascaded_pid_tracker.py`、`config/action_executor.yaml`、`msg/ActionRequest.msg`（后四项相对 robot_control 包）。状态消息和启动配置按需核对。

调用链：按键 → ActionRequest（/underwater_objectnav/action_request）→ action_executor → PoseStamped（/aquaflow/nominal_pose）→ cascaded_pid_tracker → WrenchStamped（/controller/generalized_force）→ allocator → PWM → Stonefish → Odometry 反馈。action_executor 也订阅实际里程计，负责目标生成及动作进度判断。

## 第一小段

先读键盘节点的 ACTIONS 映射和 process_key() 的消息构造、发布部分：合法且通过重复间隔检查的按键，生成带编号、动作标签、请求时间的自定义 ActionRequest；键盘节点不直接计算力或 PWM。ROS 1 自定义 .msg 与 ROS 2 自定义消息在接口描述用途上相近，当前实际运行与构建仍是 ROS 1 Noetic/Catkin。

接下来阅读 action_executor.make_target() 的 FORWARD 分支，解释如何用实际位置与航向把动作变成世界坐标中的目标点。

当前 YAML 前后距离为 0.25 m，turn_angle_deg=-15.0（旁边注释含旧数值，不作为参数依据），command_delay_s=5.0、target_z=3.0，initialize_depth_before_actions=true。后续讲解和实验按实际加载参数核对，不将收到请求视为立即前进。

本轮仅静态核对计划与源码，未检查运行状态、启动节点或执行前进动作。正式实验前须检查现有控制链、Master 及发布者，先禁用分配器核对链路；不同时启动第七天的 planar_pid_tracker。未修改控制代码，未创建实验临时文件。

## 键盘读取与消息发布的职责

学习者询问 KeyboardActionPublisher 与 ActionRequest：已解释前者是普通 Python 类，封装节点功能，ROS 1 节点由 main() 中 rospy.init_node 初始化；后者是由 ActionRequest.msg 生成的消息类，ActionRequest() 创建消息对象。学习者已表示理解普通类与节点的区别。

学习者进一步询问键盘输入是否通过 ROS 订阅，要求流程图。重新核对实际 main()：tty.setcbreak 配置终端按字符输入；select.select 检查标准输入是否可读（单次等待最多 0.05 s）；sys.stdin.read(1).lower() 读取并转小写；main 直接调用 node.process_key(key)。process_key 不是键盘订阅回调，通过有效按键和间隔检查后构造 ActionRequest，再由 request_pub.publish(msg) 发布。ROS 订阅发生在下游 action_executor；当前键盘节点未创建 Subscriber。此轮用源码流程图讲解，未实际读取终端按键、运行 ROS 通信或验证运动。

## 前进请求到目标位置

学习者表示已理解键盘读取与 ROS 发布，要求继续。下一小段读 action_executor.make_target 的 FORWARD 分支，桥接调用链：request_cb 收到请求并排队 → Timer/update 尝试 start_next → 满足条件后读取 current_pose → make_target。目标基于动作实际开始时的有效里程计，而非按键时的位姿。

讲解世界坐标 x/y 与机体前向的区别，前进距离用 cos(yaw)/sin(yaw) 分解到世界坐标；距离单位 m，yaw 输入三角函数时为 rad。按当前 YAML forward_distance_m=0.25，给出当前 (1,2) m、yaw=0 时目标 (1.25,2) m 的说明。待学习者预测同样起点、yaw=π/2 时目标 x/y。本轮尚未运行计算或 ROS 实验；目标深度、姿态打包和动作调度分支后续继续。

学习者回答目标为 (1,2.25) m，正确，通过本次前进方向预测。继续讲解 make_target 末段：创建 PoseStamped，frame_id 使用配置 world_ned，时间戳取当前 ROS 时间，写入目标 x/y；z 使用配置 target_z=3.0 m，而非直接沿用当前 z。FORWARD 分支保持当前 yaw，再转成四元数写入 orientation。make_target 返回消息本身，不在函数内发布；start_next 随后调用 reference_pub.publish(target)。此轮只读源码及更新学习记录，未执行 ROS 或运动实验。

## 请求回调与排队时间

学习者要求继续，转入 request_cb。核对 Subscriber 将 ActionRequest 关联到 request_cb；类方法名本身不自动产生 ROS 回调。回调规范化并检查动作名，按回调当前 ROS 时间计算 due，保存 (due,msg) 到 pending，并更新 last_scheduled_time。首次用 initial_delay，之后用 max(now,last_scheduled_time)+command_delay；due 是最早允许开始的调度时间，实际开始仍受当前动作、深度就绪与里程计有效性限制。

当前 YAML 及 keyboard_reference.launch 默认 initial_delay=0 s、command_delay=5 s。待学习者预测：没有历史请求，三条有效请求依次在 ROS 时间 10.0、10.2、10.4 s 到达，排除队列重置等其他变化，各自 due 为多少？第一条 due=10.0 s 作为示例，后两条待回答。尚未运行定时或队列实验，本轮仅源码讲解。

学习者要求直接给答案并继续，不等待本题预测。Agent 说明第二条 due=15 s，第三条 due=20 s；不记为学习者独立答题通过。当前段按直接讲解推进，不再插入预测题。

## 开始执行队首动作

继续读 start_next 的共同入口：已有 current_request 时返回；depth_ready 为 false 时返回；队列为空或队首 due 尚未到时返回。满足后 popleft 取出队首，current_pose 检查实际里程计；无有效位姿则将请求放回队首，最早重试时间设为当前传入 now+0.1 s。current_pose 将零时间戳临时视为当前时间，并按非零配置的超时阈值检查过期消息，因此不能笼统描述为所有无新鲜时间戳数据都会被拒绝。

有效位姿下调用已学过的 make_target，保存 hold_target/current_target/start_time，发布 PoseStamped 及 STARTED 状态；普通运动动作最终设置 current_request，STOP 有单独分支，后续再读。update 由 Timer 名义 20 Hz 调用，未开始的请求会在后续周期继续尝试，不是在 return 内原地等待。实际先决条件、目标发布及机器人运动本轮均未运行验证。

## 动作完成、超时与保持目标

学习者要求下一步，继续直接讲解 update 中的动作执行分支及 finish_current。distance=math.hypot(dx,dy) 是世界坐标中的水平距离（m）；yaw_error 是 wrap 到最短角差后取绝对值（rad）。FORWARD/BACKWARD 仅比较水平距离是否 <=position_tolerance，转向动作比较航向误差是否 <=yaw_tolerance。当前 YAML 为 0.035 m、3 deg，action_timeout=15 s；此判断没有要求平移动作同时达到深度/航向容差或速度为零。

达到条件调用 finish_current("SUCCEEDED")；未到位且执行时间超过正值超时阈值才调用 TIMEOUT。源码中这两项均在 pose is not None 内，实际里程计不可用时本轮不会进入成功或动作超时判断；未修复此实现。finish_current 发布带 action_id、目标及起止时间的 ActionStatus，成功时保留原目标；失败时若有可用位姿，则生成保持当前水平位置和航向、配置深度的目标。随后清空当前执行状态并立即尝试 start_next，仍须满足下一个请求的调度条件。

update 末尾持续刷新时间戳并发布 current_target；没有当前动作时按 hold_reference_when_idle 等条件发布 hold_target。动作完成不等于关闭控制器或将 PWM 清零。此轮仅源码与配置核对，未运行到位、超时或失去里程计实验，暂不标记第九天运行验收完成。接下来可读串级控制器如何将 PoseStamped 目标与 Odometry 反馈变成广义力；STOP 与转向符号等分支仍可按需补读。

## 键盘输入与开闭环的区别

学习者询问键盘控制是否开环、前两天路径控制是否闭环。核对 cascaded_pid_tracker 的里程计与目标订阅、位置误差、速度误差和力发布源码，说明当前完整键盘动作链同样使用反馈，属于闭环控制；键盘仅决定目标来源，不能据此判定开闭环。action_executor 也使用 odometry 生成目标和判断完成。与第七天区别在于输入为动作生成的 PoseStamped、控制器为位置/速度串级，而第七天是 Path 选点和 planar_pid_tracker；两者均用 odometry 反馈。

用 STM32 对照：按键后不依据反馈修正的固定 PWM/固定时长驱动属于该运动层面的开环例子；按键设定移动距离，再依据位置或编码器反馈不断修正输出则是闭环。此轮仅确认源码结构，未运行当前键盘控制链，学习者尚未复述此判断。

学习者随后正确概括：前面用 nominal 发布的参考作为目标，现在由键盘请求提供目标来源，两者后面都有 PID 控制。Agent 补充键盘发布动作标签，由 action_executor 转换成目标位姿；两条链使用不同控制器，反馈都来自 odometry。学习者表示理解并要求继续。

## 串级控制器的位置外环

核对 cascaded_pid_tracker.update 的位置模式，以及 cascaded_pid_control.launch 加载的 bricsbot_cascade_pid.yaml。按学习者当前要求直接讲解，不插入预测题。位置误差从世界坐标转换为机体系 ex/ey，结合深度与航向误差形成外环输入；位置环输出 vref=[期望前向速度、期望侧向速度、期望垂向速度、期望航向角速度]，前三项单位 m/s，末项 rad/s，并非直接输出力。

只展开前向分量：vref[0]=clamp(kp_pos_xy[0]*ex+ki_pos_xy[0]*pos_i[0],-max_vel_xy[0],max_vel_xy[0])。当前 YAML 的 kp_xy=[2,2]、ki_xy=[0,0]、max_velocity_xy=[1,1]，不同于脚本缺省值；需由实际启动加载且无其他覆盖才采用这些值。以机体系前向误差 0.25 m 说明外环期望速度 0.5 m/s；误差 0.05 m 对应 0.1 m/s，均未触发限幅。例子为按公式手算，非实际机器人测得速度；期望速度仍需下一层速度环结合实际速度反馈来实现。本轮只读源码、配置及更新记录，无新实验或控制源码修改。

## 略过重复 PID 公式，核对接口和数据超时

学习者确认控制环与此前内容相近，同意略过重复 PID 细节，转向接口连接与超时行为。控制器将经过限幅的 [Fx,Fy,Nz,Fz] 填入 frame_id=base_link 的 WrenchStamped，发布 /controller/generalized_force；Brics6Allocator 订阅同名同类型话题，分配推力并发布六元素 Float64MultiArray 至 /bricsbot/setpoint/pwm，连接到已学过的仿真执行链。

当前控制器配置 odom_timeout=max_state_age=0.25 s、reference_timeout=0.50 s；丢失输入或时间戳过旧等条件触发积分状态重置与零广义力发布。分配器仍运行且没有其他发布者干扰时，若广义力停止更新，默认 wrench_timeout=0.25 s 后在其下一次 update 发布六个零；禁用分配器同样发布六个零。这里是 ROS 时间与消息时间戳判断，不是距离最近一次按键的时间。action_executor 会周期更新并发布目标位姿，因此松开按键不会直接使参考超时或令 PWM 归零。

查阅 tracking_urchin.launch 发现它还启动 data_generate/objectnav_data_collector，不能未经核对就把它当作只含键盘控制的最小实验入口。本轮未启动此 launch、采集数据或进行超时/运动实验，亦未修改控制代码。保留第九天禁用链路与单次前进实验待办。

## 转向标签、角度与 NED

学习者表示理解接口和超时，要求继续。读取键盘 ACTIONS、make_target 转向分支与当前 YAML：a→TURN_LEFT→target_yaw+=turn_angle；d→TURN_RIGHT→target_yaw-=turn_angle。配置 turn_angle_deg=-15.0，经 math.radians 转为弧度；因此当前配置下左转目标航向变化 -15 deg、右转 +15 deg。NED/FRD 的 z 向下，按右手规则从水面上方向下观察，正 yaw 对应顺时针右转；与常见 ROS ENU/FLU（z 向上）正 yaw 俯视左转的差别来自坐标约定，不是 ROS 版本。

当前 YAML 同行注释中的 -30.0 为不符实际参数的旧说明，教学以数值 -15.0 和执行分支为准，未修改用户文件。转向分支保留起始水平目标位置，改变目标航向；深度仍为配置 target_z。本轮结论来自配置、源码和坐标规则推导，未运行按键转向或验证实际旋转。下一小段可补 STOP 的保持目标与排队行为，然后准备禁用链路和单次前进的最小运行实验。

## 键盘链的 launch 入口

学习者表示已理解上述内容，询问具体由哪个 launch 组合启动。核对当前文件：aquaflow_stonefish/launch/tracking_urchin.launch 通过 include 启动 rock_seaurchin.launch→stonefish_ros/simulator.launch（stonefish_simulator），并 include visualization.launch（RViz、相机静态 TF、robot_description）；通过 node 直接声明 keyboard_reference_publisher、action_executor、cascaded_pid_tracker、按 vehicle_model 选择的推进器分配器，还启动 objectnav_data_collector。没有 include robot_control 的三个拆分 launch；nominal/reference_tuning 发布节点目前被注释。

拆分入口 keyboard_reference.launch 只启动键盘发布器和 action_executor；cascaded_pid_control.launch 只启动串级控制器；action_executor.launch 只启动执行器。第七天 tracking.launch 是 nominal Path+planar_pid_tracker 另一条链，不可拿它代替当前键盘串级链或叠加同时控制。

当前 tracking_urchin.launch 参数 command_delay_s=2、target_z=3.3，会覆盖 action_executor.yaml 的 5 s/3 m；enable 默认 false。选择 scene=random_pillars、vehicle_model=bricsbot 才按该场景映射选 BricsBot，不能只凭根 launch 的 vehicle_model 默认值认为 empty_pool 场景自动更换机器人。节点的通信由话题名及消息类型匹配，launch 负责拉起进程、加载配置，XML 顺序不保证下游已就绪。

键盘脚本 main 中明确检查 sys.stdin.isatty()，否则退出；output=screen 只说明输出位置，不能证明有可读键盘的交互终端。当前仅静态检查 launch，未实测这些入口的 stdin/TTY 或启动仿真、采集。后续实验须采用真正可交互的键盘终端并避免同名节点重复启动；不把 launch 配置声明视为键盘已可操作的运行证据。未修改任何启动或控制文件。

## 准备手动键盘体验

学习者要求提供启动步骤，想亲自操作键盘。宿主机权限下只读检查 docker ps、相关进程与端口：无运行容器或匹配的 ROS 进程，11311/11391 未监听。沙箱内 Docker socket 拒绝访问不视为 Docker 故障。现有 Noetic 镜像可用，DISPLAY=:1、XAUTHORITY=/run/user/1000/gdm/Xauthority；devel/lib 下已有可执行 Catkin Python 转发脚本，原始 .py 没有执行位不妨碍通过这些已有入口启动。

新增可复用学习入口 learning/day-09/keyboard_control.launch：包含 random_pillars BricsBot 仿真、可选 RViz、action_executor.launch、cascaded_pid_control.launch，及默认禁用的 Brics6Allocator；键盘程序单独在交互终端 rosrun，不启动采集器或第七天的 planar tracker。沿用当前 YAML，不修改原始算法、参数文件或既有总 launch。实际解析参数为目标深度3 m、前进距离0.25 m、后续请求调度间隔5 s，enabled=false。

通过无网络、只读挂载的自动删除临时容器运行 roslaunch --nodes /workspace/learning/day-09/keyboard_control.launch 和 --dump-params，节点及上述关键参数解析通过。未启动 roscore、图形仿真、控制器或运动；参数转储仅在临时容器内，容器已删除。git diff --check 通过。

提供两终端手动步骤：两个宿主机终端各执行 ./docker/run.sh shell；容器内均 export ROS_MASTER_URI=http://localhost:11391。终端1先 roslaunch -p 11391 /workspace/learning/day-09/keyboard_control.launch enable:=false；终端2 rostopic echo -n 1 /bricsbot/setpoint/pwm 检查六个零。通过后终端1 Ctrl+C 完整停止，再以 enable:=true 重启；等待到达深度提示后，在终端2 rosrun robot_control keyboard_reference_publisher.py，轻按 w/s/a/d 体验。告知空格 STOP 也排队，非立即中断；q 仅退出键盘进程，停止整套仿真用终端1 Ctrl+C。等待学习者实际执行反馈，不预记实验完成。

## 学习者手动体验反馈

学习者报告已体验完键盘控制，并反馈请求积压、不能连续控制（原话“这个请求确实太满了，不能连续控制说是”）。结合当前源码解释：每个有效按键生成独立动作请求，长按的键盘重复输入可能持续入队；后续调度时间按5 s间隔排列，且 current_request 未完成时不启动下一动作，因此松键后仍可能继续执行积累的请求。此链面向离散移动/转向动作，连续速度遥控需不同的参考与输入更新策略，仅调小调度间隔不能自动变成连续遥控。

此项为学习者实际操作自述，Agent 未独立采样轨迹、消息、动作状态或量测位移，未确认每个禁用检查步骤及深度/到位容差均通过；不能据此登记所有运行验收通过。未更改队列逻辑、调度间隔或控制参数，也未停止任何学习者运行的进程。第九天结束题仍未进行。

## 第九天收尾与后续安排

学习者表示已经理解本日内容，明确要求本次不出题，转而了解后续学习内容和能力提升。本日按要求收尾，不再安排结束题；状态为源码主线已讲解、学习者报告完成键盘体验，正式结束题跳过，定量运行指标仍未独立核验。保留可复用启动文件 learning/day-09/keyboard_control.launch 与本记录；本次解析验证的临时容器已自动删除，没有可丢弃实验脚本或大日志留在工程。学习者手动启动的进程状态未在收尾时检查或更改。

按当前《学习方法.md》后续依次为：第十天 Stonefish 本体与 ROS 接口调用链；第十一天 MPC 的模型、预测窗口、代价与滚动执行；第十二天教师路径及专家数据同步采集；第十三天 RGB-D、检测、地图与 BC 动作策略；第十四天单次在线观察到动作执行的全链及跨进程推理。后续先继续第十天，不自动在本轮开始新课程。目标是独立追踪源码、解释接口和用小实验定位问题，理论推导与大规模训练不作为这五天的入门完成标准。
