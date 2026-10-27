# 第 4 天：机器人、传感器怎样放进场景

日期：2026-10-25（Asia/Shanghai）。源码分段阅读、Agent 副本实验、用户修复后的手动观察及结束理解验收均已完成。

## 目标与核心文件

`aquaflow_pool.launch` 选场景 → `aquaflow_random_pillars.scn:366` include 机器人 → `bricsbot_reference.scn` 定义本体、执行器、传感器及 ROS 接口。对照 `bluerov2_reference.scn`。这些路径均在 `src/aquaflow_stonefish/` 下。

- BricsBot：T1–T6 六个推进器，前四个水平布置，后两个竖直布置；另有 odometry、pressure、depth_camera、rgb_camera。BlueROV2 对照文件实际定义八个推进器及其他传感器；本日未启动 BlueROV2。
- `base_link` 是复合刚体，由外壳、配重、浮力部件组成。`physics="submerged"` 选择水下物理处理；整机初始位姿由 robot/world_transform 设置。
- 相机 link 指向 base_link；origin 是相对本体的安装位姿，RGB 与 Depth 同为 `(0.20,0,-0.02)` m。机体轴 FRD（前、右、下），相机光学轴 Z 向前。示意图见 `day-04/camera_mount.png`，可编辑矢量版同名 SVG；外形未按比例。
- 相机 15 Hz 与物理 100 Hz 职责不同，不是每个物理步都生成新图。相同安装位姿和配置频率不能证明 RGB/Depth 消息严格同步。
- 深度图有效像素是相机光轴方向深度，单位米，不是池底水深或机器人下潜深度。本地 Stonefish 源码与 Dockerfile 固定提交均为 `b21eb8e194c570ff2f61e91aeffb38d73dc25f42`；已核对默认 depth 模式、单位及本工程投影公式。深度图中的 0 不能直接当成 0 m 有效距离，本版本对未命中有效范围的背景输出 0。
- 里程计来自仿真状态，不是通过 RGB/Depth 定位得到。类型 nav_msgs/Odometry；pose 相对 world_ned，速度在传感器坐标系表达；当前零安装偏移使其与本体对齐。位置用 pose.pose.position，姿态为四元数。
- `parsed_simulator` 是可执行程序，当前 launch 设置节点名 stonefish_simulator。ROSInterface 是工具类，其源码编译进共享库，由同一程序调用；SCN 不启动这个类或独立节点。先初始化 ROS，再读 SCN 创建仿真实体及发布/订阅接口；一个节点可以有多个接口。学习者已准确复述并确认理解。
- SCN 的 publisher/subscriber 将仿真实体接到话题，不按 cpp/py 文件名直接连接节点。运行时控制数组属于消息数据，不是 ROS 参数。已用 rclpy create_subscription/create_publisher 作 ROS 2 对照；该教学代码只保存指令，没有物理后端，未运行或用于驱动。
- 推进器数组按 SCN 定义顺序对应 T1–T6，不按名称自动排序。ROS 回调检查长度等于六，否则拒绝整条消息；接收后保存数组，仿真更新中再逐个调用 setSetpoint。原四元素预测未作答，后续 ROS 2 示例已展示答案，不算独立通过。
- 六个推进器 normalized_setpoint 均为 true，输入为 [-1,1] 无量纲指令；T1 的 max_setpoint=376.991118 rad/s，inverted_setpoint=false。话题名 pwm 不意味着实际微秒值，也不是牛顿或推力百分比；后续转子及推力模型再计算力。具体力学推导留待后续学习日。

## 已回答的预测与应用

1. 整机初始 Z 从 0 改为 1：学习者答向下移动、柱子不动，正确；未修改或运行此位置实验。
2. 机器人初始 `(-5,0,1)` m、姿态全零，相机位置：答 `(-4.8,0,0.98)` m，正确。内联读取 SCN 偏移并作数值计算，非实测位置。
3. 100 Hz 物理、15 Hz 相机是否每步生成新图：答不会，正确。
4. 识别柱子再判断远近：答先 RGB 再 Depth，正确；随后补充深度含义。
5. 检查是否到达世界目标位置：答 pose.pose.position，正确。
6. T1 输入 2 是否被限制：答被限制，正确。内联按源码公式数值复现：输入 0/0.5/1/2/-2 对应角速度设定值 0/188.495559/376.991118/376.991118/-376.991118 rad/s；未执行 C++ Thruster 对象、未驱动或实测转速/推力。

## 运行实验与故障边界

宿主机检查确认无运行容器，未发现相关 ROS 进程，11311/11321/11331/11341 未监听；镜像存在、DISPLAY=:1，显示授权文件可读。沙箱无法访问 Docker/网络检查时改用宿主机执行，不将权限报错当成 Docker 故障。实验使用独立 Master 11341，仅启动仿真，无控制器、无推进器命令。

首次用原父 launch、scene=random_pillars、vehicle_model=bricsbot 启动失败：当前原机器人文件第 119 行存在未提交修改 `$(arg vehicle_nam e)`，比正确的 vehicle_name 多一个空格，触发解析器重复报错；只见初始节点/话题，未收到 RGB，观察超时。已停止该实例，不能将其记为原场景运行成功。

保留原文件修改不动，在 `day-04/scenarios/` 建学习副本：机器人副本只修正这个空格；场景副本只将机器人 include 指向副本。随后从宿主机工程根目录运行：

```bash
docker compose -f docker/compose.yaml run -d --rm \
  --name underwater-objectnav-day4-sensors \
  -e ROS_MASTER_URI=http://localhost:11341 \
  -e XAUTHORITY=/tmp/day4.xauth \
  -v /run/user/1000/gdm/Xauthority:/tmp/day4.xauth:ro \
  underwater-nav roslaunch -p 11341 stonefish_ros simulator.launch \
  simulation_data:=/workspace/src/aquaflow_stonefish/data \
  scenario_description:=/workspace/learning/day-04/scenarios/sensor_scene.scn \
  simulation_rate:=100.0 graphics_resolution:='1200 800' graphics_quality:=low
```

容器内观察进程加载 `/opt/ros/noetic/setup.bash` 及 `/workspace/devel/setup.bash`，确认连接 localhost:11341，使用 rospy.wait_for_message 获取样本，ROS Master 查询发布/订阅者；只读结果保存在 `day-04/sensor_evidence.json`。

| 实际话题 | 类型 | 本次样本 |
|---|---|---|
| /bricsbot/rgb/image_color | sensor_msgs/Image | 640×480，rgb8 |
| /bricsbot/depth/image_depth | sensor_msgs/Image | 640×480，32FC1，中心像素约 1.134860 m |
| /bricsbot/odometry | nav_msgs/Odometry | frame=world_ned，位置约 (-4.998766,-0.003220,0.258076) m |
| /bricsbot/rgb/camera_info | sensor_msgs/CameraInfo | 640×480，fx/fy≈457.007 |
| /bricsbot/depth/camera_info | sensor_msgs/CameraInfo | 640×480，fx/fy≈457.007 |

这些传感器发布者均为 /stonefish_simulator。`/bricsbot/setpoint/pwm` 无发布者，订阅者为 /stonefish_simulator，直接查询节点接口确认类型 std_msgs/Float64MultiArray。观察进程是额外的临时 ROS 节点，另有 /rosout；不能说整个实验系统只有一个节点。

首次成功采样后，一次补充 rosnode info 未能联系该节点，原因未确认。因有此疑点，顺序停止并重启同一学习副本，直接查询节点 XML-RPC 接口并作约 8.01 秒墙钟持续接收：RGB 121 条、Depth 121 条、Odometry 801 条，结束时节点仍可查询。此结果只证明短时数据链工作，不代表长期稳定性；采样顺序不同，未验证 RGB/Depth 时间同步，也未独立测量物理步频或图形 FPS。

## 改动、清理与剩余项

只新增本记录、两份学习场景副本、精简 JSON 证据及相机安装示意图 PNG/SVG。未修改原 SCN、控制逻辑或构建产物；原压力话题拼写问题保留，默认 BricsBot 启动路线仍需修正后才能正常使用。运行容器、观察进程均已结束，确认本次容器不存在、11341 已释放；未更改 X11 权限，授权文件只读挂载，无额外临时脚本或缓存遗留。

结论范围：修正副本上的传感器接口已实际核对；未验证推进器驱动、受控跟踪或在线导航。结束验收（3 个理解题和 1 个找代码题）待学习者回答，不提前认定第四天全部通过。

## 后续手动观察准备

学习者暂缓结束题，查看原文件拼写错误后自行修复。再次读取原文件确认第 119 行现为 `$(arg vehicle_name)`；前文“原错误保留”描述的是 Agent 实验当时的状态，不再代表当前文件。尚未运行用户修复后的原父 launch。

学习者要求亲自在终端启动并用命令行或 RQT 看数据流。只读检查时存在一个以 bash 为命令的用户容器，默认 Master 为 localhost:11311，11311 和 11341 均未监听。容器中已确认 rqt_graph、rqt_image_view、rqt_plot 可执行文件存在。Agent 未启动新仿真、未停止用户容器；提供宿主机进入 shell、容器内启动原 BricsBot 场景、另一观察终端查询节点/话题/消息及 RQT 的步骤。此手动操作结果待用户反馈，不能记为原场景已成功。

## 用户手动实验结果

学习者随后提交终端原始输出与两张 RQT 截图，确认已按步骤完成实验。本阶段属于用户自行修复并启动后的结果，区别于前面的 Agent 修正副本实验。

- 初始 rosnode list 为 /rosout、/stonefish_simulator。rosnode info 列出 RGB/Depth、里程计、压力等发布接口，推进器订阅为 std_msgs/Float64MultiArray；初始实际 Connections 只有日志，表示当时还没有接入图像查看器等传感器订阅者。
- rostopic info /bricsbot/odometry 显示发布者为 /stonefish_simulator、当时无订阅者。三次 echo position 得到 x≈-5.00044、y≈0.000291、z≈0.04859 m；位置接近初始值，小幅变化不能用来证明控制器工作。
- rostopic info /bricsbot/odometry/pose/pose/position 报 Unknown topic：这是消息字段路径，不是独立话题；rostopic echo 支持在话题后附加字段路径，故能读取该位置字段。
- 用户测得里程计平均约 100.020 Hz、RGB 约 14.982 Hz，支持消息流持续输出及与配置频率接近；不是图形 FPS 或独立物理步频测量。
- RQT Image View 已选择 /bricsbot/rgb/image_color 并显示水池/柱子场景。用户追问谁订阅图像，Agent 只读查询仍在运行的 localhost:11311 Master：发布者 /stonefish_simulator，实际订阅者 /rqt_gui_cpp_node_226。直接查询后者 getSubscriptions 也确认订阅 sensor_msgs/Image。该节点来自 rqt_image_view；命令后台进程 226 与图中节点后缀对应。rqt_graph 对应 /rqt_gui_py_node_214。
- 图截图里 Group/Images 勾选，使图像传输话题聚合显示为 image_topics；图中未呈现已实时确认的图像订阅边，可能是选择话题前的快照或尚未刷新，具体原因不能仅凭截图确认。可关闭 Images 分组并刷新，检查 simulator → image_color → rqt_gui_cpp_node_226。

当前原场景已经得到用户手动运行的节点、消息频率和实际图像证据；原拼写导致无法启动的历史状态不再代表当前结果。用户仿真与 RQT 保持运行，Agent 未停止任何用户程序。第四天结束题仍按用户要求暂缓，未判定整日验收通过。

## 第四天结束题首轮评价

1. 相机 origin 是否随机器人前进而修改：答“不需要，因为是相对机器人的位置”，正确，通过。
2. 能否由相机 15 Hz 判断物理步频/FPS：答“不可以，因为这是相机自己仿真的结果”。判断正确，理由不够准确；已补充 rate=15 Hz 是相机更新频率配置，物理步频是物理状态更新节奏，图形 FPS 是画面绘制节奏，不能由其中之一推出另两者。待学习者补充确认。
3. 为什么 ROSInterface 不作为节点出现：答“只是发布者和订阅者的声明的文件，节点只有一个”。已抓住无需独立节点，但对类与实现的描述有误；ROSInterface 是带实际消息组装/发布方法的 C++ 工具类，头文件声明方法、cpp 实现方法，ROSScenarioParser 主要负责根据 SCN 创建通信接口。方法在 parsed_simulator 进程内被调用，不因类名或文件名而自动成为节点；当前仿真程序的一个节点不能推广为整个系统只有一个节点。待学习者补充确认。
4. 找 RGB 类型和频率所在行：贴出 `<sensor name="rgb_camera" rate="15.0" type="camera">`，与当前第 136 行完全匹配，正确，通过。

本轮仅更新记录与核对源码，没有运行、停止或修改用户的仿真与 RQT。第四天尚未标记全部验收通过，等待第 2、3 题补充理解。

## 补充回答与最终验收

学习者补充第 2 题：“15hz 描述的是相机的更新频率配置”，正确，结合此前已能区分物理步频与显示 FPS，本题通过。

学习者补充第 3 题：“因为它是一个 cpp 的工具类，直接在进程中被调用”，理解正确，精确术语为 C++ 工具类；其方法由 parsed_simulator 进程内的代码调用，无需单独启动节点，本题经纠正和复述后通过。

第四天四道结束题全部通过：第 1、4 题首轮正确，第 2、3 题补充确认后通过。当天完成范围为机器人/传感器/推进器接口的源码阅读、相机安装几何与限幅公式的数值核对，以及实际 ROS 节点、传感器消息、消息频率和 RQT 图像观察。未验证受控运动、推进器实际推力、长期稳定性或在线导航。

Agent 创建的实验容器和观察进程已清理；用户自行运行的仿真和 RQT 本轮未检查、未停止，不假定其仍在运行或已退出。学习记录、场景副本、图示和精简证据保留在 learning/day-04.md 与 learning/day-04/。本轮只更新本记录，没有新增临时文件。

下一学习日按第五天计划：对照 BricsBot URDF、visualization.launch 与 SCN 加载链，理解当前工程中显示模型和物理仿真的职责，并按计划安排隔离副本实验。开始前重新核对用户当前文件与运行状态。

第四天后续扩展：学习者询问以后自行用 ROS 2 编写同一节点中多个发布/订阅接口的组织方式。继续沿用中文 ROS 工程陪练，核对本机 Jazzy rclpy 与官方示例；以一个 Node 类中保存多个 publisher/subscription 对象、回调接收指令、专门方法发布已有传感消息为结构对照。代码仅展示接口组织，未接入 Stonefish、未实现物理或图像生成、未运行；发布方法仍需由真实后端数据就绪时调用，spin 不会自动产生传感器数据。没有修改原 ROS 1 实现或启动新节点。

进一步扩展：学习者询问 Stonefish 中 SCN 与自写 ROS 2 节点应如何规划。核对本机 `/home/xiela/Downloads/stonefish_workspace/stonefish_ros2`：ROS2ScenarioParser.cpp 根据 ros_subscriber 创建 Float64MultiArray 推进器订阅，根据相机 ros_publisher 创建 image_color/image_depth 与 camera_info 发布；现有标准仿真节点已承担仿真与 ROS 桥接。建议先列接口约定表，再让 SCN 配置物理实体和传感/执行器话题，自写感知/控制节点订阅观测并发布指令，launch 同时启动仿真及算法节点并传场景路径。SCN 无需指定算法 py/cpp 文件路径。未来 ROS 2 结构为设计示例，未迁移当前 ROS 1 项目或实际启动；本机 ROS 2 启动参数名为 scenario_desc，区别于当前 ROS 1 的 scenario_description。
