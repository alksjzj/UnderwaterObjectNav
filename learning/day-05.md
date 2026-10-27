# 第 5 天：SCN 与 URDF，用加载链回答

日期：2026-10-26（Asia/Shanghai），学习进行中。

## 已阅读与理解

- 当前工程的 visualization.launch 将 BricsBot URDF 内容载入 robot_description，RViz RobotModel 读取；Stonefish 的 parsed_simulator 通过场景 SCN 引入 bricsbot_reference.scn，不读取这份 URDF。
- 学习者已准确复述两条加载链与配置一致性的要求；补充明确：SCN 定义物理属性、执行器、传感器和初始状态，实际运动由物理计算与运行时输入共同决定。RViz 正确显示还依赖 TF、位姿及传感器数据。
- scale 是网格缩放比例，不是米制长宽高；只改 URDF visual 不会改变当前 Stonefish 的外形或碰撞。显示缩放实验随后已运行，详见下节。
- Gazebo 扩展对照：URDF 可用于创建仿真机器人，加载时转换为 SDF；visual、collision、inertial 分别配置，完整世界和仿真功能仍需对应配置。只查阅官方资料，没有迁移或启动 Gazebo。
- 阅读 depth_camera_mount：固定连接 base_link 与 depth_camera，xyz 为相对位置（米），rpy 为相对旋转（弧度）。

## 预测与最小核对

预测：只把 URDF 的相机安装 X 改成 0.30 m，不改 launch 中静态 TF 发布器参数，实际相机 TF 会否改变？学习者答“会修改吧”，此预测不正确。

Agent 用 Python 标准库读取实际 XML，仅在内存副本中修改 joint/origin。输出：原 URDF 为 0.20 0.0 -0.02，修改副本为 0.30 0.0 -0.02；visualization.launch 的 bricsbot_depth_camera_tf 位置参数仍为 0.20 0.0 -0.02。该发布器直接读取 args，不从 URDF 读取安装位置；当前 launch 没有启动 robot_state_publisher。学习者回复“好的”，表示接受说明，尚未用独立应用题验证纠正后的理解。

此核对只证明两处配置输入相互独立，没有运行 TF 发布器、采样 TF 或启动仿真。没有修改原 URDF、SCN 或 launch，没有创建临时脚本。

## 当前小段与后续

继续拆解 ROS 1 tf/static_transform_publisher 参数。已核对 ros/geometry 官方 noetic-devel 源码：欧拉角参数顺序为 x y z yaw pitch roll parent child period_ms；末尾 100 为 100 ms 周期，即名义 10 Hz。它与 URDF 的 rpy 顺序不同，当前 roll 与 yaw 数值相等，不能据此把两种顺序当作相同。

官方源码：https://github.com/ros/geometry/blob/noetic-devel/tf/src/static_transform_publisher.cpp

学习者回答：末尾 100 改成 50 并重启发布器后，TF 发布更快，安装位置不改变。正确；50 ms 对应名义 20 Hz，本小段应用题通过。

下一小段核对：bricsbot_reference.scn 第 13 行 ros_base_link_transforms publish=true；ROSScenarioParser 读取该开关，ROSSimulationManager.cpp 第 398–399 行根据仿真机器人 getTransform() 发布 world_ned → bricsbot/base_link。visualization.launch 的两个静态发布器补齐 base_link → 相机；RViz 配置 Fixed Frame=world_ned。以上为源码/配置核对，尚未查询运行时 TF。

## URDF 显示副本实验（已运行）

宿主机重新检查时无运行容器、未发现相关 ROS 程序，11311/11341/11351 未监听；镜像存在，DISPLAY=:1，当前显示授权文件可读。沙箱 Docker/网络权限错误在宿主机检查中排除，没有据此诊断环境故障。

使用独立 Master 11351 和本次容器 underwater-objectnav-day5-display，只运行 RViz、相机静态 TF 及一个合成的固定 world_ned → bricsbot/base_link 变换。不启动 Stonefish 或运动控制器；本体固定在世界原点，是测试输入，不是实际仿真位姿。

- `day-05/urdf/original.urdf` 是当前原 URDF 的完整副本，`scaled.urdf` 仅把 visual/geometry/mesh 的 scale 从 1 1 1 改为 2 2 2；joint/origin 未修改。
- `day-05/display.launch` 包含工程原 visualization.launch，通过参数选择副本 URDF 和简化 RViz 配置；另增加合成世界位姿发布器。显示配置使用 0.1 m 网格。
- 实际读取 robot_description 确认两次模型缩放分别为 1 1 1、2 2 2；读取 TF 确认两次世界到相机平移均为 (0.20,0,-0.02) m，旋转四元数完全相同。
- 最终截图使用 RViz Python 接口将实际视角固定为 Distance≈1.65、Pitch≈0.65、Yaw≈0.7、Invert Z Axis=true，再读取实际应用窗口。两次记录的视角数值完全相同。original.png、scaled.png 均已查看，模型外观放大，背景网格一致；comparison.png 是相同画面区域的并排对照。透视投影下不以像素宽高比作为严格两倍的测量，缩放倍数由实际加载参数确认。
- 初步截图视角不一致，已被重拍替换。中间 Qt 内部抓取 OpenGL 区域出现图像失真，另一次观察程序缺少工程环境导致资源加载失败；均未作为成功证据保留。最终观察程序加载 Noetic 与 /workspace/devel/setup.bash 后无模型资源错误，以实际窗口截图验证显示。

宿主机工程根目录的复现入口（以 original 为例，需先核对本机显示授权路径和端口）：

```bash
docker compose -f docker/compose.yaml run -d --rm \
  --name underwater-objectnav-day5-display \
  -e ROS_MASTER_URI=http://localhost:11351 \
  -e XAUTHORITY=/tmp/day5.xauth \
  -v /run/user/1000/gdm/Xauthority:/tmp/day5.xauth:ro \
  underwater-nav roslaunch -p 11351 \
  /workspace/learning/day-05/display.launch variant:=original
```

切换副本需先停止本次容器，再用 variant:=scaled 重启。上述入口重现显示实验，最终截图另外固定了实际视角，不能假定任意交互后的视角仍相同。原始 URDF、原 visualization.launch、机器人 SCN 和父场景 SCN 的哈希在实验前后相同。

实验容器已停止并自动删除，11351 已释放；临时截图辅助程序及其容器内副本均已清理，没有改变显示访问权限。保留副本、显示配置、截图和精简 display_evidence.json。实验只验证 URDF 显示缩放及测试 TF 不变，没有验证 Stonefish 碰撞或运动。

## SCN 相机频率副本实验（已运行）

学习者预测：图像消息频率在 RViz 中仍为 15，但实际传回 5；机器人模型尺寸不变。尺寸判断正确，频率判断有误。已解释需要区分话题的新图像频率和 RViz 渲染刷新率，URDF 不保存本工程相机的更新频率。待学习者纠正后复述，不因实验成功直接判定这一理解通过。

实验前重新检查无运行容器或相关 ROS 程序，11311/11341/11351/11361 未监听，当前显示授权文件可读。使用独立 Master 11361，容器 underwater-objectnav-day5-rate，顺序运行 rgb15 与 rgb5 两组；每次切换先停止上一组。不启动控制器，两次均确认推进器话题无发布者。

- `day-05/scenarios/rgb15_robot.scn` 是当前机器人 SCN 的原样副本；rgb5_robot.scn 仅修改 RGB 传感器 rate=15.0 → 5.0。两个父场景副本仅将机器人 include 指向各自副本；环境及初始位姿保持相同。
- `day-05/sensor_rate.launch` 包含原 Stonefish simulator.launch 及原 visualization.launch，物理配置 100 Hz、图形 low，加载原 BricsBot URDF；sensor_view.rviz 启用 RobotModel、TF、Grid 与 RGB Image 显示。
- 观察进程先等 RGB/Depth/Odometry 均到达，再以约 10.01 秒墙钟窗口采样。频率用窗口内首末到达时间计算，另记录 header 时间频率及样本格式，精简结果见 sensor_rate_evidence.json。

| SCN RGB 配置 | RGB 条数 / 实测到达频率 | Depth 条数 / 频率 | Odometry 条数 / 频率 |
|---|---|---|---|
| 15 Hz | 150 / 15.002 Hz | 150 / 15.005 Hz | 1000 / 99.937 Hz |
| 5 Hz | 50 / 5.001 Hz | 150 / 15.004 Hz | 1002 / 100.019 Hz |

两次 RGB 发布者均为 /stonefish_simulator，订阅者包括 /aquaflow_rviz 及本次观察节点；实际加载的 robot_description 哈希完全相同，visual scale 均为 1 1 1。RGB 均为 640×480 rgb8，Depth 为 640×480 32FC1。这证明发布话题的新图像频率随 SCN 配置变化，RViz 订阅的是这个实际图像源；没有独立测量 RViz 内部回调频率、渲染 FPS 或物理步频，不能将观察节点测值直接称作这些测值。URDF 模型尺寸配置没有改变，未另做模型截图像素测量。

宿主机工程根目录复现入口（先核对端口与显示授权，以 rgb15 为例）：

```bash
docker compose -f docker/compose.yaml run -d --rm \
  --name underwater-objectnav-day5-rate \
  -e ROS_MASTER_URI=http://localhost:11361 \
  -e XAUTHORITY=/tmp/day5.xauth \
  -v /run/user/1000/gdm/Xauthority:/tmp/day5.xauth:ro \
  underwater-nav roslaunch -p 11361 \
  /workspace/learning/day-05/sensor_rate.launch variant:=rgb15
```

停止本次容器后，将 variant 改为 rgb5 重启。SCN 在仿真器启动时加载，所以本次修改需要重启仿真；无 C++ 改动，不需要编译。实验入口同时启动 RViz，顺序重启整套实验只是统一条件，并不表示传感器改频率必须靠重启 RViz生效。

实验结束已停止并自动删除容器，11361 已释放；临时观察脚本及其容器内副本均已清理，未改显示访问权限。原始 URDF、visualization.launch、机器人 SCN 与原父场景 SCN 的哈希在前后均相同。保留副本、两个实验启动/显示配置及精简证据，没有保存大日志或图像数据集。

频率纠正核对：针对“相机每秒发 5 张新图，RViz 每秒绘制 30 次，能否看到 30 张不同新图”，学习者答“不会”，判断正确。原因由 Agent 补充：重复绘制同一张图不会产生新的图像；学习者本轮未独立解释原因，不扩大为所有频率概念均独立通过。

两项计划内副本实验已运行。进入当天 3 个理解题和 1 个找代码题，第一题结合约 200 字的 SCN/URDF 职责说明；等待学习者回答后逐项评价，不认定第五天完成。未验证推进器驱动、受控运动、长期稳定性或在线导航。

## 第五天结束题首轮评价

1. SCN 由 Stonefish 读取、URDF 经 robot_description 给 RViz RobotModel 使用，方向正确；但将 URDF 说成负责“正确反映仿真状态”不够精确。URDF 提供模型结构和显示外观，动态位姿来自 TF，图像等来自传感器话题；保证模型外观与 SCN 对应、TF 与仿真对应是关键。频率应与预期匹配，但不能取代位姿或模型一致性。本题待学习者补充。
2. 学习者仍预测只改 URDF 相机 joint/origin 为 0.30 m 后 TF 发布到 0.30 m，并认为 TF 发布器读取 URDF。与源码及前述内存副本实验相矛盾：visualization.launch 的 tf/static_transform_publisher 的 args 写死 0.20 0.0 -0.02；只有该发布器参数也修改并重启，发布的相对位移才会改变。此题未通过，待纠正复述。
3. 学习者答无需重新编译 C++，需重启仿真程序，正确，通过。这里改的是运行时加载的 SCN，不是 C++ 源码。
4. 学习者准确贴出 bricsbot_depth_camera_tf 的 node，末尾 100 是 100 ms 周期，正确；父坐标系“baselink”缺少当前真实全名 bricsbot/base_link，子坐标系误答为“tf坐标系”，实际是 bricsbot/depth_camera。找代码与周期通过，坐标系识别待补。

本轮只核对源码和更新学习记录，没有运行或修改仿真。第五天尚未验收完成，后续只需补清第 1、2、4 题，不重复已通过的第 3 题。

## 学习者收尾确认

学习者随后表示“已经明白了，我已完成第五天的学习”。按学习者意愿，第五天课程内容与两项副本实验标记为**学习完成**，下一次可进入第六天；不再要求本日补答。

结束题的评价保持证据边界：第 3 题完整通过；第 4 题找代码与 100 ms 周期通过，父/子坐标系名称经讲解纠正；第 1 题 URDF/SCN 入口方向正确，但 TF 与传感器话题的职责由 Agent 补充；第 2 题首轮答案错误，Agent 依据当前 launch 中写死的 0.20 m 参数和内存副本实验作了纠正。学习者确认理解，但未再次独立复述第 1、2、4 题，因此这些细节不记为“独立验收通过”，也不把未写出的约 200 字说明记录为已提交。

第五天保留成果：本记录、URDF 与 SCN 学习副本、两份实验入口/显示配置、RViz 对照图和精简数值证据。原工程文件未改；本次所有实验容器、观察进程和临时脚本已清理，实验端口已释放。仿真只覆盖模型显示与传感器频率，没有验证受控运动或导航。
