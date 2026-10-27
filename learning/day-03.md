# 第 3 天：读懂并改变一个场景

日期：2026-10-25（Asia/Shanghai）。已完成静态物体、池底几何、水环境、物理材质与外观的分段阅读，一次单变量位置实验及当天理解验收。

## 本次阅读与预测

核心入口：`src/stonefish_ros/launch/simulator.launch` → `parsed_simulator` → 显式指定的 SCN → Stonefish 创建物体并显示。重点阅读 `src/aquaflow_stonefish/scenarios/aquaflow_single_obstacle.scn:96` 的 `CenterObstacle`，后续回到 `aquaflow_empty_pool.scn` 阅读环境、材质、外观和池底。

圆柱定义：`static` 固定物体；`type="cylinder"`；半径 0.45 m、高 4 m；物理材质 `Fiberglass`、外观 `obstacle_orange`；原位置 `(0, 0, 2)` m，姿态 rpy 全为 0。

预测题：把 `xyz="0.0 0.0 2.0"` 改为 `xyz="0.0 2.0 2.0"` 会怎样？学习者回答“Z方向归0,Y方向到2.0”。已纠正：字段顺序是 x、y、z，因此只有 Y 从 0 增至 2 m，Z 仍为 2 m，X 仍为 0；尺寸和颜色配置不变。后续学习者正确回答：若 X=0、Y=2、Z=0，应写 `0 2 0`，坐标顺序经补充题确认。

已说明原版 `run.sh demo` 写死 `scene:=empty_pool` 且未转发额外参数。学习者理解通过宿主机 `./docker/run.sh shell` 进入容器，再在容器内运行 `roslaunch aquaflow_stonefish aquaflow_pool.launch scene:=single_obstacle graphics_quality:=low` 来选择单障碍场景；该说明后未再次启动仿真。学习者明确表示不需要重新启动，后续优先继续源码阅读。

## 实际改动与运行

原场景未改动。为聚焦已有圆柱，复制单障碍物场景到 `learning/day-03/scenarios/baseline.scn`，再复制为 `moved.scn`。两份文件只相差 `CenterObstacle/world_transform` 的 Y 坐标一处。两份副本其余内容与原单障碍物场景一致，包括机器人引用。

实验前在宿主机核对：Docker 无运行容器；未发现运行中的 rosmaster、roslaunch、parsed_simulator；11331 未监听；本工程镜像存在。沙箱内 Docker/网络检查受限，改在宿主机执行后成功，不据沙箱报错判断 Docker 故障。

两次顺序运行，先停止 baseline 再启动 moved。宿主机工程根目录入口如下，以 baseline 为例：

```bash
docker compose -f docker/compose.yaml run -d --rm \
  --name underwater-objectnav-day3-baseline \
  -e ROS_MASTER_URI=http://localhost:11331 \
  -e XAUTHORITY=/tmp/day3.xauth \
  -v /run/user/1000/gdm/Xauthority:/tmp/day3.xauth:ro \
  underwater-nav roslaunch -p 11331 stonefish_ros simulator.launch \
  simulation_data:=/workspace/src/aquaflow_stonefish/data \
  scenario_description:=/workspace/learning/day-03/scenarios/baseline.scn \
  simulation_rate:=100.0 graphics_resolution:='1200 800' graphics_quality:=low
```

第二次将容器名尾部 `baseline` 和场景文件名 `baseline.scn` 分别换成 `moved`、`moved.scn`。这是本次独立端口实验命令；显示授权路径及运行环境以后重用前要重新检查。直接使用子 launch 的 `scenario_description` 传入学习副本完整路径，没有修改父 launch 的场景选择表。没有启动运动控制器。

两次均在对应容器内执行：

```bash
source /opt/ros/noetic/setup.bash
rosnode list
rostopic info /bluerov2/odometry
```

实际均返回 `/rosout`、`/stonefish_simulator`；里程计类型为 `nav_msgs/Odometry`，发布者为 `/stonefish_simulator`，无订阅者。未测话题频率，也未评价控制性能。

## 截图与证据边界

- 修改前：`learning/day-03/before.png`。
- 修改后：`learning/day-03/after.png`。
- 两张图片只截取 Stonefish 内容窗口，已查看确认：圆柱由池中央移至靠近一侧的位置，仍呈橙色；截图是实际仿真画面。
- 两次默认取景略有差异，仿真时刻分别约为 20.53 s、13.68 s。截图不用于像素位移或严格相同投影的测量；Y 增加 2 m、Z 不变由配置差异确定。截图中的侧面可见程度及明暗变化不能解释为尺寸、材质或颜色配置改变。
- 未通过物体状态接口独立测量圆柱坐标；静态 XML 核对只证明输入配置，窗口及节点检查证明两套副本实际加载显示并运行。

## 清理与下一步

已停止本次两只临时容器，确认自动删除，11331 端口已释放。未改变 X11 访问控制；显示授权文件只读挂载。截图转换直接使用内存，无额外临时截图、脚本或探针需要清理。

保留两份学习场景、两张截图和本记录。原有工程改动未纳入本实验。

池底小段已讲解：`dimensions` 是完整尺寸，`world_transform` 是物体中心位姿；NED 的 Z 向下。池底中心 4.1 m、厚度 0.2 m，因此上表面在 4.0 m、下表面在 4.2 m。学习者正确回答：若中心改到 4.0 m、厚度不变，则上表面在 3.9 m，水变浅。本段理解题通过，未修改场景或重新启动仿真。

`environment/ocean` 小段已讲解：水密度单位 kg/m³；Jerlov 为水体光学参数；零波高表示平坦水面；颗粒显示关闭；均匀水流的 `velocity xyz` 表示 m/s，与尺寸/位置的 `xyz` 不同。学习者表示明白并要求继续，尚未做独立应用题。本段未改文件、未运行仿真。

池底 `Rock` 物理材质与 `pool_floor` 显示外观的引用关系已讲解；已区分 `roughness` 的视觉作用与 `friction_table` 的物理摩擦。学习者正确回答：要把池底改红，把 `pool_floor` 的 `rgb` 改为 `1 0 0`，`Rock` 是物理材质，无需修改。本题通过；名称书写 `poorfloor` 已提醒实际名称为 `pool_floor`。

## 第三天结束题与评价

1. 坐标：学习者回答中心 Z 从 2 改到 3 是“向下移动 1 米”，正确。
2. 增加圆柱：学习者列出 dimension、material、look、worldtransform，抓住主要信息。补充精确标签名为 `dimensions`、`material`、`look`、`world_transform`；外层 `<static>` 还要指定物体名称 `name` 和形状 `type="cylinder"`。不要求默写标签。
3. 修改生效：学习者正确判断保存 SCN 不会立即更新，但最初误认为需要重新编译。已解释：SCN 是运行时读取的场景配置，本工程绑定挂载下修改后停止并重新启动仿真即可；修改 C++ 源码才需要重新编译并重启。当前源码证据：`parsed_simulator.cpp:44` 从 argv 取得场景路径，`ROSSimulationManager.cpp:148` 调用 `parser.Parse(scnFilePath)` 读取。补充回答“SCN是程序运行时候读取的文件，不需要编译”，已确认关键区别，本题经纠正后通过。
4. 找代码：学习者准确贴出 `PoolWallRight` 的尺寸和位姿两行（当前第 86、89 行）；厚度回答“。2”，按 0.2 m 理解，正确，对应 Y 方向尺寸。

阶段结论：第三天理解验收完成。第 2 题在主要字段正确的基础上补充了外层物体名称与形状，第 3 题经纠正及学习者复述后通过；不要求默写 XML。运行验证范围限于此前两份场景的实际显示、节点及里程计发布者检查，没有验证控制或导航效果。此次收尾未启动新仿真。

下次按第 4 天计划：阅读机器人 include 与机器人 SCN 内的 link、actuator、sensor，追踪机器人和传感器如何放入场景。先核对当前文件及运行状态，不沿用本日临时端口作为永久配置。
