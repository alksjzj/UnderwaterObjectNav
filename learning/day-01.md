# 第 1 天：启动链与环境边界

日期：2026-10-24（Asia/Shanghai）。范围：空水池演示；未验证控制跟踪或在线导航。理解题已初评，C++ 重新编译的作用已复述确认；完整启动链与 Master 职责仍待独立复述。

## 入口与源码事实

五个核心文件按运行配置连接：

`docker/run.sh demo` → `docker/compose.yaml` → `docker/entrypoint.sh` → `src/aquaflow_stonefish/launch/aquaflow_pool.launch` → `src/stonefish_ros/launch/simulator.launch`。

最后的 `<node>` 运行 `parsed_simulator` 可执行程序，ROS 节点名为 `/stonefish_simulator`。实现入口是 `src/stonefish_ros/src/parsed_simulator.cpp`；其中创建仿真管理器与图形应用，并调用 `app.Run()`。

- `scene:=empty_pool` 选择 `aquaflow_empty_pool.scn`，该场景包含 BlueROV2。
- 父 launch 的 `simulation_rate=100.0` 传给子 launch，覆盖子文件默认值 `500`；单位是步/秒，不是画面 FPS。
- `graphics_resolution="1200 800"` 表示宽 1200、高 800 像素；展开成两个命令行参数。`demo` 传入 `graphics_quality:=low`。
- 节点名决定 ROS 中的身份，`type="parsed_simulator"` 决定选择哪个程序。改节点名不会更换程序，但会影响依赖节点名的私有接口与管理命令。

## 环境与目录对应表

| 内容 | 宿主机位置或入口 | 容器内位置／实测结果 |
|---|---|---|
| 工程源码 | `/home/xiela/bishe/UnderwaterObjectNav` | `/workspace`，绑定挂载、可读写 |
| ROS 1 环境 | 宿主机调用 Docker 入口 | `/opt/ros/noetic`；`rosversion -d` 输出 `noetic` |
| 系统 Python | 容器中执行 `python3` | `/usr/bin/python3`，Python 3.8.10 |
| 推理 Python | 容器中显式选择 Conda 环境 | `/opt/conda/envs/objectnav-gfl/bin/python`，Python 3.10.22 |
| 编译产物 | 工程目录下的 `devel/` | `/workspace/devel/lib/stonefish_ros/parsed_simulator` |
| X11 通道 | `/tmp/.X11-unix` | 同路径绑定挂载；日志确认窗口及 OpenGL 上下文初始化 |

实际进程检查确认：容器内 `roslaunch` 启动 ROS Master、`rosout` 与 `parsed_simulator`；宿主机进程列表也能看到这些容器进程，但程序使用容器内的路径和依赖。

## 本轮实际运行结果

1. 普通 `python3` 使用 Python 3.8.10，符合学习者的预测；推理环境是独立的 Python 3.10.22。
2. 初次按原 `run.sh demo` 启动时，日志出现同名节点重新注册提示，并记录仿真进程退出码 `-5`。后续检查发现默认端口已有另一套演示。未证明异常退出的具体根因，也未修改或停止另一套演示。
3. 为隔离验证，使用相同镜像、入口、场景、步频和画质，单独运行容器 `underwater-objectnav-day1-check`；仅将 `ROS_MASTER_URI` 改为 `http://localhost:11321`，同时给 `roslaunch` 传入 `-p 11321`。这不是原默认端口无冲突运行的证明。
4. 独立演示日志记录 `Window created. OpenGL 4.3 contexts created.`、加载 `aquaflow_empty_pool.scn` 与 `bluerov2_reference.scn`，以及 `Ready for running...`。未另外保存或审查窗口截图。
5. `rosnode list` 返回 `/rosout`、`/stonefish_simulator`。
6. `/bluerov2/odometry` 类型为 `nav_msgs/Odometry`，发布者为 `/stonefish_simulator`。位置参考系 `header.frame_id=world_ned`；`child_frame_id=bluerov2/odometry`。位置单位为米，四元数无量纲；本轮未更改坐标系。
7. 一个实际位置样本约为 `(-5.000093, -0.000003, 0.046265)` 米。不能据此认定机器人严格静止或受控跟踪成功。
8. 约 6 秒观察中，`rostopic hz -w 50` 输出约 99.176–101.352 Hz。这是话题接收频率，不是对物理步进频率或渲染 FPS 的独立测量。观察命令由 `timeout` 按计划结束，状态码为 `124`。

| 本次检查方式 | 用途 |
|---|---|
| `rosversion -d`、`command -v python3`、`python3 --version` | 确认 ROS 与默认解释器 |
| 显式运行 Conda 环境中的 Python | 确认另一套解释器路径、版本 |
| `docker inspect`、`docker top` | 确认挂载和进程 |
| `rosnode list`、`rostopic info`、`rostopic echo -n 1` | 确认节点、发布者和实际消息 |
| 有时限的 `rostopic hz` | 观察消息接收频率 |

## 文件生效与清理

- 挂载使容器能看到宿主机文件变化，不保证已运行程序自动重新读取它。
- 对直接读取挂载源码的 Python 节点，修改后通常重启节点即可；安装复制或生成入口时，还要检查其构建方式。C++ 源码修改后需要重新编译，再重启程序。
- 镜像构建时 `COPY` 的入口脚本是另一种情况：修改宿主机 `docker/entrypoint.sh` 不会自动替换镜像中的 `/usr/local/bin/underwater-entrypoint`，需要重新构建相应镜像。
- 停止本次独立验证后，已确认验证容器自动删除、端口 11321 不再监听；另一套原有演示容器保留。
- 未写入实验脚本，未修改工程逻辑；容器内日志随本次临时容器删除。仅保留本学习记录作为可复查证据摘要。

## 回答与初评

1. 启动职责：部分正确。学习者知道入口负责准备环境、roslaunch 负责启动节点，但省略了调用链。需要修正：入口脚本设置 `ROS_MASTER_URI` 等环境变量，不会自行启动 ROS Master；`roslaunch` 可以在需要时自动启动 Master。
2. 挂载与保留：正确。这里是绑定挂载，容器访问宿主机同一份源码；删除临时容器不会删除宿主机文件。
3. C++ 修改：经讲解后，学习者准确复述：“因为修改后没有进行编译，没有形成新的可以运行的文件。”已理解源码变更不会自动更新可执行程序。已介绍 `./docker/run.sh compile` 的操作方法，但未要求学习者实际改代码或编译。
4. 找代码：已找到正确的 `<node>` 内容并识别两种名称。完整位置是 `src/stonefish_ros/launch/simulator.launch:7`；`type="parsed_simulator"` 是可执行程序名，`name="stonefish_simulator"` 是 ROS 节点名，C++ 源码文件名另为 `parsed_simulator.cpp`。

C++ 修改后的操作方法：先在原演示终端停止旧仿真，在宿主机工程根目录执行 `./docker/run.sh compile`，确认编译成功后再用 `./docker/run.sh demo` 启动。当前 `compile` 分支在容器内执行 `catkin_make -j2` 并指定系统 Python。仅修改挂载工程中的 C++ 通常不需重建 Docker 镜像。

本轮只说明编译方法，没有修改 C++、没有实际重新编译，也没有额外启动仿真。阶段评价：挂载、节点命名与 C++ 编译生效机制已建立基本理解；完整启动链及入口与 Master 的职责区别，尚未由学习者独立复述，不计为已验证通过。
