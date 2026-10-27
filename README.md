# UnderwaterObjectNav

初学者请先阅读 [学习方法.md](学习方法.md)：14 天、每天约 2–3 小时，以 Agent 辅助实验和读代码为主，依次学习场景、SCN/URDF、控制闭环、Stonefish 源码与导航模块。

面向水下目标导航学习的 ROS 1 工程。项目使用 **Stonefish 1.6** 构建水下仿真环境，以 **BricsBot / BlueROV2** 为载体，包含场景仿真、底层控制、专家数据采集、GFL 海胆检测、语义地图、行为克隆（BC）策略和在线闭环导航。

仓库提供一套隔离的 Docker 环境：宿主机继续使用 Ubuntu 24.04 + ROS 2 Jazzy，容器内部使用 Ubuntu 20.04 + ROS Noetic。宿主机的 ROS 2 环境不会被修改，也不会与容器的 Python 3.8 / 3.10 环境混用。

![Stonefish 空水池场景](docs/images/stonefish-empty-pool.png)

---

## 1. 当前可运行状态

以下结果是本机实际构建、编译和运行得到的，不是“应该可以”的推断：

| 能力 | 状态 | 验证结果 |
|---|---|---|
| ROS Noetic + Catkin | ✅ | `rosversion -d` = `noetic` |
| Stonefish 1.6.0 | ✅ | `pkg-config --modversion stonefish` = `1.6.0` |
| Catkin 工作空间 | ✅ | 72 个 target 编译通过，`rospack find stonefish_ros/objectnav` 正常 |
| GPU / OpenGL | ✅ | RTX 4060 Laptop，驱动 595.99.02，OpenGL 4.6.0，`direct rendering: Yes` |
| PyTorch + CUDA | ✅ | `torch 2.1.0+cu121`，`torch.cuda.is_available() == True` |
| OpenMMLab | ✅ | MMCV 2.1.0 / MMEngine 0.10.7 / MMDetection 3.3.0 |
| 单元测试 | ✅ | `103 passed` |
| Stonefish 图形场景 | ✅ | 空水池 1200×800，实时渲染约 467 FPS |
| RGB / 里程计话题 | ✅ | 相机约 30 Hz，里程计约 100 Hz |
| 完整在线 ObjectNav 闭环 | ⚠️ 未验证 | 仓库缺少 GFL 权重、阈值 JSON 和 BC 权重 |

**结论：现在可以学习和运行仿真、控制、数据采集和单元测试；完整 AI 导航还需要补齐模型资产。** 模型文件不在仓库中，属于论文/训练产物，不应由容器凭空生成。

---

## 2. 项目结构

```text
UnderwaterObjectNav/
├── docker/
│   ├── Dockerfile              ROS Noetic + Stonefish + Python 3.10 镜像
│   ├── compose.yaml            GPU、X11、主机网络和源码挂载
│   ├── entrypoint.sh           ROS / Stonefish / Conda 环境初始化
│   └── run.sh                  统一的构建、编译、验证和启动入口
├── docs/images/                README 使用的截图
├── scripts/
│   └── setup_stonefish_ros.sh  已改为使用容器内的 /opt/stonefish
├── src/
│   ├── stonefish_ros/          Stonefish 1.6 的 ROS 1 接口
│   ├── aquaflow_stonefish/     水下机器人、场景、推力分配
│   ├── robot_control/          PID、动作执行、键盘参考
│   ├── data_generate/          RGB-D-Pose 专家数据采集
│   ├── image_process_ResNet50/ GFL 海胆/岩石检测
│   ├── ObjectNav/              地图、BC 策略和在线导航
│   ├── aquaflow_ref/           教师轨迹与地图可视化
│   └── hofa_mpc_ros/           三自由度 HOFA-MPC
├── build/                      Catkin 构建产物（不提交）
└── devel/                      Catkin 开发环境（不提交）
```

在线阶段的 Catkin 包名是小写 `objectnav`，目录名仍是 `ObjectNav`：

```bash
roslaunch objectnav objectnav_online.launch
```

---

## 3. 为什么用两个 Python 环境

ROS Noetic 的 `rospy`、`cv_bridge` 等模块是为系统 Python 3.8 编译的；GFL 检测和 BC 推理需要 PyTorch 2.1、MMCV 2.1 和 Python 3.10。直接混用会让 `cv_bridge` 或 PyTorch 的 C 扩展加载失败。

容器内因此保留两套解释器：

```text
/usr/bin/python3                           Python 3.8  ROS 节点 / rospy / cv_bridge
/opt/conda/envs/objectnav-gfl/bin/python   Python 3.10 PyTorch / GFL / BC
```

两者通过本机 TCP（`127.0.0.1:29500`）传递“长度前缀 JSON + NumPy 数组”，不传 pickle。ROS 侧只负责传感器同步、状态机和动作发布；推理侧不导入 ROS。

镜像的 `PATH` 已把系统 Python 放在 Conda 前面，因此 `catkin_make`、`roslaunch` 默认使用 Python 3.8；需要深度学习环境时显式执行：

```bash
conda run -n objectnav-gfl python ...
```

`conda` 仍可直接使用，因为它位于 `PATH` 末尾的 `/opt/conda/bin`。

---

## 4. 宿主机要求

- Linux + NVIDIA 显卡；
- Docker Engine 与 Docker Compose v2；
- `docker buildx`（构建缓存挂载和 BuildKit 需要）；
- NVIDIA Container Toolkit（`--gpus all` 可用）；
- X11 图形会话（`DISPLAY` 非空）；
- 建议至少 16 GB 内存、50 GB 可用磁盘（镜像解压后约 22 GB，构建缓存另计）。

先检查：

```bash
nvidia-smi
docker version
docker compose version
docker buildx version
echo "$DISPLAY"
```

### 4.1 安装或补齐 buildx

如果 `docker buildx version` 报 `unknown command`，安装 buildx 插件。优先尝试系统包：

```bash
sudo apt install docker-buildx-plugin
```

如果软件源里没有该包，可以把官方二进制放到当前用户插件目录（不需要 root，也不需要改系统 Docker）：

```bash
mkdir -p ~/.docker/cli-plugins
curl -fL -o ~/.docker/cli-plugins/docker-buildx \
  https://github.com/docker/buildx/releases/download/v0.38.1/buildx-v0.38.1.linux-amd64
chmod +x ~/.docker/cli-plugins/docker-buildx
docker buildx version
```

本项目实际使用的就是这种方式；`docker buildx ls` 能看到 `default` builder 即可。

### 4.2 代理与镜像

`docker/run.sh build` 使用主机网络，并把当前 shell 的 `HTTP_PROXY` / `HTTPS_PROXY` 传进构建过程。Docker 守护进程拉取基础镜像时使用的是守护进程自己的代理配置，和终端代理是两套设置；如果 `docker pull` 超时，需要单独配置 Docker daemon 的代理。

构建默认使用以下镜像/下载源，避免几 GB 的 CUDA 依赖从公网慢速下载：

```text
PyPI          https://pypi.tuna.tsinghua.edu.cn/simple
PyTorch cu121 https://mirrors.aliyun.com/pytorch-wheels/cu121/
MMCV          https://download.openmmlab.com/mmcv/dist/cu121/torch2.1/index.html
```

在海外或镜像不可达时，可以覆盖：

```bash
PIP_INDEX_URL=https://pypi.org/simple \
TORCH_FIND_LINKS=https://download.pytorch.org/whl/cu121 \
./docker/run.sh build
```

---

## 5. 第一次构建

所有日常操作都通过 `docker/run.sh` 完成：

```bash
cd ~/bishe/UnderwaterObjectNav
chmod +x docker/run.sh

./docker/run.sh build
./docker/run.sh compile
./docker/run.sh verify
```

三条命令分别完成：

1. 构建 `underwater-object-nav:noetic-stonefish-1.6` 镜像；
2. 在容器内用系统 Python 3.8 运行 `catkin_make`；
3. 检查 GPU/OpenGL、ROS、Stonefish、PyTorch/OpenMMLab，并运行 103 个单元测试。

镜像下载内容约 6.6 GB，解压后磁盘占用约 22 GB；首次构建还会产生 Docker 构建缓存（本机实测约 17 GB），因此建议预留 50 GB 以上磁盘。首次构建会下载 ROS Noetic、Stonefish 源码、Miniforge、PyTorch CUDA 依赖，通常需要十几分钟到半小时，取决于网络。后续只修改项目代码时不需要重新构建镜像。

构建完成后可以单独确认镜像：

```bash
docker images | grep underwater-object-nav
```

应看到：

```text
underwater-object-nav:noetic-stonefish-1.6
```

### 5.1 `verify` 的正常输出

```text
NVIDIA GeForce RTX 4060 Laptop GPU, 595.99.02, 8188 MiB
direct rendering: Yes
OpenGL vendor string: NVIDIA Corporation
OpenGL renderer string: NVIDIA GeForce RTX 4060 Laptop GPU/PCIe/SSE2
OpenGL core profile version string: 4.6.0 NVIDIA 595.99.02
1.6.0
/workspace/src/aquaflow_stonefish
PyTorch 2.1.0+cu121 CUDA 12.1 available True
MMCV 2.1.0 MMEngine 0.10.7 MMDetection 3.3.0
103 passed
```

显卡型号、驱动版本和 OpenGL 版本会因机器而异；关键是 `direct rendering: Yes`、`torch.cuda.is_available() == True` 和测试全部通过。

---

## 6. 运行 Stonefish 空水池

```bash
./docker/run.sh demo
```

脚本会打开低画质的 Stonefish 图形窗口，并运行：

```bash
roslaunch aquaflow_stonefish aquaflow_pool.launch \
  scene:=empty_pool graphics_quality:=low
```

正常情况下可以看到：

- 1200×800 的 Stonefish Simulator 窗口；
- 水面、池壁和 BlueROV2；
- 左侧 DEBUG / SUN POSITION / OCEAN / VIEW 面板；
- 底部实时 FPS、CPU 和仿真时间。

另开一个终端进入同一环境观察话题：

```bash
./docker/run.sh shell
source devel/setup.bash        # entrypoint 已自动 source，可省略
rostopic list
rostopic hz /bluerov2/left/image_color
rostopic hz /bluerov2/odometry
```

在本机实测中，相机约 30 Hz，里程计约 100 Hz。按 `Ctrl+C` 关闭仿真。

> 第一次运行时只观察场景和话题，不要立即启用推进器。

---

## 7. BricsBot 跟踪演示

先用安全模式启动，推进器使能参数为 `false`：

```bash
./docker/run.sh tracking
```

该命令对应：

```bash
roslaunch aquaflow_stonefish tracking.launch \
  scene:=random_pillars \
  vehicle_model:=bricsbot \
  enable:=false \
  graphics_quality:=low
```

会启动以下节点：

```text
stonefish_simulator
nominal_reference_publisher
planar_pid_tracker
brics6_thruster_allocator
actual_path_recorder
```

确认场景、里程计、参考轨迹和分配器都正常后，再在容器 Shell 中手动启用控制：

```bash
source devel/setup.bash
roslaunch aquaflow_stonefish tracking.launch \
  scene:=random_pillars \
  vehicle_model:=bricsbot \
  enable:=true \
  graphics_quality:=low
```

控制链：

```text
参考轨迹
  -> /aquaflow/nominal_path
  -> planar_pid_tracker
  -> 平面推力
  -> brics6_thruster_allocator
  -> Stonefish 推进器
```

`enable:=true` 会让机器人真正运动。第一次务必在空水池或开阔场景中、确认急停方式后再启用。

---

## 8. 专家数据采集

需要三个终端。每个终端都通过 `./docker/run.sh shell` 进入同一容器环境。

终端 1：仿真和底层控制

```bash
source devel/setup.bash
roslaunch aquaflow_stonefish tracking.launch \
  vehicle_model:=bricsbot scene:=random_pillars enable:=true
```

终端 2：RGB-D-Pose 数据采集

```bash
source devel/setup.bash
roslaunch data_generate data_collector.launch \
  dataset_root:=/workspace/underwater_objectnav_dataset
```

终端 3：键盘专家（终端必须保持焦点）

```bash
source devel/setup.bash
roslaunch robot_control keyboard_reference.launch
```

开始一个 Episode：

```bash
rosservice call /objectnav_data_collector/start_episode \
"scene_id: 'scene_001'
goal_category: 'echinus'"
```

键盘动作：

```text
W       前进
S       后退（仅用于人工调整，不保存为训练动作）
A       左转
D       右转
SPACE   停止
Q       退出键盘节点
```

结束 Episode：

```bash
rosservice call /objectnav_data_collector/end_episode \
"success: true failure_reason: ''"
```

数据默认写入 `/workspace/underwater_objectnav_dataset/episode_NNNN/`，包含 RGB PNG、Depth NPY、`trajectory.csv` 和 `episode.yaml`。详细字段和采集原理见 [`src/data_generate/README.md`](src/data_generate/README.md)。

---

## 9. 深度学习环境

检查 Conda 环境：

```bash
./docker/run.sh shell
conda run -n objectnav-gfl python \
  src/image_process_ResNet50/tools/check_env.py --strict
```

手工确认版本：

```bash
conda run -n objectnav-gfl python -c \
  "import torch, mmcv, mmengine, mmdet; print(torch.__version__, torch.cuda.is_available()); print(mmcv.__version__, mmengine.__version__, mmdet.__version__)"
```

训练数据建议放在：

```text
src/image_process_ResNet50/data/underwater_objectnav_rgb/
```

相关文档：

- GFL 训练、校准和推理：[`src/image_process_ResNet50/README.md`](src/image_process_ResNet50/README.md)
- BC 数据处理、训练和评估：[`src/ObjectNav/objectnav_bc/README.md`](src/ObjectNav/objectnav_bc/README.md)
- 在线架构、服务和记录：[`src/ObjectNav/underwater_objectnav_online_usage.md`](src/ObjectNav/underwater_objectnav_online_usage.md)

---

## 10. 模型资产与在线 ObjectNav

仓库**不包含**以下三个文件，因此不能直接完成在线闭环：

| 文件 | 默认路径 | 用途 |
|---|---|---|
| GFL 检测权重 | `src/ObjectNav/objectnav_bc/perception/model/best_echinus_recall_at_precision_95_epoch_5.pth` | 海胆/岩石检测 |
| 检测阈值 | `src/ObjectNav/objectnav_bc/perception/model/echinus_threshold.json` | 置信度阈值与权重文件名校验 |
| BC 策略权重 | `src/ObjectNav/work_dirs/objectnav_bc/best_policy.pt` | 离散动作策略 |

按上表路径放好三个文件后，先以控制关闭模式检查所有节点：

```bash
./docker/run.sh objectnav
```

该入口显式设置 `enable:=false`。当前未放权重时，Worker 会先报 GFL checkpoint 不存在，例如：

```text
FileNotFoundError: Detector checkpoint not found:
/workspace/src/ObjectNav/objectnav_bc/perception/model/best_echinus_recall_at_precision_95_epoch_5.pth
```

这是当前仓库的预期状态，不是容器环境损坏。

确认 RGB、Depth、Odometry、Worker 和动作状态正常后，再手动启用控制：

```bash
./docker/run.sh shell
source devel/setup.bash
roslaunch objectnav objectnav_online.launch \
  scene:=rock_seaurchin \
  vehicle_name:=bricsbot \
  vehicle_model:=bricsbot \
  enable:=true \
  conda_executable:=/opt/conda/bin/conda \
  conda_env:=objectnav-gfl \
  record_root:=/workspace/underwater_objectnav_online_runs \
  graphics_quality:=low
```

开始导航：

```bash
rosservice call /underwater_objectnav/start_navigation \
"goal_category: 'echinus'"
```

在线状态机、服务字段、终止条件和录制格式见 [`src/ObjectNav/underwater_objectnav_online_usage.md`](src/ObjectNav/underwater_objectnav_online_usage.md)。

---

## 11. 常用命令

```bash
./docker/run.sh build      # 构建或更新镜像
./docker/run.sh compile    # 编译 Catkin 工作空间
./docker/run.sh verify     # GPU/OpenGL/ROS/Stonefish/PyTorch/测试
./docker/run.sh shell      # 进入已配置的容器 Shell
./docker/run.sh demo       # 空水池基础仿真
./docker/run.sh tracking   # BricsBot 安全跟踪（enable=false）
./docker/run.sh objectnav  # 在线导航启动入口（需要模型权重）
```

修改 Python 脚本、launch、YAML 或场景文件后通常不必重建镜像，工作区通过 `/workspace` 实时挂载。修改 `CMakeLists.txt`、消息或服务后重新执行：

```bash
./docker/run.sh compile
```

只有修改 `docker/Dockerfile`、系统依赖或 Python 包版本时才需要重新执行 `build`。

---

## 12. 已验证结果

验证环境：Ubuntu 24.04 宿主机、RTX 4060 Laptop 8 GB、驱动 595.99.02、Docker 29.1.3、Compose 2.40.3。

| 检查 | 命令 | 实测结果 |
|---|---|---|
| 镜像构建 | `./docker/run.sh build` | `underwater-object-nav:noetic-stonefish-1.6`，下载约 6.6 GB / 磁盘约 22 GB |
| Catkin 编译 | `./docker/run.sh compile` | 72 个 target，`rospack find` 正常 |
| 环境与测试 | `./docker/run.sh verify` | `103 passed` |
| Stonefish | `./docker/run.sh demo` | 空水池窗口，约 467 FPS |
| 传感器 | `rostopic hz` | 相机约 30 Hz，里程计约 100 Hz |
| 跟踪节点 | `./docker/run.sh tracking` | 5 个节点正常启动，分配器 `enabled=false` |

未验证的部分：

- 完整在线 ObjectNav 闭环：缺 GFL 权重、阈值 JSON、BC 权重；
- GFL / BC 的训练精度：仓库不含训练数据；
- 真实水下机器人：本环境只覆盖 Stonefish 仿真。

---

## 13. 常见问题

### 13.1 `docker buildx` 不存在

见 [4.1](#41-安装或补齐-buildx)。`docker/run.sh build` 会在构建前检查并给出提示。

### 13.2 拉取基础镜像超时

这是 Docker 守护进程的代理问题，不是 Dockerfile 的问题。先确认：

```bash
docker info | grep -i proxy
```

如果 `HTTP Proxy` 为空，需要给 Docker daemon 配置代理并重启 Docker。构建阶段的 `RUN` 使用主机网络，会继承 `docker/run.sh` 传入的 `HTTP_PROXY/HTTPS_PROXY`；但 `FROM` 拉镜像走的是 daemon。

### 13.3 Stonefish 窗口打不开

```bash
echo "$DISPLAY"
xhost
```

`DISPLAY` 必须非空；`docker/run.sh` 会临时执行 `xhost +SI:localuser:$(id -un)`，退出时撤销。通过 SSH 运行时需要 X11 转发或使用本机图形会话。

### 13.4 容器里看不到 NVIDIA GPU

```bash
nvidia-smi
docker run --rm --gpus all ubuntu:20.04 nvidia-smi
```

第二条失败通常说明 NVIDIA Container Toolkit 没有正确接入 Docker。

### 13.5 `catkin_make` 报找不到 `empy` 或 `em`

说明 CMake 选中了 Conda 的 Python，而不是 ROS Noetic 的系统 Python 3.8。本镜像已经把系统 Python 放在 `PATH` 前面，并给 `run.sh compile` 显式传入：

```text
-DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON3_EXECUTABLE=/usr/bin/python3
```

如果你在自定义镜像里删除了这个 `PATH` 设置，需要把 `/opt/conda/bin` 放到 `PATH` 末尾。

### 13.6 `pkg-config --modversion stonefish` 失败

确认容器内的：

```bash
echo "$PKG_CONFIG_PATH"
# /opt/stonefish/lib/pkgconfig:...
```

`entrypoint.sh` 会自动设置；手工执行 `docker run` 而不经过 entrypoint 时不会自动设置。

### 13.7 在线导航报缺少 checkpoint

这是当前仓库的预期状态。按第 10 节补齐三个模型文件，或者在 `roslaunch` 中用 `checkpoint:=/absolute/path/to/best_policy.pt` 覆盖。

### 13.8 出现 `Found orphan containers [...]` 警告

这是因为宿主机上还有别的 Compose 项目（例如 `rlai`）的容器。它不影响本项目；如果确认那些容器不再需要，可以在对应项目里执行 `docker compose down`，不要在本项目里随意使用 `--remove-orphans`。

### 13.9 修改代码后要不要重建镜像

一般不需要。`/workspace` 是实时挂载的：

| 修改内容 | 需要执行 |
|---|---|
| Python / launch / YAML / 场景文件 | 直接重新运行 |
| `CMakeLists.txt`、msg、srv | `./docker/run.sh compile` |
| `docker/Dockerfile`、apt/pip 依赖 | `./docker/run.sh build` |

### 13.10 磁盘占用与清理

本项目镜像下载内容约 6.6 GB，解压后磁盘占用约 22 GB；再加上 Docker 构建缓存，首次构建后本机 Docker 可能多占 35–40 GB。项目源码本身只有约 0.8 GB。

查看占用：

```bash
docker system df
docker images
```

如果磁盘紧张，优先清理**构建缓存**，它不影响已经构建好的镜像：

```bash
docker builder prune
```

本机实测可回收约 16 GB；代价是下次重建镜像会慢一些。不要直接执行 `docker system prune -a`，它会删除本项目镜像以及其他项目的镜像。如果只需要保留运行环境、不再重建，可以保留 `underwater-object-nav:noetic-stonefish-1.6`，只清理缓存和悬空镜像：

```bash
docker image prune
```

如果还想进一步缩小镜像，需要改 `docker/Dockerfile`，例如把基础镜像从 `osrf/ros:noetic-desktop-full-focal` 换成更小的 `ros-base` 再按需安装 RViz/PCL；这能省几 GB，但更容易漏装运行依赖，不建议在课程/毕设初期做。

---

## 14. 相对原仓库的改动

为了让项目能在当前机器上运行，做了以下最小改动：

- `scripts/setup_stonefish_ros.sh`：把 `/home/yzw/...` 改为 `${STONEFISH_ROOT:-/opt/stonefish}` 和当前工作空间路径；
- `src/ObjectNav/config/online.yaml`：`record_root` 改为 `/workspace/underwater_objectnav_online_runs`；
- `src/ObjectNav/launch/objectnav_online.launch`：Conda 路径改为 `/opt/conda/bin/conda`，环境名改为 `objectnav-gfl`；
- `src/aquaflow_stonefish/launch/tracking.launch`：补充 `graphics_resolution` / `graphics_quality` 参数；
- `src/aquaflow_stonefish/scenarios/models/Rock4/source/Rock4.mtl`：把写死的绝对纹理路径改为相对路径；
- 新增 `docker/`、`.dockerignore`、`.gitignore` 和本 README。

这些改动不改变算法逻辑，只解决原仓库绑定到作者机器路径和旧环境的问题。
