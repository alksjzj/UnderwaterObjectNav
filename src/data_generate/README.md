# 水下 ObjectNav 专家数据采集

依据当前节点和配置整理：采集器将同步 RGB、Depth、Odometry 与动作执行开始事件配对，保存动作前观测和专家标签。

## 当前数据链

```text
键盘 → ActionRequest → action_executor → 目标 PoseStamped → 串级 PID → 推进器分配 → Stonefish
                             ↓
                        ActionStatus (STARTED)
                             ↓
RGB / Depth / Odometry → 时间同步缓存 → data_collector → PNG / NPY / CSV / YAML
```

键盘节点只发送带编号的动作请求；动作执行器负责延迟、排队、生成目标和完成判断。采集器订阅 ActionStatus，在 STARTED 时间附近寻找同步观测，同一 action_id 避免重复记录。旧版 `expert_action` 字符串发布流程已不适用于当前实现。

## 启动顺序

以下命令在已加载 ROS Noetic 和工程 `devel/setup.bash` 的环境中执行。Docker 用户在宿主机运行 `./docker/run.sh shell` 进入容器；工程路径为 `/workspace`。各命令使用独立终端。

1. 启动 BricsBot 仿真：

```bash
roslaunch aquaflow_stonefish aquaflow_pool.launch scene:=random_pillars vehicle_model:=bricsbot graphics_quality:=low
```

2. 启动串级 PID：

```bash
roslaunch robot_control cascaded_pid_control.launch vehicle_name:=bricsbot
```

3. 启动分配器。先以 `_enabled:=false` 检查连接，确认后停止并以 true 重启：

```bash
rosrun aquaflow_stonefish brics6_thruster_allocator.py _vehicle_name:=bricsbot _enabled:=true
```

4. 启动采集器，建议学习时使用独立目录：

```bash
roslaunch data_generate data_collector.launch dataset_root:=/workspace/learning_dataset
```

5. 在交互式终端启动键盘和动作执行器：

```bash
roslaunch robot_control keyboard_reference.launch command_delay_s:=5.0
```

6. 在另一终端开始记录，再返回键盘终端操作：

```bash
rosservice call /objectnav_data_collector/start_episode \
  "scene_id: 'scene_001'
goal_category: 'echinus'"
```

当前动作执行器会先到达配置的初始化深度，再执行排队动作。不要同时运行 `tracking.launch` 内的轨迹 PID 或 HOFA-MPC；它们可能竞争 `/controller/generalized_force`。

| 按键 | 动作 |
|---|---|
| W | FORWARD |
| S | BACKWARD，人工调整，不属于四类训练动作 |
| A / D | TURN_LEFT / TURN_RIGHT |
| SPACE | STOP |
| Q | 退出键盘节点 |

当前 `collector.yaml` 的 `auto_end_on_stop: true` 会把 STOP 保存为最后一步并成功结束 episode。这是人工标记规则，不代表采集器自动证实找到目标。

也可以通过服务结束：

```bash
rosservice call /objectnav_data_collector/end_episode "success: true failure_reason: ''"
rosservice call /objectnav_data_collector/end_episode "success: false failure_reason: '目标未找到'"
```

以上两个结束命令按实际结果选择一个。`cancel_episode` 服务会删除当前 episode 目录，仅在需要放弃本轮数据时使用。

## 参数归属

| 配置文件 | 负责什么 |
|---|---|
| `robot_control/config/keyboard_reference.yaml` | 请求话题、按键防重复 |
| `robot_control/config/action_executor.yaml` | 距离、转角、延迟、深度、动作完成判断 |
| `robot_control/config/bricsbot_cascade_pid.yaml` | 控制器增益和限幅 |
| `data_generate/config/collector.yaml` | 传感器话题、同步、记录和结束规则 |

当前执行器默认前进 0.25 m，转角配置值 -15°。左右转符号应结合 `make_target()` 和 NED 坐标阅读。旧文档的 0.5 m / 30°不是当前执行器默认值。

`command_delay_s` 属于执行器；排队和前一个动作完成也影响实际开始时间，不能保证每次按键后恰好第 5 秒执行。修改配置后应重启实际读取该参数的节点。采集器中的历史动作容差参数不能替代执行器的完成判断。

## 输出

```text
learning_dataset/
├── metadata.yaml
└── episode_0001/
    ├── rgb/000000.png
    ├── depth/000000.npy
    ├── trajectory.csv
    └── episode.yaml
```

Depth 为 float32 NumPy 数组。CSV 字段如下，每行对应动作开始附近的观测与标签：

```text
step_id,action_id,action_start_time,timestamp,rgb_path,depth_path,x,y,z,roll,pitch,yaw,goal_category,expert_action
```

`episode.yaml` 保存场景、目标类别、起始位姿、成功状态、失败原因和结束时步数。当前实现不写入目标真实坐标。

## 排查记录缺失

```bash
rostopic info /underwater_objectnav/action_request
rostopic echo /underwater_objectnav/action_status
rostopic hz /bricsbot/rgb/image_color
rostopic hz /bricsbot/depth/image_depth
rostopic hz /bricsbot/odometry
rostopic info /controller/generalized_force
```

依次确认 episode 已开始、动作进入 STARTED、同步数据足够新、控制输出发布者唯一。当前采集器没有发布旧文档中的 `collector_status` 话题。

本文启动组合依据代码连接关系整理，本次仅修改文档，未重新运行完整采集。首次使用请记录一个短 episode，核对图像、时间戳和 action_id。
