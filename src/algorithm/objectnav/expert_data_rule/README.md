# Underwater ObjectNav Rule Expert

该 ROS 包使用冻结的 GFL + ResNet-50 检测器、RGB-D 语义地图和硬编码规则，
替代 `keyboard_reference_publisher` 发布 `robot_control/ActionRequest`。它复用
现有 `action_executor` 执行动作，并通过 `data_generate` 服务自动管理 Episode。

所有策略、感知、建图、场景解析和循环采集代码均位于本包。`ObjectNav` 仅作为
实现参考，不是运行时 Python 依赖。

## 启动

先完成 catkin 编译并 source 工作空间，然后提供已经训练好的检测 checkpoint：

```bash
roslaunch expert_data_rule continuous_expert_collection.launch \
  detector_checkpoint:=/absolute/path/to/BEST.pth \
  detector_threshold_file:=/absolute/path/to/echinus_threshold.json \
  enable_thrusters:=true
```

`detector_threshold_file` 可省略，此时使用 `config/rule_expert.yaml` 中的分类阈值。
默认场景是 `Rock_SeaUrchin.scn`，也可覆盖：

```bash
roslaunch expert_data_rule continuous_expert_collection.launch \
  scenario_file:=/absolute/path/to/custom.scn \
  scene_id:=custom_scene \
  detector_checkpoint:=/absolute/path/to/BEST.pth \
  enable_thrusters:=true
```

默认启动后无限循环采集。每轮从 `.scn` 的 `PoolFloor` 推导池区，从活动的
`RockSurface*` OBJ 几何推导岩石包围盒，在固定 z 高度采样安全位姿并调用
`/stonefish_simulator/respawn_robot`。成功和失败 Episode 均由
`objectnav_data_collector` 落盘。

## 运行控制

```bash
rosservice call /rule_based_expert/pause_collection
rosservice call /rule_based_expert/start_collection
rostopic echo /rule_based_expert/collection_state
```

暂停请求不会打断正在执行的离散动作；动作完成后，当前 Episode 以
`operator_paused` 结束，并停止开启下一轮。恢复后会重新随机出生并创建新 Episode。

## 关键约束

- `action_executor` 的一次转角必须与 `policy.turn_primitive_rad` 的绝对值一致；
- `spawn.fixed_z_m` 应与 action executor 的目标深度一致；
- `data_generate` 在组合 launch 中被设置为 `auto_end_on_stop=false`，由本包在
  `STOPPED` 后显式结束成功 Episode；
- 第 100 个运动动作完成后仍会做一次新观察，成功优先，否则以
  `max_steps_exceeded` 结束，不生成第 101 个运动动作；
- 模型类别顺序固定为 `0=echinus, 1=rock`。

主要参数及动作间隔均在 `config/rule_expert.yaml` 中调整。
