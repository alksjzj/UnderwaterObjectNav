# 第 6 天：ROS 消息、回调与反馈

日期：2026-10-26（Asia/Shanghai），第六天核心目标与主线验收已完成。

## 目标与范围

沿实际位姿记录链理解 Publisher、Subscriber、回调、Timer 和 spin；区分实际轨迹与期望路径。本日先读 actual_path_recorder.py，再读 nominal_reference_publisher.py，使用 tracking.launch 核对启动配置。

源码链：Stonefish 发布 /bricsbot/odometry（nav_msgs/Odometry）→ actual_path_recorder 的 odom_cb 保存轨迹点 → publish_path 发布 /aquaflow/actual_path（nav_msgs/Path）。另一条 nominal_reference_publisher → /aquaflow/nominal_path 是期望路径。

## 当前进度

- 已核对当前计划、第五天记录及上述三份源码；第五天记录仍有 SCN/URDF、静态 TF 的补充验收项，按用户要求先进入第六天。
- 第一小段从 actual_path_recorder.py 的程序入口开始，随后辨认初始化函数内的订阅与定时器注册。
- 学习者正确预测：Odometry 到达触发 odom_cb；Timer 周期为 0.1 秒。将定时输出理解为“当前位置”，需补充：publish_path 将 self.points 中累计保存的位姿组成 Path，发布的是轨迹。
- 当前小段核对 publish_path 的 snapshot → Path.poses → publish；下一段追踪 odom_cb 如何写入 self.points，以及消息位姿的嵌套结构。
- 学习者表示已理解定时发布累计轨迹；继续阅读 odom_cb 中 msg.pose.pose.position 的取值、PoseStamped 包装和 self.points.append。暂略坐标系、时间戳与采样筛选分支，后续补读。
- 继续讲解 odom_cb 的距离/时间联合筛选：仅当 distance 与 interval 同时小于各自阈值时跳过；比较基准是上一个保存点。核对脚本默认 0.02 m、0.05 s，而 tracking.launch 覆盖距离为 0.05 m。给出脚本默认阈值下的 0.01 m / 0.06 s 预测题，待学习者回答。
- 学习者预测“跳过，因为要两个都成立才能被保存”，将跳过条件与保存条件看反，尚未通过。Agent 从当前源码 AST 提取实际 and 判断式，以合成数值在内存中求值：0.01 m / 0.06 s 得 False，继续保存；0.01 m / 0.03 s 得 True，执行 return 跳过。仅验证筛选表达式，没有运行 ROS 回调或实际消息链，无临时文件。待学习者判断距离足够、时间很短的情况以核对纠正后的理解。
- 补充应用题：0.03 m / 0.01 s，学习者回答“会保存，因为移动的足够远了”，判断与理由正确。距离足够这一分支通过；时间足够这一分支此前由 Agent 解释，尚未单独复述。继续阅读回调开头的 source_frame 检查，明确只比较坐标系名称，不执行坐标变换；消息 frame_id 为空时采用配置值。
- 下一小段阅读时间戳：零时间戳用 rospy.Time.now() 补齐（ROS 当前时间，不是恢复采样时刻）；当前 stamp 早于上一个保存点时清空 points 并将 last_position 置 None，随后接纳当前点并更新 last_stamp，防止新旧时间段的轨迹混合。仅源码讲解，尚未用 ROS 时间倒退实验验证。
- 学习者回复“好的”后转入 nominal_reference_publisher.py，先读 main() 的节点初始化及默认参数：输出 /aquaflow/nominal_path，world_ned，默认 line，起止 x=-5/5 m，y=z=0 m，spacing=0.25 m。说明它生成期望路径，不订阅实际 odometry，也不直接驱动推进器；后续再读路径点生成与 2 Hz 发布循环。尚未运行节点实验。
- 学习者询问节点来源及 demo 是否启动它们。已核对 run.sh 的 demo/tracking 分支、aquaflow_pool.launch、simulator.launch 与 tracking.launch：两个 Python 节点属于工程 aquaflow_stonefish 包，通过 rospy 使用 ROS 接口；demo 的 launch 链只显式启动 stonefish_simulator，不启动这两个节点，tracking.launch 才显式列出它们。demo 仍由仿真器发布机器人 odometry，记录器负责将其整理成 Path。该结论是启动配置核对，未查询当前运行节点。
- 继续直线路径生成段：count=max(2,ceil(abs(x1-x0)/spacing)+1)，默认 -5 到 5 m、spacing=0.25 m 得 41 个点；u=i/(count-1)，x=x0+u*(x1-x0)，y/z 固定，PoseStamped 追加到 msg.poses。说明 range 不含上界、+1 用于包含两个端点，以及 u 是空间插值比例而非时间或速度；未运行 ROS 节点。随后待读发布循环。
- 继续发布循环：rospy.Rate(2.0) 配合 rate.sleep() 设定名义 2 Hz，每轮更新 Path 和各 PoseStamped 的时间戳，重复发布此前生成的整条路径；发布循环没有追加点、改变坐标或驱动机器人。待学习者预测 spacing 从 0.25 改为 0.5 m 并重启节点后，每条消息点数及发布频率如何变化，再安排最小运行核对。

## 路径发布实验（已运行）

学习者正确预测：spacing=0.5 m 时为 10/0.5+1=21 个点，发布频率不变。并提出“像 GPS 打点后逐点走到末端”的类比。

实验前在宿主机核对：没有运行容器或相关 ROS 进程，11311/11371 未监听，已有 Noetic 镜像可用。初次沙箱检查无法访问 Docker 和 socket，宿主机权限检查成功，未据此诊断 Docker 故障。

使用临时容器 underwater-objectnav-day6-path、只读工程挂载、独立 Master 11371，仅启动 roscore、路径发布节点及观察节点；不启动 Stonefish、控制器或分配器。容器内设置 ROS_MASTER_URI=http://localhost:11371，先运行 roscore -p 11371，再顺序运行：

```bash
/usr/bin/python3 /workspace/src/aquaflow_stonefish/scripts/nominal_reference_publisher.py _spacing:=0.25 __name:=day6_nominal_reference
/usr/bin/python3 /workspace/src/aquaflow_stonefish/scripts/nominal_reference_publisher.py _spacing:=0.5 __name:=day6_nominal_reference
```

每组收到至少 7 条消息，舍去首条后对 6 条采样。第一组节点停止后才启动第二组。

| spacing | 每条 Path 点数 | 到达频率 | header 时间频率 |
|---|---|---|---|
| 0.25 m | 六条均为 41 | 2.00008 Hz | 1.99997 Hz |
| 0.5 m | 六条均为 21 | 1.99989 Hz | 1.99994 Hz |

两组类型均为 nav_msgs/Path、frame_id=world_ned，首尾为 (-5,0,0)/(5,0,0) m；每组点坐标保持不变，各点时间戳与本条 Path 时间戳一致。发布者均为 /day6_nominal_reference。这是真实 ROS 节点与消息通信，路径按参数生成，没有采样仿真里程计或验证机器人运动。

附加源码核对：planar_pid_tracker.py:219 将 nearest 固定为 0，:220 按 nearest+lookahead 选点，默认 lookahead=2；tracking.launch 直接订阅完整 nominal_path，没有启动注释所假定的 reference_processor。故当前默认链不能直接解释成逐点推进到终点：41 点时始终选下标 2，即 (-4.5,0,0) m，21 点时对应 (-4,0,0) m。这是静态源码结论，未运行控制器或修复实现。

实验结束后容器已自动删除，11371 已释放。观察程序通过标准输入运行，没有创建临时脚本或修改原程序；只保留本记录。实际 odometry→回调状态→定时 Path 发布实验及当天结束题仍待完成。

## 实际里程计到轨迹实验（已运行）

本次沿用已讨论的订阅/定时器触发与保存筛选预期。实验前重新核对：没有运行容器或相关 ROS 进程，11311/11371 未监听，DISPLAY=:1，当前 /run/user/1000/gdm/Xauthority 可读。

临时容器 underwater-objectnav-day6-odom 只读挂载工程与显示授权，使用 GPU 图形模式和独立 Master 11371。启动原始 random_pillars BricsBot 场景、原始 actual_path_recorder.py 和观察节点；没有启动 tracking.launch、PID 或推进器分配器。容器内主要入口依次为：

```bash
roscore -p 11371
roslaunch -p 11371 aquaflow_stonefish aquaflow_pool.launch scene:=random_pillars vehicle_model:=bricsbot graphics_quality:=low
/usr/bin/python3 /workspace/src/aquaflow_stonefish/scripts/actual_path_recorder.py _vehicle_name:=bricsbot __name:=day6_actual_path_recorder
```

以上入口在 ROS_MASTER_URI=http://localhost:11371 环境下分别运行。等待仿真里程计及非空 Path 到达后，以约 5.020 秒墙钟窗口采样；到达频率使用窗口内首末消息的单调时钟计算。

| 话题 | 类型 | 发布者 | 条数 | 到达频率 |
|---|---|---|---|---|
| /bricsbot/odometry | nav_msgs/Odometry | /stonefish_simulator | 501 | 99.923 Hz |
| /aquaflow/actual_path | nav_msgs/Path | /day6_actual_path_recorder | 50 | 9.999 Hz |

两者 header.frame_id 均为 world_ned。实际里程计的 child_frame_id 为 bricsbot/odometry（按收到的消息记录，不推断为 base_link）。ROS 图确认记录器订阅 /bricsbot/odometry。窗口内首条 Path 有 4 个点，末条有 89 个点；末条的全部 89 个位姿均能按时间戳在收到的真实 Odometry 中找到，位置与四元数完全相同。Path.header.stamp 等于末尾保存点的时间戳。

例：末条 Path 的末尾位置约为 (-4.999081, -0.002449, 0.299919) m；这是仿真实际输出，不是期望路径，也不是注入的合成位姿。501 是本观察窗口里程计消息数，89 是自记录器启动以来末条 Path 的累计点数，二者不对应同一统计起点，不能据此计算精确保留比例。

这支持“仿真里程计→回调保存位姿→定时器发布累计 Path”真实链路；没有直接读取节点进程内的 self.points，也没有测量物理步频。ROS 图中 /bricsbot/setpoint/pwm 和 /controller/generalized_force 均无发布者，因此本实验不证明受控跟踪；机器人可有被动运动。

实验容器已自动删除，11371 已释放，未发现残留相关 ROS 进程；没有改显示访问权限、原始源码或配置。观察代码通过标准输入执行，无临时脚本或大日志留存。第六天两条核心消息链均已运行核对，结束题待学习者回答。

## 结束题评价与交接

1. 学习者正确区分消息到达触发 odom_cb 与定时器触发 Path 发布，并说明累计位姿组成轨迹。主线通过。补充纠正：odom_cb 是 ROS 订阅回调，不是硬件中断服务函数；发布器按 0.1 秒周期发布当前快照，并不等待攒够一定数量的消息；无新点时也会重复发布已有轨迹。
2. 学习者准确指出 and 条件：位置不变时，只要与上一个保存点的时间间隔达到阈值，仍会保存。通过，先前时间条件的薄弱点已通过独立回答补齐。强调比较的是上一个保存点，不是上一条收到的消息。
3. 学习者正确区分实际轨迹来自 odometry、期望路径来自 nominal_reference_publisher，且认识到需要控制器。主线通过。补充精确职责：实际位姿由仿真器产生，记录器筛选、保存与打包；控制器结合期望路径和实际位姿计算控制输出，经推进器分配、执行器和物理响应才使机器人运动。控制器本身不“运动”。
4. 学习者准确找到 self.last_position = current 的写入，以及距离公式中 self.last_position[i] 的读取。通过。

第六天核心目标已达到；上述术语与控制职责细化由 Agent 补充，未另出应用题复测。已验证范围仍限于路径发布和真实里程计到轨迹记录链，未验证受控运动或完整逐点跟踪。

第七天从 tracking.launch → planar_pid_tracker.update → brics6_thruster_allocator 的链路开始，先核对当前代码及运行环境。保留重点问题：当前 tracker 的 nearest=0 与完整 nominal_path 直接连接，选点不会随运动逐点推进；讲解与实验须如实处理，不能预先保证沿完整路径到达终点。不要为此自动修复代码或直接启用推进器。
