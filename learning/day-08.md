# 第 8 天：PID 和推进器分配

日期：2026-10-26（Asia/Shanghai），按学习者要求跳过。

## 目标与范围

已读取当前《学习方法.md》第八天及第七天收尾记录；第七天主线与综合关卡已完成。本日理解误差、积分、限幅、超时，以及期望广义力经推进器几何分配与标定曲线变为六路命令的过程。

三个核心文件：`src/aquaflow_stonefish/launch/tracking.launch`、`src/aquaflow_stonefish/scripts/planar_pid_tracker.py`、`src/aquaflow_stonefish/scripts/brics6_thruster_allocator.py`。后续核对推进器符号时按需读取实际加载的机器人 SCN。

调用链：期望 Path + 实际 Odometry → tracker.update（误差、反馈、积分、力限幅）→ WrenchStamped → allocator（伪逆分配、推力到归一化命令）→ 六路 PWM → Stonefish → 新的 Odometry。

当前 tracker 仍为 nearest=0、默认 lookahead=2；tracking.launch 未启动 reference_processor，默认直线路径目标为下标 2。单增益实验应在相同场景、初始状态、固定目标下比较响应，不能当作完整路径跟踪。

## 第一小段与待回答预测

先读 tracker.update 中 `fx = self.kp_xy * ex - self.kd_xy * vx + self.ki_xy * self.int_x` 及发布前的 max/min 限幅。说明 ex 为机体系前向误差（m），vx 为代码读取的前向速度反馈（m/s），fx 为限幅前前向力需求（N）；当前默认 kp_xy=15、kd_xy=3、ki_xy=0、max_fx=4 N，tracking.launch 未覆盖这些参数。

待学习者预测：假设 ex=0.5 m、vx=0、ki_xy=0，其他条件相同，将 kp_xy 从 15 改为 30，最终发布的 force.x 是否翻倍，各是多少？先保留预测，不提前记录数值答案。

本轮只核对源码与记录，未检查当前 ROS 运行状态、启动仿真、执行调参或纯计算实验。保留用户已有 tracker 注释及其他未提交修改；未修改控制源码，未创建临时探针。后续实际实验前重新核对容器、进程、Master 和控制输出发布者。单增益误差图、推力分配计算及当日验收待完成。

## 学习安排调整

学习者表示相关知识在 STM32 小车控制学习时已接触，明确要求跳过第八天。停止本日预测、调参实验和验收安排，直接进入第九天。此状态表示按已有基础跳过，不表示第八天实验或工程特有的推进器分配、标定与符号检查已经验证完成；后续需要时再结合具体代码补充。
