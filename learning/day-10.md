# 第 10 天：Stonefish 内部与仿真器分层

日期：2026-10-27（Asia/Shanghai），进行中。

## 目标与阅读范围

按当前《学习方法.md》第十天，分三小段追踪：SCN 解析与机器人创建；ROS 推进器数组接收与执行器设定；执行器更新与物理施力。最后串起配置→解析→实体→执行器→物理更新→传感器→ROS。

已核对第九天记录：源码主线已讲解，学习者报告完成键盘体验；结束题按要求跳过，定量运行验收未独立核验。本日不重讲键盘/PID，也不假定旧仿真仍在运行。

核心文件：工程内 `src/stonefish_ros/src/ROSSimulationManager.cpp`、`ROSScenarioParser.cpp`；外部 `/home/xiela/Downloads/stonefish_workspace/stonefish/Library/src/core/ScenarioParser.cpp`、`../actuators/Thruster.cpp`。调用处按需补读。计划提到的推进器回调当前实际位于 ROSSimulationManager.cpp 的 ThrustersCallback::operator()，后续按真实定义追踪。

外部 Stonefish 源码 HEAD 为 b21eb8e194c570ff2f61e91aeffb38d73dc25f42，与当前 docker/Dockerfile 的 STONEFISH_COMMIT 相同；此核对不证明既有容器二进制的版本或当前运行状态。

## 第一小段：BuildScenario 的解析入口

阅读 ROSSimulationManager.cpp:144 起的 BuildScenario，先展开 `ROSScenarioParser parser(this)` 与 `bool success = parser.Parse(scnFilePath)`：构造解析器，把当前管理器指针交给它，再将场景文件路径传入解析入口，返回布尔值保存为 success。后续 SaveLog 保存解析日志，错误分支报告解析或日志保存失败；末尾注册服务并启动 ROS spinner。spinner 处理 ROS 回调，不能据此认定它承担物理步进或图形循环。

下一小段定位 ROSScenarioParser::ParseRobot：基础 ScenarioParser::ParseRobot 与额外 ROS 接口的职责划分。待学习者预测：仅移除机器人 SCN 中的推进器 ros_subscriber 标签、其他定义保留，机器人实体是否仍会创建？尚未独立作答或实验。

本轮只静态读取本机源码、计划与记录；未启动 ROS、检查现有 Master、发送推进器命令或执行仿真实验。未修改控制、场景和启动文件，未创建临时实验文件。

## 代码整理后的继续位置

之后按学习者要求，将 `src/` 切换到师兄整理后的 `f3abe92`，重新编译并更新第十天以后的计划；这次迁移不算第十天的教学或验收。Stonefish ROS 的上述两个解析源码与迁移前一致。下一个对话仍先从上面的 `ros_subscriber` 预测及 `ROSScenarioParser::ParseRobot()` 继续，不重复第九天或假定预测已回答。旧源码和构建产物留在 `.local-backups/pre-upstream-f3abe92/`，该本地备份不随 Git 克隆。
