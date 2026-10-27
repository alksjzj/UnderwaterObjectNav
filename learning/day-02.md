# 第 2 天：从 launch 找到真正的场景

学习跨 2026-10-24 至 2026-10-25（Asia/Shanghai）；本记录整理于 2026-10-25。范围：场景选择、启动参数传递和 C++ 入口；未启动本日仿真。

## 入口与关键路径

`aquaflow_pool.launch` 根据 `scene`、`vehicle_model` 计算启动参数 `scenario_file` → 拼出 `scenario_description` 完整路径 → include `simulator.launch` 并传入启动参数 → 子文件的 `<node>` 启动 `parsed_simulator`，节点名为 `stonefish_simulator` → `main()` 从 `argv[2]` 读取场景路径，保存到 `scenarioPath` → `ROSSimulationManager manager(rate, scenarioPath)`。

核心文件均在 `src/` 下：
- `aquaflow_stonefish/launch/aquaflow_pool.launch:13–33`
- `stonefish_ros/launch/simulator.launch:2–7`
- `stonefish_ros/src/parsed_simulator.cpp:43–51,88–90`

`simulation_data` 是模型、纹理等仿真资源的目录，不仅限于机器人；`scenario_description` 指向具体 SCN 文件，包含文件名。二者通过 `<node args="...">` 作为普通命令行参数传入，不是节点参数服务器上的参数。

## 预测、核对及实际证据

学习者最初预测 `scene:=empty_pool vehicle_model:=bricsbot` 会得到空水池和 BricsBot。读取选择表达式及 SCN 后修正：空水池固定选择 `aquaflow_empty_pool.scn`，其中 include `bluerov2_reference.scn`。通过字典展开和选择路线图学习按键取值，已回答随机柱子分支还需要查内部表。

执行了一次内联 Python 静态核对：用 ElementTree 读取实际 launch；计算实际 `scenario_file` 表达式；检查目标 SCN 存在及其顶层 include。结果：

| scene | vehicle_model | scenario_file | 机器人引用 |
|---|---|---|---|
| empty_pool | bluerov2 | aquaflow_empty_pool.scn | bluerov2_reference.scn |
| empty_pool | bricsbot | aquaflow_empty_pool.scn | bluerov2_reference.scn |
| random_pillars | bricsbot | aquaflow_random_pillars.scn | bricsbot_reference.scn |

按工程约定容器挂载路径 `/workspace` 展示参数展开：`argv[1]` 为 `/workspace/src/aquaflow_stonefish/data`，`argv[2]` 为对应 SCN 完整路径，`argv[3..6]` 为 `100.0`、`1200`、`800`、`medium`。这是静态计算和路径示意，没有调用真实 roslaunch 解析器，没有验证容器当前挂载、节点、窗口或话题。`demo` 入口另传 low 画质，本次展示的是父 launch 的默认 medium。

字典路线图只表达选择关系：Python 构建外层字典时会先计算各值，内部 `vehicle_model` 查找也会求值；不是 if/elif 的惰性分支执行，不能据图断言无效 vehicle_model 在 empty_pool 下不会报错。

## 回答与当前验收状态

1. 调用链：基本正确。学习者说明了父 launch → scenario_file → 拼接 → 子 launch → C++ scenarioPath。细化：scene 是字典选择的输入；子 launch 使用 args 按顺序传参；实际运行的是编译后的 parsed_simulator 可执行程序，cpp 是其源码。
2. 两种路径：基本正确。学习者能区分资源目录和 SCN 路径。补充：资源不只属于机器人；scenario_description 包含具体 SCN 文件名，不只是所在目录。
3. 步频与 FPS：补充回答后通过。学习者先准确指出 100 Hz 对应每秒仿真时间 100 个物理步，并不保证现实一秒完成计算；随后复述“物理配置是在更新机器人的位置，FPS 表明画面每秒绘制的张数”，已能区分物理状态更新与画面绘制。进一步精确：物理步频是每秒仿真时间的物理更新次数，状态还包括速度、姿态等，不只是位置。
4. 找代码：正确。准确找到 `sf::ROSSimulationManager manager(rate, scenarioPath);`（第 88 行）。此前也正确指出 `scenarioPath` 是保存场景路径的变量。

阶段结论：第 2 天的源码阅读与理解验收完成，配置静态核对完成；未进行本日运行时验证。下一学习日按第 3 天计划阅读场景组成，再在预测后安排单变量场景实验；不自动启动或修改场景。

## 改动与清理

未修改工程逻辑，未创建实验脚本、容器或进程；静态检查使用内联命令，没有额外临时文件需要清理。只新增本学习记录。没有重新验证第 1 天的运行状态，也未将配置核对描述为仿真运行成功。
