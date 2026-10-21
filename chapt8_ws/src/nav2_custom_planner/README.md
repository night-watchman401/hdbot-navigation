# nav2_custom_planner：ROS 2 Jazzy 修改说明

本模块为本项目自主开发的直线全局规划器，包含 Jazzy 接口适配与边界处理。代码中可搜索 `[修改01]` 到 `[修改08]`，定位每组修改及其中文解释。

这是一个直线全局规划器：根据起点、目标和插值分辨率生成直线采样点，遇到致命障碍物时报规划失败。

## 改了哪些地方，为什么改

| 编号 | 文件/位置 | 修改内容 | 原因 |
| --- | --- | --- | --- |
| 修改01 | .hpp 声明、.cpp 定义及循环 | 引入 `<functional>`，给 `createPlan()` 增加 `std::function<bool()> cancel_checker`，在入口和两个循环中检查取消 | Jazzy 的 GlobalPlanner 使用三个参数。声明、定义和基类必须一致；收到取消后抛出 PlannerCancelled，让服务器停止本次规划 |
| 修改01 | .cpp 头文件 | 将 `nav2_core/exceptions.hpp` 改成 `nav2_core/planner_exceptions.hpp` | 使用 Jazzy 的规划器异常头文件 |
| 修改02 | CMakeLists.txt、package.xml、头文件 | 显式声明和链接直接依赖，设置目标使用 C++17，生成 compile_commands.json，直接包含用到的类型和代价值定义 | 让构建依赖完整，同时让编辑器能读取真实编译参数 |
| 修改03 | .hpp 成员、configure()、cleanup()、createPlan() | 明确节点指针类型；初始化地图指针和分辨率；检查失效父节点和空地图；验证成功后才保存地图指针；清理时释放引用 | 减少空指针、未初始化成员、配置失败后仍继续规划等问题。节点类型写法的调整也便于理解 parent.lock() 返回什么 |
| 修改04 | configure()、createPlan() | 检查分辨率为有限正数、平面坐标与距离有效；坐标系不匹配时抛出 PlannerTFError；输出实际配置参数 | 避免无效数值参与插值，并让错误原因和参数加载结果更容易定位 |
| 修改05 | 路径插值 | 使用 ceil() 向上取整；非零距离至少一段；重合起终点只保留目标；整数转换前检查范围 | 修复距离小于分辨率时循环次数为 0、步长除零的问题；使实际采样间距不大于请求的分辨率（允许浮点舍入误差） |
| 修改06 | 中间路径点 | 根据 atan2(dy, dx) 计算朝向，用 sin(yaw/2) 与 cos(yaw/2) 生成单位四元数；统一路径时间戳和坐标系 | 默认的全零四元数不能表示有效旋转；中间点应具备有效姿态。末点继续采用请求中的目标朝向 |
| 修改07 | 目标点加入顺序 | 在障碍物检查之前加入目标点 | 原代码先检查已有点、再加入目标，导致目标所在栅格没有被检查 |
| 修改08 | 代价地图检查 | 读取期间持有地图互斥锁；按 const 引用遍历；越界明确失败；致命障碍物抛出 NoValidPathCouldBeFound；增加路径点数 debug 日志 | 避免和地图更新并发读写，避免把越界当成规划成功，减少消息复制，并明确规划失败原因 |

补充理解：

- `costmap_` 是借用的裸指针，地图由 Costmap2DROS 管理。因此 cleanup() 只将它置空，不执行 delete。
- `node_` 和 `tf_` 是共享指针，reset() 释放本插件持有的那份引用。
- `unique_lock` 使用作用域管理锁；正常返回或抛异常时都会解锁。
- `interpolation_resolution` 在 configure() 时读取。本示例没有实现运行时参数更新回调，修改 YAML 后需重新配置或重启。

## 安装与编译

将本目录放到工作空间的 `src/nav2_custom_planner`。替换旧包时，将备份放到工作空间之外，避免 colcon 扫描出同名包。

在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash

# 缺少依赖时执行；需要系统已配置 rosdep。
rosdep install --from-paths src/nav2_custom_planner --ignore-src -r -y --rosdistro jazzy

colcon build \
  --packages-select nav2_custom_planner \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 上一步成功后再执行。
source install/setup.bash
```

CMake 已启用编译命令导出，通常可以在以下位置找到：

```text
build/nav2_custom_planner/compile_commands.json
```

VS Code 打开整个工作空间，在 C/C++ 扩展配置中将 compileCommands 指向该文件。这个文件负责编辑器解析；实际编译是否成功以 colcon 输出为准。

## Nav2 参数片段

将下面内容合并到实际使用的完整 Nav2 YAML。它只是规划器配置片段，不包含 global_costmap、定位和其他导航节点的完整参数。

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_custom_planner/CustomPlanner"
      interpolation_resolution: 0.1
```

这里 `plugin` 匹配 custom_planner_plugin.xml 中的 `name`；XML 中的 `type` 则是 C++ 类型 `nav2_custom_planner::CustomPlanner`。当前注册名含 `/`，因此 YAML 也应使用这个精确的注册名。

启动 Nav2 的终端也需要 source 本工作空间的 install/setup.bash。重新编译后需重启加载插件的进程，使其使用新的共享库。

启动后可检查（使用命名空间时相应调整节点路径）：

```bash
ros2 pkg prefix nav2_custom_planner
ros2 param get /planner_server GridBased.plugin
ros2 param get /planner_server GridBased.interpolation_resolution
ros2 lifecycle get /planner_server
```

## 调试建议

插件是共享库，调试目标是加载它的 planner_server。可在原 Nav2 启动命令中加入 `use_composition:=False`，使 planner_server 独立运行，再通过 VS Code 或 GDB 附加这个进程。

适合设置断点的位置：

1. configure() 中读取参数之后：查看 name_、global_frame_ 和 interpolation_resolution_。
2. createPlan() 入口：查看 start、goal 和坐标系。
3. 插值段数计算后：查看 distance、required_steps、total_number_of_loop。
4. worldToMap() 和 getCost() 之后：查看转换结果、mx、my 和 cost。

configure() 在配置阶段调用；如果在节点激活后才附加，只能观察后续发生的调用。createPlan() 在新的规划请求到达时调用，可发送导航目标触发。

如果在断点停留时生命周期管理器因心跳超时关闭导航，可在启动 lifecycle_manager_navigation 的 Node 的 parameters 列表中临时增大 bond_timeout，调试后恢复原值。

## 已完成的验证及范围

2026-10-14：

- 对照 Jazzy 官方 GlobalPlanner、规划器异常和 Costmap2D 锁接口核对了本次使用的声明。
- 使用 g++ 以 C++17、`-Wall -Wextra -Wpedantic -Werror` 编译实际修改后的 .cpp；ROS 消息、节点、地图和导出宏由临时测试替身提供。
- 启用了未定义行为、浮点除零和浮点转整数越界检查。18 项行为检查全部通过：正常插值、短距离、重合点、姿态、末点障碍物、中间点障碍物、地图越界、坐标系、非法分辨率、非法坐标、距离溢出、段数溢出、三种取消时机、清理后调用、空依赖以及配置失败后调用。
- 检查了 package.xml、插件 XML、依赖声明和最终压缩包。

上述为早期替身测试记录。后续已在 ROS 2 Jazzy 环境完成真实 colcon 构建；完整导航验证仍需实际执行，见根目录 docs/VALIDATION.md。

算法仍只生成直线；碰撞检查只检查采样点所在栅格的 LETHAL_OBSTACLE。没有实现绕障搜索、完整机器人足迹检测或采样点之间的连续碰撞检测，也没有新增未知区域或膨胀代价的通行策略。

## 核对用的官方资料

- [Jazzy GlobalPlanner 接口](https://api.nav2.org/nav2-jazzy/html/global__planner_8hpp_source.html)
- [Jazzy 规划器异常](https://api.nav2.org/nav2-jazzy/html/planner__exceptions_8hpp_source.html)
- [Jazzy Costmap2D 接口](https://api.nav2.org/nav2-jazzy/html/costmap__2d_8hpp_source.html)
- [Nav2 调试方法](https://docs.nav2.org/jazzy/tutorials/general_tutorials/get_backtrace/)
