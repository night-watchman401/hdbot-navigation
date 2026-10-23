# nav2_custom_planner

ROS 2 Jazzy 全局规划插件，支持直线、A*、Theta*、Dijkstra 规划。

## 配置

将以下片段合并到 Nav2 的参数文件：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_custom_planner/CustomPlanner"
      algorithm: "A_STAR"
      interpolation_resolution: 0.1
      allow_unknown: false
```

`algorithm` 可选值：

| 值 | 行为 |
| --- | --- |
| `STRAIGHT` | 直线规划，遇障碍失败；默认值，兼容原配置 |
| `A_STAR` | 八邻域 A*，欧氏距离启发式，搜索绕障路径 |
| `THETA_STAR` | Theta*，在 A* 搜索中使用父节点视线连接，生成任意角度路径 |
| `DIJKSTRA` | 八邻域 Dijkstra，不使用启发式 |

参数在 configure 时读取，修改后需重新配置或重启 planner_server。
也可以配置三个插件实例，分别设置三种 algorithm，调用 Nav2 规划动作时通过 planner_id 选择实例。

`interpolation_resolution` 为输出路径最大点间距（米），必须为有限正数。
`allow_unknown` 控制是否允许经过未知栅格，默认禁止。
所有模式都禁止经过致命障碍和内切膨胀障碍（代价 253、254），禁止斜向穿过障碍角点。
线段检查遍历经过的栅格，不依赖输出采样间距。搜索优化几何距离，不对可通行栅格的膨胀代价值加权。
碰撞检查基于栅格，完整机器人足迹的安全距离应由代价地图膨胀层配置保证。

路径保留精确起点、目标及目标朝向，中间点沿路径朝向。搜索、插值和碰撞检查均支持取消；无路可达时抛出 Nav2 规划异常。

## 构建与测试

在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav2_custom_planner --symlink-install
source install/setup.bash
colcon test --packages-select nav2_custom_planner
colcon test-result --verbose
```

`planner_smoke` 使用真实 ROS 节点和 Costmap2D，检查三种搜索模式的绕障、路径端点、路径点通行性、无路可达、取消和禁止斜向穿角。
完整导航效果仍需在机器人或仿真中验证。
