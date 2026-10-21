# 模块接入与使用

本仓库不包含机器人模型、地图、GNSS 接收机驱动或完整 Nav2 启动配置。先启动你自己的机器人/仿真，并在各终端加载 ROS 与本工作区 `install/setup.bash`。

## 建图后端

基本输入包括 `/scan`、`/odom`、`/imu` 和 TF。默认坐标系为 `map`、`odom`、`base_footprint`、`imu_link`。前端参数默认来自已安装的 slam_toolbox：

```bash
ros2 launch hdbot_gtsam_backend slam_gnss.launch.py use_sim_time:=true
```

实车按实际时钟设置 `use_sim_time:=false`。该命令不启动机器人。可通过 `slam_params_file:=/绝对路径/slam.yaml` 提供自定义前端参数。

GNSS 默认等待地理标定；未启用时仍可优化前端位姿图。复制后端的 `config/gnss_backend.yaml`，设置 map 到 ECEF 的旋转和平移、天线杆臂、IMU 旋转与零偏，确认完成标定后设置 `georeference_ready: true`，然后运行：

```bash
ros2 launch hdbot_gtsam_backend slam_gnss.launch.py \
  gnss_params_file:=/绝对路径/gnss.yaml use_sim_time:=false
```

GNSS 数据类型为 `hdbot_gnss_msgs/msg/DopplerEpoch`。需要原始多普勒、卫星状态等字段，单条 NavSatFix 不足以运行多普勒优化。所有输入须使用一致的时钟；参数、单位、符号和可观测性见 [后端说明](../chapt8_ws/src/hdbot_gtsam_backend/README.md)。

不要同时运行其他发布 `map -> odom` 的建图器或 AMCL。`/gnss_corrected_pose` 是修正位姿输出，不含后验协方差。

## Nav2 规划与控制插件

将下列片段合并到你自己的完整 Nav2 参数文件，其他定位、地图、代价地图、行为树等配置由机器人系统提供：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: [GridBased]
    GridBased:
      plugin: nav2_custom_planner/CustomPlanner
      interpolation_resolution: 0.1
controller_server:
  ros__parameters:
    controller_plugins: [FollowPath]
    FollowPath:
      plugin: nav2_custom_controller::CustomController
      max_linear_speed: 0.1
      max_angular_speed: 1.0
```

规划器只生成直线路径，不自动绕障；只检查采样点的致命障碍物栅格，没有完整足迹或连续碰撞检测。控制器为基础路径跟随实现，不具备完整局部避障和安全停车保障；应先在受控仿真中验证。

## 循环巡检

先使自己的 Nav2 导航系统可用，再修改 `autopartol_robot/config/partol_config.yaml` 中的实际初始位置和目标点：

```bash
ros2 run autopartol_robot partol_node --ros-args \
  --params-file "$PWD/src/autopartol_robot/config/partol_config.yaml"
```

上例在 `chapt8_ws` 目录执行；安装后的配置也在包的 `share/autopartol_robot/config` 下。目标点数组每三个数为一个 `[x, y, yaw]`，单位为米、米、弧度，坐标系为 `map`。初始位姿恰好一个点；目标点不得为空。

| 参数 | 含义 |
| --- | --- |
| `use_sim_time` | 仿真为 true；实车根据时钟设置 |
| `image_save_path` | 图像目录，自动创建，空值为当前工作目录 |
| `image_max_age_s` | 图像最大年龄，默认 2 秒 |
| `initial_point` | 初始 `[x, y, yaw]` |
| `target_points` | 顺序执行并循环的目标点数组 |

节点成功到点后尝试保存 PNG，失败或取消时跳过该点图像。文件名包含地图位置、采集时间和随机标识。默认相机话题 `/camera_sensor/image_raw` 可通过 `-r /camera_sensor/image_raw:=/你的相机话题` 重映射。图像与位姿为近似关联，非严格时间同步。按 Ctrl+C 退出节点；当前未提供独立任务管理界面，实车需另行验证导航取消与底盘停止行为。

## 常见问题

- 找不到包或插件：确认编译成功、加载工作区环境，并重启加载插件的进程。
- 后端找不到 GTSAM：检查固定提交、`GTSAM_DIR` 和共享库依赖；不要随意升级外部库。
- GNSS 因子为零：检查地理参考、消息坐标系、有效信号数量和同步容差。
- 规划失败：检查地图范围和直线路径中的障碍物，当前规划器不能绕行。
- 无图片：检查导航结果、相机消息、时钟、TF 和目录权限。
