# HDBot 建图与巡检模块

基于 ROS 2 Jazzy 的二维建图优化、Nav2 插件与巡检模块。仓库发布范围为下表五个包；GTSAM、ROS 2、Nav2 和 slam_toolbox 是外部依赖，不随本项目分发。本地机器人模型、地图及整机演示包不属于发布内容。

| 软件包 | 功能 |
| --- | --- |
| `hdbot_gtsam_backend` | Karto 求解器插件、二维多普勒适配、观测关联与批量优化 |
| `hdbot_gnss_msgs` | 原始 GNSS 多普勒消息接口 |
| `nav2_custom_planner` | 直线采样规划与路径点栅格检查 |
| `nav2_custom_controller` | 简单路径跟随控制器 |
| `autopartol_robot` | 多目标点循环巡检与图像记录 |

## 安装与使用

- [依赖安装和构建](docs/INSTALL.md)：Ubuntu 24.04、ROS 2 Jazzy、固定版本 GTSAM。
- [模块接入与使用](docs/USAGE.md)：传感器、Nav2 配置、巡检参数与故障排查。
- [后端模型和限制](chapt8_ws/src/hdbot_gtsam_backend/README.md)。
- [规划器说明](chapt8_ws/src/nav2_custom_planner/README.md)。
- [验证记录](docs/VALIDATION.md)。
- [来源与许可证范围](THIRD_PARTY.md)。

## 能力与验证范围

后端保留前端相对位姿观测及源坐标系下的完整协方差，使用车轮速度/无侧滑软约束与 GNSS 多普勒联合优化。GNSS 地理参考默认关闭，需标定后显式启用。软件不提供接收机驱动、星历传播或自动地理标定。

规划器不搜索绕障路线，控制器不提供完整局部碰撞预测。后端采用批量优化；尚未完成真实 GNSS 数据性能评估和完整 Gazebo 端到端验证，不将合成数据误差作为实测定位精度。

## 发布范围和许可证

`.gitignore` 已排除外部 GTSAM、本机演示包、备份压缩包、构建产物及本地软著草稿。它不会自动取消已经跟踪的文件；如果已有 Git 提交，需要另行检查暂存区和历史。

本项目五个发布模块采用 [Apache-2.0](LICENSE)，版权信息见 [NOTICE](NOTICE)。GTSAM 等外部依赖采用各自的许可证，说明见 [THIRD_PARTY.md](THIRD_PARTY.md)。

使用 `python3 scripts/export_source.py /tmp/hdbot-source-review` 可导出只含上述模块与公共文档的审阅目录；该命令不创建远程仓库或上传内容。
