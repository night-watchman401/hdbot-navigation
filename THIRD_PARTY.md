# 自主开发模块与外部依赖

## 自主开发模块

项目维护者已确认以下五个发布模块为自主开发，统一采用 Apache-2.0。
维护者：hd。许可证全文见根目录及各模块的 LICENSE，版权通知见根目录 NOTICE。

| 模块 | 许可证 | 内容 |
| --- | --- | --- |
| hdbot_gtsam_backend | Apache-2.0 | GTSAM 后端适配、自定义因子、同步关联与优化实现 |
| hdbot_gnss_msgs | Apache-2.0 | GNSS 原始观测消息定义 |
| nav2_custom_planner | Apache-2.0 | 直线规划插件及边界检查 |
| nav2_custom_controller | Apache-2.0 | 路径跟随控制插件 |
| autopartol_robot | Apache-2.0 | 多目标点巡检及图像记录 |

Python 包中的 ROS 测试模板保留其已有的 Open Source Robotics Foundation 版权头及 Apache-2.0 声明。
自主开发声明针对模块实现，不包括以下外部库及已有测试模板。

## 外部依赖（不随仓库分发）

| 依赖 | 上游及使用方式 |
| --- | --- |
| GTSAM | https://github.com/borglab/gtsam ，固定提交见 INSTALL；链接官方多普勒因子，许可依上游 LICENSE 及组件声明 |
| slam_toolbox / Karto | https://github.com/SteveMacenski/slam_toolbox ，前端位姿图、求解器接口及默认前端配置 |
| Nav2 | https://github.com/ros-navigation/navigation2 ，规划/控制接口及导航客户端 |
| ROS 2 | https://github.com/ros2 ，节点、消息、TF、生命周期、插件和构建基础设施，各包许可分别适用 |
| OpenCV / cv_bridge | https://github.com/opencv/opencv 、https://github.com/ros-perception/vision_opencv ，图像转换与保存 |
| Boost、Eigen、TBB 等 | 由 GTSAM 或 ROS 依赖链引入，按实际安装版本的上游声明处理 |

外部库不入库不等于没有许可义务；若未来分发包含外部库的容器、二进制或源码包，需要按实际包含内容补齐相应许可证和通知。本文件仅为来源清单，不替代任何上游许可证。

Apache 官方许可全文：https://www.apache.org/licenses/LICENSE-2.0.txt
