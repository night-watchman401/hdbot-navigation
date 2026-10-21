# Copyright 2026 hd
# SPDX-License-Identifier: Apache-2.0

"""基于 Nav2 的循环巡检、导航结果检查与到点图像记录。"""

import math
from pathlib import Path

import rclpy

# 位姿消息：PoseStamped 包含坐标系和时间戳信息
from geometry_msgs.msg import PoseStamped

# Nav2 导航接口及导航任务结果枚举
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# TF 坐标变换工具：Buffer 保存变换,TransformListener 接收变换
from tf2_ros import TransformListener, Buffer, TransformException

# 将平面航向角转换为四元数
from tf_transformations import quaternion_from_euler

# ROS2 时间间隔类型,用于解析导航反馈中的剩余时间
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

# ROS2 图像消息类型
from sensor_msgs.msg import Image

# ROS2 图像消息与 OpenCV 图像之间的转换工具
from cv_bridge import CvBridge, CvBridgeError
from autopartol_robot.patrol_utils import parse_points, image_filename

# OpenCV,用于将图像保存到文件
import cv2


class PatrolNode(BasicNavigator):
    """
    自动巡逻节点。

    继承 BasicNavigator,使用其提供的 Nav2 导航功能。
    主要功能：
    1. 初始化机器人位姿。
    2. 依次导航到参数中配置的目标点。
    3. 报告导航结果，仅在成功到达后保存巡检图像。
    4. 获取机器人当前位置,并保存相机图像。
    """

    def __init__(self, node_name='patrol_node'):
        # 初始化父类,创建导航节点
        super().__init__(node_name)

        # 声明初始位姿参数,格式为 [x, y, yaw]
        # x、y 的单位为米,yaw 为绕 Z 轴的旋转角,单位为弧度
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])

        # 声明巡逻目标点参数,每三个数表示一个目标点 [x, y, yaw]
        # 默认包含两个目标点：[0.0, 0.0, 0.0] 和 [1.0, 1.0, 1.57]
        self.declare_parameter(
            'target_points',
            [0.0, 0.0, 0.0, 1.0, 1.0, 1.57]
        )

        # 读取参数值,保存到成员变量中
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        # 在启动阶段拒绝不完整或非有限的坐标，避免静默丢弃目标点。
        parse_points(self.initial_point_, single=True)
        parse_points(self.target_points_)

        # 创建 TF 缓冲区,用于保存和查询坐标系之间的变换
        self.buffer_ = Buffer()

        # 创建 TF 监听器,将接收到的坐标变换写入缓冲区
        self.listener_ = TransformListener(self.buffer_, self)

        # 声明图像保存路径参数
        # 默认空字符串表示将图片保存到程序的当前工作目录
        self.declare_parameter('image_save_path', '')
        self.image_save_path = self.get_parameter('image_save_path').value
        self.declare_parameter('image_max_age_s', 2.0)
        self.image_max_age_s = self.get_parameter('image_max_age_s').value
        if not math.isfinite(self.image_max_age_s) or self.image_max_age_s <= 0:
            raise ValueError('image_max_age_s 必须是有限正数')

        # 创建图像转换工具
        self.bridge = CvBridge()

        # 保存最近收到的一帧 ROS2 图像消息
        # 尚未收到图像时为 None
        self.latest_image = None

        # 订阅相机原始图像话题
        # 使用传感器 QoS，兼容相机的 best-effort 发布方式
        self.subscription_image = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

    def image_callback(self, msg):
        """
        相机图像订阅回调。

        每收到一帧图像,就更新 latest_image,
        供后续 record_image 方法保存。
        """
        self.latest_image = msg

    def record_image(self):
        """
        保存最近收到的图像。

        使用保存时查询到的机器人 x、y 坐标命名图片。
        """
        if self.latest_image is None:
            self.get_logger().warn('尚未收到相机图像，本次不保存')
            return False
        try:
            # 相机与节点必须使用相同的 ROS 时钟，过期或未来图像不作为本次记录。
            age = (self.get_clock().now().nanoseconds -
                   rclpy.time.Time.from_msg(self.latest_image.header.stamp).nanoseconds) / 1e9
            if age < 0 or age > self.image_max_age_s:
                self.get_logger().warn('相机图像时间无效或已过期，本次不保存')
                return False
            pose = self.get_current_pose()
            # 使用 Path 拼接路径，目录参数无需手工补斜杠。
            directory = Path(self.image_save_path or '.').expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            stamp = self.latest_image.header.stamp
            age = (self.get_clock().now().nanoseconds -
                   rclpy.time.Time.from_msg(stamp).nanoseconds) / 1e9
            if age < 0 or age > self.image_max_age_s:
                self.get_logger().warn('等待定位期间图像已过期，本次不保存')
                return False
            filename = image_filename(
                pose.translation.x, pose.translation.y, stamp.sec, stamp.nanosec)
            # 明确转换为 BGR 编码，保证 OpenCV 保存的彩色图像通道正确。
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
            output = directory / filename
            if not cv2.imwrite(str(output), cv_image):
                raise OSError(f'图像写入失败：{output}')
            self.get_logger().info(f'巡检图像已保存：{output}')
            return True
        except (TransformException, CvBridgeError, cv2.error, OSError) as error:
            self.get_logger().error(f'无法保存巡检图像：{error}')
            return False

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        根据平面位置和朝向生成 PoseStamped 消息。

        参数：
            x：地图坐标系中的 x 坐标,单位为米。
            y：地图坐标系中的 y 坐标,单位为米。
            yaw：绕 Z 轴的旋转角,单位为弧度。

        返回：
            包含位置和朝向的 PoseStamped 消息。
        """
        # 创建带坐标系信息的位姿消息
        pose = PoseStamped()

        # 指定该位姿使用地图坐标系
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # 设置平面位置,z 保持默认值 0
        pose.pose.position.x = x
        pose.pose.position.y = y

        # 将欧拉角转换为四元数
        # 参数顺序为 roll、pitch、yaw,此处仅设置 yaw
        rotation_quat = quaternion_from_euler(0, 0, yaw)

        # 转换结果的排列顺序为 [x, y, z, w]
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]

        return pose

    def init_robot_pose(self):
        """
        设置机器人初始位姿,并等待 Nav2 导航系统激活。
        """
        # 重新读取初始位姿参数,使用当前参数值
        self.initial_point_ = self.get_parameter('initial_point').value
        parse_points(self.initial_point_, single=True)

        # 将 [x, y, yaw] 转换为 PoseStamped,并提交初始位姿
        self.setInitialPose(
            self.get_pose_by_xyyaw(
                self.initial_point_[0],
                self.initial_point_[1],
                self.initial_point_[2]
            )
        )

        # 等待定位和导航相关节点进入可用状态
        self.waitUntilNav2Active()

    def get_target_points(self):
        """
        从参数中读取并整理巡逻目标点。

        将扁平数组：
            [x1, y1, yaw1, x2, y2, yaw2, ...]

        转换为目标点列表：
            [[x1, y1, yaw1], [x2, y2, yaw2], ...]
        """
        self.target_points_ = self.get_parameter('target_points').value
        return parse_points(self.target_points_)

    def nav_to_pose(self, target_pose):
        """
        导航到指定位姿,等待任务结束并输出导航结果。

        参数：
            target_pose：目标位置和朝向,类型为 PoseStamped。
        """
        # 确认 Nav2 导航系统已经激活
        self.waitUntilNav2Active()

        # 发送导航目标,启动导航任务
        if not self.goToPose(target_pose):
            self.get_logger().error('导航目标被拒绝')
            return None

        # 在导航任务完成前,持续读取反馈
        while rclpy.ok() and not self.isTaskComplete():
            feedback = self.getFeedback()

            # 有反馈时,输出预计剩余导航时间
            if feedback:
                # 将反馈中的时间间隔转换为纳秒,再除以 1e9 得到秒
                self.get_logger().info(
                    f'预计: '
                    f'{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9}'
                    f' s 后到达',
                    throttle_duration_sec=5.0
                )

        if not rclpy.ok():
            return None

        # 导航任务结束后,获取最终结果
        result = self.getResult()

        # 根据结果枚举输出对应日志
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航结果：失败')
        else:
            self.get_logger().error('导航结果：返回状态无效')
        return result

    def get_current_pose(self):
        """
        通过 TF 查询机器人在地图坐标系中的当前位姿。

        返回：
            Transform 对象,其中：
            translation 表示位置；
            rotation 表示四元数形式的朝向。
        """
        # 有限重试并主动处理回调，避免 TF 缺失时永久阻塞巡检流程。
        # 零超时查询配合 spin_once，允许同一线程中的 TF 订阅更新缓冲区。
        for _ in range(30):
            if not rclpy.ok():
                break
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                return self.buffer_.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time()).transform
            except TransformException:
                continue
        raise TransformException('无法在有限重试次数内获取 map 到 base_footprint 的变换')


def main():
    """
    程序主入口：初始化机器人,并循环执行巡逻任务。
    """
    # 初始化 ROS2 通信环境
    rclpy.init()

    patrol = None
    try:
        patrol = PatrolNode()
        patrol.init_robot_pose()
        while rclpy.ok():
            for x, y, yaw in patrol.get_target_points():
                if not rclpy.ok():
                    break
                result = patrol.nav_to_pose(patrol.get_pose_by_xyyaw(x, y, yaw))
                # 导航失败或取消时不得将当前位置的图像标记为成功到点记录。
                if result == TaskResult.SUCCEEDED:
                    rclpy.spin_once(patrol, timeout_sec=0.1)
                    patrol.record_image()
    except KeyboardInterrupt:
        pass
    finally:
        # 正常退出和异常退出均释放节点及 ROS 通信资源。
        if patrol is not None:
            patrol.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# 直接运行该 Python 文件时,执行 main()
# 如果该文件被其他模块导入,则不会自动执行 main()
if __name__ == '__main__':
    main()