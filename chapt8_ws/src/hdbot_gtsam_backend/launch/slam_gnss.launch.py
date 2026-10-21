# Copyright 2026 hd
# SPDX-License-Identifier: Apache-2.0

"""Start mapping with the planar GNSS backend; Gazebo is launched separately."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    backend = get_package_share_directory('hdbot_gtsam_backend')
    # 使用外部 slam_toolbox 的通用前端配置，不依赖本地机器人模型包。
    frontend = get_package_share_directory('slam_toolbox')
    node = LifecycleNode(
        package='hdbot_gtsam_backend', executable='slam_gnss_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[LaunchConfiguration('slam_params_file'),
                    LaunchConfiguration('gnss_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}])
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(
            frontend, 'config', 'mapper_params_online_async.yaml')),
        DeclareLaunchArgument('gnss_params_file', default_value=os.path.join(
            backend, 'config', 'gnss_backend.yaml')),
        activate, node,
        EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                  transition_id=Transition.TRANSITION_CONFIGURE)),
    ])
