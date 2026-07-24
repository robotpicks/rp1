"""Standalone ELRS teleop test: elrs_node + rp1_teleop only, publishing /cmd_vel.

The ELRS analogue of rp1_teleop/launch/teleop.launch.py -- but there is no joy_node here;
elrs_node is itself the /joy source (CRSF RC channels -> sensor_msgs/Joy). Useful for verifying
the channel/switch mapping (`ros2 topic echo /cmd_vel`, or a small rclpy subscriber on /joy)
before wiring in the rest of the rp1_bringup pipeline.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    elrs_config = os.path.join(
        get_package_share_directory('rp1_elrs'), 'config', 'rp1_elrs.yaml')
    joy_elrs_config = os.path.join(
        get_package_share_directory('rp1_elrs'), 'config', 'joy_elrs.yaml')

    return LaunchDescription([
        Node(
            package='rp1_elrs',
            executable='elrs_node',
            name='elrs_node',
            parameters=[elrs_config],
            output='screen',
        ),
        Node(
            package='rp1_teleop',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_elrs_config],
            output='screen',
        ),
    ])
