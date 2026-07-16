"""Standalone teleop test: joy_node + rp1_teleop only, publishing /cmd_vel.

Useful for verifying the Xbox Series X mapping (`ros2 topic echo /cmd_vel`) before wiring the
rest of the rp1_bringup pipeline in on top of it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rp1_teleop'), 'config', 'joy_xbox_series_x.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='rp1_teleop',
            executable='teleop_node',
            name='teleop_node',
            parameters=[config],
            output='screen',
        ),
    ])
