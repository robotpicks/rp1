"""Full rp1 MVP teleop pipeline: joy -> rp1_teleop -> rp1_control -> rp1_dronecan_bridge.

Each node loads its package's own default config first, then rp1_bringup's rp1_mvp.yaml on top
for the handful of values that are robot-specific (track width, CAN interface, deadman button).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rp1_bringup')
    bringup_config = os.path.join(bringup_share, 'config', 'rp1_mvp.yaml')

    teleop_config = os.path.join(
        get_package_share_directory('rp1_teleop'), 'config', 'joy_xbox_series_x.yaml')
    control_config = os.path.join(
        get_package_share_directory('rp1_control'), 'config', 'rp1_control.yaml')
    bridge_config = os.path.join(
        get_package_share_directory('rp1_dronecan_bridge'), 'config',
        'rp1_dronecan_bridge.yaml')

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
            parameters=[teleop_config, bringup_config],
            output='screen',
        ),
        Node(
            package='rp1_control',
            executable='control_node',
            name='control_node',
            parameters=[control_config, bringup_config],
            output='screen',
        ),
        Node(
            package='rp1_dronecan_bridge',
            executable='bridge_node',
            name='bridge_node',
            parameters=[bridge_config, bringup_config],
            output='screen',
        ),
    ])
