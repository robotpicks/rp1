"""rp1 MVP teleop pipeline against rp1_sim instead of real DroneCAN/VESC hardware.

Identical to rp1_mvp.launch.py except rp1_dronecan_bridge is swapped out for rp1_sim's
sim_bridge_node -- no CAN adapter, no DroneCAN, no VESCs required. Useful for checking the
rp1_teleop -> rp1_control -> (wheel_cmd/wheel_feedback/odom) ROS2 graph end to end, e.g. with
`ros2 topic echo` or rviz2, before any hardware is wired up.
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
    sim_config = os.path.join(
        get_package_share_directory('rp1_sim'), 'config', 'rp1_sim.yaml')

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
            package='rp1_sim',
            executable='sim_bridge_node',
            name='sim_bridge_node',
            parameters=[sim_config],
            output='screen',
        ),
    ])
