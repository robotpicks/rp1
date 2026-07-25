"""Standalone ELRS teleop test: elrs_driver + rp1_teleop, publishing /cmd_vel.

The generic elrs_driver (elrs_ros submodule) is the /joy source (CRSF RC channels ->
sensor_msgs/Joy); rp1_teleop maps it to /cmd_vel with the rp1 ELRS mapping (joy_elrs.yaml).
Useful for verifying the channel/switch mapping (watch /joy, or `ros2 topic echo /cmd_vel`)
before wiring in the rest of the pipeline. Telemetry back to the handset needs a /battery source
(the wheel_feedback_to_battery adapter, present in the full rp1_mvp_elrs pipeline).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    elrs_config = os.path.join(
        get_package_share_directory('elrs_driver'), 'config', 'elrs_driver.yaml')
    joy_elrs_config = os.path.join(
        get_package_share_directory('rp1_elrs'), 'config', 'joy_elrs.yaml')

    return LaunchDescription([
        Node(
            package='elrs_driver',
            executable='elrs_driver',
            name='elrs_driver',
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
