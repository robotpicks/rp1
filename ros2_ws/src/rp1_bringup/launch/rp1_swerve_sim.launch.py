"""Standalone launch for rp1_sim's swerve_sim_node -- the full 4-wheel + 4-steering-actuator
simulator, no CAN/DroneCAN/VESCs required.

Unlike rp1_mvp_sim.launch.py, this does NOT chain from rp1_teleop/rp1_control: there is no
swerve inverse-kinematics controller in rp1_control yet (see CLAUDE.md's "Extending to full
swerve" section), so there's nothing upstream to resolve a body-twist command into per-wheel
speed+angle commands. Drive this node directly, e.g.:

  ros2 topic pub /wheel_cmd rp1_msgs/msg/WheelCommand "{velocity: [0.5, 0.5, 0.5, 0.5]}"
  ros2 topic pub /steering_cmd rp1_msgs/msg/SteeringCommand "{angle: [0.3, 0.3, 0.3, 0.3]}"

and watch /odom, /wheel_feedback, /steering_feedback, or rviz2 (TF: odom -> base_link).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_config = os.path.join(
        get_package_share_directory('rp1_sim'), 'config', 'swerve_sim.yaml')

    return LaunchDescription([
        Node(
            package='rp1_sim',
            executable='swerve_sim_node',
            name='swerve_sim_node',
            parameters=[sim_config],
            output='screen',
        ),
    ])
