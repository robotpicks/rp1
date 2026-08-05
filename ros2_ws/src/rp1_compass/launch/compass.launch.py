"""Starts the IST8310 compass driver alone, for bench testing without the rest of the rp1 stack.

require_i2c:=false runs it as a dry-run (no I2C device opened, no data published) -- useful for
confirming the node starts cleanly before hardware is wired up.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rp1_compass'), 'config', 'rp1_compass.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'require_i2c', default_value='true',
            description='false runs a dry-run with no I2C device opened'),

        Node(
            package='rp1_compass',
            executable='ist8310_driver',
            name='ist8310_driver',
            output='screen',
            # ParameterValue(..., value_type=bool) is required here, not decorative -- a bare
            # LaunchConfiguration substitution in a parameters dict is always a string, and
            # rclpy's parameter type checking rejects 'false' (str) against a bool-typed
            # declare_parameter default.
            parameters=[config, {
                'require_i2c': ParameterValue(LaunchConfiguration('require_i2c'), value_type=bool),
            }],
        ),
    ])
