"""Starts the ICM20948 IMU driver alone, for bench testing without the rest of the rp1 stack.

require_spi:=false runs it as a dry-run (no SPI device opened, no data published) -- useful for
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
        get_package_share_directory('rp1_imu'), 'config', 'rp1_imu.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'require_spi', default_value='true',
            description='false runs a dry-run with no SPI device opened'),

        Node(
            package='rp1_imu',
            executable='icm20948_driver',
            name='icm20948_driver',
            output='screen',
            # ParameterValue(..., value_type=bool) is required here, not decorative -- a bare
            # LaunchConfiguration substitution in a parameters dict is always a string, and
            # rclpy's parameter type checking rejects 'false' (str) against a bool-typed
            # declare_parameter default.
            parameters=[config, {
                'require_spi': ParameterValue(LaunchConfiguration('require_spi'), value_type=bool),
            }],
        ),
    ])
