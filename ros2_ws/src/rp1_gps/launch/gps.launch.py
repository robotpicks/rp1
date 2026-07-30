"""Starts the u-blox M10 GNSS driver (ublox_gps, stock package) with rp1's config.

Unlike rp1_imu/rp1_elrs, ublox_gps has no built-in dry-run/require_serial escape hatch -- it
opens the configured device on startup and will fail if that port doesn't exist or isn't a real
u-blox receiver. device:=/dev/pts/N against a socat virtual serial pair (`socat -d -d pty,raw
pty,raw`) confirms the launch/config wiring reaches the node correctly (package, executable,
config values all resolve, and it gets as far as opening the port) but does NOT fully stand in
for hardware: confirmed 2026-07-30 that ublox_gps's baud-rate configuration step throws
std::runtime_error against a pty ("Could not configure serial baud rate") since a pty doesn't
support real termios baud-rate configuration the way genuine UART hardware does. That crash is
expected from the test method, not a bug in this config -- it's the furthest a hardware-free
smoke test can go for this particular driver.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rp1_gps'), 'config', 'rp1_gps.yaml')

    return LaunchDescription([
        # Default matches rp1_gps.yaml's own device: -- same always-override pattern
        # rp1_mvp.launch.py uses for can_iface, so this stays in sync if the yaml default
        # ever changes without needing a second edit here.
        DeclareLaunchArgument(
            'device', default_value='/dev/ttyUSB0',
            description="Serial device for the GNSS receiver; overrides rp1_gps.yaml's "
                        'device: (e.g. a socat test pty for a hardware-free smoke test)'),

        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            parameters=[config, {'device': LaunchConfiguration('device')}],
        ),
    ])
