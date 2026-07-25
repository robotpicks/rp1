"""rp1 MVP teleop pipeline driven by an ExpressLRS radio instead of the Xbox pad:
elrs_driver -> /joy -> rp1_teleop -> rp1_control -> rp1_dronecan_bridge.

Identical to rp1_mvp.launch.py except the /joy source: there is no joy_node here; the generic
elrs_driver (elrs_ros submodule) reads the CRSF link and publishes /joy itself. Telemetry back to
the handset is the second node: rp1_elrs's wheel_feedback_to_battery adapter turns the bridge's
/wheel_feedback into the /battery that elrs_driver forwards to the RX. Each node loads its
package's own default config first, then rp1_bringup's rp1_mvp.yaml on top for the robot-specific
values (serial port, track width, CAN interface, deadman button).

Do not also start joy_node against this pipeline -- two publishers on /joy would fight.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rp1_bringup')
    bringup_config = os.path.join(bringup_share, 'config', 'rp1_mvp.yaml')

    elrs_driver_config = os.path.join(
        get_package_share_directory('elrs_driver'), 'config', 'elrs_driver.yaml')
    elrs_config = os.path.join(
        get_package_share_directory('rp1_elrs'), 'config', 'rp1_elrs.yaml')
    joy_elrs_config = os.path.join(
        get_package_share_directory('rp1_elrs'), 'config', 'joy_elrs.yaml')
    control_config = os.path.join(
        get_package_share_directory('rp1_control'), 'config', 'rp1_control.yaml')
    bridge_config = os.path.join(
        get_package_share_directory('rp1_dronecan_bridge'), 'config',
        'rp1_dronecan_bridge.yaml')

    return LaunchDescription([
        Node(
            package='elrs_driver',
            executable='elrs_driver',
            name='elrs_driver',
            parameters=[elrs_driver_config, bringup_config],
            output='screen',
        ),
        Node(
            package='rp1_elrs',
            executable='wheel_feedback_to_battery',
            name='wheel_feedback_to_battery',
            parameters=[elrs_config, bringup_config],
            output='screen',
        ),
        Node(
            package='rp1_teleop',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_elrs_config, bringup_config],
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
