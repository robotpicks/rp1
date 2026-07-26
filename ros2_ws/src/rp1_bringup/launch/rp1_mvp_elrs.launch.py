"""rp1 MVP teleop pipeline driven by an ExpressLRS radio instead of the Xbox pad:
elrs_driver -> /joy -> rp1_teleop -> diff_drive_controller -> rp1_hardware_interface -> DroneCAN.

Identical to rp1_mvp.launch.py except the /joy source: there is no joy_node here; the generic
elrs_driver (elrs_ros submodule) reads the CRSF link and publishes /joy itself. Telemetry back to
the handset is the second node: rp1_elrs's esc_telemetry_to_battery adapter turns the hardware
component's /dynamic_joint_states into the /battery that elrs_driver forwards to the RX. Each node
loads its package's own default config first, then rp1_bringup's rp1_mvp.yaml on top for the
robot-specific values (serial port, CAN interface, deadman button).

Do not also start joy_node against this pipeline -- two publishers on /joy would fight.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rp1_bringup')
    bringup_config = os.path.join(bringup_share, 'config', 'rp1_mvp.yaml')
    controllers_config = os.path.join(bringup_share, 'config', 'rp1_controllers.yaml')

    elrs_driver_config = os.path.join(
        get_package_share_directory('elrs_driver'), 'config', 'elrs_driver.yaml')
    elrs_config = os.path.join(
        get_package_share_directory('rp1_elrs'), 'config', 'rp1_elrs.yaml')
    joy_elrs_config = os.path.join(
        get_package_share_directory('rp1_elrs'), 'config', 'joy_elrs.yaml')
    urdf_path = os.path.join(
        get_package_share_directory('rp1_hardware_interface'), 'urdf', 'rp1_drive.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # --param-file is required: a controller node does not inherit the params file given to
    # ros2_control_node. See the longer note in rp1_mvp.launch.py.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--param-file', controllers_config],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller_spawner',
        output='screen',
        arguments=['diff_drive_controller', '--param-file', controllers_config],
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            # No robot_description parameter: controller_manager reads it off the
            # /robot_description topic that robot_state_publisher latches above. See the note in
            # rp1_mvp.launch.py.
            parameters=[controllers_config],
            output='screen',
        ),
        Node(
            package='elrs_driver',
            executable='elrs_driver',
            name='elrs_driver',
            parameters=[elrs_driver_config, bringup_config],
            output='screen',
        ),
        Node(
            package='rp1_elrs',
            executable='esc_telemetry_to_battery',
            name='esc_telemetry_to_battery',
            parameters=[elrs_config, bringup_config],
            output='screen',
        ),
        Node(
            package='rp1_teleop',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_elrs_config, bringup_config],
            remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel')],
            output='screen',
        ),

        # Sequential spawning, same controller_manager service/update-loop contention as
        # rp1_mvp.launch.py -- see the comment there.
        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        ),
    ])
