"""rp1_swerve_controller against rp1_swerve.urdf on the REAL vesc_dronecan_driver/
VescDroneCanSystem plugin -- actual DroneCAN wire framing over SocketCAN, not
mock_components/GenericSystem (see rp1_swerve_mock.launch.py for that tier). No physics; this
is the "does the real hardware path actually carry correct traffic" tier, one step before real
VESCs or Gazebo.

  can_iface (default can0) overrides rp1_swerve.urdf's <param name="can_iface"> the same way
  rp1_mvp.launch.py's can_iface does for rp1_drive.urdf -- point it at vcan0 plus
  simulation/sim_vesc_node.py (drive) and simulation/sim_actuator_node.py --actuator-ids 4,5,6,7
  (steering) to exercise this against software VESC/actuator stand-ins with zero physical
  hardware, same technique as rp1/simulation/README.md's existing drive-only and steering-bench
  loopback checks, just against the merged 4-drive+4-steering description and
  rp1_swerve_controller instead of diff_drive_controller/a raw position controller.

  Only esc.RPMCommand/Status (drive) and actuator.ArrayCommand/Status position control (steering)
  are exercised here -- the home_0deg/home_90deg status bits and COMMAND_TYPE_HOME (see the bldc
  repo's swerve branch) are a private wire-format extension the generic `dronecan` Python
  library's stock decoder doesn't know to populate, so seek_home/home_* aren't verified by this
  launch file or the simulator scripts; only a real bldc-flashed VESC (or a simulator taught the
  extension) would exercise those.

  rviz:=true / keyboard:=true / rqt_steering:=true match rp1_swerve_mock.launch.py.

  Verified against `can_iface:=vcan0` plus both simulator scripts, for both drive and steering:
  driving straight produced real, nonzero drive joint velocity/position in `/joint_states`, and
  a turn-in-place command produced steering joint positions matching the hand-computed
  full-swerve angles exactly -- both fed back over actual DroneCAN framing (not just commands
  looping back, like the mock tier). Getting this far required three bug fixes, all on the
  `vesc_dronecan_ros` swerve branch except the first: `dronecan`'s driver-dispatch bug (see the
  simulator scripts' `_force_native_socketcan_driver()`); `pumpRx()` passing a hardcoded
  `timestamp_usec=0` to libcanard, which broke multi-frame transfer reassembly for both
  esc.Status and actuator.Status; and actuator.Status's decoder hard-failing its *entire* decode
  (not just the 2 extension fields) whenever the home_0deg/home_90deg private-extension bits
  were absent -- which they always are from a standard, non-extended sender like
  sim_actuator_node.py. See that repo's swerve-branch history for each fix.
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_description(context):
    """Read the URDF and apply the can_iface launch argument to it.

    Done in an OpaqueFunction, matching rp1_mvp.launch.py's _robot_description(): the CAN
    interface is a <hardware> block param in the URDF text, not a node parameter, so it can only
    be overridden by editing the text, and the value is only known at launch time.
    """
    urdf_path = os.path.join(
        get_package_share_directory('rp1_description'), 'urdf', 'rp1_swerve.urdf')
    with open(urdf_path, 'r') as f:
        description = f.read()

    can_iface = LaunchConfiguration('can_iface').perform(context)
    description, substitutions = re.subn(
        r'(<param name="can_iface">)[^<]*(</param>)',
        lambda m: m.group(1) + can_iface + m.group(2), description, count=1)
    if substitutions != 1:
        raise RuntimeError(
            f'{urdf_path} has no <param name="can_iface"> to override -- the launch argument '
            'would be silently ignored, so refusing to start.')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': description}],
        ),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('rp1_bringup')
    controllers_config = os.path.join(bringup_share, 'config', 'rp1_swerve_controllers.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'rp1_swerve.rviz')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--param-file', controllers_config],
    )

    rp1_swerve_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='rp1_swerve_controller_spawner',
        output='screen',
        arguments=['rp1_swerve_controller', '--param-file', controllers_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_iface', default_value='can0',
            description="SocketCAN interface for the hardware component (e.g. vcan0 for a "
                        "virtual bus); overrides the URDF's <param name=\"can_iface\">"),
        DeclareLaunchArgument(
            'rviz', default_value='false', description='Also start rviz2'),
        DeclareLaunchArgument(
            'keyboard', default_value='false',
            description='Start teleop_twist_keyboard. Needs a real foreground terminal -- see '
                        "rp1_mvp.launch.py's module docstring for why."),
        DeclareLaunchArgument(
            'rqt_steering', default_value='false',
            description='Start rqt_robot_steering (mouse-driven slider GUI) instead.'),

        OpaqueFunction(function=_robot_description),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[controllers_config],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True,
            parameters=[{'stamped': True, 'frame_id': 'base_link'}],
            remappings=[('cmd_vel', '/rp1_swerve_controller/cmd_vel')],
            condition=IfCondition(LaunchConfiguration('keyboard')),
        ),
        Node(
            package='rqt_robot_steering',
            executable='rqt_robot_steering',
            name='rqt_robot_steering',
            output='screen',
            parameters=[{
                'default_topic': '/rp1_swerve_controller/cmd_vel',
                'default_stamped': True,
            }],
            condition=IfCondition(LaunchConfiguration('rqt_steering')),
        ),

        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rp1_swerve_controller_spawner],
            )
        ),
    ])
