"""rp1_swerve_controller against rp1_swerve.urdf on mock hardware -- the "IK result on screen
before the real robot" tier: no CAN, no Gazebo physics, just ros2_control's mock_components/
GenericSystem looping commands straight back to state, robot_state_publisher computing the
resulting TF from those joint states, and rviz2 showing the actual mesh. Fastest way to see
whether the swerve inverse kinematics (rp1_swerve_controller) points each wheel the right way
and spins it at a sane speed for a given /cmd_vel, before trusting it against Gazebo physics or
real VESCs.

  rviz:=true starts rviz2 (rp1_swerve.rviz -- RobotModel + TF + Odometry, Fixed Frame odom).
  rp1_swerve_controller computes odometry (forward kinematics from wheel/steering *state*, not
  the commands -- on mock hardware that state is just last cycle's command looped back) and
  publishes it on ~/odom plus an odom->base_link TF, the same as diff_drive_controller, so the
  robot does actually drive across the grid here, not just articulate its joints in place.
  keyboard:=true / rqt_steering:=true drive it, same convention as rp1_mvp.launch.py -- both
  publish TwistStamped directly to /rp1_swerve_controller/cmd_vel, no translator node needed.

  can_iface is NOT a launch argument here: mock hardware never opens a CAN socket, so there is
  nothing to point at a virtual bus. Unlike rp1_mvp.launch.py this file has no use_mock:=false
  path yet either -- pointing rp1_swerve.urdf's DroneCAN plugin at real/virtual hardware and
  driving the full 4-actuator ArrayCommand is follow-up work, not done here.

  Publishing ~/mode=1/2/3 (LOCKED_0/LOCKED_90/TWO_WHEEL) here will NOT actually drive the wheels:
  mock hardware's home_0deg/home_90deg gpio state has nothing simulating the proximity sensors,
  so it never reports "confirmed" and rp1_swerve_controller's homing gate correctly holds every
  drive wheel at zero indefinitely, including TWO_WHEEL's free-steering pair (the gate is global
  once any locked corner is unconfirmed, not just the corners that need the reference -- see
  rp1_swerve_controller/README.md's homing-gated locked-mode transitions section). This is the
  gate working as intended, not a bug. Only FULL_SWERVE (mode 0, the default) drives here, since
  it has no fixed reference to confirm.
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rp1_bringup')
    controllers_config = os.path.join(bringup_share, 'config', 'rp1_swerve_controllers.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'rp1_swerve.rviz')

    urdf_path = os.path.join(
        get_package_share_directory('rp1_description'), 'urdf', 'rp1_swerve.urdf')
    with open(urdf_path, 'r') as f:
        description = f.read()

    # Same string-substitution technique as rp1_mvp.launch.py's _robot_description(), simplified:
    # this launch file only ever runs mock, so the swap isn't conditional on a launch argument.
    description, swapped = re.subn(
        r'<plugin>[^<]*</plugin>', '<plugin>mock_components/GenericSystem</plugin>', description,
        count=1)
    if swapped != 1:
        raise RuntimeError(
            f'{urdf_path} has no <plugin> to swap for mock_components/GenericSystem -- refusing '
            'to start rather than silently opening a real CAN device.')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': description}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[controllers_config],
    )

    # Controllers are spawned strictly sequentially (TimerAction delay, then chained via
    # OnProcessExit), matching rp1_mvp.launch.py/steering_bench.launch.py -- concurrent spawner
    # processes were observed to hang controller_manager in this sandboxed environment (no
    # CAP_SYS_NICE for real RT scheduling), see those files' comments for the upstream issue.
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
            'rviz', default_value='false', description='Also start rviz2'),
        DeclareLaunchArgument(
            'keyboard', default_value='false',
            description='Start teleop_twist_keyboard. Needs a real foreground terminal -- see '
                        "rp1_mvp.launch.py's module docstring for why."),
        DeclareLaunchArgument(
            'rqt_steering', default_value='false',
            description='Start rqt_robot_steering (mouse-driven slider GUI) instead.'),

        robot_state_publisher,
        controller_manager,
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
