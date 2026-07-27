"""Full rp1 MVP teleop pipeline (joy -> rp1_teleop -> diff_drive_controller) driven through
Gazebo Sim physics instead of real hardware or ros2_control's mock_components.

Unlike rp1_mvp.launch.py, there is no separate ros2_control_node here: gz_ros2_control is a
Gazebo SYSTEM plugin (the <gazebo> block injected below), and it instantiates its own
controller_manager INSIDE the gz sim process once the spawned model loads. The same
`ros2 run controller_manager spawner` calls used elsewhere reach that internal instance over
the same services -- nothing about spawning controllers changes.

collision geometry: rp1_drive.urdf's <collision> tags are box/cylinder primitives sized from
the CAD, not the detailed visual meshes (see that file's header) -- the ~40MB concave chassis
mesh would be a slow/unstable physics collision shape. Don't swap that back without profiling.

Requires ros-lyrical-gz-ros2-control (not part of a stock ROS2 desktop install; installed via
apt in this sandbox, not rosdep -- see rp1_bringup/package.xml).

keyboard:=true starts teleop_twist_keyboard instead of joy_node/rp1_teleop, remapped straight to
/diff_drive_controller/cmd_vel (it publishes TwistStamped itself via its own stamped:=true param,
so it doesn't need rp1_teleop as a translator the way a joystick's Joy messages do). Pass
teleop:=false alongside it -- both would otherwise publish to the same topic. This MUST be run
from a real interactive terminal you're typing into directly: it reads raw keypresses via
termios, which needs an actual foreground tty. Launching it from an automation/background
context (a CI job, a detached/backgrounded `ros2 launch`, an assistant's tool call) gives it no
controlling tty, and it crashes immediately with `termios.error: (25, 'Inappropriate ioctl for
device')` -- confirmed 2026-07-28, not a silent no-op as you might expect.

Also confirmed 2026-07-28 (a real end-to-end test, not just launch-file inspection): a
teleop_twist_keyboard instance with a real controlling tty does drive the simulated robot
correctly through this launch file (odometry moved under simulated keypresses). If keyboard
input seems to do nothing, the most common cause isn't this wiring -- it's OS window focus:
teleop_twist_keyboard reads from its own terminal's stdin, not from whatever window has focus,
so clicking the Gazebo window and typing there sends keystrokes to Gazebo, not to the teleop
node. Click the terminal actually running `ros2 launch` instead.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_description(context, controllers_config):
    """Read the URDF, swap its ros2_control plugin for Gazebo's, and inject the <gazebo>
    system-plugin block that tells gz_ros2_control to host a controller_manager for it.

    Same string-substitution approach rp1_mvp.launch.py uses for its use_mock swap, for the
    same reason: ros2_control reads hardware parameters from the <hardware> block of whatever
    description controller_manager actually loaded, not from a node parameter, so there is no
    other way to reach this.
    """
    urdf_path = os.path.join(
        get_package_share_directory('rp1_description'), 'urdf', 'rp1_drive.urdf')
    with open(urdf_path, 'r') as f:
        description = f.read()

    description, swapped = re.subn(
        r'<plugin>[^<]*</plugin>',
        '<plugin>gz_ros2_control/GazeboSimSystem</plugin>', description, count=1)
    if swapped != 1:
        raise RuntimeError(
            f'{urdf_path} has no <plugin> to swap for gz_ros2_control/GazeboSimSystem -- '
            'refusing to start, since that would leave the real DroneCAN plugin loaded.')

    gazebo_block = (
        '\n  <gazebo>\n'
        '    <plugin filename="gz_ros2_control-system" '
        'name="gz_ros2_control::GazeboSimROS2ControlPlugin">\n'
        f'      <parameters>{controllers_config}</parameters>\n'
        '    </plugin>\n'
        '  </gazebo>\n'
    )
    description, inserted = re.subn(r'</robot>', gazebo_block + '</robot>', description, count=1)
    if inserted != 1:
        raise RuntimeError(
            f'{urdf_path} has no </robot> closing tag to inject the gazebo plugin before')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': description, 'use_sim_time': True}],
        ),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('rp1_bringup')
    controllers_config = os.path.join(bringup_share, 'config', 'rp1_controllers.yaml')
    bringup_config = os.path.join(bringup_share, 'config', 'rp1_mvp.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'rp1_bringup.rviz')
    teleop_config = os.path.join(
        get_package_share_directory('rp1_teleop'), 'config', 'joy_xbox_series_x.yaml')

    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    # The URDF's <mesh filename="package://rp1_description/meshes/..."> becomes a
    # "model://rp1_description/meshes/..." URI once urdf2sdf converts it -- gz-sim resolves
    # that by looking for a "rp1_description" directory under each GZ_SIM_RESOURCE_PATH entry,
    # which is exactly one level above get_package_share_directory('rp1_description')
    # (.../share/rp1_description/meshes/...). Without this, physics/control still work (the
    # errors are tagged [GUI]) but the GUI can't find any of the meshes to render.
    rp1_description_share = get_package_share_directory('rp1_description')
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(rp1_description_share))

    # --param-file is required, not decorative -- see rp1_mvp.launch.py's comment: a spawner
    # doesn't inherit the params file given to whatever controller_manager it's talking to.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--param-file', controllers_config],
        parameters=[{'use_sim_time': True}],
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller_spawner',
        output='screen',
        arguments=['diff_drive_controller', '--param-file', controllers_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false', description='Also start rviz2'),
        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='Start joy_node + teleop_node. false leaves '
                        '/diff_drive_controller/cmd_vel free for another publisher'),
        DeclareLaunchArgument(
            'keyboard', default_value='false',
            description='Start teleop_twist_keyboard instead. Pass teleop:=false alongside '
                        'this -- both publish to the same topic. Needs a real foreground '
                        'terminal (see module docstring); does nothing if backgrounded.'),
        DeclareLaunchArgument(
            'world', default_value='empty.sdf', description='Gazebo world SDF to load'),

        set_gz_resource_path,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                'gz_args': [LaunchConfiguration('world'), ' -r'],
            }.items(),
        ),

        # Bridges Gazebo's simulated clock onto ROS's /clock -- controller_manager and
        # robot_state_publisher both run with use_sim_time:=true above, and without this
        # bridge they'd have no sim time source to read.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),

        OpaqueFunction(function=lambda context: _robot_description(context, controllers_config)),

        # Spawns from the /robot_description topic robot_state_publisher just latched, so this
        # depends on that node existing -- OpaqueFunction above returns before this runs, but
        # launch doesn't guarantee robot_state_publisher is up yet; ros_gz_sim's create waits on
        # the topic itself, so this isn't a race in practice. z=0.05 lifts it slightly clear of
        # the ground plane at spawn (base_link's own z=0 is exactly the wheels' contact patch).
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', 'robot_description', '-name', 'rp1', '-z', '0.05'],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
        Node(
            package='rp1_teleop',
            executable='teleop_node',
            name='teleop_node',
            parameters=[teleop_config, bringup_config, {'use_sim_time': True}],
            remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True,
            parameters=[{'stamped': True, 'frame_id': 'base_link'}],
            remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel')],
            condition=IfCondition(LaunchConfiguration('keyboard')),
        ),

        # Same rationale as rp1_mvp.launch.py: spawn joint_state_broadcaster first and only
        # start diff_drive_controller once it exits, rather than both at once. The delay here
        # is longer (8s vs 5s) to give Gazebo time to actually load the model and gz_ros2_control
        # time to bring its internal controller_manager up before the first spawner call hits it.
        TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        ),
    ])
