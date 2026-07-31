"""rp1_swerve_controller against rp1_swerve.urdf on real Gazebo (gz sim) physics --
gz_ros2_control's GazeboSimSystem instead of vesc_dronecan_driver/VescDroneCanSystem
(rp1_swerve_dronecan.launch.py) or mock_components/GenericSystem (rp1_swerve_mock.launch.py).
This is the first tier where drive/steering feedback comes from an actual simulated rigid-body
dynamics solver (inertia, gravity, contact) rather than commands looping straight back to state
or a real DroneCAN round-trip with no physics at all.

  GazeboSimSystem only backs STANDARD joint interfaces (position/velocity/effort) with a real
  Gazebo entity -- it has no physical concept of the seek_home/brake command interfaces or the
  home_0deg/home_90deg/ESC-telemetry <gpio> blocks rp1_swerve.urdf also declares (those exist for
  vesc_dronecan_driver's benefit and have no Gazebo-entity equivalent to back them with).
  rp1_swerve_controller's own resource manager refuses to activate a controller that DECLARES an
  interface no hardware component actually exports, so this can't be requested-then-tolerated-
  if-missing -- rp1_swerve_gazebo_overrides.yaml (layered on top of rp1_swerve_controllers.yaml
  via a second --param-file below) sets steering_home_sensors_available/steering_brake_available
  to false, telling the controller not to request either at all here (see its README).
  Practically this means home_0deg/home_90deg are permanently unconfirmed here, so LOCKED_0/
  LOCKED_90/TWO_WHEEL are gated at zero drive forever -- the same situation
  rp1_swerve_mock.launch.py's docstring already describes for the mock tier, for the same reason
  (nothing simulates the proximity sensors). Only FULL_SWERVE (mode 0, the default) drives here.

  headless (default true) runs gz sim server-only (`-s`), no GUI -- this sandbox has no display
  and gz sim's rendering stack needs one even for a mostly-invisible run. Set headless:=false on
  a real desktop to watch the robot in the Gazebo GUI.

  rviz:=true / keyboard:=true / rqt_steering:=true match rp1_swerve_mock.launch.py.
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_description(context):
    """Read the URDF and swap in the Gazebo hardware plugin + controller params path.

    Same OpaqueFunction/text-substitution technique rp1_swerve_dronecan.launch.py's
    _robot_description() uses for can_iface: the URDF is plain text, not xacro, so a
    launch-time-only value (a package share path, in this case) can only be filled in by editing
    the text after reading it, not via a URDF-native substitution.
    """
    urdf_path = os.path.join(
        get_package_share_directory('rp1_description'), 'urdf', 'rp1_swerve.urdf')
    with open(urdf_path, 'r') as f:
        description = f.read()

    description, plugin_subs = re.subn(
        r'<plugin>vesc_dronecan_driver/VescDroneCanSystem</plugin>',
        '<plugin>gz_ros2_control/GazeboSimSystem</plugin>', description, count=1)
    if plugin_subs != 1:
        raise RuntimeError(
            f'{urdf_path} has no vesc_dronecan_driver/VescDroneCanSystem <plugin> to swap for '
            'gz_ros2_control/GazeboSimSystem -- refusing to start with the wrong hardware '
            'plugin.')

    controllers_config = os.path.join(
        get_package_share_directory('rp1_bringup'), 'config', 'rp1_swerve_controllers.yaml')
    description, params_subs = re.subn(
        r'PARAMS_FILE_PLACEHOLDER', controllers_config, description, count=1)
    if params_subs != 1:
        raise RuntimeError(
            f'{urdf_path} has no PARAMS_FILE_PLACEHOLDER for the gz_ros2_control <parameters> '
            'path -- refusing to start with an unresolved plugin config.')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': description, 'use_sim_time': True}],
        ),
    ]


def _start_gz_sim(context):
    """Start gz sim, headless (-s, server-only) or not per the headless launch argument.

    An OpaqueFunction because gz_args needs to branch on a launch argument's value, which is
    only known at launch time, not a plain LaunchDescription substitution.
    """
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    gz_args = '-s -r -v 3 empty.sdf' if headless else '-r -v 3 empty.sdf'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items(),
        ),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('rp1_bringup')
    rviz_config = os.path.join(bringup_share, 'rviz', 'rp1_swerve.rviz')
    # gz_ros2_control's plugin already reads this file for controller_manager's own
    # ros__parameters (the joint_state_broadcaster/rp1_swerve_controller type declarations --
    # confirmed working without this), but a controller node never inherits the file its
    # controller_manager was given, same gotcha rp1_mvp.launch.py's docstring documents for the
    # other two tiers: without --param-file here, rp1_swerve_controller's own drive_joints/
    # steering_joints/etc. parameters are simply never declared, and on_init() sees empty lists.
    controllers_config = os.path.join(bringup_share, 'config', 'rp1_swerve_controllers.yaml')
    # Tells rp1_swerve_controller not to request seek_home/home_0deg/home_90deg/brake at all on
    # this tier -- see the URDF comment above the <gazebo> plugin tag and this file's own module
    # docstring for why GazeboSimSystem can't back them with anything.
    gazebo_overrides_config = os.path.join(
        bringup_share, 'config', 'rp1_swerve_gazebo_overrides.yaml')

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_rp1',
        output='screen',
        parameters=[{'topic': '/robot_description', 'name': 'rp1'}],
    )

    # gz sim publishes sim time on /clock (gz.msgs.Clock); this bridges it to ROS so
    # robot_state_publisher/controller_manager/spawners all agree on the simulated clock instead
    # of the wall clock racing ahead of physics.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['joint_state_broadcaster', '--param-file', controllers_config],
    )

    rp1_swerve_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='rp1_swerve_controller_spawner',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            'rp1_swerve_controller', '--param-file', controllers_config, '--param-file',
            gazebo_overrides_config,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run gz sim server-only (-s), no GUI -- this sandbox has no display; '
                        'set false on a real desktop to watch the robot in the Gazebo GUI.'),
        DeclareLaunchArgument(
            'rviz', default_value='false', description='Also start rviz2'),
        DeclareLaunchArgument(
            'keyboard', default_value='false',
            description='Start teleop_twist_keyboard. Needs a real foreground terminal -- see '
                        "rp1_mvp.launch.py's module docstring for why."),
        DeclareLaunchArgument(
            'rqt_steering', default_value='false',
            description='Start rqt_robot_steering (mouse-driven slider GUI) instead.'),

        # The URDF's <mesh filename="package://rp1_description/meshes/..."> becomes a
        # "model://rp1_description/meshes/..." URI once urdf2sdf converts it -- gz-sim resolves
        # that by looking for a "rp1_description" directory under each GZ_SIM_RESOURCE_PATH
        # entry, one level above get_package_share_directory('rp1_description'). Without this,
        # physics/control still work (only the GUI can't find meshes to render) -- matches
        # rp1_gazebo.launch.py's identical fix for rp1_drive.urdf.
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(get_package_share_directory('rp1_description'))),

        OpaqueFunction(function=_start_gz_sim),

        clock_bridge,
        OpaqueFunction(function=_robot_description),
        spawn_entity,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}],
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

        # gz_ros2_control's plugin only finishes registering the controller_manager services
        # once the entity has actually spawned and the plugin has initialized -- give it a real
        # head start before the first spawner tries to talk to it, same reasoning as the other
        # two tiers' TimerAction delay.
        TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rp1_swerve_controller_spawner],
            )
        ),
    ])
