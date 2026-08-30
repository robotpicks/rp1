"""Full rp1 MVP teleop pipeline on ros2_control: joy -> rp1_teleop -> diff_drive_controller ->
vesc_dronecan_driver -> DroneCAN -> the 4 drive VESCs.

This replaced the old rp1_teleop -> rp1_control -> rp1_dronecan_bridge topic chain. The
skid-steer kinematics, the /cmd_vel watchdog and the odometry are diff_drive_controller's now,
and the DroneCAN encoding moved into the hardware component's read()/write() -- see
docs/can_id_map.md. DroneCAN itself is unchanged; only the process that speaks it moved.

  use_mock:=true swaps the real VescDroneCanSystem plugin for ros2_control's mock_components/
  GenericSystem, which loops commands straight back to states. That is the no-hardware,
  no-CAN way to exercise the whole controller stack (it replaced rp1_sim's sim_bridge_node).
  rviz:=true additionally starts rviz2.
  can_iface:=vcan0 points the hardware component at a different SocketCAN interface -- what
  `robotpicks.sh smoke dronecan` uses to run this whole pipeline against a virtual bus.
  teleop:=false drops joy_node and teleop_node, leaving /diff_drive_controller/cmd_vel free for
  something else to drive (the smoke check, a nav stack, `ros2 topic pub`).
  keyboard:=true starts teleop_twist_keyboard instead (pass teleop:=false alongside it -- both
  publish to the same topic). It publishes TwistStamped itself via its own stamped:=true param,
  so it doesn't need rp1_teleop as a translator the way a joystick's Joy messages do. This MUST
  be run from a real interactive terminal you're typing into directly -- it reads raw keypresses
  via termios, which needs an actual foreground tty; a detached/backgrounded `ros2 launch` has no
  controlling tty and it crashes immediately with `termios.error: (25, 'Inappropriate ioctl for
  device')` -- confirmed 2026-07-28, not a silent no-op as you might expect.

  Also confirmed 2026-07-28 (an end-to-end test, not just launch-file inspection): a
  teleop_twist_keyboard instance with a real controlling tty does drive the robot correctly
  through this launch file (with use_mock:=true, odometry moved under simulated keypresses). If
  keyboard input seems to do nothing, the most common cause isn't this wiring -- it's OS window
  focus: teleop_twist_keyboard reads from its own terminal's stdin, not from whatever window has
  focus, so clicking some other window (e.g. rviz2, if rviz:=true) and typing there sends
  keystrokes there, not to the teleop node. Click the terminal actually running `ros2 launch`.
  rqt_steering:=true starts rqt_robot_steering (mouse-driven slider GUI) instead -- pass
  teleop:=false alongside it. Its own window, not a panel inside rviz. Natively supports
  TwistStamped (default_stamped param, set true below), so it needs no translator either.

  imu:=true / gps:=true / compass:=true start the second-wave sensor drivers (rp1_imu's
  ICM20948-over-SPI node, rp1_gps's ublox_gps-based NEO-F10N-over-UART1 node, rp1_compass's
  IST8310-over-I2C node) -- all three default off since none of this hardware is wired up yet as
  of 2026-08-05. All three share one physical Waveshare USB-to-UART/I2C/SPI/JTAG converter,
  confirmed running in Mode 1 (UART1 + I2C + SPI, UART0 disabled) and identified on the bench via
  `lsusb` as a WCH/QinHeng CH34x/CH347-family chipset (1a86:55db). ELRS does NOT share this
  converter -- confirmed 2026-08-05 to be its own separate FTDI USB device instead (see
  rp1_mvp.yaml's elrs_driver.serial_port comment), correcting the earlier assumption that GPS,
  IMU, and ELRS all shared one converter. None of the three sensor nodes' dry-run stories are as
  solid as the drive path's: the IMU/compass drivers are UNVERIFIED against real hardware (see
  icm20948_driver_node.py's and ist8310_driver_node.py's module docstrings), and ublox_gps has no
  require_serial-style escape hatch at all, so gps:=true against a nonexistent device crashes the
  node rather than dry-running it. See rp1_imu/config/rp1_imu.yaml, rp1_gps/config/rp1_gps.yaml,
  and rp1_compass/config/rp1_compass.yaml for the SPI/I2C bus/device and serial port/baud
  placeholders that still need confirming against the real Waveshare wiring.

controller_manager in this ROS 2 release takes the robot description from the /robot_description
TOPIC, not from a parameter -- it logs "Waiting for data on 'robot_description' topic" and
blocks until robot_state_publisher latches it. That is why the description is built once here
and handed only to robot_state_publisher: passing a different one to controller_manager as a
parameter would be silently ignored, which with use_mock:=true would mean quietly loading the
real DroneCAN plugin and opening can0.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_description(context):
    """Read the URDF and apply the use_mock / can_iface launch arguments to it.

    Done in an OpaqueFunction because both edits depend on launch arguments whose values are
    only known at launch time, and every node below must agree on one description.

    Editing the URDF text is the only way to reach these: ros2_control takes hardware parameters
    from the <hardware> block of the robot description and nowhere else, so neither the plugin
    name nor the CAN interface can be overridden as a node parameter.
    """
    urdf_path = os.path.join(
        get_package_share_directory('rp1_description'), 'urdf', 'rp1_drive.urdf')
    with open(urdf_path, 'r') as f:
        description = f.read()

    if LaunchConfiguration('use_mock').perform(context).lower() in ('true', '1'):
        # Swapping the plugin by string substitution keeps a single URDF as the one description
        # of the robot; a xacro arg would pull xacro into the runtime path for one line.
        #
        # Matched by <plugin> tag rather than by the plugin's literal name, and checked: a bare
        # .replace() that silently matches nothing would leave the REAL DroneCAN plugin loaded
        # under use_mock:=true and open the CAN device -- the exact failure this argument exists
        # to prevent, and one that renaming the plugin would otherwise cause. Fail loudly instead.
        description, swapped = re.subn(
            r'<plugin>[^<]*</plugin>',
            '<plugin>mock_components/GenericSystem</plugin>', description, count=1)
        if swapped != 1:
            raise RuntimeError(
                f'{urdf_path} has no <plugin> to swap for mock_components/GenericSystem -- '
                'use_mock:=true would silently run the real hardware, so refusing to start.')

    can_iface = LaunchConfiguration('can_iface').perform(context)
    # Matched by name rather than by the literal default so this keeps working if the URDF's
    # can0 ever changes. count=1: only the <hardware> block declares it.
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
    bringup_config = os.path.join(bringup_share, 'config', 'rp1_mvp.yaml')
    controllers_config = os.path.join(bringup_share, 'config', 'rp1_controllers.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'rp1_bringup.rviz')

    teleop_config = os.path.join(
        get_package_share_directory('rp1_teleop'), 'config', 'joy_xbox_series_x.yaml')

    # --param-file is required, not decorative: a controller node does NOT inherit the params
    # file passed to ros2_control_node. Without it diff_drive_controller declares wheel_separation
    # at its 0.0 default, which violates that parameter's own "> 0" range, and the controller
    # fails to load with "doesn't comply with floating point range".
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

    esc_telemetry_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='esc_telemetry_broadcaster_spawner',
        output='screen',
        arguments=['esc_telemetry_broadcaster', '--param-file', controllers_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock', default_value='false',
            description='Use mock_components/GenericSystem instead of the DroneCAN hardware'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also start rviz2 (odom/TF from diff_drive_controller)'),
        DeclareLaunchArgument(
            'can_iface', default_value='can0',
            description="SocketCAN interface for the hardware component (e.g. vcan0 for a "
                        "virtual bus); overrides the URDF's <param name=\"can_iface\">"),
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
            'rqt_steering', default_value='false',
            description='Start rqt_robot_steering (mouse-driven slider GUI) instead. Pass '
                        'teleop:=false alongside this -- both publish to the same topic.'),
        DeclareLaunchArgument(
            'imu', default_value='false',
            description='Start the ICM20948 IMU driver (rp1_imu). Unverified against real '
                        'hardware -- see module docstring.'),
        DeclareLaunchArgument(
            'gps', default_value='false',
            description='Start the u-blox NEO-F10N GPS driver (rp1_gps). No dry-run mode -- '
                        'crashes if the configured device does not exist.'),
        DeclareLaunchArgument(
            'compass', default_value='false',
            description="Start the IST8310 compass driver (rp1_compass) -- the GPS module's "
                        'onboard magnetometer, separate from the GPS fix itself. Unverified '
                        'against real hardware -- see module docstring.'),

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
            parameters=[teleop_config, bringup_config],
            # diff_drive_controller subscribes on its own namespaced topic, and takes
            # TwistStamped -- see the note in teleop_node.py.
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
        Node(
            package='rqt_robot_steering',
            executable='rqt_robot_steering',
            name='rqt_robot_steering',
            output='screen',
            parameters=[{
                'default_topic': '/diff_drive_controller/cmd_vel',
                'default_stamped': True,
            }],
            condition=IfCondition(LaunchConfiguration('rqt_steering')),
        ),
        Node(
            package='rp1_imu',
            executable='icm20948_driver',
            name='icm20948_driver',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('rp1_imu'), 'config', 'rp1_imu.yaml')],
            condition=IfCondition(LaunchConfiguration('imu')),
        ),
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('rp1_gps'), 'config', 'rp1_gps.yaml')],
            condition=IfCondition(LaunchConfiguration('gps')),
        ),
        Node(
            package='rp1_compass',
            executable='ist8310_driver',
            name='ist8310_driver',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('rp1_compass'), 'config', 'rp1_compass.yaml')],
            condition=IfCondition(LaunchConfiguration('compass')),
        ),

        # Controllers are spawned strictly sequentially (TimerAction delay, then chained via
        # OnProcessExit) rather than concurrently -- controller_manager's service handling runs
        # on the same thread as its real-time update loop (see ros-controls/ros2_control#2808),
        # and two spawner processes hitting its services at once was observed to hang
        # controller_manager entirely in this sandboxed environment (no CAP_SYS_NICE for real RT
        # scheduling). One spawner at a time avoids that contention.
        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=diff_drive_controller_spawner,
                on_exit=[esc_telemetry_broadcaster_spawner],
            )
        ),
    ])
