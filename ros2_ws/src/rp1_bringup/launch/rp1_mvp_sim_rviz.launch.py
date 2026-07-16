"""rp1_mvp_sim.launch.py plus rviz2, pre-configured to watch /odom and TF move.

No physical hardware and no Xbox controller required -- drive it by publishing to /cmd_vel
directly (see the top-level README's "Run without hardware" section) while watching the robot
move in rviz2.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rp1_bringup')
    sim_launch = os.path.join(bringup_share, 'launch', 'rp1_mvp_sim.launch.py')
    rviz_config = os.path.join(
        get_package_share_directory('rp1_sim'), 'rviz', 'rp1_sim.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch)),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
