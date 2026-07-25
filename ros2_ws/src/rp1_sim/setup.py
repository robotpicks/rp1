import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rp1_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uzi.mor',
    maintainer_email='uzi.mor@gmail.com',
    description=(
        'Pure-ROS2 stand-in for rp1_dronecan_bridge: simulates wheel dynamics and odometry '
        'so the teleop/control graph can be exercised with zero CAN hardware.'
    ),
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sim_bridge_node = rp1_sim.sim_bridge_node:main',
            'swerve_sim_node = rp1_sim.swerve_sim_node:main',
        ],
    },
)
