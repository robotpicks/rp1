import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rp1_dronecan_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uzi.mor',
    maintainer_email='uzi.mor@gmail.com',
    description='ROS2 <-> DroneCAN bridge for rp1.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = rp1_dronecan_bridge.bridge_node:main',
        ],
    },
)
