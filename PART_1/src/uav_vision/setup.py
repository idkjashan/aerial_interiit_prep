from glob import glob
import os
from setuptools import setup

package_name = 'uav_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jashan',
    maintainer_email='hello@pathnovo.com',
    description='GPS-denied visual odometry and OFFBOARD control for PX4.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vo_node = uav_vision.vo_node:main',
            'px4_odometry_bridge = uav_vision.px4_odometry_bridge:main',
            'vision_health_node = uav_vision.vision_health_node:main',
            'offboard_mission_node = uav_vision.offboard_mission_node:main',
            'manual_control = uav_vision.manual_control:main',
            'monitor = uav_vision.monitor:main',
            'vision_cut = uav_vision.vision_cut:main',
        ],
    },
)
