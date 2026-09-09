import os
from glob import glob
from setuptools import setup

package_name = 'uav_sim_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aerial Inter-IIT Team',
    maintainer_email='prep@interiit.org',
    description='Simulation bringup, world coordination, and UAV spawner for GPS-denied visual odometry',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_drone = uav_sim_bringup.spawner:main',
            'sim_manager = uav_sim_bringup.sim_manager:main',
        ],
    },
)
