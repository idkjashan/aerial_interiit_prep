"""Bring up the full GPS-denied vision stack.

Assumes MicroXRCEAgent, PX4 SITL and the ros_gz_bridge are already running -- see
tools/run_sim.sh, which starts those in the right order.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('uav_vision'), 'config', 'vo_params.yaml')
    args = [
        DeclareLaunchArgument('params', default_value=cfg),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('run_mission', default_value='true'),
    ]
    params = LaunchConfiguration('params')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription(args + [
        Node(package='uav_vision', executable='vo_node', name='uav_visual_odometry',
             output='screen', parameters=[params, sim_time]),
        Node(package='uav_vision', executable='px4_odometry_bridge',
             name='uav_px4_odometry_bridge', output='screen',
             parameters=[params, sim_time]),
        Node(package='uav_vision', executable='vision_health_node',
             name='uav_vision_health', output='screen', parameters=[params, sim_time]),
        Node(package='uav_vision', executable='offboard_mission_node',
             name='uav_offboard_mission', output='screen', parameters=[params, sim_time],
             condition=IfCondition(LaunchConfiguration('run_mission'))),
    ])
