"""Spawn UAV into an already running Gazebo Sim world."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uav_sim_bringup.world_config import get_uav_pose


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)
    name = LaunchConfiguration('name').perform(context)

    # Get default pose for world if not overridden
    default_pose = get_uav_pose(world)

    spawner_node = Node(
        package='uav_sim_bringup',
        executable='spawn_drone',
        name='uav_spawner',
        output='screen',
        parameters=[{
            'world': world,
            'model': model,
            'name': name,
            'wait_for_world': True,
        }],
    )

    return [spawner_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='vo_ground',
                              description='Gazebo world name (vo_ground, drdo_world1, drdo_world2, etc.)'),
        DeclareLaunchArgument('model', default_value='x500_depth_down',
                              description='UAV model name'),
        DeclareLaunchArgument('name', default_value='x500_depth_down_0',
                              description='UAV instance name in world'),
        OpaqueFunction(function=launch_setup),
    ])
