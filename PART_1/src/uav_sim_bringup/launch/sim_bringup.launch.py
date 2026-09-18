"""Bring up Gazebo, PX4 SITL, the DDS agent, the camera bridge and the vision stack.

    ros2 launch uav_sim_bringup sim_bringup.launch.py world:=vo_ground

PX4 is looked up in $PX4_DIR (default ~/PX4-Autopilot). Worlds are searched in PX4's
worlds folder and then in every directory on $GZ_SIM_RESOURCE_PATH, so extra world
packages only need to be added to that variable.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from uav_sim_bringup.world_config import get_uav_pose_str

PX4_DIR = os.environ.get('PX4_DIR', os.path.expanduser('~/PX4-Autopilot'))
PX4_GZ = os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz')


def find_world_sdf(world_name):
    dirs = [os.path.join(PX4_GZ, 'worlds')]
    dirs += [d for d in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':') if d]
    for d in dirs:
        for candidate in (os.path.join(d, f'{world_name}.sdf'),
                          os.path.join(d, 'worlds', f'{world_name}.sdf')):
            if os.path.isfile(candidate):
                return candidate
    return os.path.join(PX4_GZ, 'worlds', f'{world_name}.sdf')   # let gz report it


def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    run_mission = LaunchConfiguration('run_mission').perform(context).lower() == 'true'
    instance = LaunchConfiguration('instance').perform(context)

    pkg_bringup = get_package_share_directory('uav_sim_bringup')
    pkg_vision = get_package_share_directory('uav_vision')

    world_sdf = find_world_sdf(world_name)
    # instance 1 uses 8890 so the default 8888 stays free for another session
    agent_port = str(8890 if instance == '1' else (8888 + int(instance)))
    model_name = f'x500_depth_down_{instance}'

    gz_args = ['-r', '-s', world_sdf] if headless else ['-r', world_sdf]
    gz_process = ExecuteProcess(cmd=['gz', 'sim'] + gz_args, output='screen', name='gz_sim')

    agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', agent_port],
        output='screen',
        name='micro_xrce_agent',
    )

    rootfs = os.path.join(PX4_DIR, 'build', 'px4_sitl_default', 'rootfs')
    px4_bin = os.path.join(PX4_DIR, 'build', 'px4_sitl_default', 'bin', 'px4')
    px4_process = ExecuteProcess(
        cmd=[px4_bin, '-i', instance],
        cwd=rootfs,
        additional_env={
            'PX4_SIM_MODEL': 'gz_x500_depth_down',
            'PX4_GZ_WORLD': world_name,
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_MODEL_POSE': get_uav_pose_str(world_name),
            'PX4_SYS_AUTOSTART': '4022',
            'PX4_UXRCE_DDS_PORT': agent_port,
            'PX4_UXRCE_DDS_NS': '',
        },
        output='screen',
        name='px4_sitl',
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'uav_bridge.launch.py')),
        launch_arguments={'world': world_name, 'model_name': model_name}.items(),
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_vision, 'launch', 'gps_denied_vo.launch.py')),
        launch_arguments={
            'run_mission': 'true' if run_mission else 'false',
            'use_sim_time': 'true',
        }.items(),
    )

    return [gz_process, agent_process, px4_process, bridge_launch, vision_launch]


def generate_launch_description():
    resource_paths = [os.path.join(PX4_GZ, 'models'), os.path.join(PX4_GZ, 'worlds')]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resource_paths.append(os.environ['GZ_SIM_RESOURCE_PATH'])

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(resource_paths)),
        SetEnvironmentVariable('GZ_PARTITION', os.environ.get('GZ_PARTITION', 'uav_vision_interiit')),
        SetEnvironmentVariable('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '77')),
        DeclareLaunchArgument('world', default_value='vo_ground',
                              description='Gazebo world name (see world_config.py)'),
        DeclareLaunchArgument('instance', default_value='1',
                              description='PX4 SITL instance number'),
        DeclareLaunchArgument('headless', default_value='true',
                              description='Run Gazebo without the GUI'),
        DeclareLaunchArgument('run_mission', default_value='true',
                              description='Run the 90 s hover mission'),
        OpaqueFunction(function=launch_setup),
    ])
