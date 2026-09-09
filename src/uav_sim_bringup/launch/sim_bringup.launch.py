"""Top-level simulation bringup launch file for GPS-denied UAV operations."""
import os
import shutil

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

from uav_sim_bringup.world_config import get_uav_pose_str, get_world_config


def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    run_mission = LaunchConfiguration('run_mission').perform(context).lower() == 'true'
    takeoff_alt = LaunchConfiguration('takeoff_altitude').perform(context)
    hold_s = LaunchConfiguration('hold_seconds').perform(context)

    px4_dir = '/home/jashan/PX4-Autopilot'
    workspace_dir = '/home/jashan/aerial_interiit_prep'
    pkg_bringup = get_package_share_directory('uav_sim_bringup')
    pkg_vision = get_package_share_directory('uav_vision')

    world_cfg = get_world_config(world_name)
    uav_pose_str = get_uav_pose_str(world_name)

    # Locate world SDF
    world_sdf = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds', f'{world_name}.sdf')
    if not os.path.isfile(world_sdf):
        drdo_sdf = f'/home/jashan/training_pool/src/drdo_gz_worlds/worlds/{world_name}.sdf'
        if os.path.isfile(drdo_sdf):
            world_sdf = drdo_sdf

    instance = LaunchConfiguration('instance').perform(context)
    agent_port = str(8890 if instance == '1' else (8888 + int(instance)))
    model_name = f'x500_depth_down_{instance}'

    # 1. Gazebo Sim
    gz_args = f"-r -s {world_sdf}" if headless else f"-r {world_sdf}"
    gz_process = ExecuteProcess(
        cmd=['gz', 'sim'] + gz_args.split(),
        output='screen',
        name='gz_sim',
    )

    # 2. MicroXRCEAgent
    agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', agent_port],
        output='screen',
        name='micro_xrce_agent',
    )

    # 3. PX4 SITL process
    rootfs = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'rootfs')
    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    px4_env = {
        'PX4_SIM_MODEL': 'gz_x500_depth_down',
        'PX4_GZ_WORLD': world_name,
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_MODEL_POSE': uav_pose_str,
        'PX4_SYS_AUTOSTART': '4022',
        'PX4_UXRCE_DDS_PORT': agent_port,
        'PX4_UXRCE_DDS_NS': '',
    }
    px4_process = ExecuteProcess(
        cmd=[px4_bin, '-i', instance],
        cwd=rootfs,
        additional_env=px4_env,
        output='screen',
        name='px4_sitl',
    )

    # 4. UAV Camera & Clock Bridge
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'uav_bridge.launch.py')
        ),
        launch_arguments={
            'world': world_name,
            'model_name': model_name,
        }.items(),
    )

    # 5. UAV Vision & Health & Mission stack
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vision, 'launch', 'gps_denied_vo.launch.py')
        ),
        launch_arguments={
            'run_mission': 'true' if run_mission else 'false',
            'use_sim_time': 'true',
        }.items(),
    )

    return [
        gz_process,
        agent_process,
        px4_process,
        bridge_launch,
        vision_launch,
    ]


def generate_launch_description():
    px4_dir = '/home/jashan/PX4-Autopilot'
    training_pool = '/home/jashan/training_pool'
    existing_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    resource_paths = [
        f"{px4_dir}/Tools/simulation/gz/models",
        f"{px4_dir}/Tools/simulation/gz/worlds",
        f"{training_pool}/src/drdo_gz_worlds/models",
        f"{training_pool}/src/drdo_gz_worlds/worlds",
        "/home/jashan/uav_guided_ugv/install/ackermann_gz_bringup/share/ackermann_gz_bringup/models",
    ]
    if existing_resource:
        resource_paths.append(existing_resource)

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(resource_paths)
    )
    set_partition = SetEnvironmentVariable(
        name='GZ_PARTITION',
        value=os.environ.get('GZ_PARTITION', 'uav_vision_interiit')
    )
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=os.environ.get('ROS_DOMAIN_ID', '77')
    )

    return LaunchDescription([
        set_resource_path,
        set_partition,
        set_domain_id,
        DeclareLaunchArgument('world', default_value='vo_ground',
                              description='Gazebo world (vo_ground, drdo_world1, drdo_world2, drdo_world3)'),
        DeclareLaunchArgument('instance', default_value='1',
                              description='PX4 SITL instance number (default 1 to avoid conflicts)'),
        DeclareLaunchArgument('headless', default_value='true',
                              description='Run Gazebo without GUI'),
        DeclareLaunchArgument('run_mission', default_value='true',
                              description='Run autonomous offboard 90s hover mission'),
        DeclareLaunchArgument('takeoff_altitude', default_value='10.0',
                              description='Target hover altitude in meters'),
        DeclareLaunchArgument('hold_seconds', default_value='90.0',
                              description='Target hold duration in seconds'),
        OpaqueFunction(function=launch_setup),
    ])
