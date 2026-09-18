"""Bridge UAV downward camera and simulation clock from Gazebo Sim to ROS 2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)

    # Gazebo topics for downward OakD-Lite camera
    gz_rgb_image = f'/world/{world}/model/{model_name}/link/camera_link/sensor/IMX214/image'
    gz_rgb_info = f'/world/{world}/model/{model_name}/link/camera_link/sensor/IMX214/camera_info'
    gz_depth_image = '/depth_camera'
    gz_depth_points = '/depth_camera/points'
    gz_clock = f'/world/{world}/clock'

    clock_bridge_args = [
        f'{gz_clock}@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '--ros-args',
        '-r', f'{gz_clock}:=/clock',
    ]

    camera_bridge_args = [
        f'{gz_rgb_image}@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{gz_rgb_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        f'{gz_depth_image}@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{gz_depth_points}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '--ros-args',
        '-r', f'{gz_rgb_image}:=/uav/rgb',
        '-r', f'{gz_rgb_info}:=/uav/camera_info',
        '-r', f'{gz_depth_image}:=/uav/depth',
        '-r', f'{gz_depth_points}:=/uav/points',
    ]

    clock_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_clock_bridge',
        output='screen',
        arguments=clock_bridge_args,
    )

    camera_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_camera_bridge',
        output='screen',
        arguments=camera_bridge_args,
    )

    return [clock_node, camera_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='vo_ground',
            description='World name containing the UAV model (vo_ground, drdo_world1, drdo_world2, drdo_world3)'
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='x500_depth_down_0',
            description='Spawned UAV model instance name'
        ),
        OpaqueFunction(function=launch_setup),
    ])
