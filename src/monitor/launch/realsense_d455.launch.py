from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """独立启动 RealSense D455 相机节点，只发布图像话题"""

    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='相机名称'
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='camera',
            description='相机命名空间'
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='true',
            description='是否启用深度流'
        ),
        DeclareLaunchArgument(
            'depth_profile',
            default_value='640x480x30',
            description='深度流配置 (WxHxFPS)'
        ),
        DeclareLaunchArgument(
            'color_profile',
            default_value='640x480x30',
            description='彩色流配置 (WxHxFPS)'
        ),
        DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='false',
            description='是否启用点云'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
            launch_arguments={
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_namespace': LaunchConfiguration('camera_namespace'),
                'enable_color': 'true',
                'enable_depth': LaunchConfiguration('enable_depth'),
                'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
                'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
                'enable_accel': 'false',
                'enable_gyro': 'false',
                'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),
                'align_depth.enable': 'true',
                'enable_sync': 'true',
            }.items()
        ),
    ])
