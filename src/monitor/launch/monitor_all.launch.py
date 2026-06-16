from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    一键启动：RealSense D455相机 + 床位检测 + 人脸识别

    分步启动（推荐用于调试）：
      ros2 launch monitor realsense_d455.launch.py
      ros2 launch monitor bed_detection.launch.py
      ros2 launch monitor face_identify.launch.py

    模拟相机测试：
      ros2 launch monitor monitor_all.launch.py use_mock_camera:=true camera_topic:=/camera/rgb/image_raw
    """

    monitor_launch_dir = os.path.join(
        get_package_share_directory('monitor'), 'launch'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_camera',
            default_value='false',
            description='使用模拟相机替代真实相机（测试用）'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/camera/color/image_raw',
            description='RGB图像话题名（mock相机需设为 /camera/rgb/image_raw）'
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'depth_profile',
            default_value='640x480x30',
        ),
        DeclareLaunchArgument(
            'color_profile',
            default_value='640x480x30',
        ),

        # ==================== 相机 ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(monitor_launch_dir, 'realsense_d455.launch.py')
            ),
            condition=UnlessCondition(
                LaunchConfiguration('use_mock_camera')
            ),
        ),

        # ==================== 床位检测 ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(monitor_launch_dir, 'bed_detection.launch.py')
            ),
            launch_arguments={
                'use_mock_camera': LaunchConfiguration('use_mock_camera'),
                'camera_topic': LaunchConfiguration('camera_topic'),
            }.items(),
        ),

        # ==================== 人脸识别 ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(monitor_launch_dir, 'face_identify.launch.py')
            ),
            launch_arguments={
                'use_mock_camera': LaunchConfiguration('use_mock_camera'),
                'camera_topic': LaunchConfiguration('camera_topic'),
            }.items(),
        ),
    ])
