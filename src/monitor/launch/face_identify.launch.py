from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """启动人脸识别服务节点（相机节点需独立启动）"""

    return LaunchDescription([
        # ==================== 启动参数 ====================
        DeclareLaunchArgument(
            'use_mock_camera',
            default_value='false',
            description='是否使用模拟相机（无真机时用于测试）'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/camera/color/image_raw',
            description='RGB图像话题名（RealSense默认: /camera/camera/color/image_raw，模拟相机: /camera/rgb/image_raw）'
        ),
        DeclareLaunchArgument(
            'max_recognition_attempts',
            default_value='10',
            description='最大识别尝试次数'
        ),
        DeclareLaunchArgument(
            'recognition_timeout',
            default_value='10.0',
            description='识别超时时间（秒）'
        ),

        # ==================== 模拟相机节点（仅测试用） ====================
        Node(
            package='monitor',
            executable='mock_camera',
            name='mock_camera_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('use_mock_camera'))
        ),

        # ==================== 人脸识别服务节点 ====================
        Node(
            package='monitor',
            executable='face_identify_server',
            name='face_identify_server',
            output='screen',
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic'),
                'max_recognition_attempts': LaunchConfiguration('max_recognition_attempts'),
                'recognition_timeout': LaunchConfiguration('recognition_timeout'),
            }],
        ),
    ])
