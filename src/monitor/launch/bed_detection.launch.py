from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """启动床位检测服务节点（相机节点需独立启动）"""

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
            'yolo_model_path',
            default_value='',
            description='YOLO模型路径（留空使用包内路径）'
        ),
        DeclareLaunchArgument(
            'clip_model_path',
            default_value='',
            description='CLIP模型路径（留空使用包内路径）'
        ),
        DeclareLaunchArgument(
            'max_beds',
            default_value='10',
            description='最大床位数'
        ),
        DeclareLaunchArgument(
            'yolo_conf_threshold',
            default_value='0.5',
            description='YOLO检测置信度阈值（低于此值的检测结果被过滤）'
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

        # ==================== 床位检测服务节点 ====================
        Node(
            package='monitor',
            executable='bed_detection_server',
            name='bed_detection_server',
            output='screen',
            parameters=[{
                'yolo_model_path': LaunchConfiguration('yolo_model_path'),
                'clip_model_path': LaunchConfiguration('clip_model_path'),
                'max_beds': LaunchConfiguration('max_beds'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'yolo_conf_threshold': LaunchConfiguration('yolo_conf_threshold'),
            }],
            emulate_tty=True
        ),
    ])
