from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import ament_index_python.packages

def generate_launch_description():
    """启动床位检测节点和模拟相机节点"""
    
    package_path = ament_index_python.packages.get_package_share_directory('monitor')
    
    return LaunchDescription([
        # 启动参数
        DeclareLaunchArgument(
            'use_mock_camera',
            default_value='true',
            description='是否使用模拟相机'
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
        
        # 模拟相机节点
        Node(
            package='monitor',
            executable='mock_camera',
            name='mock_camera_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(
                PythonExpression([LaunchConfiguration('use_mock_camera'), " == 'true'"])
            )
        ),
        
        # 床位检测服务节点
        Node(
            package='monitor',
            executable='bed_detection_server',
            name='bed_detection_server',
            output='screen',
            parameters=[{
                'yolo_model_path': LaunchConfiguration('yolo_model_path'),
                'clip_model_path': LaunchConfiguration('clip_model_path'),
                'max_beds': LaunchConfiguration('max_beds'),
            }],
            emulate_tty=True
        ),

    ])