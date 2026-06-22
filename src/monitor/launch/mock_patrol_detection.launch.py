from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动巡诊检测模拟节点（无需相机/YOLO/CLIP，替代 bed_detection_server）。"""

    return LaunchDescription([
        DeclareLaunchArgument(
            'service_name',
            default_value='/detect_anomaly',
            description='检测服务名，与行为树调用的 /detect_anomaly 保持一致',
        ),
        DeclareLaunchArgument(
            'fall_risk_bed_id',
            default_value='4',
            description='Bed 模式返回坠床风险的床位 index',
        ),

        Node(
            package='monitor',
            executable='mock_patrol_detection_server',
            name='mock_patrol_detection_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'service_name': LaunchConfiguration('service_name'),
                'area_occupied_beds': [2, 4],
                'area_urgencies': [1, 1],
                'fall_risk_bed_id': LaunchConfiguration('fall_risk_bed_id'),
                'fall_risk_details': 'fall risk detected at bed 4',
            }],
        ),
    ])
