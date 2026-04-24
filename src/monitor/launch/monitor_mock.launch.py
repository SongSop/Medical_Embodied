from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='monitor',
            executable='monitor_mock_pub',
            name='monitor_mock_pub',
            output='screen',
            parameters=[{
                'battery_soc': 50.0,
                'battery_charging': False,
                'battery_voltage': 24.0,
                'fault_type': '',
                'fault_severity': 0,
                'fault_details': '',
                'call_signal': False,
            }],
        ),
        Node(
            package='monitor',
            executable='anomaly_detect_server',
            name='anomaly_detect_server',
            output='screen',
        ),
    ])
