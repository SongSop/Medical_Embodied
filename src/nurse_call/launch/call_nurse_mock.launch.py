from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nurse_call',
            executable='call_nurse_mock_server',
            name='call_nurse_mock_server',
            output='screen',
        )
    ])
