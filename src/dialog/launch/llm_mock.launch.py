from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dialog',
            executable='llm_mock_server',
            name='llm_mock_server',
            output='screen',
        )
    ])
