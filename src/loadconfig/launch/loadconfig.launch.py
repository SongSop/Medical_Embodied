from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='loadconfig',
            executable='loadconfig_server',
            name='loadconfig_server',
            output='screen',
        )
    ])
