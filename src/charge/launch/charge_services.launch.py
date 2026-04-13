from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='charge',
            executable='charge_services',
            name='charge_services',
            output='screen',
        )
    ])
