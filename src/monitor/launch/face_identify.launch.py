from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='monitor',
            executable='face_identify_server',
            name='face_identify_server',
            output='screen',
        )
    ])
