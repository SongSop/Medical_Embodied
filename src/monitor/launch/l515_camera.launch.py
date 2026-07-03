"""Launch Intel RealSense L515 camera via librealsense2 C++ node.

Replaces realsense_d455.launch.py.  Publishes the same topic set as
realsense2_camera so existing monitor nodes work without modification.

Published topics:
  /camera/camera/color/image_raw
  /camera/camera/aligned_depth_to_color/image_raw
  /camera/camera/color/camera_info
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_no', default_value='',
            description='L515 serial number (empty = auto-detect first device)'
        ),
        DeclareLaunchArgument(
            'camera_name', default_value='camera',
            description='Camera name prefix for topic and frame_id'
        ),
        DeclareLaunchArgument(
            'camera_namespace', default_value='camera',
            description='Camera namespace prefix for topic'
        ),
        DeclareLaunchArgument(
            'color_width', default_value='640',
            description='Color stream width'
        ),
        DeclareLaunchArgument(
            'color_height', default_value='480',
            description='Color stream height'
        ),
        DeclareLaunchArgument(
            'color_fps', default_value='30',
            description='Color stream framerate'
        ),
        DeclareLaunchArgument(
            'depth_width', default_value='640',
            description='Depth stream width'
        ),
        DeclareLaunchArgument(
            'depth_height', default_value='480',
            description='Depth stream height'
        ),
        DeclareLaunchArgument(
            'depth_fps', default_value='30',
            description='Depth stream framerate'
        ),
        # L515-specific visual preset:
        #   short_range    – 0.3–3 m indoor scenes (default, best for wards)
        #   no_ambient     – no ambient light
        #   low_ambient    – low ambient light
        #   max_range      – maximum range
        #   default / automatic
        DeclareLaunchArgument(
            'l515_preset', default_value='short_range',
            description='L515 visual preset'
        ),
        DeclareLaunchArgument(
            'publish_depth', default_value='true',
            description='Publish aligned depth image topic'
        ),

        Node(
            package='monitor',
            executable='l515_camera_node',
            name='l515_camera_node',
            output='screen',
            parameters=[{
                'serial_no': LaunchConfiguration('serial_no'),
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_namespace': LaunchConfiguration('camera_namespace'),
                'color_width': LaunchConfiguration('color_width'),
                'color_height': LaunchConfiguration('color_height'),
                'color_fps': LaunchConfiguration('color_fps'),
                'depth_width': LaunchConfiguration('depth_width'),
                'depth_height': LaunchConfiguration('depth_height'),
                'depth_fps': LaunchConfiguration('depth_fps'),
                'l515_preset': LaunchConfiguration('l515_preset'),
                'publish_depth': LaunchConfiguration('publish_depth'),
            }],
        ),
    ])
