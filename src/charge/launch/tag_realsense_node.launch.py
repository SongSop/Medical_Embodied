import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument("image_topic", default_value="image_raw")
    camera_name_arg = DeclareLaunchArgument("camera_name", default_value="/charge_cam")
    detections_topic_arg = DeclareLaunchArgument(
        "detections_topic", default_value="/apriltag_detections"
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("charge"), "cfg", "tags_36h11_node.yaml"
        ),
    )

    image_topic = [LaunchConfiguration("camera_name"), "/", LaunchConfiguration("image_topic")]
    info_topic = [LaunchConfiguration("camera_name"), "/camera_info"]

    apriltag_node = Node(
        package="apriltag_ros",
        executable="tag_detector",
        name="apriltag",
        namespace="apriltag",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[
            ("image", image_topic),
            ("camera_info", info_topic),
            ("apriltag_detections", LaunchConfiguration("detections_topic")),
        ],
    )

    return LaunchDescription(
        [image_topic_arg, camera_name_arg, detections_topic_arg, params_file_arg, apriltag_node]
    )
