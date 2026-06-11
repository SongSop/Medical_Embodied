from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_scan_topic",
                default_value="/scan_raw",
                description="Raw LaserScan topic from pointcloud_to_laserscan",
            ),
            DeclareLaunchArgument(
                "output_scan_topic",
                default_value="/scan_filtered",
                description="Filtered LaserScan topic consumed by Nav2",
            ),
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                name="scan_to_scan_filter_chain",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("xjrobot_navigation"),
                            "config",
                            "shadow_filter_example.yaml",
                        ]
                    )
                ],
                remappings=[
                    ("scan", LaunchConfiguration("input_scan_topic")),
                    ("scan_filtered", LaunchConfiguration("output_scan_topic")),
                ],
            ),
        ]
    )
