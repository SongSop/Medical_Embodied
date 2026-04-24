from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_waypoints_path = PathJoinSubstitution(
        [FindPackageShare("xjrobot_bridge"), "config", "waypoints.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=default_waypoints_path,
                description="桥接节点点位参数文件绝对路径",
            ),
            Node(
                package="xjrobot_bridge",
                executable="xjrobot_bridge_node",
                name="xjrobot_bridge_node",
                output="screen",
                parameters=[LaunchConfiguration("waypoints_file")],
            ),
        ]
    )
