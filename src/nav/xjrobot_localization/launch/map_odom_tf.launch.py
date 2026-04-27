from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_params_file = PathJoinSubstitution(
        [FindPackageShare("xjrobot_localization"), "config", "map_odom_tf.yaml"]
    )

    map_odom_tf_publisher = Node(
        package="xjrobot_localization",
        executable="map_odom_tf_publisher",
        name="map_odom_tf_publisher",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="仿真环境请设为 true，实机请设为 false",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="map->odom 发布器参数文件",
            ),
            map_odom_tf_publisher,
        ]
    )

