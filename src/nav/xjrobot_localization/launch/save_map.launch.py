from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os

# ros2 launch xjrobot_localization save_map.launch.py \
#   map_topic:=/map \
#   save_dir:=/home/medical/maps \
#   map_name:=ward_20260421
def launch_setup(context, *args, **kwargs):
    save_dir = LaunchConfiguration("save_dir").perform(context)
    os.makedirs(save_dir, exist_ok=True)

    params_file = LaunchConfiguration("params_file")
    map_topic = LaunchConfiguration("map_topic")
    map_name = LaunchConfiguration("map_name")
    map_file = PathJoinSubstitution([LaunchConfiguration("save_dir"), map_name])

    map_saver = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-t",
            map_topic,
            "-f",
            map_file,
            "--ros-args",
            "--params-file",
            params_file,
        ],
        output="screen",
    )

    return [map_saver]


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("xjrobot_localization"), "config", "save_map.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="map_saver_cli 参数文件路径",
            ),
            DeclareLaunchArgument(
                "map_topic",
                default_value="/map",
                description="待保存地图话题",
            ),
            DeclareLaunchArgument(
                "save_dir",
                default_value="/home/medical/maps",
                description="地图输出目录",
            ),
            DeclareLaunchArgument(
                "map_name",
                default_value="map",
                description="输出地图文件名（不含扩展名）",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
