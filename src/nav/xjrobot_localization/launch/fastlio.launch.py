"""单独启动 xjrobot_localization 内的 Fast-LIO（laser_mapping）节点。

典型用法（E1R）:
  # 终端 1：传感器驱动 + 点云/IMU 转换
  ros2 launch fast_lio rslidar_mapping.launch.py run_fastlio:=false

  # 终端 2：仅 Fast-LIO
  ros2 launch xjrobot_localization fastlio.launch.py
  ros2 launch xjrobot_localization fastlio.launch.py config_file:=e1r.yaml rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os
import yaml


DEFAULT_CONFIG_FILE = "e1r.yaml"
DEFAULT_USE_SIM_TIME = "false"
DEFAULT_RVIZ = "false"
DEFAULT_PCD_MAP = ""


def launch_setup(context, *args, **kwargs):
    package_path = get_package_share_directory("xjrobot_localization")
    config_file = LaunchConfiguration("config_file").perform(context)
    localization_config = os.path.join(package_path, "config", config_file)
    map_file_path = os.path.join("/home/medical/maps", "test.pcd")

    try:
        with open(localization_config, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}
        map_file_path = (
            config.get("laser_mapping", {})
            .get("ros__parameters", {})
            .get("map_file_path", map_file_path)
        )
    except OSError:
        pass

    pcd_map_arg = LaunchConfiguration("pcd_map").perform(context).strip()
    if pcd_map_arg:
        map_file_path = pcd_map_arg

    fast_lio_node = Node(
        package="xjrobot_localization",
        executable="fastlio_mapping",
        name="laser_mapping",
        output="screen",
        parameters=[
            localization_config,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_file_path": map_file_path,
            },
        ],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("xjrobot_localization"), "rviz", "fastlio_localization.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return [fast_lio_node, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=DEFAULT_CONFIG_FILE,
                description="配置文件名，位于 xjrobot_localization/config/，例如 e1r.yaml 或 mid360.yaml",
            ),
            DeclareLaunchArgument(
                "pcd_map",
                default_value=DEFAULT_PCD_MAP,
                description="覆盖配置中 laser_mapping.map_file_path 的 PCD 路径；为空则使用配置文件默认值",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=DEFAULT_USE_SIM_TIME,
                description="是否使用仿真时间",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value=DEFAULT_RVIZ,
                description="是否启动 RViz2",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
