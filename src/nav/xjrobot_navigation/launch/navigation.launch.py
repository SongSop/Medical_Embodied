from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 默认加载地图名（对应 maps/<MAP_NAME>.yaml）
MAP_NAME = "scans"


def generate_launch_description():
    # Nav2 官方 bringup 入口
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )

    # RViz 配置（已按 xjrobot 的话题和坐标系设置）
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("xjrobot_navigation"), "rviz", "xjrobot_navigation.rviz"]
    )

    # 默认地图路径；也可通过 launch 参数 map:=... 覆盖
    default_map_path = PathJoinSubstitution(
        [FindPackageShare("xjrobot_navigation"), "maps", f"{MAP_NAME}.yaml"]
    )

    # Nav2 参数总配置
    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare("xjrobot_navigation"), "config", "navigation.yaml"]
    )

    return LaunchDescription(
        [
            # 仿真时必须为 true（使 Nav2 与 Gazebo 统一使用 /clock）
            DeclareLaunchArgument(
                name="sim", default_value="true", description="是否启用仿真时间（use_sim_time）"
            ),
            # 是否同时启动 RViz（调试阶段建议 true）
            DeclareLaunchArgument(name="rviz", default_value="true", description="是否启动 RViz2"),
            DeclareLaunchArgument(
                name="map", default_value=default_map_path, description="导航地图 yaml 的绝对路径"
            ),
            DeclareLaunchArgument(
                name="initial_pose_x", default_value="0.5", description="初始位姿 X（米）"
            ),
            DeclareLaunchArgument(
                name="initial_pose_y", default_value="0.0", description="初始位姿 Y（米）"
            ),
            DeclareLaunchArgument(
                name="initial_pose_yaw", default_value="0.0", description="初始朝向 yaw（弧度）"
            ),
            # 启动 Nav2 栈：map_server / amcl / planner / controller / bt_navigator 等
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": LaunchConfiguration("sim"),
                    "params_file": nav2_config_path,
                    "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                    "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                    "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                }.items(),
            ),
            # 可选 RViz；注意 Fixed Frame 建议设置为 map
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
                condition=IfCondition(LaunchConfiguration("rviz")),
                parameters=[{"use_sim_time": LaunchConfiguration("sim")}],
            ),
        ]
    )
