from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # world_path 支持外部直接传完整路径；若未传则由 world_name 组装
    world_path = LaunchConfiguration("world_path")

    # Gazebo Sim 官方启动入口
    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    # 机器人描述发布入口（xacro -> robot_description -> TF）
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare("xjrobot_description"), "launch", "description.launch.py"]
    )
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("xjrobot_description"), "urdf", "robots", "mid360.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("xjrobot_gazebo"), "rviz", "xjrobot.rviz"]
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("xjrobot_gazebo"), "config", "ekf.yaml"]
    )

    # Gazebo Server（物理+仿真计算）
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={"gz_args": [" -r -s ", world_path]}.items(),
    )

    # Gazebo GUI（可视化界面）
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        condition=IfCondition(LaunchConfiguration("gui")),
        launch_arguments={"gz_args": [" -g"]}.items(),
    )

    # 发布 robot_description + 机器人各关节 TF（与 lino 模版一致）
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            "urdf": LaunchConfiguration("urdf"),
            "publish_joints": "false",
            "rviz": "false",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # 将 URDF 机器人实体生成到 Gazebo 世界中
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "xjrobot",
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
            "-Y",
            LaunchConfiguration("spawn_yaw"),
        ],
    )

    # Gazebo <-> ROS 话题桥
    # 按当前 xjrobot 实际发布路径，仅保留原始话题链路。
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            # 基础链路
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            # 里程计：仅使用原始话题 /odom/unfiltered
            "/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # 2D 雷达：仅使用原始话题 /scan
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # 3D 雷达：仅使用原始话题 /livox/scan3d/points
            "/livox/scan3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
    )

    # EKF 融合原始里程计+IMU，输出稳定 /odom（odom->base_footprint）
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ekf_config_path,
        ],
        remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))],
    )

    # 可选仿真 RViz（默认关闭，避免和导航 RViz 重复）
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("simu_rviz")),
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="是否启动 Gazebo GUI"),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="是否启用仿真时间（/clock）"
            ),
            DeclareLaunchArgument(
                "simu_rviz",
                default_value="false",
                description="是否在仿真 launch 中额外启动 RViz",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="xjrobot/myworld2.sdf",
                description=(
                    "world 相对路径（相对 xjrobot_gazebo/worlds）"
                    "，例如 xjrobot/myworld2.sdf"
                ),
            ),
            DeclareLaunchArgument(
                "world_path",
                default_value=[
                    FindPackageShare("xjrobot_gazebo"),
                    "/worlds/",
                    LaunchConfiguration("world_name"),
                ],
                description="world 文件完整路径；显式传入时会覆盖 world_name",
            ),
            DeclareLaunchArgument("urdf", default_value=urdf_path, description="URDF/Xacro 路径"),
            DeclareLaunchArgument("spawn_x", default_value="0.0", description="机器人初始 X（米）"),
            DeclareLaunchArgument("spawn_y", default_value="0.0", description="机器人初始 Y（米）"),
            # 车体最低点约在 base_link 下方 0.10m，0.07 会初始插地导致物理发散/模型跑飞
            DeclareLaunchArgument("spawn_z", default_value="0.12", description="机器人初始 Z（米）"),
            DeclareLaunchArgument(
                "spawn_yaw", default_value="0.0", description="机器人初始朝向 yaw（弧度）"
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="EKF 融合里程计输出话题",
            ),
            gz_server,
            gz_gui,
            description_launch,
            spawn_robot,
            bridge,
            ekf,
            rviz,
        ]
    )
