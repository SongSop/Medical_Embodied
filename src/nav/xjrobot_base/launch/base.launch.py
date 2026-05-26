from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_params = PathJoinSubstitution(
        [FindPackageShare("xjrobot_base"), "config", "base.yaml"]
    )
    ekf_params = PathJoinSubstitution(
        [FindPackageShare("xjrobot_base"), "config", "ekf.yaml"]
    )
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("xjrobot_base"), "urdf", "robots", "mid360.urdf.xacro"]
    )
    livox_launch_path = PathJoinSubstitution(
        [FindPackageShare("livox_ros_driver2"), "launch_ROS2", "msg_MID360_launch.py"]
    )
    ground_segmentation_launch_path = PathJoinSubstitution(
        [FindPackageShare("ground_segmentation_ros2"), "launch", "ground_segmentation.launch.py"]
    )
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("urdf")]), value_type=str
    )

    return LaunchDescription(
        [
            # 实机底盘 bringup 默认不使用仿真时间，但保留参数便于后续统一调试。
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            # 直接复用与仿真相同的机器人描述，确保固定 TF 链保持一致。
            DeclareLaunchArgument(
                "urdf", default_value=urdf_path, description="Robot URDF/Xacro path"
            ),
            # 实机通常只需要 robot_state_publisher 发布固定关节 TF；
            # 如果想连轮子等非固定关节也一起发布零位姿，可手动改成 true。
            DeclareLaunchArgument(
                "publish_joints",
                default_value="false",
                description="Launch joint_state_publisher for non-fixed joints",
            ),
            # 可选启动 joy_node，便于实机直接接入手柄控制。
            DeclareLaunchArgument(
                "launch_joy",
                default_value="true",
                description="Launch joy_node for joystick input",
            ),
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="Joystick device path",
            ),
            DeclareLaunchArgument(
                "launch_livox",
                default_value="true",
                description="Launch Livox MID360 driver",
            ),
            DeclareLaunchArgument(
                "launch_ground_segmentation",
                default_value="true",
                description="Launch ground segmentation for rotated lidar pointcloud",
            ),
            DeclareLaunchArgument(
                "chassis_only",
                default_value="false",
                description="If true, launch only chassis-related nodes and joy node",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(livox_launch_path),
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("launch_livox"),
                            "' == 'true' and '",
                            LaunchConfiguration("chassis_only"),
                            "' != 'true'",
                        ]
                    )
                ),
            ),
            # 机器人底盘相关的驱动和控制程序
            Node(
                package="xjrobot_base",
                executable="base_controller_node",
                name="xjrobot_base_controller",
                parameters=[base_params],
            ),
            Node(
                package="xjrobot_base",
                executable="can_hardware_node",
                name="xjrobot_can_hardware",
                parameters=[base_params],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                condition=IfCondition(LaunchConfiguration("launch_joy")),
                parameters=[
                    {
                        "dev": LaunchConfiguration("joy_dev"),
                        "deadzone": 0.05,
                        "autorepeat_rate": 0.0,
                        "default_trig_val": True,
                        "coalesce_interval": 0.01,
                        "symmetric_motion": True,
                        "autorepeat_after": 20,
                    }
                ],
            ),
            Node(
                package="xjrobot_base",
                executable="wheel_odom_fusion_node",
                name="xjrobot_wheel_odom_fusion",
                parameters=[base_params],
            ),
            Node(
                package="xjrobot_base",
                executable="mid360_rotation_node",
                name="mid360_rotation_node",
                condition=UnlessCondition(LaunchConfiguration("chassis_only")),
                parameters=[base_params],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ground_segmentation_launch_path),
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("launch_ground_segmentation"),
                            "' == 'true' and '",
                            LaunchConfiguration("chassis_only"),
                            "' != 'true'",
                        ]
                    )
                ),
                launch_arguments={
                    "pointcloud_topic": "/livox/lidar_rotated/points",
                    "imu_topic": "/livox/imu_rotated",
                    "sim": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("publish_joints"),
                            "' == 'true' and '",
                            LaunchConfiguration("chassis_only"),
                            "' != 'true'",
                        ]
                    )
                ),
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("chassis_only")),
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "robot_description": robot_description,
                    }
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("chassis_only")),
                parameters=[ekf_params],
                remappings=[("odometry/filtered", "/odom")],
            ),
        ]
    )
