"""底盘与状态估计 bringup（不含 3D 雷达驱动、点云旋转、IMU 旋转）。

启动节点：
  - base_controller_node / can_hardware_node / wheel_odom_fusion_node
  - joy_node（可选）
  - robot_state_publisher / ekf_node
  - ground_segmentation（可选，需外部已提供旋转后的点云与 IMU 话题）
  - joint_state_publisher（可选）

雷达驱动与 mid360_rotation_node 请单独启动，例如 E1R 驱动 + 自定义旋转节点。
"""

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
    ground_segmentation_launch_path = PathJoinSubstitution(
        [FindPackageShare("ground_segmentation_ros2"), "launch", "ground_segmentation.launch.py"]
    )
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("urdf")]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "urdf", default_value=urdf_path, description="Robot URDF/Xacro path"
            ),
            DeclareLaunchArgument(
                "publish_joints",
                default_value="false",
                description="Launch joint_state_publisher for non-fixed joints",
            ),
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
                "launch_ground_segmentation",
                default_value="true",
                description=(
                    "Launch ground segmentation; requires external rotated pointcloud/IMU topics"
                ),
            ),
            DeclareLaunchArgument(
                "ground_segmentation_pointcloud_topic",
                default_value="/rslidar_points_fastlio_frame",
                description="PointCloud2 topic for ground segmentation",
            ),
            DeclareLaunchArgument(
                "ground_segmentation_imu_topic",
                default_value="/rslidar_imu_data_rotated",
                description="IMU topic for ground segmentation",
            ),
            DeclareLaunchArgument(
                "chassis_only",
                default_value="false",
                description=(
                    "If true, launch only chassis-related nodes and joy_node "
                    "(no robot_state_publisher, EKF, or ground segmentation)"
                ),
            ),
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
                    "pointcloud_topic": LaunchConfiguration(
                        "ground_segmentation_pointcloud_topic"
                    ),
                    "imu_topic": LaunchConfiguration("ground_segmentation_imu_topic"),
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
