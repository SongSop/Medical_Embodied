"""HDL navigation base bringup without robot TF publishers.

This launch is intended for the HDL localization TF tree:

  map -> rslidar -> base_link

It deliberately does not start robot_state_publisher or EKF, because those
publish the old odom/base_footprint/base_link tree and conflict with HDL.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_params = PathJoinSubstitution(
        [FindPackageShare("xjrobot_base"), "config", "base.yaml"]
    )
    ground_segmentation_launch_path = PathJoinSubstitution(
        [FindPackageShare("ground_segmentation_ros2"), "launch", "ground_segmentation.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "launch_joy",
                default_value="true",
                description="Launch joy_node for manual/auto mode switching",
            ),
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="Joystick device path",
            ),
            DeclareLaunchArgument(
                "launch_ground_segmentation",
                default_value="true",
                description="Launch ground segmentation for Nav2 /scan generation",
            ),
            DeclareLaunchArgument(
                "ground_segmentation_pointcloud_topic",
                default_value="/rslidar_points",
                description="PointCloud2 topic consumed by ground segmentation",
            ),
            DeclareLaunchArgument(
                "ground_segmentation_imu_topic",
                default_value="/imu/link1",
                description="IMU topic consumed by ground segmentation",
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
                condition=IfCondition(LaunchConfiguration("launch_ground_segmentation")),
                launch_arguments={
                    "pointcloud_topic": LaunchConfiguration(
                        "ground_segmentation_pointcloud_topic"
                    ),
                    "imu_topic": LaunchConfiguration("ground_segmentation_imu_topic"),
                    "sim": LaunchConfiguration("use_sim_time"),
                    "robot_frame": "rslidar",
                    "output_frame_id": "rslidar",
                }.items(),
            ),
        ]
    )
