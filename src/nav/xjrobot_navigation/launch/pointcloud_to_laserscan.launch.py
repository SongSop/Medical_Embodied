from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/ground_segmentation/obstacle_points",
                description="Input obstacle pointcloud topic after ground removal",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="Output 2D LaserScan topic consumed by Nav2",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="lidar_3d_link",
                description="Frame used to generate the 2D scan",
            ),
            DeclareLaunchArgument(
                "transform_tolerance",
                default_value="0.1",
                description="TF lookup tolerance in seconds",
            ),
            DeclareLaunchArgument(
                "min_height",
                default_value="-0.1",
                description="Minimum point height kept during 3D to 2D projection",
            ),
            DeclareLaunchArgument(
                "max_height",
                default_value="2.0",
                description="Maximum point height kept during 3D to 2D projection",
            ),
            DeclareLaunchArgument(
                "angle_min",
                default_value="-3.141592653589793",
                description="Minimum scan angle in radians",
            ),
            DeclareLaunchArgument(
                "angle_max",
                default_value="3.141592653589793",
                description="Maximum scan angle in radians",
            ),
            DeclareLaunchArgument(
                "angle_increment",
                default_value="0.0034906585",
                description="Angular resolution of the generated scan in radians",
            ),
            DeclareLaunchArgument(
                "scan_time",
                default_value="0.1",
                description="Scan period used to populate LaserScan metadata",
            ),
            DeclareLaunchArgument(
                "range_min",
                default_value="0.2",
                description="Minimum valid range in meters",
            ),
            DeclareLaunchArgument(
                "range_max",
                default_value="4.0",
                description="Maximum valid range in meters",
            ),
            DeclareLaunchArgument(
                "queue_size",
                default_value="1",
                description="Input queue size for pointcloud messages",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                remappings=[
                    ("cloud_in", LaunchConfiguration("pointcloud_topic")),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
                parameters=[
                    {
                        # 这里显式输出到 lidar_3d_link，是为了让 scan 的 frame 和当前导航链路保持一致，
                        # 避免后续 costmap / rviz 因 frame 切换再引入额外 TF 歧义。
                        "target_frame": LaunchConfiguration("target_frame"),
                        "transform_tolerance": LaunchConfiguration("transform_tolerance"),
                        "min_height": LaunchConfiguration("min_height"),
                        "max_height": LaunchConfiguration("max_height"),
                        "angle_min": LaunchConfiguration("angle_min"),
                        "angle_max": LaunchConfiguration("angle_max"),
                        "angle_increment": LaunchConfiguration("angle_increment"),
                        "scan_time": LaunchConfiguration("scan_time"),
                        "range_min": LaunchConfiguration("range_min"),
                        "range_max": LaunchConfiguration("range_max"),
                        "queue_size": LaunchConfiguration("queue_size"),
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
            ),
        ]
    )
