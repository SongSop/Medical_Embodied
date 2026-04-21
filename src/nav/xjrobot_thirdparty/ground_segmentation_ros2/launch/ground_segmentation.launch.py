import os
import shutil
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _java_env():
    javac_path = shutil.which("javac")
    if not javac_path:
        return {}

    java_home = Path(javac_path).resolve().parent.parent
    java_lib = str(java_home / "lib")
    java_server_lib = str(java_home / "lib" / "server")

    ld_library_path_parts = [java_lib, java_server_lib]
    existing_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    if existing_ld_library_path:
        ld_library_path_parts.append(existing_ld_library_path)

    return {
        "JAVA_HOME": str(java_home),
        "LD_LIBRARY_PATH": ":".join(ld_library_path_parts),
    }


def launch_setup(context, *args, **kwargs):

    parameters_file = PathJoinSubstitution(
        [FindPackageShare("ground_segmentation_ros2"), "config", "parameters.yaml"]
    )

    ground_segmentation_ros2_node = Node(
        package="ground_segmentation_ros2",
        executable="ground_segmentation_ros2_node",
        parameters=[
            parameters_file,
            {
                "use_sim_time": LaunchConfiguration("sim")
            }
        ],
        remappings=[
            ("/ground_segmentation/input_pointcloud", LaunchConfiguration("pointcloud_topic")),
            ("/ground_segmentation/input_imu", LaunchConfiguration("imu_topic")),
        ],
        additional_env=_java_env(),
        output="screen",
    )

    return [ground_segmentation_ros2_node]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/ground_segmentation/input_pointcloud",
            description="Topic name for the pointcloud",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/ground_segmentation/input_imu",
            description="Topic name for the imu",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Use simulation time (set true for Gazebo / bag playback)",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
