import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_loc = get_package_share_directory('hdl_localization')
    base_launch = os.path.join(pkg_loc, 'launch', 'hdl_localization.launch.py')

    include_hdl_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'use_sim_time': 'false',
            'points_topic': '/rslidar_points',
            'imu_topic': '/imu/link1',
            'odom_topic': '/odom/unfiltered',
            'robot_odom_frame_id': 'odom',
            'odom_child_frame_id': 'base_footprint',
            'lidar_frame_id': 'rslidar',
            'globalmap_pcd': '/home/medical/rs_lidar/src/FAST_LIO/PCD/test.pcd',
            'use_global_localization': 'true',
            'use_imu': 'true',
            'send_tf_transforms': 'false',
            'publish_base_to_lidar_tf': 'true',
            'base_to_livox_x': '0.0',
            'base_to_livox_y': '0.0',
            'base_to_livox_z': '0.0',
            'base_to_livox_qx': '0.0',
            'base_to_livox_qy': '0.0',
            'base_to_livox_qz': '0.0',
            'base_to_livox_qw': '1.0',
        }.items(),
    )

    static_tf_base_to_link1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_link1_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'link1',
        ],
    )

    return LaunchDescription([
        include_hdl_localization,
        static_tf_base_to_link1,
    ])
