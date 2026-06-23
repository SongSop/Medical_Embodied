import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _load_static_transform(config_path, parent_frame, child_frame):
    with open(config_path, 'r', encoding='utf-8') as config_file:
        frame_config = yaml.safe_load(config_file)

    for relation in frame_config.get('relations', []):
        if relation.get('parent') == parent_frame and relation.get('child') == child_frame:
            xyz = relation.get('xyz', [0.0, 0.0, 0.0])
            rpy = relation.get('rpy', [0.0, 0.0, 0.0])
            return [str(value) for value in xyz], [str(value) for value in rpy]

    raise RuntimeError(
        f'Static transform {parent_frame} -> {child_frame} not found in {config_path}'
    )


def generate_launch_description():
    pkg_loc = get_package_share_directory('hdl_localization')
    base_launch = os.path.join(pkg_loc, 'launch', 'hdl_localization.launch.py')
    frame_config = os.path.join(pkg_loc, 'config', 'rslidar_local_frames.yaml')
    rslidar_to_base_link_xyz, rslidar_to_base_link_rpy = _load_static_transform(
        frame_config, 'rslidar', 'base_link'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    use_imu = LaunchConfiguration('use_imu')
    use_global_localization = LaunchConfiguration('use_global_localization')
    invert_imu_acc = LaunchConfiguration('invert_imu_acc')
    invert_imu_gyro = LaunchConfiguration('invert_imu_gyro')
    log_output_frequency_hz = LaunchConfiguration('log_output_frequency_hz')

    # Debug/stable mode:
    # - localize directly as map -> rslidar
    # - disable IMU prediction and global relocalization side effects
    # This isolates frame-extrinsic issues that typically cause "aligned_points sinking".
    include_hdl_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'points_topic': '/rslidar_points',
            'imu_topic': imu_topic,
            'odom_topic': odom_topic,
            'robot_odom_frame_id': 'rslidar',
            'odom_child_frame_id': 'rslidar',
            'lidar_frame_id': 'rslidar',
            'globalmap_pcd': '/home/medical/rs_lidar/src/FAST_LIO/PCD/test.pcd',
            'use_global_localization': use_global_localization,
            'use_imu': use_imu,
            'invert_imu_acc': invert_imu_acc,
            'invert_imu_gyro': invert_imu_gyro,
            'log_output_frequency_hz': log_output_frequency_hz,
            'send_tf_transforms': 'true',
            'publish_base_to_lidar_tf': 'false',
            'specify_init_pose': 'false',
        }.items(),
    )

    static_rslidar_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rslidar_to_base_link_tf',
        arguments=[
            '--x', rslidar_to_base_link_xyz[0],
            '--y', rslidar_to_base_link_xyz[1],
            '--z', rslidar_to_base_link_xyz[2],
            '--roll', rslidar_to_base_link_rpy[0],
            '--pitch', rslidar_to_base_link_rpy[1],
            '--yaw', rslidar_to_base_link_rpy[2],
            '--frame-id', 'rslidar',
            '--child-frame-id', 'base_link',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/link1'),
        DeclareLaunchArgument('odom_topic', default_value='/hdl/odom'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_global_localization', default_value='true'),
        DeclareLaunchArgument('invert_imu_acc', default_value='true'),
        DeclareLaunchArgument('invert_imu_gyro', default_value='false'),
        DeclareLaunchArgument('log_output_frequency_hz', default_value='1.0'),
        static_rslidar_to_base_link_tf,
        include_hdl_localization,
    ])
