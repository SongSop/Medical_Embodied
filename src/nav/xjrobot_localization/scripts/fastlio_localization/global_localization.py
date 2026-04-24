#!/usr/bin/env python3

import copy
import logging
import sys
import time

import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
# from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import tf2_ros
import tf_transformations
import ros2_numpy

logging.getLogger("ros2_numpy.point_cloud2").setLevel(logging.ERROR)


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")
        self.global_map = None
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        self.initialized = False
        self.last_status_log_time = 0.0
        self.scan_count = 0
        self.odom_count = 0
        self.last_scan_frame_id = "n/a"
        self.last_odom_frame_id = "n/a"
        self.last_odom_child_frame_id = "n/a"
        self.last_scan_points = 0
        self.last_submap_points = 0
        self.last_fitness = None
        self.last_map_to_odom_xyz = None
        self.last_initial_pose_xyz = None
        self.last_status = "starting"
        self.last_warning = "none"
        self.last_error = "none"
        self.map_publish_count = 0
        self.map_to_odom_publish_count = 0

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("freq_global_map", 0.25),
                ("localization_threshold", 0.8),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
                ("odom_topic", "/odom_fastlio"),
                ("map_frame", "map_fastlio"),
                ("status_print", True),
                ("status_clear_screen", True),
                ("status_print_period", 1.0),
                ("coarse_correspondence_distance", 15.0),
                ("fine_correspondence_distance", 3.0),
            ],
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_global_map = self.create_publisher(PointCloud2, self.get_parameter("pcd_map_topic").value, 10)
        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)

        self.get_logger().info(f"Loading global map from: {self.get_parameter('pcd_map_path').value}")
        self.initialize_global_map()
        self.get_logger().info("Global map loaded.")
        
        self.create_subscription(PointCloud2, "/cloud_registered", self.cb_save_cur_scan, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.cb_save_cur_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.cb_initialize_pose, 10)

        self.timer_localisation = self.create_timer(1.0 / self.get_parameter("freq_localization").value, self.localisation_timer_callback)
        self.timer_global_map = self.create_timer(1.0 / self.get_parameter("freq_global_map").value, self.global_map_callback)
        self.timer_status = self.create_timer(self.get_parameter("status_print_period").value, self.status_timer_callback)

    def global_map_callback(self):
        if self.global_map is None:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter("map_frame").value
        self.publish_point_cloud(self.pub_global_map, header, np.array(self.global_map.points))
        self.map_publish_count += 1
        
    def pose_to_mat(self, pose):
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans
    
    def msg_to_array(self, pc_msg):
        pc_array = ros2_numpy.numpify(pc_msg)
        xyz = pc_array["xyz"]
        finite_mask = np.isfinite(xyz).all(axis=1)
        return xyz[finite_mask], int(np.size(finite_mask) - np.count_nonzero(finite_mask))
    
    def registration_at_scale(self, scan, map, initial, scale):
        if scale >= 5:
            max_correspondence_distance = self.get_parameter("coarse_correspondence_distance").value
        else:
            max_correspondence_distance = self.get_parameter("fine_correspondence_distance").value
        result_icp = o3d.pipelines.registration.registration_icp(
        self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale),
        self.voxel_down_sample(map, self.get_parameter("map_voxel_size").value * scale),
        max_correspondence_distance,
        initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        return result_icp.transformation, result_icp.fitness
            
    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def describe_points(self, points):
        if points is None or len(points) == 0:
            return "empty"
        mins = np.min(points, axis=0)
        maxs = np.max(points, axis=0)
        center = np.mean(points, axis=0)
        return (
            f"center=({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}), "
            f"min=({mins[0]:.3f}, {mins[1]:.3f}, {mins[2]:.3f}), "
            f"max=({maxs[0]:.3f}, {maxs[1]:.3f}, {maxs[2]:.3f})"
        )

    def transform_points(self, points, transform):
        if points is None or len(points) == 0:
            return np.empty((0, 3))
        homo = np.column_stack([points, np.ones(len(points))])
        return (transform @ homo.T).T[:, :3]

    def publish_point_cloud(self, publisher, header, pc):
        data = dict()
        data["xyz"] = pc[:, :3]
        
        if pc.shape[1] == 4:
            data["intensity"] = pc[:, 3]
        # else:
            # data["rgb"] = np.ones_like(pc)
        msg = ros2_numpy.msgify(PointCloud2, data)
        msg.header = header
        if len(msg.fields) == 4:
            msg.point_step = 16
        else:
            msg.point_step = 12
            
        publisher.publish(msg)
        
    def crop_global_map_in_FOV(self, pose_estimation):
        T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        if self.get_parameter("fov").value > 3.14:
            indices = np.where(
                (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        else:
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0)
                & (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))
        self.last_submap_points = len(global_map_in_FOV.points)

        header = self.cur_odom.header
        header.frame_id = self.get_parameter("map_frame").value
        self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV

    def global_localization(self, pose_estimation):
        if self.cur_scan is None:
            self.last_warning = "waiting for /cloud_registered"
            self.get_logger().warn("Global localization skipped: no current scan available yet.")
            return
        if self.cur_odom is None:
            self.last_warning = f"waiting for {self.get_parameter('odom_topic').value}"
            self.get_logger().warn("Global localization skipped: no odometry received yet.")
            return
        if self.global_map is None or len(self.global_map.points) == 0:
            self.last_error = "global map is empty"
            self.get_logger().warn("Global localization skipped: global map is empty.")
            return

        self.last_status = "running_icp"
        guess_xyz = pose_estimation[:3, 3]
        self.get_logger().info(
            f"Running global localization. scan_points={len(self.cur_scan.points)}, "
            f"map_points={len(self.global_map.points)}"
        )
        self.get_logger().info(
            "Initial guess map_to_odom translation="
            f"({guess_xyz[0]:.3f}, {guess_xyz[1]:.3f}, {guess_xyz[2]:.3f})"
        )
        scan_tobe_mapped = copy.copy(self.cur_scan)
        scan_points = np.asarray(scan_tobe_mapped.points)
        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)
        self.get_logger().info(f"Submap points in FOV: {len(global_map_in_FOV.points)}")
        if len(global_map_in_FOV.points) == 0:
            self.last_warning = "submap in FOV is empty"
            self.get_logger().warn("Global localization skipped: submap in FOV is empty.")
            return

        transformed_scan_points = self.transform_points(scan_points, pose_estimation)
        submap_points = np.asarray(global_map_in_FOV.points)
        self.get_logger().info(
            "Transformed scan stats in map frame: "
            + self.describe_points(transformed_scan_points)
        )
        self.get_logger().info(
            "Submap stats in map frame: "
            + self.describe_points(submap_points)
        )
        
        transformation, coarse_fitness = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5
        )
        self.get_logger().info(f"Coarse global localization fitness={coarse_fitness:.4f}")

        transformation, fitness = self.registration_at_scale(
            scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1
        )
        self.last_fitness = float(fitness)
        self.get_logger().info(
            f"Global localization fitness={fitness:.4f}, "
            f"threshold={self.get_parameter('localization_threshold').value:.4f}"
        )
        
        if fitness > self.get_parameter("localization_threshold").value:
            self.T_map_to_odom = transformation
            xyz = transformation[:3, 3]
            self.last_map_to_odom_xyz = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
            self.last_status = "localized"
            self.last_warning = "none"
            self.get_logger().info(
                f"Global localization accepted. map_to_odom translation="
                f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})"
            )
            self.publish_odom(transformation)
        else:
            self.last_status = "fitness_below_threshold"
            self.last_warning = (
                f"fitness {fitness:.4f} below threshold "
                f"{self.get_parameter('localization_threshold').value:.4f}"
            )
            self.get_logger().warn(f"Fitness score {fitness} less than localization threshold {self.get_parameter('localization_threshold').value}")

    def voxel_down_sample(self, pcd, voxel_size):
        # print(pcd)
        
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        
        except Exception as e:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
            
        return pcd_down

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg
        self.odom_count += 1
        self.last_odom_frame_id = msg.header.frame_id
        self.last_odom_child_frame_id = msg.child_frame_id
        if self.odom_count == 1:
            self.get_logger().info(
                f"Received first odometry message on {self.get_parameter('odom_topic').value}. "
                f"frame_id={msg.header.frame_id}, child_frame_id={msg.child_frame_id}"
            )
        
    def cb_save_cur_scan(self, msg):
        pc, filtered_count = self.msg_to_array(msg)
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc)
        self.scan_count += 1
        self.last_scan_frame_id = msg.header.frame_id
        self.last_scan_points = len(pc)
        if filtered_count > 0:
            self.last_warning = f"filtered {filtered_count} invalid points from /cloud_registered"
            # self.get_logger().warn(
            #     f"Filtered {filtered_count} invalid points (NaN/Inf) from /cloud_registered."
            # )
        if self.scan_count == 1:
            self.get_logger().info(
                f"Received first registered scan on /cloud_registered with {len(pc)} points. "
                f"frame_id={msg.header.frame_id}"
            )
        self.publish_point_cloud(self.pub_pc_in_map, msg.header, pc)
        
    def initialize_global_map(self): #, pc_msg):
        map_path = self.get_parameter("pcd_map_path").value
        self.global_map = o3d.io.read_point_cloud(map_path)
        if self.global_map is None or len(self.global_map.points) == 0:
            self.last_error = f"failed to load map: {map_path}"
            self.get_logger().error(f"Failed to load map or map is empty: {map_path}")
            return
        self.global_map = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value)
        self.last_status = "map_ready"
        self.get_logger().info(f"Global map ready with {len(self.global_map.points)} points after downsampling.")

    def cb_initialize_pose(self, msg):
        self.initialized = True
        self.last_initial_pose_xyz = [
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        ]
        self.last_status = "initialized"
        self.get_logger().info(
            "Initial pose received. "
            f"x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}, "
            f"z={msg.pose.pose.position.z:.3f}"
        )

        if self.cur_scan is None:
            return
        if self.cur_odom is None:
            self.last_warning = f"initial pose received but waiting for {self.get_parameter('odom_topic').value}"
            self.get_logger().warn(
                f"Initial pose received but no odometry on {self.get_parameter('odom_topic').value} yet."
            )
            return

        # /initialpose from RViz is a pose of the robot body in map frame.
        # Convert it to the map->odom initial guess expected by this node:
        # T_map_to_odom = T_map_to_base * inverse(T_odom_to_base)
        T_map_to_base_link = self.pose_to_mat(msg.pose.pose)
        T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
        initial_map_to_odom = np.matmul(T_map_to_base_link, self.inverse_se3(T_odom_to_base_link))

        self.get_logger().info(
            "Converted initial pose from map->base to map->odom initial guess."
        )
        guess_xyz = initial_map_to_odom[:3, 3]
        self.get_logger().info(
            "Initial map_to_odom guess from RViz pose="
            f"({guess_xyz[0]:.3f}, {guess_xyz[1]:.3f}, {guess_xyz[2]:.3f})"
        )
        self.global_localization(initial_map_to_odom)
            
    def publish_odom(self, transform):
        odom_msg = Odometry()
        xyz = transform[:3, 3]
        quat = tf_transformations.quaternion_from_matrix(transform)
        odom_msg.pose.pose = Pose(
            position = Point(x = xyz[0], y = xyz[1], z = xyz[2]), 
            orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
        )
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.get_parameter("map_frame").value
        self.pub_map_to_odom.publish(odom_msg)
        self.map_to_odom_publish_count += 1
        self.get_logger().info("Published /map_to_odom.")

    def localisation_timer_callback(self):
        if not self.initialized:
            now = time.time()
            if now - self.last_status_log_time > 5.0:
                self.last_status_log_time = now
                self.last_status = "waiting_initial_pose"
                self.get_logger().info(
                    "Waiting for initial pose... "
                    f"odom_received={self.cur_odom is not None}, "
                    f"scan_received={self.cur_scan is not None}, "
                    f"map_ready={self.global_map is not None and len(self.global_map.points) > 0}"
                )
            return
        
        if self.cur_scan is not None:
            self.global_localization(self.T_map_to_odom)
        else:
            now = time.time()
            if now - self.last_status_log_time > 5.0:
                self.last_status_log_time = now
                self.last_status = "waiting_scan"
                self.last_warning = "initialized but waiting for /cloud_registered"
                self.get_logger().warn("Initialized but still waiting for /cloud_registered scan input.")

    def status_timer_callback(self):
        if not self.get_parameter("status_print").value:
            return

        if self.get_parameter("status_clear_screen").value:
            sys.stdout.write("\033[2J\033[H")

        map_ready = self.global_map is not None and len(self.global_map.points) > 0
        map_points = len(self.global_map.points) if map_ready else 0
        fitness_text = f"{self.last_fitness:.4f}" if self.last_fitness is not None else "n/a"
        map_to_odom_text = (
            f"({self.last_map_to_odom_xyz[0]:.3f}, {self.last_map_to_odom_xyz[1]:.3f}, {self.last_map_to_odom_xyz[2]:.3f})"
            if self.last_map_to_odom_xyz is not None
            else "n/a"
        )
        initial_pose_text = (
            f"({self.last_initial_pose_xyz[0]:.3f}, {self.last_initial_pose_xyz[1]:.3f}, {self.last_initial_pose_xyz[2]:.3f})"
            if self.last_initial_pose_xyz is not None
            else "n/a"
        )

        panel_lines = [
            "========== FAST_LIO_LOCALIZATION STATUS ==========",
            f"status: {self.last_status}",
            f"map_ready: {map_ready} | map_points: {map_points} | map_publish_count: {self.map_publish_count}",
            f"initialized: {self.initialized} | initial_pose: {initial_pose_text}",
            f"odom_topic: {self.get_parameter('odom_topic').value} | odom_received: {self.cur_odom is not None} | odom_count: {self.odom_count}",
            f"odom_frame: {self.last_odom_frame_id} -> {self.last_odom_child_frame_id}",
            f"scan_received: {self.cur_scan is not None} | scan_count: {self.scan_count} | scan_points: {self.last_scan_points} | scan_frame: {self.last_scan_frame_id}",
            f"submap_points: {self.last_submap_points} | last_fitness: {fitness_text}",
            f"map_to_odom_publish_count: {self.map_to_odom_publish_count} | map_to_odom_xyz: {map_to_odom_text}",
            f"last_warning: {self.last_warning}",
            f"last_error: {self.last_error}",
            "==================================================",
        ]
        sys.stdout.write("\n".join(panel_lines) + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
