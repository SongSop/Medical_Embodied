#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class ManualInitialPosePublisher(Node):
    def __init__(self):
        super().__init__("manual_initial_pose_publisher")

        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("covariance_x", 0.25)
        self.declare_parameter("covariance_y", 0.25)
        self.declare_parameter("covariance_yaw", 0.06853891945200942)
        self.declare_parameter("publish_delay", 2.0)
        self.declare_parameter("publish_once", True)

        topic = self.get_parameter("initial_pose_topic").value
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, topic, 10)

        self.published_ = False
        delay = float(self.get_parameter("publish_delay").value)
        self.timer_ = self.create_timer(delay, self.publish_initial_pose)

        self.get_logger().info(
            "Manual initial pose publisher ready: topic=%s, frame=%s, pose=(%.3f, %.3f, %.3f)"
            % (
                topic,
                self.get_parameter("frame_id").value,
                float(self.get_parameter("x").value),
                float(self.get_parameter("y").value),
                float(self.get_parameter("yaw").value),
            )
        )

    def publish_initial_pose(self):
        if self.published_ and bool(self.get_parameter("publish_once").value):
            return

        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        z = float(self.get_parameter("z").value)
        yaw = float(self.get_parameter("yaw").value)

        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        covariance_x = float(self.get_parameter("covariance_x").value)
        covariance_y = float(self.get_parameter("covariance_y").value)
        covariance_yaw = float(self.get_parameter("covariance_yaw").value)
        msg.pose.covariance[0] = covariance_x
        msg.pose.covariance[7] = covariance_y
        msg.pose.covariance[35] = covariance_yaw

        self.publisher_.publish(msg)
        self.published_ = True

        self.get_logger().info(
            "Published initial pose to %s: x=%.3f, y=%.3f, yaw=%.3f"
            % (self.get_parameter("initial_pose_topic").value, x, y, yaw)
        )

        if bool(self.get_parameter("publish_once").value):
            self.timer_.cancel()


def main():
    rclpy.init()
    node = ManualInitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
