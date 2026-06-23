#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuUnitConverter(Node):
    def __init__(self):
        super().__init__("imu_unit_converter")

        self.declare_parameter("input_topic", "/imu/link1")
        self.declare_parameter("output_topic", "/imu/link1_g")
        self.declare_parameter("conversion", "mps2_to_g")
        self.declare_parameter("gravity", 9.80665)
        self.declare_parameter("output_frame_id", "")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.conversion = self.get_parameter("conversion").value
        self.gravity = float(self.get_parameter("gravity").value)
        self.output_frame_id = self.get_parameter("output_frame_id").value

        if self.gravity <= 0.0:
            raise ValueError("gravity must be > 0.0")

        if self.conversion not in ("mps2_to_g", "g_to_mps2"):
            raise ValueError("conversion must be one of: mps2_to_g, g_to_mps2")

        self.sub = self.create_subscription(Imu, self.input_topic, self.imu_callback, 100)
        self.pub = self.create_publisher(Imu, self.output_topic, 100)

        self.get_logger().info(
            f"imu_unit_converter started: {self.input_topic} -> {self.output_topic}, "
            f"conversion={self.conversion}, gravity={self.gravity}"
        )

    def imu_callback(self, msg: Imu):
        out = copy.deepcopy(msg)

        if self.conversion == "mps2_to_g":
            scale = 1.0 / self.gravity
            cov_scale = 1.0 / (self.gravity * self.gravity)
        else:  # g_to_mps2
            scale = self.gravity
            cov_scale = self.gravity * self.gravity

        out.linear_acceleration.x *= scale
        out.linear_acceleration.y *= scale
        out.linear_acceleration.z *= scale

        for i in range(9):
            out.linear_acceleration_covariance[i] *= cov_scale

        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuUnitConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
