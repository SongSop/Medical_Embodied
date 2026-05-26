#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class QrDockingNode(Node):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING_TAG"
    ALIGNING = "ALIGNING"
    APPROACHING = "APPROACHING"
    RETRYING = "RETRYING"
    COMPLETED = "DOCKING_COMPLETED"
    FAILED = "DOCKING_FAILED"
    ABORTED = "ABORTED"

    def __init__(self) -> None:
        super().__init__("qr_docking_node")

        self.is_running = False
        self.current_state = self.IDLE
        self.is_docking = False
        self.tag_visible = False
        self.last_tag_time = self.get_clock().now()
        self.last_valid_error_time = self.get_clock().now()
        self.start_time = None
        self.completed_stable_count = 0
        self.retry_count = 0
        self.is_retreating = False
        self.retreat_start_time = None

        self.target_distance = self._declare_float("target_distance", 0.315)
        self.horizontal_tolerance = self._declare_float("horizontal_tolerance", 0.03)
        self.angular_tolerance = self._declare_float("angular_tolerance", 0.05)
        self.linear_max_speed = self._declare_float("linear_max_speed", 0.1)
        self.angular_max_speed = self._declare_float("angular_max_speed", 0.3)
        self.search_angular_speed = self._declare_float("search_angular_speed", 0.25)
        self.max_docking_duration_sec = self._declare_float("max_docking_duration_sec", 120.0)
        self.tag_timeout_sec = self._declare_float("tag_timeout_sec", 1.0)
        self.max_tag_lost_sec = self._declare_float("max_tag_lost_sec", 10.0)
        self.max_retry_count = self._declare_int("max_retry_count", 5)
        self.done_stable_cycles = self._declare_int("done_stable_cycles", 8)
        self.stop_publish_count = self._declare_int("stop_publish_count", 5)
        self.tag_timeout = Duration(seconds=self.tag_timeout_sec)
        self.max_tag_lost = Duration(seconds=self.max_tag_lost_sec)
        self.max_docking_duration = Duration(seconds=self.max_docking_duration_sec)

        self.linear_kp = self._declare_float("linear_kp", 0.5)
        self.linear_ki = self._declare_float("linear_ki", 0.0)
        self.linear_kd = self._declare_float("linear_kd", 0.1)
        self.angular_kp = self._declare_float("angular_kp", 1.0)
        self.angular_ki = self._declare_float("angular_ki", 0.0)
        self.angular_kd = self._declare_float("angular_kd", 0.1)

        self.linear_error_sum_max = self._declare_float("linear_error_sum_max", 1.0)
        self.angular_error_sum_max = self._declare_float("angular_error_sum_max", 1.0)
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.last_linear_error = 0.0
        self.last_angular_error = 0.0

        self.retry_threshold = self._declare_float("retry_threshold", 0.5)
        self.retreat_duration = Duration(seconds=self._declare_float("retreat_duration_sec", 3.0))
        self.retreat_speed = self._declare_float("retreat_speed", 0.1)
        self.retreat_angular_factor = self._declare_float("retreat_angular_factor", 1.5)
        self.retreat_lateral_factor = self._declare_float("retreat_lateral_factor", 2.0)

        self.base_frame = self._declare_str("base_frame", "base_link")
        self.tag_frame = self._declare_str("tag_frame", "dock_frame")
        self.control_period = self._declare_float("control_period", 0.1)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel_safe", 10)
        self.state_pub = self.create_publisher(String, "/docking/state", 10)
        self.create_subscription(String, "/dock/control_cmd", self.control_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(
            f"qr_docking_node started, tracking tf {self.base_frame} -> {self.tag_frame}"
        )

    def _declare_float(self, name: str, default: float) -> float:
        self.declare_parameter(name, default)
        return float(self.get_parameter(name).value)

    def _declare_str(self, name: str, default: str) -> str:
        self.declare_parameter(name, default)
        return str(self.get_parameter(name).value)

    def _declare_int(self, name: str, default: int) -> int:
        self.declare_parameter(name, default)
        return int(self.get_parameter(name).value)

    def control_callback(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == "start" and not self.is_running:
            self.start_docking()
            self.get_logger().info("Docking started")
        elif command == "stop":
            self.stop_docking(self.ABORTED)
            self.get_logger().info("Docking stopped")
        self.publish_state()

    def publish_state(self) -> None:
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

    def get_tag_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.tag_frame, Time())
            self.tag_visible = True
            self.last_tag_time = self.get_clock().now()
            return transform
        except TransformException as exc:
            if (self.get_clock().now() - self.last_tag_time) > self.tag_timeout:
                self.tag_visible = False
            self.get_logger().debug(f"Cannot get tag transform: {exc}")
            return None

    def start_docking(self) -> None:
        self.is_running = True
        self.is_docking = True
        self.is_retreating = False
        self.retry_count = 0
        self.completed_stable_count = 0
        now = self.get_clock().now()
        self.start_time = now
        self.last_valid_error_time = now
        self.current_state = self.SEARCHING
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.last_linear_error = 0.0
        self.last_angular_error = 0.0
        self.publish_state()

    def stop_docking(self, state: str = IDLE) -> None:
        self.is_running = False
        self.is_docking = False
        self.is_retreating = False
        self.current_state = state
        self.send_stop_command()
        self.publish_state()

    def fail_docking(self, reason: str) -> None:
        self.get_logger().error(f"Docking failed: {reason}")
        self.stop_docking(self.FAILED)

    def start_retreating(self) -> None:
        self.is_retreating = True
        self.retreat_start_time = self.get_clock().now()
        self.retry_count += 1
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.current_state = self.RETRYING
        self.get_logger().warn(f"Start retry maneuver ({self.retry_count}/{self.max_retry_count})")

    def publish_zero_velocity(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def send_stop_command(self) -> None:
        cmd = Twist()
        for _ in range(max(self.stop_publish_count, 1)):
            self.cmd_vel_pub.publish(cmd)

    def send_search_command(self) -> None:
        cmd = Twist()
        cmd.angular.z = -self.search_angular_speed
        self.cmd_vel_pub.publish(cmd)

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def log_status_line(
        self,
        current_distance: float,
        lateral_error: float,
        angular_error: float,
        distance_error: float,
    ) -> None:
        abs_distance_error = abs(distance_error)
        abs_lateral_error = abs(lateral_error)
        abs_angular_error = abs(angular_error)
        self.get_logger().info(
            "state=%s retry=%d/%d current(x=%.3f,y=%.3f,yaw=%.3f) "
            "target(x=%.3f,y=0.000,yaw=0.000) error(|dx|=%.3f,|dy|=%.3f,|dyaw|=%.3f)"
            % (
                self.current_state,
                self.retry_count,
                self.max_retry_count,
                current_distance,
                lateral_error,
                angular_error,
                self.target_distance,
                abs_distance_error,
                abs_lateral_error,
                abs_angular_error,
            )
        )

    def control_loop(self) -> None:
        self.publish_state()
        if not self.is_docking:
            return

        now = self.get_clock().now()
        if self.start_time is not None and (now - self.start_time) > self.max_docking_duration:
            self.fail_docking("overall docking timeout")
            return

        transform = self.get_tag_transform()
        if transform is None or not self.tag_visible:
            self.current_state = self.SEARCHING
            self.log_status_line(float("nan"), float("nan"), float("nan"), float("nan"))
            if (now - self.last_valid_error_time) > self.max_tag_lost:
                self.fail_docking("tag lost for too long")
                return
            self.send_search_command()
            return

        # Plan in base_link frame:
        #   forward/backward -> x axis
        #   lateral          -> y axis
        current_distance = transform.transform.translation.x
        lateral_error = transform.transform.translation.y

        q = transform.transform.rotation
        angular_error = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.last_valid_error_time = now
        distance_error = current_distance - self.target_distance

        if self.is_retreating:
            self.log_status_line(current_distance, lateral_error, angular_error, distance_error)
            if (self.get_clock().now() - self.retreat_start_time) > self.retreat_duration:
                self.is_retreating = False
                self.get_logger().info("Forward retry finished, resume docking")
                return

            lateral_correction = lateral_error * self.retreat_lateral_factor
            angular_correction = -angular_error * self.retreat_angular_factor
            lateral_correction = max(
                -self.angular_max_speed / 2.0, min(self.angular_max_speed / 2.0, lateral_correction)
            )
            angular_correction = max(
                -self.angular_max_speed / 2.0, min(self.angular_max_speed / 2.0, angular_correction)
            )

            cmd = Twist()
            # Retreat direction should be opposite to docking direction, compatible with +/- target_distance.
            if abs(self.target_distance) > 1e-6:
                target_sign = math.copysign(1.0, self.target_distance)
            elif abs(current_distance) > 1e-6:
                target_sign = math.copysign(1.0, current_distance)
            else:
                target_sign = 1.0
            cmd.linear.x = -target_sign * self.retreat_speed
            cmd.angular.z = lateral_correction + angular_correction
            self.cmd_vel_pub.publish(cmd)
            return

        if abs(lateral_error) > self.horizontal_tolerance or abs(angular_error) > self.angular_tolerance:
            self.current_state = self.ALIGNING
        else:
            self.current_state = self.APPROACHING

        if (
            abs(current_distance) < self.retry_threshold
            and (abs(lateral_error) > self.horizontal_tolerance or abs(angular_error) > self.angular_tolerance)
            and not self.is_retreating
        ):
            if self.retry_count >= self.max_retry_count:
                self.fail_docking("retry count exceeded")
                return
            self.start_retreating()
            self.log_status_line(current_distance, lateral_error, angular_error, distance_error)
            return

        self.linear_error_sum += distance_error
        self.linear_error_sum = max(-self.linear_error_sum_max, min(self.linear_error_sum_max, self.linear_error_sum))
        linear_p = self.linear_kp * distance_error
        linear_i = self.linear_ki * self.linear_error_sum
        linear_d = self.linear_kd * (distance_error - self.last_linear_error)
        linear_velocity = linear_p + linear_i + linear_d
        self.last_linear_error = distance_error

        effective_angular_error = angular_error + math.atan2(lateral_error, max(abs(current_distance), 1e-6))
        self.angular_error_sum += effective_angular_error
        self.angular_error_sum = max(
            -self.angular_error_sum_max, min(self.angular_error_sum_max, self.angular_error_sum)
        )
        angular_p = self.angular_kp * effective_angular_error
        angular_i = self.angular_ki * self.angular_error_sum
        angular_d = self.angular_kd * (effective_angular_error - self.last_angular_error)
        angular_velocity = angular_p + angular_i + angular_d
        self.last_angular_error = effective_angular_error

        linear_velocity = max(-self.linear_max_speed, min(self.linear_max_speed, linear_velocity))
        angular_velocity = max(-self.angular_max_speed, min(self.angular_max_speed, angular_velocity))

        done1 = (
            abs(distance_error) < 0.01
            and abs(lateral_error) < self.horizontal_tolerance
            and abs(angular_error) < self.angular_tolerance
        )
        done2 = (
            abs(distance_error) < 0.02
            and abs(lateral_error) < self.horizontal_tolerance
            and abs(angular_error) < self.angular_tolerance
            and abs(linear_velocity) < 0.01
        )
        if done1 or done2:
            self.completed_stable_count += 1
            if self.completed_stable_count >= self.done_stable_cycles:
                self.current_state = self.COMPLETED
                self.publish_state()
                self.stop_docking(self.COMPLETED)
                self.get_logger().info("Docking completed with stable convergence")
            else:
                self.publish_zero_velocity()
            return
        self.completed_stable_count = 0

        cmd = Twist()
        # Keep linear command sign consistent with distance_error:
        # target behind (negative) -> usually command negative (reverse),
        # only when too close (error > 0) command becomes positive (forward).
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd)
        self.log_status_line(current_distance, lateral_error, angular_error, distance_error)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QrDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
