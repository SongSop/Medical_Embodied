#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        self.current_distance_error = float("nan")
        self.current_lateral_error = float("nan")
        self.current_angular_error = float("nan")
        self.last_docking_end_monotonic = None

        self.target_distance = self._declare_float("target_distance", 0.315)
        self.horizontal_tolerance = self._declare_float("horizontal_tolerance", 0.03)
        self.angular_tolerance = self._declare_float("angular_tolerance", 0.05)
        self.linear_max_speed = self._declare_float("linear_max_speed", 0.1)
        self.angular_max_speed = self._declare_float("angular_max_speed", 0.3)
        self.search_angular_speed = self._declare_float("search_angular_speed", 0.25)
        self.max_docking_duration_sec = self._declare_float("max_docking_duration_sec", 120.0)
        self.tag_timeout_sec = self._declare_float("tag_timeout_sec", 1.0)
        self.max_tag_lost_sec = self._declare_float("max_tag_lost_sec", 10.0)
        self.tag_startup_grace_sec = self._declare_float("tag_startup_grace_sec", 5.0)
        self.max_retry_count = self._declare_int("max_retry_count", 5)
        self.done_stable_cycles = self._declare_int("done_stable_cycles", 8)
        self.stop_publish_count = self._declare_int("stop_publish_count", 5)
        self.tag_timeout = Duration(seconds=self.tag_timeout_sec)
        self.max_tag_lost = Duration(seconds=self.max_tag_lost_sec)
        self.tag_startup_grace = Duration(seconds=self.tag_startup_grace_sec)
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
        self.retry_stall_linear_speed_threshold = self._declare_float(
            "retry_stall_linear_speed_threshold", 0.01
        )
        self.retry_stall_cycles = self._declare_int("retry_stall_cycles", 8)
        self.pre_align_distance_margin = self._declare_float("pre_align_distance_margin", 0.2)
        self.pre_align_linear_max_speed = self._declare_float("pre_align_linear_max_speed", 0.04)
        self.final_approach_distance_window = self._declare_float("final_approach_distance_window", 0.2)
        self.final_approach_linear_max_speed = self._declare_float("final_approach_linear_max_speed", 0.05)
        self.pre_align_lateral_gain = self._declare_float("pre_align_lateral_gain", 1.0)
        self.final_approach_lateral_gain = self._declare_float("final_approach_lateral_gain", 2.0)
        self.lateral_lookahead_min = self._declare_float("lateral_lookahead_min", 0.05)
        self.lateral_direct_k = self._declare_float("lateral_direct_k", 1.2)
        self.pre_align_lateral_direct_scale = self._declare_float("pre_align_lateral_direct_scale", 0.8)
        self.final_approach_lateral_direct_scale = self._declare_float(
            "final_approach_lateral_direct_scale", 1.8
        )
        self.lateral_direct_max = self._declare_float("lateral_direct_max", 0.12)
        self.was_pre_align_active = False
        self.stall_retry_counter = 0

        self.base_frame = self._declare_str("base_frame", "base_link")
        self.tag_frame = self._declare_str("tag_frame", "dock_frame")
        self.control_period = self._declare_float("control_period", 0.1)
        self.error_topic = self._declare_str("error_topic", "/docking/current_error")
        self.error_publish_hz = self._declare_float("error_publish_hz", 20.0)
        self.docking_restart_cooldown_sec = self._declare_float("docking_restart_cooldown_sec", 5.0)
        self.state_publish_hz = self._declare_float("state_publish_hz", 2.0)
        self.terminal_state_hold_sec = self._declare_float("terminal_state_hold_sec", 3.0)
        self._terminal_state_until_monotonic = None

        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel_safe", 10)
        self.state_pub = self.create_publisher(String, "/docking/state", state_qos)
        self.error_pub = self.create_publisher(Vector3Stamped, self.error_topic, 10)
        self.create_subscription(String, "/dock/control_cmd", self.control_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(self.control_period, self.control_loop)
        self.create_timer(1.0 / max(self.error_publish_hz, 1.0), self.publish_current_error_timer)
        self.create_timer(1.0 / max(self.state_publish_hz, 0.5), self.publish_state_timer)

        self.get_logger().info(
            f"qr_docking_node started, tracking tf {self.base_frame} -> {self.tag_frame}, "
            f"error_topic={self.error_topic}, error_publish_hz={self.error_publish_hz}"
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

    def _wait_for_restart_cooldown(self) -> None:
        if self.last_docking_end_monotonic is None:
            return
        remaining = self.docking_restart_cooldown_sec - (
            time.monotonic() - self.last_docking_end_monotonic
        )
        if remaining <= 0.0:
            return
        self.get_logger().info(
            "Docking restart cooldown: waiting %.1fs before start" % remaining
        )
        time.sleep(remaining)

    def control_callback(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == "start":
            self._wait_for_restart_cooldown()
            if self.is_running:
                self.get_logger().warn("Docking is running, restarting docking flow")
            self.start_docking()
            self.get_logger().info("Docking started")
        elif command == "stop":
            self.stop_docking(self.ABORTED)
            self.get_logger().info("Docking stopped")
        self.publish_state()

    def _is_terminal_state(self, state: str) -> bool:
        return state in {self.COMPLETED, self.FAILED, self.ABORTED}

    def _mark_terminal_state(self, state: str) -> None:
        if self._is_terminal_state(state):
            self._terminal_state_until_monotonic = (
                time.monotonic() + max(self.terminal_state_hold_sec, 0.0)
            )

    def publish_state(self) -> None:
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

    def publish_state_timer(self) -> None:
        now = time.monotonic()
        if (
            self._terminal_state_until_monotonic is not None
            and now >= self._terminal_state_until_monotonic
            and self._is_terminal_state(self.current_state)
            and not self.is_docking
        ):
            self._terminal_state_until_monotonic = None
            self.current_state = self.IDLE
        self.publish_state()

    def publish_current_error(
        self, distance_error: float, lateral_error: float, angular_error: float
    ) -> None:
        self.current_distance_error = float(distance_error)
        self.current_lateral_error = float(lateral_error)
        self.current_angular_error = float(angular_error)

    def publish_current_error_timer(self) -> None:
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # x=dx, y=dy, z=dyaw
        msg.vector.x = self.current_distance_error
        msg.vector.y = self.current_lateral_error
        msg.vector.z = self.current_angular_error
        self.error_pub.publish(msg)

    def get_tag_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.tag_frame, Time())
            now = self.get_clock().now()
            transform_time = Time.from_msg(transform.header.stamp)
            # Prevent reusing stale TF from previous docking rounds.
            if transform_time.nanoseconds > 0 and (now - transform_time) > self.tag_timeout:
                self.tag_visible = False
                self.get_logger().debug(
                    "Ignoring stale tag transform, age=%.3fs"
                    % ((now - transform_time).nanoseconds / 1e9)
                )
                return None

            self.tag_visible = True
            self.last_tag_time = transform_time if transform_time.nanoseconds > 0 else now
            return transform
        except TransformException as exc:
            if (self.get_clock().now() - self.last_tag_time) > self.tag_timeout:
                self.tag_visible = False
            self.get_logger().debug(f"Cannot get tag transform: {exc}")
            return None

    def start_docking(self) -> None:
        self._terminal_state_until_monotonic = None
        self.is_running = True
        self.is_docking = True
        self.is_retreating = False
        self.was_pre_align_active = False
        self.retry_count = 0
        self.completed_stable_count = 0
        self.stall_retry_counter = 0
        now = self.get_clock().now()
        self.start_time = now
        self.last_valid_error_time = now
        self.current_state = self.SEARCHING
        self.reset_pid_errors()
        self.publish_state()

    def reset_pid_errors(self) -> None:
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.last_linear_error = 0.0
        self.last_angular_error = 0.0

    def stop_docking(self, state: str = IDLE) -> None:
        self.is_running = False
        self.is_docking = False
        self.is_retreating = False
        self.current_state = state
        self._mark_terminal_state(state)
        self.stall_retry_counter = 0
        self.last_docking_end_monotonic = time.monotonic()
        self.publish_current_error(float("nan"), float("nan"), float("nan"))
        self.send_stop_command()
        self.publish_state()

    def fail_docking(self, reason: str) -> None:
        self.get_logger().error(f"Docking failed: {reason}")
        self.stop_docking(self.FAILED)

    def start_retreating(self) -> None:
        self.is_retreating = True
        self.was_pre_align_active = False
        self.retreat_start_time = self.get_clock().now()
        self.retry_count += 1
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.stall_retry_counter = 0
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

    @staticmethod
    def _target_sign(target_distance: float, current_distance: float) -> float:
        if abs(target_distance) > 1e-6:
            return math.copysign(1.0, target_distance)
        if abs(current_distance) > 1e-6:
            return math.copysign(1.0, current_distance)
        return 1.0

    def log_status_line(
        self,
        current_distance: float,
        lateral_error: float,
        angular_error: float,
        distance_error: float,
    ) -> None:
        self.get_logger().info(
            "state=%s retry=%d/%d current(x=%.3f,y=%.3f,yaw=%.3f) "
            "target(x=%.3f,y=0.000,yaw=0.000) error(dx=%.3f,dy=%.3f,dyaw=%.3f)"
            % (
                self.current_state,
                self.retry_count,
                self.max_retry_count,
                current_distance,
                lateral_error,
                angular_error,
                self.target_distance,
                distance_error,
                lateral_error,
                angular_error,
            )
        )

    def control_loop(self) -> None:
        if not self.is_docking:
            return
        self.publish_state()

        now = self.get_clock().now()
        if self.start_time is not None and (now - self.start_time) > self.max_docking_duration:
            self.fail_docking("overall docking timeout")
            return

        transform = self.get_tag_transform()
        if transform is None or not self.tag_visible:
            self.was_pre_align_active = False
            self.current_state = self.SEARCHING
            self.publish_current_error(float("nan"), float("nan"), float("nan"))
            self.log_status_line(float("nan"), float("nan"), float("nan"), float("nan"))
            # Allow apriltag detector startup latency at docking begin.
            if self.start_time is not None and (now - self.start_time) <= self.tag_startup_grace:
                self.publish_zero_velocity()
                return
            if (now - self.last_valid_error_time) > self.max_tag_lost:
                self.fail_docking("tag lost for too long")
                return
            # Keep still when tag is lost, avoid blind rotation that may worsen detection.
            self.publish_zero_velocity()
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
        self.publish_current_error(distance_error, lateral_error, angular_error)

        if self.is_retreating:
            self.log_status_line(current_distance, lateral_error, angular_error, distance_error)
            if (self.get_clock().now() - self.retreat_start_time) > self.retreat_duration:
                self.is_retreating = False
                self.get_logger().info("Forward retry finished, resume docking")
                return

            # Retreat correction only targets lateral offset.
            # Because retreat moves opposite to docking direction, lateral sign follows retreat direction.
            target_sign = self._target_sign(self.target_distance, current_distance)
            retreat_direction_sign = -target_sign
            lateral_correction = retreat_direction_sign * lateral_error * self.retreat_lateral_factor
            lateral_correction = max(
                -self.angular_max_speed / 2.0, min(self.angular_max_speed / 2.0, lateral_correction)
            )

            cmd = Twist()
            # Retreat direction should be opposite to docking direction, compatible with +/- target_distance.
            cmd.linear.x = -target_sign * self.retreat_speed
            cmd.angular.z = lateral_correction
            self.cmd_vel_pub.publish(cmd)
            return

        if abs(lateral_error) > self.horizontal_tolerance or abs(angular_error) > self.angular_tolerance:
            self.current_state = self.ALIGNING
        else:
            self.current_state = self.APPROACHING

        if (
            abs(current_distance) < self.retry_threshold
            and abs(lateral_error) > self.horizontal_tolerance
            and not self.is_retreating
        ):
            if self.retry_count >= self.max_retry_count:
                self.fail_docking("retry count exceeded")
                return
            self.start_retreating()
            self.log_status_line(current_distance, lateral_error, angular_error, distance_error)
            return

        target_sign = self._target_sign(self.target_distance, current_distance)
        distance_to_target = target_sign * distance_error
        pre_align_active = (
            distance_to_target > self.pre_align_distance_margin
            and (abs(lateral_error) > self.horizontal_tolerance or abs(angular_error) > self.angular_tolerance)
        )
        if pre_align_active:
            self.current_state = self.ALIGNING
            # Keep a safety margin first, then enter final docking approach.
            linear_error = distance_error - target_sign * self.pre_align_distance_margin
        else:
            linear_error = distance_error
            if self.was_pre_align_active:
                # Avoid velocity jump when switching from pre-align to final approach.
                self.reset_pid_errors()
        self.was_pre_align_active = pre_align_active

        self.linear_error_sum += linear_error
        self.linear_error_sum = max(-self.linear_error_sum_max, min(self.linear_error_sum_max, self.linear_error_sum))
        linear_p = self.linear_kp * linear_error
        linear_i = self.linear_ki * self.linear_error_sum
        linear_d = self.linear_kd * (linear_error - self.last_linear_error)
        linear_velocity = linear_p + linear_i + linear_d
        # Forward adjustment uses proportional-only control.
        forward_adjust = linear_velocity > 0.0
        if forward_adjust:
            linear_velocity = linear_p
        self.last_linear_error = linear_error

        if pre_align_active:
            lateral_heading_error = math.atan2(lateral_error, max(abs(current_distance), 1e-6))
            lateral_gain = self.pre_align_lateral_gain
        else:
            lateral_heading_error = math.atan2(
                lateral_error, max(abs(distance_to_target), self.lateral_lookahead_min)
            )
            lateral_gain = self.final_approach_lateral_gain
        # Lateral correction direction must follow docking motion direction.
        # For reverse docking (negative target_sign), lateral->yaw mapping should be inverted.
        directional_lateral_error = target_sign * lateral_heading_error
        effective_angular_error = angular_error + lateral_gain * directional_lateral_error
        self.angular_error_sum += effective_angular_error
        self.angular_error_sum = max(
            -self.angular_error_sum_max, min(self.angular_error_sum_max, self.angular_error_sum)
        )
        angular_p = self.angular_kp * effective_angular_error
        angular_i = self.angular_ki * self.angular_error_sum
        angular_d = self.angular_kd * (effective_angular_error - self.last_angular_error)
        if forward_adjust:
            angular_velocity = angular_p
        else:
            angular_velocity = angular_p + angular_i + angular_d
        self.last_angular_error = effective_angular_error

        if pre_align_active:
            lateral_direct_scale = self.pre_align_lateral_direct_scale
        else:
            lateral_direct_scale = self.final_approach_lateral_direct_scale
        lateral_direct_correction = (
            self.lateral_direct_k * lateral_direct_scale * target_sign * lateral_error
        )
        lateral_direct_correction = max(
            -self.lateral_direct_max, min(self.lateral_direct_max, lateral_direct_correction)
        )
        angular_velocity += lateral_direct_correction

        linear_velocity = max(-self.linear_max_speed, min(self.linear_max_speed, linear_velocity))
        angular_velocity = max(-self.angular_max_speed, min(self.angular_max_speed, angular_velocity))
        if pre_align_active:
            linear_velocity = max(
                -self.pre_align_linear_max_speed,
                min(self.pre_align_linear_max_speed, linear_velocity),
            )
        elif distance_to_target <= self.final_approach_distance_window:
            linear_velocity = max(
                -self.final_approach_linear_max_speed,
                min(self.final_approach_linear_max_speed, linear_velocity),
            )

        # If lateral error stays large while linear speed stalls near zero,
        # proactively trigger another retry maneuver instead of idling in place.
        stalled_with_lateral_error = (
            abs(lateral_error) > self.horizontal_tolerance
            and abs(linear_velocity) < self.retry_stall_linear_speed_threshold
            and not self.is_retreating
        )
        if stalled_with_lateral_error:
            self.stall_retry_counter += 1
        else:
            self.stall_retry_counter = 0
        if self.stall_retry_counter >= max(self.retry_stall_cycles, 1):
            if self.retry_count >= self.max_retry_count:
                self.fail_docking("retry count exceeded (stall with lateral error)")
                return
            self.start_retreating()
            self.log_status_line(current_distance, lateral_error, angular_error, distance_error)
            return

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
