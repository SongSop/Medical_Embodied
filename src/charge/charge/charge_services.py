#!/usr/bin/env python3

import os
import signal
import subprocess
import time
from threading import Event

import rclpy
from interfaces.srv import ChargeUntil, Dock
from rclpy.node import Node
from std_msgs.msg import String


class ChargeServices(Node):
    TERMINAL_STATES = {"DOCKING_COMPLETED", "DOCKING_FAILED", "ABORTED"}
    ACTIVE_STATES = {"SEARCHING_TAG", "ALIGNING", "APPROACHING", "RETRYING"}

    def __init__(self):
        super().__init__("charge_services")
        self.state_timeout_sec = float(self.declare_parameter("state_timeout_sec", 5.0).value)
        self.enable_apriltag_on_demand = bool(
            self.declare_parameter("enable_apriltag_on_demand", True).value
        )
        self.apriltag_launch_package = str(
            self.declare_parameter("apriltag_launch_package", "charge").value
        )
        self.apriltag_launch_file = str(
            self.declare_parameter("apriltag_launch_file", "tag_realsense_node.launch.py").value
        )
        self.apriltag_camera_name = str(self.declare_parameter("apriltag_camera_name", "/camera").value)
        self.apriltag_image_topic = str(self.declare_parameter("apriltag_image_topic", "image_raw").value)
        self.apriltag_startup_delay_sec = float(
            self.declare_parameter("apriltag_startup_delay_sec", 1.0).value
        )
        self.apriltag_stop_timeout_sec = float(
            self.declare_parameter("apriltag_stop_timeout_sec", 3.0).value
        )
        self.apriltag_start_retry_count = int(
            self.declare_parameter("apriltag_start_retry_count", 2).value
        )
        self.apriltag_start_retry_delay_sec = float(
            self.declare_parameter("apriltag_start_retry_delay_sec", 0.8).value
        )

        self.last_docking_state = "UNKNOWN"
        self.state_event = Event()
        self.apriltag_process = None
        self.docking_session_active = False
        self.session_seen_active_state = False

        self.control_pub = self.create_publisher(String, "/dock/control_cmd", 10)
        self.create_subscription(String, "/dock/control_cmd", self._control_cmd_callback, 10)
        self.create_subscription(String, "/docking/state", self._state_callback, 10)
        self.create_service(Dock, "dock", self.handle_dock)
        self.create_service(ChargeUntil, "charge_until", self.handle_charge)
        self.get_logger().info("charge_services started")

    def _state_callback(self, msg: String) -> None:
        self.last_docking_state = msg.data.strip()
        self.state_event.set()
        if self.docking_session_active and self.last_docking_state in self.ACTIVE_STATES:
            self.session_seen_active_state = True
        if (
            self.enable_apriltag_on_demand
            and self.docking_session_active
            and self.session_seen_active_state
            and self.last_docking_state in self.TERMINAL_STATES
        ):
            self.docking_session_active = False
            self._stop_apriltag_detector()

    def _publish_control(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.control_pub.publish(msg)

    def _wait_for_state(self, accepted_states: set[str], timeout_sec: float) -> bool:
        self.state_event.clear()
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok():
            if self.last_docking_state in accepted_states:
                return True
            if self.get_clock().now().nanoseconds > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _prepare_new_docking_session(self) -> None:
        # Drop previous-round state so new start won't be short-circuited by stale terminal state.
        self.last_docking_state = "UNKNOWN"
        self.state_event.clear()
        self.docking_session_active = True
        self.session_seen_active_state = False

    def _control_cmd_callback(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if not self.enable_apriltag_on_demand:
            return
        if command == "start":
            # If this is our mirrored command during an active session, avoid
            # resetting state to prevent start-race side effects.
            if not self.docking_session_active:
                self._prepare_new_docking_session()
            if not self._start_apriltag_detector():
                self.docking_session_active = False
                self.get_logger().error("Received /dock/control_cmd start but apriltag failed to start")
        elif command == "stop":
            self.docking_session_active = False
            self._stop_apriltag_detector()

    def _build_apriltag_launch_cmd(self) -> list[str]:
        return [
            "ros2",
            "launch",
            self.apriltag_launch_package,
            self.apriltag_launch_file,
            f"camera_name:={self.apriltag_camera_name}",
            f"image_topic:={self.apriltag_image_topic}",
        ]

    def _wait_for_apriltag_startup_result(self) -> bool:
        if self.apriltag_startup_delay_sec > 0.0:
            time.sleep(self.apriltag_startup_delay_sec)

        if self.apriltag_process is None:
            return False
        if self.apriltag_process.poll() is not None:
            self.get_logger().error(
                "apriltag detector exited unexpectedly after startup, return_code=%s"
                % self.apriltag_process.poll()
            )
            self.apriltag_process = None
            return False
        return True

    def _retry_start_apriltag(self) -> bool:
        max_attempts = max(self.apriltag_start_retry_count + 1, 1)
        launch_cmd = self._build_apriltag_launch_cmd()
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(
                "Starting apriltag detector (attempt %d/%d): %s"
                % (attempt, max_attempts, " ".join(launch_cmd))
            )
            try:
                self.apriltag_process = subprocess.Popen(launch_cmd, start_new_session=True)
            except Exception as exc:
                self.get_logger().error(f"Failed to start apriltag detector: {exc}")
                self.apriltag_process = None
            else:
                if self._wait_for_apriltag_startup_result():
                    return True

            if attempt < max_attempts:
                time.sleep(max(self.apriltag_start_retry_delay_sec, 0.0))
        return False

    def _start_apriltag_detector(self) -> bool:
        if self.apriltag_process is not None and self.apriltag_process.poll() is None:
            return True

        return self._retry_start_apriltag()

    def _stop_apriltag_detector(self) -> None:
        if self.apriltag_process is None:
            return
        if self.apriltag_process.poll() is not None:
            self.apriltag_process = None
            return

        self.get_logger().info("Stopping apriltag detector")
        try:
            os.killpg(self.apriltag_process.pid, signal.SIGINT)
            self.apriltag_process.wait(timeout=self.apriltag_stop_timeout_sec)
        except subprocess.TimeoutExpired:
            self.get_logger().warn("apriltag detector did not stop on SIGINT, sending SIGTERM")
            try:
                os.killpg(self.apriltag_process.pid, signal.SIGTERM)
                self.apriltag_process.wait(timeout=1.0)
            except Exception as exc:
                self.get_logger().error(f"Failed to force stop apriltag detector: {exc}")
        except Exception as exc:
            self.get_logger().error(f"Failed to stop apriltag detector: {exc}")
        finally:
            self.apriltag_process = None

    def handle_dock(self, request, response):
        start = bool(request.start)
        self.get_logger().info(f"dock requested start={str(start).lower()}")

        if start:
            self._prepare_new_docking_session()
            if self.enable_apriltag_on_demand and not self._start_apriltag_detector():
                self.docking_session_active = False
                response.ok = False
                return response
            self._publish_control("start")
            accepted = self._wait_for_state(
                {"SEARCHING_TAG", "ALIGNING", "APPROACHING", "RETRYING"},
                self.state_timeout_sec,
            )
            response.ok = accepted
            if not accepted:
                self.docking_session_active = False
                self.get_logger().error("Dock start rejected or timed out waiting for state transition")
                if self.enable_apriltag_on_demand:
                    self._stop_apriltag_detector()
        else:
            self.docking_session_active = False
            self._publish_control("stop")
            if self.enable_apriltag_on_demand:
                self._stop_apriltag_detector()
            # Stop command is idempotent: return success even if controller is already idle.
            response.ok = True
        return response

    def handle_charge(self, request, response):
        self.get_logger().info(f"charge_until requested soc_target={request.soc_target:.1f}")
        response.ok = True
        return response

    def destroy_node(self):
        if self.enable_apriltag_on_demand:
            self._stop_apriltag_detector()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChargeServices()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
