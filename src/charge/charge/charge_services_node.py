#!/usr/bin/env python3

import time
from threading import Event

import rclpy
from rclpy.node import Node

from charge_ros2.srv import ChargeUntil, Dock
from std_msgs.msg import String


class ChargeServicesNode(Node):
    def __init__(self) -> None:
        super().__init__("charge_services")
        self.state_timeout_sec = float(self.declare_parameter("state_timeout_sec", 5.0).value)
        self.docking_session_cooldown_sec = float(
            self.declare_parameter("docking_session_cooldown_sec", 5.0).value
        )
        self.last_docking_state = "UNKNOWN"
        self.state_event = Event()
        self.last_docking_end_monotonic = None

        self.control_pub = self.create_publisher(String, "/dock/control_cmd", 10)
        self.create_subscription(String, "/docking/state", self._state_callback, 10)
        self.create_service(Dock, "dock", self.handle_dock)
        self.create_service(ChargeUntil, "charge_until", self.handle_charge_until)
        self.get_logger().info("charge_services started")

    def _state_callback(self, msg: String) -> None:
        self.last_docking_state = msg.data.strip()
        self.state_event.set()

    def _publish_control(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.control_pub.publish(msg)

    def _prepare_new_docking_session(self) -> None:
        # Prevent previous terminal state from short-circuiting a new start.
        self.last_docking_state = "UNKNOWN"
        self.state_event.clear()

    def _record_docking_cooldown(self) -> None:
        self.last_docking_end_monotonic = time.monotonic()

    def _wait_for_docking_cooldown(self) -> None:
        if self.last_docking_end_monotonic is None:
            return
        remaining = self.docking_session_cooldown_sec - (
            time.monotonic() - self.last_docking_end_monotonic
        )
        if remaining <= 0.0:
            return
        self.get_logger().info(
            "Docking cooldown: waiting %.1fs before next start" % remaining
        )
        time.sleep(remaining)

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

    def handle_dock(self, request: Dock.Request, response: Dock.Response) -> Dock.Response:
        start = bool(request.start)
        self.get_logger().info(f"dock requested start={str(start).lower()}")

        if start:
            self._wait_for_docking_cooldown()
            self._prepare_new_docking_session()
            self._publish_control("start")
            accepted = self._wait_for_state(
                {"SEARCHING_TAG", "ALIGNING", "APPROACHING", "RETRYING"},
                self.state_timeout_sec,
            )
            response.ok = accepted
            if not accepted:
                self.get_logger().error("Dock start rejected or timed out waiting for state transition")
                self._publish_control("stop")
                self._record_docking_cooldown()
        else:
            self._publish_control("stop")
            self._record_docking_cooldown()
            # Stop command is idempotent: return success even if controller is already idle.
            response.ok = True
        return response

    def handle_charge_until(
        self, request: ChargeUntil.Request, response: ChargeUntil.Response
    ) -> ChargeUntil.Response:
        self.get_logger().info(f"charge_until requested soc_target={request.soc_target:.1f}")
        # Battery/SOC orchestration is platform-specific and should be integrated with BMS topics/services.
        response.ok = False
        self.get_logger().warn("charge_until is not wired to BMS yet")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChargeServicesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
