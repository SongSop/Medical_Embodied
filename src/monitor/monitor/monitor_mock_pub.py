#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from interfaces.msg import Battery, Fault


class MonitorMockPub(Node):
    def __init__(self):
        super().__init__('monitor_mock_pub')
        self.battery_pub = self.create_publisher(Battery, 'battery', 10)
        self.fault_pub = self.create_publisher(Fault, 'fault', 10)
        self.call_signal_pub = self.create_publisher(Bool, 'call_signal', 10)

        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('battery_soc', 50.0)
        self.declare_parameter('battery_charging', False)
        self.declare_parameter('battery_voltage', 24.0)
        self.declare_parameter('fault_type', '')
        self.declare_parameter('fault_severity', 0)
        self.declare_parameter('fault_details', '')
        self.declare_parameter('call_signal', False)
        rate_hz = float(self.get_parameter('rate_hz').value)
        period = 1.0 / max(0.1, rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

    def _on_timer(self):
        battery = Battery()
        battery.soc = float(self.get_parameter('battery_soc').value)
        battery.charging = bool(self.get_parameter('battery_charging').value)
        battery.voltage = float(self.get_parameter('battery_voltage').value)
        self.battery_pub.publish(battery)

        fault = Fault()
        fault.fault_type = str(self.get_parameter('fault_type').value)
        fault.severity = int(self.get_parameter('fault_severity').value)
        fault.details = str(self.get_parameter('fault_details').value)
        self.fault_pub.publish(fault)

        call_signal = bool(self.get_parameter('call_signal').value)
        self.call_signal_pub.publish(Bool(data=call_signal))


def main(args=None):
    rclpy.init(args=args)
    node = MonitorMockPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
