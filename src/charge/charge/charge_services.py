#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from interfaces.srv import ChargeUntil, Dock


class ChargeServices(Node):
    def __init__(self):
        super().__init__('charge_services')
        self.create_service(Dock, 'dock', self.handle_dock)
        self.create_service(ChargeUntil, 'charge_until', self.handle_charge)
        self.get_logger().info('charge_services started')

    def handle_dock(self, request, response):
        self.get_logger().info(f'dock requested start={str(request.start).lower()}')
        response.ok = True
        return response

    def handle_charge(self, request, response):
        self.get_logger().info(f'charge_until requested soc_target={request.soc_target:.1f}')
        response.ok = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ChargeServices()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
