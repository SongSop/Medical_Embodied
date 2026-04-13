#!/usr/bin/env python3
import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from interfaces.action import CallNurse


class CallNurseMockClient(Node):
    def __init__(self):
        super().__init__('call_nurse_mock_client')
        self._client = ActionClient(self, CallNurse, 'call_nurse')

    def send_goal(self, bed_ids, summarys):
        goal = CallNurse.Goal()
        goal.bed_ids = [int(x) for x in bed_ids]
        goal.summarys = [str(x) for x in summarys]

        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('call_nurse action server not available')
            return 1

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('goal rejected')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'call_nurse result={result.status.status} message={result.message}')
        return 0


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--beds', type=str, default='1')
    parser.add_argument('--summarys', type=str, default='mock')
    parsed = parser.parse_args(args=args)

    bed_ids = [x for x in parsed.beds.split(',') if x.strip()]
    summarys = [x for x in parsed.summarys.split(',') if x.strip()]

    rclpy.init(args=None)
    node = CallNurseMockClient()
    rc = node.send_goal(bed_ids, summarys)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
