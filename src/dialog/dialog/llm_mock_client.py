#!/usr/bin/env python3
import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from interfaces.action import LLMInteraction


class LLMMockClient(Node):
    def __init__(self):
        super().__init__('llm_mock_client')
        self._client = ActionClient(self, LLMInteraction, 'llm_interaction')

    def send_goal(self, mode, person_id, context):
        goal = LLMInteraction.Goal()
        goal.mode = int(mode)
        goal.person_id = int(person_id)
        goal.context = str(context)

        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('llm action server not available')
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
        self.get_logger().info(f'llm result={result.status.status} call_nurse={result.need_call_nurse} summary={result.summary}')
        return 0


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=int, default=1)
    parser.add_argument('--person-id', type=int, default=1)
    parser.add_argument('--context', type=str, default='none')
    parsed = parser.parse_args(args=args)

    rclpy.init(args=None)
    node = LLMMockClient()
    rc = node.send_goal(parsed.mode, parsed.person_id, parsed.context)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
