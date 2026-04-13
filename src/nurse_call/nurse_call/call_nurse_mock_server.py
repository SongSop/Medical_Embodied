#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from interfaces.action import CallNurse
from interfaces.msg import ActionStatus


class CallNurseMockServer(Node):
    def __init__(self):
        super().__init__('call_nurse_mock_server')
        self._server = ActionServer(
            self,
            CallNurse,
            'call_nurse',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('call_nurse_mock_server started')

    def goal_callback(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = CallNurse.Result()

        if not goal.bed_ids:
            result.status.status = ActionStatus.ABORTED
            result.message = 'empty bed_ids'
            goal_handle.abort()
            return result

        for i in range(3):
            if goal_handle.is_cancel_requested:
                result.status.status = ActionStatus.PREEMPTED
                result.message = 'preempted'
                goal_handle.canceled()
                return result
            feedback = CallNurse.Feedback()
            feedback.progress = f'calling nurse step {i + 1}'
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)

        result.status.status = ActionStatus.OK
        result.message = 'ok'
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CallNurseMockServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
