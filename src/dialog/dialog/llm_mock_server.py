#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from interfaces.action import LLMInteraction
from interfaces.msg import ActionStatus

INTERACTION_ALERT = 0
INTERACTION_PASSIVE = 1
INTERACTION_INTERRUPT = 2


class LLMMockServer(Node):
    def __init__(self):
        super().__init__('llm_mock_server')
        self._server = ActionServer(
            self,
            LLMInteraction,
            'llm_interaction',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('llm_mock_server started')

    def goal_callback(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = LLMInteraction.Result()

        for part in ['analyzing', 'responding', 'done']:
            if goal_handle.is_cancel_requested:
                result.status.status = ActionStatus.PREEMPTED
                result.summary = 'preempted'
                result.need_call_nurse = False
                goal_handle.canceled()
                return result
            feedback = LLMInteraction.Feedback()
            feedback.partial = part
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)

        need_call_nurse = (goal.mode == INTERACTION_ALERT and (goal.person_id % 2 == 1))
        result.status.status = ActionStatus.OK
        result.summary = 'mock_summary'
        result.need_call_nurse = need_call_nurse
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LLMMockServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
