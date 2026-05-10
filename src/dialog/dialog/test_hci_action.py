#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试 bt_hci_interface.py 中的 llm_interaction action。

示例：
python3 src/dialog/dialog/test_hci_action.py --mode alert
python3 src/dialog/dialog/test_hci_action.py --mode passive --person-id 2 --context "患者说自己有点头晕"
python3 src/dialog/dialog/test_hci_action.py --mode interrupt --person-id 3 --context "立即打断当前对话"
python3 src/dialog/dialog/test_hci_action.py --mode 1
"""

import argparse
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


ROOT_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "../../../install/interfaces/lib/python3.12/site-packages",
    )
)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)


from interfaces.action import LLMInteraction


INTERACTION_ALERT = 0
INTERACTION_PASSIVE = 1
INTERACTION_INTERRUPT = 2


def parseModeValue(mode_text: str) -> int:
    mode_map = {
        "alert": INTERACTION_ALERT,
        "passive": INTERACTION_PASSIVE,
        "interrupt": INTERACTION_INTERRUPT,
        "0": INTERACTION_ALERT,
        "1": INTERACTION_PASSIVE,
        "2": INTERACTION_INTERRUPT,
    }

    mode_key = str(mode_text).strip().lower()
    if mode_key not in mode_map:
        raise ValueError(
            f"unknown mode: {mode_text}, use alert/passive/interrupt or 0/1/2"
        )

    return mode_map[mode_key]


class TestHciActionClient(Node):
    def __init__(self):
        super().__init__("test_hci_action_client")
        self.actionClient = ActionClient(
            self,
            LLMInteraction,
            "llm_interaction",
        )

    def sendGoal(self, mode: int, person_id: int, context: str) -> int:
        goal = LLMInteraction.Goal()
        goal.mode = mode
        goal.person_id = person_id
        goal.context = context

        if not self.actionClient.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("llm_interaction action server not available")
            return 1

        self.get_logger().info(
            f"send goal: mode={goal.mode}, person_id={goal.person_id}, context={goal.context}"
        )

        sendGoalFuture = self.actionClient.send_goal_async(
            goal,
            feedback_callback=self.handleFeedback,
        )
        rclpy.spin_until_future_complete(self, sendGoalFuture)
        goalHandle = sendGoalFuture.result()

        if goalHandle is None or not goalHandle.accepted:
            self.get_logger().error("goal rejected")
            return 1

        resultFuture = goalHandle.get_result_async()
        rclpy.spin_until_future_complete(self, resultFuture)
        resultWrap = resultFuture.result()
        if resultWrap is None:
            self.get_logger().error("failed to get action result")
            return 1

        result = resultWrap.result
        self.get_logger().info(
            "action result: "
            f"status={result.status.status}, "
            f"need_call_nurse={result.need_call_nurse}, "
            f"summary={result.summary}"
        )
        return 0

    def handleFeedback(self, feedbackMsg) -> None:
        feedback = feedbackMsg.feedback
        self.get_logger().info(f"feedback: {feedback.partial}")


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        type=str,
        default="passive",
        help="alert/passive/interrupt or 0/1/2",
    )
    parser.add_argument("--person-id", type=int, default=1)
    parser.add_argument("--context", type=str, default="none")
    parsed = parser.parse_args(args=args)

    try:
        mode = parseModeValue(parsed.mode)
    except ValueError as exc:
        print(exc)
        raise SystemExit(1)

    rclpy.init(args=None)
    node = TestHciActionClient()
    rc = node.sendGoal(mode, parsed.person_id, parsed.context)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
