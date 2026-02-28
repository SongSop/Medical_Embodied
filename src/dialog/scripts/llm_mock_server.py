#!/usr/bin/env python3
import rospy
import actionlib

from interfaces.msg import LLMInteractionAction, LLMInteractionResult, LLMInteractionFeedback, ActionStatus

INTERACTION_ALERT = 0
INTERACTION_PASSIVE = 1
INTERACTION_INTERRUPT = 2


class LLMMockServer:
    def __init__(self):
        self._server = actionlib.SimpleActionServer(
            "llm_interaction",
            LLMInteractionAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._server.start()
        rospy.loginfo("llm_mock_server started")

    def _execute(self, goal):
        result = LLMInteractionResult()
        feedback = LLMInteractionFeedback()

        partials = ["analyzing", "responding", "done"]
        rate = rospy.Rate(5)
        for part in partials:
            if self._server.is_preempt_requested():
                result.status.status = ActionStatus.PREEMPTED
                result.summary = "preempted"
                result.need_call_nurse = False
                self._server.set_preempted(result, result.summary)
                return
            feedback.partial = part
            self._server.publish_feedback(feedback)
            rate.sleep()

        need_call_nurse = goal.mode == INTERACTION_ALERT and (goal.person_id % 2 == 1)
        result.status.status = ActionStatus.OK
        result.summary = "mock_summary"
        result.need_call_nurse = need_call_nurse
        self._server.set_succeeded(result, result.summary)


if __name__ == "__main__":
    rospy.init_node("llm_mock_server")
    LLMMockServer()
    rospy.spin()
