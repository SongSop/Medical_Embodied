#!/usr/bin/env python3
import rospy
import actionlib

from interfaces.msg import NavigateAction, NavigateResult, NavigateFeedback, ActionStatus

NAV_GOAL = 0
NAV_STOP = 1
NAV_DOCK = 2


class NavigateMockServer:
    def __init__(self):
        self._server = actionlib.SimpleActionServer(
            "navigate",
            NavigateAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._server.start()
        rospy.loginfo("navigate_mock_server started")

    def _execute(self, goal):
        result = NavigateResult()
        feedback = NavigateFeedback()

        if goal.target_index < -10:
            result.status.status = ActionStatus.NO_PATH
            result.message = "no path"
            self._server.set_aborted(result, result.message)
            return

        steps = 5 if goal.nav_type == NAV_DOCK else 3
        rate = rospy.Rate(10)
        for i in range(steps):
            if self._server.is_preempt_requested():
                result.status.status = ActionStatus.PREEMPTED
                result.message = "preempted"
                self._server.set_preempted(result, result.message)
                return
            feedback.progress = float(i + 1) / float(steps)
            self._server.publish_feedback(feedback)
            rate.sleep()

        result.status.status = ActionStatus.OK
        result.message = "ok"
        self._server.set_succeeded(result, result.message)


if __name__ == "__main__":
    rospy.init_node("navigate_mock_server")
    NavigateMockServer()
    rospy.spin()
