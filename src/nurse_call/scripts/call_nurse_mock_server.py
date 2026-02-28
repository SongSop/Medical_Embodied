#!/usr/bin/env python3
import rospy
import actionlib

from interfaces.msg import CallNurseAction, CallNurseResult, CallNurseFeedback, ActionStatus


class CallNurseMockServer:
    def __init__(self):
        self._server = actionlib.SimpleActionServer(
            "call_nurse",
            CallNurseAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._server.start()
        rospy.loginfo("call_nurse_mock_server started")

    def _execute(self, goal):
        result = CallNurseResult()
        feedback = CallNurseFeedback()
        #是否完成对应修改？
        if not goal.bed_ids:
            result.status.status = ActionStatus.ABORTED
            result.message = "empty bed_ids"
            self._server.set_aborted(result, result.message)
            return

        rate = rospy.Rate(5)
        for i in range(3):
            if self._server.is_preempt_requested():
                result.status.status = ActionStatus.PREEMPTED
                result.message = "preempted"
                self._server.set_preempted(result, result.message)
                return
            feedback.progress = "calling nurse step {}".format(i + 1)
            self._server.publish_feedback(feedback)
            rate.sleep()

        result.status.status = ActionStatus.OK
        result.message = "ok"
        self._server.set_succeeded(result, result.message)


if __name__ == "__main__":
    rospy.init_node("call_nurse_mock_server")
    CallNurseMockServer()
    rospy.spin()
