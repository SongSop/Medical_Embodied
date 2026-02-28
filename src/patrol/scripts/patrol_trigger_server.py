#!/usr/bin/env python3
import datetime
import rospy
from std_msgs.msg import Bool
from interfaces.srv import PatrolTrigger, PatrolTriggerResponse


class PatrolTriggerNode:
    def __init__(self):
        self._state = False
        self._pub = rospy.Publisher("patrol_triggered", Bool, queue_size=10, latch=True)
        self._trigger_srv = rospy.Service("patrol_trigger", PatrolTrigger, self._handle_button_trigger)
        self._mode = "button"
        self._time_str = "08:00"
        self._duration = 5
        self._last_trigger_date = None
        self._pulse_end = None
        rospy.loginfo("patrol_trigger node started (service: /patrol_trigger, topic: /patrol_triggered)")

    def _handle_button_trigger(self, req):
        self._state = bool(req.enable)
        self._pulse_end = None
        self._pub.publish(Bool(data=self._state))
        return PatrolTriggerResponse(ok=True)

    def _refresh_params(self):
        self._mode = rospy.get_param("/loadconfig/patrol_trigger_mode", "button").strip().lower()
        self._time_str = rospy.get_param("/loadconfig/patrol_trigger_time", "08:00").strip()
        self._duration = int(rospy.get_param("/loadconfig/patrol_trigger_duration_sec", 5))
        if self._duration <= 0:
            self._duration = 5

    def _check_time_trigger(self, now):
        if self._mode != "time":
            return
        parts = self._time_str.split(":")
        if len(parts) < 2:
            return
        try:
            hour = int(parts[0])
            minute = int(parts[1])
            second = int(parts[2]) if len(parts) > 2 else 0
        except ValueError:
            return
        if (now.hour, now.minute, now.second) != (hour, minute, second):
            return
        today = now.date()
        if self._last_trigger_date == today:
            return
        self._last_trigger_date = today
        self._state = True
        self._pulse_end = rospy.Time.now() + rospy.Duration(self._duration)
        self._pub.publish(Bool(data=True))
        rospy.loginfo("patrol_trigger_server: time trigger fired")

    def spin(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self._refresh_params()
            self._check_time_trigger(datetime.datetime.now())
            if self._pulse_end and rospy.Time.now() >= self._pulse_end:
                self._state = False
                self._pulse_end = None
                self._pub.publish(Bool(data=False))
            self._pub.publish(Bool(data=self._state))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("patrol_trigger_server")
    node = PatrolTriggerNode()
    node.spin()
