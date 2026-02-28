#!/usr/bin/env python3
import time
import threading

import rospy
import actionlib
from std_msgs.msg import Bool
from rosgraph_msgs.msg import Log

from interfaces.msg import Battery, Fault, ActionStatus
from interfaces.msg import NavigateAction, NavigateResult
from interfaces.msg import LLMInteractionAction, LLMInteractionResult
from interfaces.msg import CallNurseAction, CallNurseResult
from interfaces.srv import DetectAnomaly, DetectAnomalyResponse
from interfaces.srv import FaceIdentify, FaceIdentifyResponse
from interfaces.srv import SetConfig, SetConfigResponse

INTERACTION_ALERT = 0
INTERACTION_PASSIVE = 1
INTERACTION_INTERRUPT = 2

NAV_GOAL = 0
NAV_STOP = 1
NAV_DOCK = 2

DETECT_AREA = 0
DETECT_BED = 1


class Metrics:
    def __init__(self):
        self.llm_call_response = 0
        self.llm_abnormal = 0
        self.llm_passive = 0
        self.nav_stop = 0
        self.nav_dock = 0
        self.nav_patrol = 0
        self.call_nurse = 0


class MedicalBtRosTestDriver:
    def __init__(self):
        self.metrics = Metrics()
        self.section_marks = {}
        self.lock = threading.Lock()

        self.battery_pub = rospy.Publisher("/battery", Battery, queue_size=10)
        self.fault_pub = rospy.Publisher("/fault", Fault, queue_size=10)
        self.call_signal_pub = rospy.Publisher("/call_signal", Bool, queue_size=10)
        self.patrol_trigger_pub = rospy.Publisher("/patrol_triggered", Bool, queue_size=1, latch=False)

        self.anomaly_srv = rospy.Service("/detect_anomaly", DetectAnomaly, self.handle_detect_anomaly)
        self.face_identify_srv = rospy.Service("/face_identify", FaceIdentify, self.handle_face_identify)
        self.set_config_srv = rospy.Service("/loadconfig/set_config", SetConfig, self.handle_set_config)

        self.nav_server = actionlib.SimpleActionServer(
            "navigate", NavigateAction, execute_cb=self.handle_navigate, auto_start=False
        )
        self.llm_server = actionlib.SimpleActionServer(
            "llm_interaction", LLMInteractionAction, execute_cb=self.handle_llm, auto_start=False
        )
        self.call_nurse_server = actionlib.SimpleActionServer(
            "call_nurse", CallNurseAction, execute_cb=self.handle_call_nurse, auto_start=False
        )

        self.nav_server.start()
        self.llm_server.start()
        self.call_nurse_server.start()

        rospy.Subscriber("/rosout_agg", Log, self.handle_rosout)

        self.patrol_triggered = False
        self.publish_patrol_triggered(False)

    def publish_patrol_triggered(self, value, tick=None):
        if value == self.patrol_triggered:
            return
        self.patrol_triggered = value
        self.patrol_trigger_pub.publish(Bool(data=value))
        if tick is not None:
            rospy.loginfo(f"[SIM ] tick={tick} patrol_triggered={str(value).lower()}")


    def handle_detect_anomaly(self, req):
        if req.mode == DETECT_AREA:
            return DetectAnomalyResponse(
                is_anomaly=False,
                details="scan",
                bed_ids=[1, 0],
                urgencies=[1, 2],
            )
        is_anomaly = (req.area_bed_id % 2 == 0)
        details = "anomaly" if is_anomaly else "normal"
        return DetectAnomalyResponse(
            is_anomaly=is_anomaly,
            details=details,
            bed_ids=[],
            urgencies=[],
        )

    def handle_face_identify(self, _req):
        return FaceIdentifyResponse(
            success=True,
            person_id=1,
            confidence=0.9,
            message="ok",
        )

    def handle_set_config(self, req):
        config_id = (req.config_id or "default").strip()
        if config_id == "default":
            route_id = "route_a"
            cycles = 2
            points = ["p0", "p1"]
        else:
            route_id = "route_a"
            cycles = 1
            points = ["p0"]
        rospy.set_param("/loadconfig/patrol_route_id", route_id)
        rospy.set_param("/loadconfig/patrol_cycles", cycles)
        rospy.set_param("/loadconfig/patrol_points", points)
        return SetConfigResponse(ok=True, message="ok")

    def handle_navigate(self, goal):
        with self.lock:
            if goal.nav_type == NAV_STOP:
                self.metrics.nav_stop += 1
            if goal.nav_type == NAV_DOCK:
                self.metrics.nav_dock += 1
            if goal.nav_type == NAV_GOAL:
                self.metrics.nav_patrol += 1
        time.sleep(0.05)
        result = NavigateResult()
        result.status.status = ActionStatus.OK
        result.message = "ok"
        self.nav_server.set_succeeded(result)

    def handle_llm(self, goal):
        need_call_nurse = (goal.mode == INTERACTION_ALERT)
        with self.lock:
            if goal.mode == INTERACTION_INTERRUPT:
                self.metrics.llm_call_response += 1
            elif goal.mode == INTERACTION_ALERT:
                self.metrics.llm_abnormal += 1
            elif goal.mode == INTERACTION_PASSIVE:
                self.metrics.llm_passive += 1
        time.sleep(0.05)
        result = LLMInteractionResult()
        result.status.status = ActionStatus.OK
        result.summary = "ok"
        result.need_call_nurse = need_call_nurse
        self.llm_server.set_succeeded(result)

    def handle_call_nurse(self, goal):
        with self.lock:
            self.metrics.call_nurse += 1
        time.sleep(0.02)
        result = CallNurseResult()
        result.status.status = ActionStatus.OK
        result.message = "ok"
        self.call_nurse_server.set_succeeded(result)

    def handle_rosout(self, msg):
        # ros_bt_runner uses std::cout, so this is best-effort for any ROS logs only.
        pass

    def mark_section(self, name):
        with self.lock:
            self.section_marks[name] = (
                self.metrics.llm_call_response,
                self.metrics.nav_stop,
                self.metrics.nav_dock,
                self.metrics.nav_patrol,
                self.metrics.call_nurse,
            )

    def delta_section(self, name):
        with self.lock:
            start = self.section_marks.get(name, (0, 0, 0, 0, 0))
            return (
                self.metrics.llm_call_response - start[0],
                self.metrics.nav_stop - start[1],
                self.metrics.nav_dock - start[2],
                self.metrics.nav_patrol - start[3],
                self.metrics.call_nurse - start[4],
            )

    def publish_inputs(self, tick):
        call_signal = 5 <= tick <= 8 or 95 <= tick <= 110
        battery_soc = 10.0 if 68 <= tick <= 74 else 50.0
        fault_type = ""
        fault_severity = 0
        if 15 <= tick <= 18:
            fault_type = "localization"
            fault_severity = 1
        elif 25 <= tick <= 28:
            fault_type = "navigation"
            fault_severity = 1
        elif 35 <= tick <= 38:
            fault_type = "self"
            fault_severity = 1

        battery = Battery()
        battery.soc = float(battery_soc)
        battery.charging = False
        battery.voltage = 24.0
        self.battery_pub.publish(battery)

        fault = Fault()
        fault.fault_type = fault_type
        fault.severity = fault_severity
        fault.details = ""
        self.fault_pub.publish(fault)

        self.call_signal_pub.publish(Bool(data=call_signal))
        if not hasattr(self, "_prev_call_signal"):
            self._prev_call_signal = call_signal
            self._prev_fault_type = fault_type
            self._prev_fault_severity = fault_severity
            self._prev_battery_soc = battery_soc
        if call_signal != self._prev_call_signal:
            rospy.loginfo(f"[SIM ] tick={tick} call_signal={str(call_signal).lower()}")
            self._prev_call_signal = call_signal
        if fault_type != self._prev_fault_type or fault_severity != self._prev_fault_severity:
            rospy.loginfo(f"[SIM ] tick={tick} fault_type={fault_type} severity={fault_severity}")
            self._prev_fault_type = fault_type
            self._prev_fault_severity = fault_severity
        if battery_soc != self._prev_battery_soc:
            rospy.loginfo(f"[SIM ] tick={tick} battery_soc={battery_soc:.1f}")
            self._prev_battery_soc = battery_soc

    def run(self):
        start_delay = rospy.get_param("~start_delay", 1.0)
        total_ticks = rospy.get_param("~ticks", 140)
        tick_hz = rospy.get_param("~tick_hz", 20)
        rate = rospy.Rate(tick_hz)
        rospy.loginfo(f"medical_bt_ros_test_driver starting in {start_delay:.1f}s")
        time.sleep(start_delay)
        try:
            rospy.wait_for_service("/detect_anomaly", timeout=5.0)
            rospy.wait_for_service("/face_identify", timeout=5.0)
            rospy.wait_for_service("/loadconfig/set_config", timeout=5.0)
        except rospy.ROSException:
            rospy.logwarn("services not ready; proceeding without initial patrol reset")

        for tick in range(total_ticks):
            if rospy.is_shutdown():
                break

            if tick == 0:
                rospy.loginfo(f"[TEST] tick={tick} Section: baseline idle.")
                rospy.loginfo(f"[TEST] tick={tick} 目标: 无触发，树保持空闲。")
                rospy.loginfo(f"[TEST] tick={tick} 结果: (区段结束后给出)")
                self.mark_section("baseline")
            if tick == 4:
                d = self.delta_section("baseline")
                unexpected = d[0] + d[1] + d[2] + d[3] + d[4]
                rospy.loginfo(f"[TEST] tick={tick} Section end: baseline idle.")
                rospy.loginfo(f"[TEST] tick={tick} 结果: {'PASS' if unexpected == 0 else 'FAIL'} "
                              f"(unexpected_events={unexpected})")

            if tick == 5:
                rospy.loginfo(f"[TEST] tick={tick} Section: call signal response (ticks 5-8).")
                rospy.loginfo(f"[TEST] tick={tick} 目标: 触发呼叫响应，LLMInteraction 模式为 interrupt。")
                rospy.loginfo(f"[TEST] tick={tick} 结果: (区段结束后给出)")
                self.mark_section("call_signal_1")
            if tick == 12:
                d = self.delta_section("call_signal_1")
                rospy.loginfo(f"[TEST] tick={tick} Section end: call signal response.")
                rospy.loginfo(f"[TEST] tick={tick} 结果: {'PASS' if d[0] >= 1 else 'FAIL'} "
                              f"(llm_call_response+={d[0]})")

            if tick == 15:
                rospy.loginfo(f"[TEST] tick={tick} Section: localization fault handling (ticks 15-18).")
                rospy.loginfo(f"[TEST] tick={tick} 目标: 触发定位故障处理并触发 stop 导航。")
                rospy.loginfo(f"[TEST] tick={tick} 结果: (区段结束后给出)")
                self.mark_section("fault_loc")
                self.publish_patrol_triggered(True, tick=tick)
            if tick == 20:
                d = self.delta_section("fault_loc")
                rospy.loginfo(f"[TEST] tick={tick} Section end: localization fault handling.")
                rospy.loginfo(f"[TEST] tick={tick} 结果: {'PASS' if d[1] >= 1 else 'FAIL'} "
                              f"(nav_stop+={d[1]})")
                self.publish_patrol_triggered(False, tick=tick)

            if tick == 25:
                rospy.loginfo(f"[TEST] tick={tick} Section: navigation fault handling (ticks 25-28).")
                rospy.loginfo(f"[TEST] tick={tick} 目标: 触发导航故障处理并触发 stop 导航。")
                rospy.loginfo(f"[TEST] tick={tick} 结果: (区段结束后给出)")
                self.mark_section("fault_nav")
                self.publish_patrol_triggered(True, tick=tick)
            if tick == 31:
                d = self.delta_section("fault_nav")
                rospy.loginfo(f"[TEST] tick={tick} Section end: navigation fault handling.")
                rospy.loginfo(f"[TEST] tick={tick} 结果: {'PASS' if d[1] >= 1 else 'FAIL'} "
                              f"(nav_stop+={d[1]})")
                self.publish_patrol_triggered(False, tick=tick)

            if tick == 35:
                rospy.loginfo(f"[TEST] tick={tick} Section: self fault handling (ticks 35-38).")
                rospy.loginfo(f"[TEST] tick={tick} 目标: 触发自检故障处理并触发 stop 导航。")
                rospy.loginfo(f"[TEST] tick={tick} 结果: (区段结束后给出)")
                self.mark_section("fault_self")
                self.publish_patrol_triggered(True, tick=tick)
            if tick == 41:
                d = self.delta_section("fault_self")
                rospy.loginfo(f"[TEST] tick={tick} Section end: self fault handling.")
                rospy.loginfo(f"[TEST] tick={tick} 结果: {'PASS' if d[1] >= 1 else 'FAIL'} "
                              f"(nav_stop+={d[1]})")
                self.publish_patrol_triggered(False, tick=tick)

            if tick == 45:
                rospy.loginfo(f"[TEST] tick={tick} Section: patrol run (cycles=2).")
                rospy.loginfo(f"[TEST] tick={tick} 目标: 多次巡检并触发异常检测与交互。")
                rospy.loginfo(f"[TEST] tick={tick} 结果: (区段结束后给出)")
                self.mark_section("patrol_2")
                self.publish_patrol_triggered(True, tick=tick)
                rospy.loginfo(f"[TEST] tick={tick} Section: nurse call handling (during patrol).")
                rospy.loginfo(f"[TEST] tick={tick} 目标: 异常告警触发后产生呼叫护士。")
                rospy.loginfo(f"[TEST] tick={tick} 结果: (区段结束后给出)")
                self.mark_section("nurse_call")
            if tick == 61:
                d = self.delta_section("patrol_2")
                rospy.loginfo(f"[TEST] tick={tick} Section end: patrol run (cycles=2).")
                rospy.loginfo(f"[TEST] tick={tick} 结果: {'PASS' if d[3] >= 1 else 'FAIL'} "
                              f"(nav_patrol+={d[3]})")

            if tick == 61:
                d = self.delta_section("nurse_call")
                rospy.loginfo(f"[TEST] tick={tick} Section end: nurse call handling.")
                rospy.loginfo(f"[TEST] tick={tick} 结果: {'PASS' if d[4] >= 1 else 'FAIL'} "
                              f"(call_nurse+={d[4]})")

            if tick == 68:
                rospy.loginfo(f"[TEST] tick={tick} Section: low battery handling (ticks 68-74).")
                rospy.loginfo(f"[TEST] tick={tick} 目标: 触发低电量流程并进入充电导航。")
                rospy.loginfo(f"[TEST] tick={tick} 结果: (区段结束后给出)")
                self.mark_section("battery_low")
            if tick == 75:
                d = self.delta_section("battery_low")
                rospy.loginfo(f"[TEST] tick={tick} Section end: low battery handling.")
                rospy.loginfo(f"[TEST] tick={tick} 结果: {'PASS' if d[2] >= 1 else 'FAIL'} "
                              f"(nav_dock+={d[2]})")

            if tick == 90:
                rospy.loginfo(f"[TEST] tick={tick} Section: patrol run (cycles=1).")
                rospy.loginfo(f"[TEST] tick={tick} 目标: 单次巡检流程。")
                rospy.loginfo(f"[TEST] tick={tick} 结果: (区段结束后给出)")
                self.mark_section("patrol_1")
                self.publish_patrol_triggered(True, tick=tick)
            if tick == 110:
                d = self.delta_section("patrol_1")
                rospy.loginfo(f"[TEST] tick={tick} Section end: patrol run (cycles=1).")
                rospy.loginfo(f"[TEST] tick={tick} 结果: {'PASS' if d[3] >= 1 else 'FAIL'} "
                              f"(nav_patrol+={d[3]})")

            if tick == 95:
                rospy.loginfo(f"[TEST] tick={tick} Section: call signal late in run (ticks 95-110).")
                rospy.loginfo(f"[TEST] tick={tick} 目标: 后期呼叫响应。")
                rospy.loginfo(f"[TEST] tick={tick} 结果: (区段结束后给出)")
                self.mark_section("call_signal_2")
            if tick == 120:
                d = self.delta_section("call_signal_2")
                rospy.loginfo(f"[TEST] tick={tick} Section end: call signal late in run.")
                rospy.loginfo(f"[TEST] tick={tick} 结果: {'PASS' if d[0] >= 1 else 'FAIL'} "
                              f"(llm_call_response+={d[0]})")

            self.publish_inputs(tick)
            patrol_active = (
                (15 <= tick <= 18) or
                (25 <= tick <= 28) or
                (35 <= tick <= 38) or
                (68 <= tick <= 74) or
                (45 <= tick <= 61) or
                (90 <= tick <= 110)
            )
            self.publish_patrol_triggered(patrol_active, tick=tick)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("medical_bt_ros_test_driver")
    driver = MedicalBtRosTestDriver()
    driver.run()
