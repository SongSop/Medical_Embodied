#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import Bool

from interfaces.action import CallNurse, LLMInteraction, Navigate
from interfaces.msg import ActionStatus, Battery, Fault
from interfaces.srv import DetectAnomaly, FaceIdentify, SetConfig

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


class MedicalBtRosTestDriver(Node):
    def __init__(self):
        super().__init__('medical_bt_ros_test_driver')
        self.metrics = Metrics()
        self.section_marks = {}
        self.lock = threading.Lock()

        self.battery_pub = self.create_publisher(Battery, '/battery', 10)
        self.fault_pub = self.create_publisher(Fault, '/fault', 10)
        self.call_signal_pub = self.create_publisher(Bool, '/call_signal', 10)
        self.patrol_trigger_pub = self.create_publisher(Bool, '/patrol_triggered', 1)

        # 注释掉检测服务，让行为树调用实际的检测节点
        # self.create_service(DetectAnomaly, '/detect_anomaly', self.handle_detect_anomaly)
        self.create_service(FaceIdentify, '/face_identify', self.handle_face_identify)
        self.create_service(SetConfig, '/loadconfig/set_config', self.handle_set_config)

        self.nav_server = ActionServer(
            self, Navigate, 'navigate',
            execute_callback=self.handle_navigate,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
        )
        self.llm_server = ActionServer(
            self, LLMInteraction, 'llm_interaction',
            execute_callback=self.handle_llm,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
        )
        self.call_nurse_server = ActionServer(
            self, CallNurse, 'call_nurse',
            execute_callback=self.handle_call_nurse,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
        )

        self.declare_parameter('start_delay', 1.0)
        self.declare_parameter('ticks', 140)
        self.declare_parameter('tick_hz', 20)
        self.declare_parameter('patrol_route_id', 'route_a')
        self.declare_parameter('patrol_cycles', 2)
        self.declare_parameter('patrol_points', ['p0', 'p1'])
        self.patrol_triggered = False
        self.publish_patrol_triggered(False)

    def publish_patrol_triggered(self, value, tick=None):
        if value == self.patrol_triggered:
            return
        self.patrol_triggered = value
        self.patrol_trigger_pub.publish(Bool(data=value))
        if tick is not None:
            self.get_logger().info(f'[SIM ] tick={tick} patrol_triggered={str(value).lower()}')

    def handle_detect_anomaly(self, request, response):
        if request.mode == DETECT_AREA:
            response.is_anomaly = False
            response.details = 'scan'
            response.bed_ids = [1, 0]
            response.urgencies = [1, 2]
            return response
        is_anomaly = (request.area_bed_id % 2 == 0)
        response.is_anomaly = is_anomaly
        response.details = 'anomaly' if is_anomaly else 'normal'
        response.bed_ids = []
        response.urgencies = []
        return response

    def handle_face_identify(self, _request, response):
        response.success = True
        response.person_id = 1
        response.confidence = 0.9
        response.message = 'ok'
        return response

    def handle_set_config(self, request, response):
        config_id = (request.config_id or 'default').strip()
        if config_id == 'default':
            route_id, cycles, points = 'route_a', 2, ['p0', 'p1']
        else:
            route_id, cycles, points = 'route_a', 1, ['p0']
        self.set_parameters([
            rclpy.parameter.Parameter('patrol_route_id', value=route_id),
            rclpy.parameter.Parameter('patrol_cycles', value=cycles),
            rclpy.parameter.Parameter('patrol_points', value=points),
        ])
        response.ok = True
        response.message = 'ok'
        return response

    def handle_navigate(self, goal_handle):
        goal = goal_handle.request
        with self.lock:
            if goal.nav_type == NAV_STOP:
                self.metrics.nav_stop += 1
            if goal.nav_type == NAV_DOCK:
                self.metrics.nav_dock += 1
            if goal.nav_type == NAV_GOAL:
                self.metrics.nav_patrol += 1
        time.sleep(0.05)
        result = Navigate.Result()
        result.status.status = ActionStatus.OK
        result.message = 'ok'
        goal_handle.succeed()
        return result

    def handle_llm(self, goal_handle):
        goal = goal_handle.request
        need_call_nurse = (goal.mode == INTERACTION_ALERT)
        with self.lock:
            if goal.mode == INTERACTION_INTERRUPT:
                self.metrics.llm_call_response += 1
            elif goal.mode == INTERACTION_ALERT:
                self.metrics.llm_abnormal += 1
            elif goal.mode == INTERACTION_PASSIVE:
                self.metrics.llm_passive += 1
        time.sleep(0.05)
        result = LLMInteraction.Result()
        result.status.status = ActionStatus.OK
        result.summary = 'ok'
        result.need_call_nurse = need_call_nurse
        goal_handle.succeed()
        return result

    def handle_call_nurse(self, goal_handle):
        with self.lock:
            self.metrics.call_nurse += 1
        time.sleep(0.02)
        result = CallNurse.Result()
        result.status.status = ActionStatus.OK
        result.message = 'ok'
        goal_handle.succeed()
        return result

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
        fault_type = ''
        fault_severity = 0
        if 15 <= tick <= 18:
            fault_type, fault_severity = 'localization', 1
        elif 25 <= tick <= 28:
            fault_type, fault_severity = 'navigation', 1
        elif 35 <= tick <= 38:
            fault_type, fault_severity = 'self', 1

        battery = Battery()
        battery.soc = float(battery_soc)
        battery.charging = False
        battery.voltage = 24.0
        self.battery_pub.publish(battery)

        fault = Fault()
        fault.fault_type = fault_type
        fault.severity = fault_severity
        fault.details = ''
        self.fault_pub.publish(fault)

        self.call_signal_pub.publish(Bool(data=call_signal))

    def run(self):
        start_delay = float(self.get_parameter('start_delay').value)
        total_ticks = int(self.get_parameter('ticks').value)
        tick_hz = float(self.get_parameter('tick_hz').value)
        period = 1.0 / max(1.0, tick_hz)

        self.get_logger().info(f'medical_bt_ros_test_driver starting in {start_delay:.1f}s')
        time.sleep(start_delay)

        for tick in range(total_ticks):
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
            time.sleep(period)
        
        # 保持运行，持续发布最后一个状态，让Action服务器保持可用
        self.get_logger().info('Tick cycle completed, keeping node alive for action servers...')
        while rclpy.ok():
            time.sleep(1.0)



def main(args=None):
    rclpy.init(args=args)
    node = MedicalBtRosTestDriver()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
