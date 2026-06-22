#!/usr/bin/env python3
"""
巡诊检测模拟节点：替代 bed_detection_server，无需相机/YOLO/CLIP。

行为：
  - Area 模式（到达巡诊点）：固定返回床位 2、4 有人
  - Bed 模式（到达床位 2）：正常，无异常
  - Bed 模式（到达床位 4）：有坠床风险
"""

import rclpy
from rclpy.node import Node

from interfaces.srv import DetectAnomaly

MODE_AREA = 0
MODE_BED = 1


def _parse_int_list(value) -> list[int]:
    if isinstance(value, list):
        return [int(v) for v in value]
    if not value:
        return []
    return [int(item.strip()) for item in str(value).split(',') if item.strip()]


class MockPatrolDetectionServer(Node):
    def __init__(self):
        super().__init__('mock_patrol_detection_server')

        self.declare_parameter('service_name', '/detect_anomaly')
        self.declare_parameter('area_occupied_beds', [2, 4])
        self.declare_parameter('area_urgencies', [1, 1])
        self.declare_parameter('fall_risk_bed_id', 4)
        self.declare_parameter('fall_risk_details', 'fall risk detected at bed 4')

        service_name = self.get_parameter('service_name').value
        self._srv = self.create_service(DetectAnomaly, service_name, self.handle_request)
        self.get_logger().info(
            f'mock_patrol_detection_server started on {service_name} '
            f'(area -> beds {self._area_beds()}, bed 2 normal, bed 4 fall risk)'
        )

    def _area_beds(self) -> list[int]:
        return _parse_int_list(self.get_parameter('area_occupied_beds').value)

    def _area_urgencies(self, bed_count: int) -> list[int]:
        urgencies = _parse_int_list(self.get_parameter('area_urgencies').value)
        while len(urgencies) < bed_count:
            urgencies.append(1)
        return urgencies[:bed_count]

    def _handle_area_mode(self, patrol_id: int, response: DetectAnomaly.Response):
        bed_ids = self._area_beds()
        urgencies = self._area_urgencies(len(bed_ids))

        response.is_anomaly = bool(bed_ids)
        response.bed_ids = bed_ids
        response.urgencies = urgencies
        response.details = (
            f'[mock] patrol_id={patrol_id}: occupied beds {bed_ids}'
        )
        self.get_logger().info(
            f'Area mode patrol_id={patrol_id} -> bed_ids={bed_ids}, urgencies={urgencies}'
        )
        return response

    def _handle_bed_mode(self, bed_id: int, response: DetectAnomaly.Response):
        fall_risk_bed_id = int(self.get_parameter('fall_risk_bed_id').value)
        fall_risk_details = str(self.get_parameter('fall_risk_details').value)

        if bed_id == fall_risk_bed_id:
            response.is_anomaly = True
            response.details = fall_risk_details
        else:
            response.is_anomaly = False
            response.details = f'[mock] bed {bed_id}: normal'

        response.bed_ids = []
        response.urgencies = []
        self.get_logger().info(
            f'Bed mode bed_id={bed_id} -> '
            f'{"fall risk" if response.is_anomaly else "normal"}'
        )
        return response

    def handle_request(self, request, response):
        if request.mode == MODE_AREA:
            return self._handle_area_mode(int(request.area_bed_id), response)
        if request.mode == MODE_BED:
            return self._handle_bed_mode(int(request.area_bed_id), response)

        response.is_anomaly = False
        response.details = f'[mock] unknown mode: {request.mode}'
        response.bed_ids = []
        response.urgencies = []
        self.get_logger().warn(response.details)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockPatrolDetectionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
