#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from interfaces.srv import DetectAnomaly

DETECT_AREA = 0
DETECT_BED = 1


def _parse_int_csv(value: str):
    if not value:
        return []
    out = []
    for item in value.split(','):
        item = item.strip()
        if not item:
            continue
        try:
            out.append(int(item))
        except ValueError:
            out.append(0)
    return out


class AnomalyDetectServer(Node):
    def __init__(self):
        super().__init__('anomaly_detect_server')
        self.declare_parameter('scan_beds', '0,2')
        self.declare_parameter('scan_urgencies', '2,1')
        self.declare_parameter('force_anomaly', '')
        self._srv = self.create_service(DetectAnomaly, 'detect_anomaly', self.handle_request)
        self.get_logger().info('anomaly_detect_server started')

    def handle_request(self, request, response):
        if request.mode == DETECT_AREA:
            beds = _parse_int_csv(str(self.get_parameter('scan_beds').value))
            urgencies = _parse_int_csv(str(self.get_parameter('scan_urgencies').value))
            while len(urgencies) < len(beds):
                urgencies.append(0)
            pairs = list(zip(beds, urgencies))
            pairs.sort(key=lambda x: x[1], reverse=True)
            response.bed_ids = [b for b, _ in pairs]
            response.urgencies = [u for _, u in pairs]
            response.is_anomaly = bool(response.bed_ids)
            response.details = 'scan'
            return response

        bed_id = int(request.area_bed_id)
        force = str(self.get_parameter('force_anomaly').value)
        if force.lower() in ('true', '1', 'yes'):
            is_anomaly = True
        elif force.lower() in ('false', '0', 'no'):
            is_anomaly = False
        else:
            is_anomaly = (bed_id % 2 == 0)

        details = 'mock anomaly' if is_anomaly else 'normal'
        self.get_logger().info(f'detect_anomaly mode=bed bed_id={bed_id} -> {details}')
        response.is_anomaly = is_anomaly
        response.details = details
        response.bed_ids = []
        response.urgencies = []
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetectServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
