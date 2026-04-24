#!/usr/bin/env python3
"""
床位检测测试客户端 - Area模式测试
整合版本: 用于Medical_Embodied项目的monitor包

功能: 测试床位检测服务的Area模式
"""
import rclpy
from rclpy.node import Node
from interfaces.srv import DetectAnomaly

MODE_AREA = 0

class BedDetectionTestClient(Node):
    def __init__(self):
        super().__init__('bed_detection_test_client')
        self.client = self.create_client(DetectAnomaly, '/detect_anomaly')

        self.get_logger().info('等待床位检测服务上线...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务未就绪,等待中...')

        self.get_logger().info('床位检测服务已连接')

    def test_area_mode(self, area_bed_id=1):
        """
        测试Area模式

        Args:
            area_bed_id: 区域ID
        """
        self.get_logger().info(f'=== 开始测试Area模式 (区域ID: {area_bed_id}) ===')

        request = DetectAnomaly.Request()
        request.mode = MODE_AREA
        request.area_bed_id = area_bed_id

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info('')
            self.get_logger().info('===== 检测结果 =====')
            self.get_logger().info(f'是否异常: {response.is_anomaly}')
            self.get_logger().info(f'详情: {response.details}')
            self.get_logger().info(f'有人床位ID列表: {list(response.bed_ids)}')
            self.get_logger().info(f'紧急程度列表: {list(response.urgencies)}')
            self.get_logger().info('==================')
            self.get_logger().info('')
        else:
            self.get_logger().error('服务调用失败!')


def main(args=None):
    rclpy.init(args=args)
    test_client = BedDetectionTestClient()

    try:
        # 测试Area模式
        test_client.test_area_mode(area_bed_id=1)

    except KeyboardInterrupt:
        pass
    finally:
        test_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()