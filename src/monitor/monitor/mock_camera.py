#!/usr/bin/env python3
"""
模拟相机发布节点 - 用于测试
整合版本: 用于Medical_Embodied项目的monitor包

功能:
  - 订阅 /camera/mode (std_msgs/String) 切换模式: area / bed / face
  - 根据模式从 test_images/{area_pic,bed_pic,face_pic} 中随机抽取图片
  - 发布图像到 /camera/rgb/image_raw (10Hz)
  - 默认模式为 area
"""
import os
import random

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


# 模式 -> 文件夹名映射
MODE_FOLDER_MAP = {
    'area': 'area_pic',
    'bed': 'bed_pic',
    'face': 'face_pic',
}

# 支持的图片扩展名
IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')


class MockCameraNode(Node):
    def __init__(self):
        super().__init__('mock_camera_node')
        self.bridge = CvBridge()

        # 确定源代码目录路径
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        src_monitor_dir = os.path.dirname(current_file_dir)
        self.test_images_dir = os.path.join(src_monitor_dir, 'test_images')

        # 当前模式
        self.current_mode = 'area'

        # 预加载各模式图片路径
        self.mode_image_paths = {}
        for mode, folder in MODE_FOLDER_MAP.items():
            folder_path = os.path.join(self.test_images_dir, folder)
            paths = []
            if os.path.isdir(folder_path):
                paths = [
                    os.path.join(folder_path, f)
                    for f in sorted(os.listdir(folder_path))
                    if f.lower().endswith(IMAGE_EXTENSIONS)
                ]
            self.mode_image_paths[mode] = paths
            self.get_logger().info(
                f'模式 [{mode}]: 加载 {len(paths)} 张图片 from {folder_path}'
            )

        # 当前图片缓存
        self.current_image = None
        self._load_random_image()

        # 如果没有加载到任何图片，生成占位图
        if self.current_image is None:
            self.current_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                self.current_image, 'NO IMAGE', (180, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3
            )
            self.get_logger().warn('没有找到任何测试图片，使用占位图')

        # 创建图像发布者
        self.image_pub = self.create_publisher(
            Image, '/camera/rgb/image_raw', 10
        )

        # 订阅模式切换话题
        self.mode_sub = self.create_subscription(
            String, '/camera/mode', self._mode_callback, 10
        )

        # 定时发布图像 (10Hz)
        self.timer = self.create_timer(0.1, self._publish_image)
        self.get_logger().info(
            f'模拟相机节点已启动, 默认模式: {self.current_mode}'
        )
        self.get_logger().info(
            '切换模式: ros2 topic pub --once /camera/mode std_msgs/String "{data: bed}"'
        )

    def _load_random_image(self):
        """根据当前模式随机加载一张图片"""
        paths = self.mode_image_paths.get(self.current_mode, [])
        if not paths:
            self.get_logger().warn(
                f'模式 [{self.current_mode}] 没有可用图片'
            )
            return

        chosen = random.choice(paths)
        img = cv2.imread(chosen)
        if img is not None:
            self.current_image = img
            self.get_logger().info(
                f'模式 [{self.current_mode}] 加载图片: {os.path.basename(chosen)} '
                f'({img.shape[1]}x{img.shape[0]})'
            )
        else:
            self.get_logger().error(f'图片读取失败: {chosen}')

    def _mode_callback(self, msg):
        """接收模式切换指令"""
        new_mode = msg.data.strip().lower()
        if new_mode not in MODE_FOLDER_MAP:
            self.get_logger().warn(
                f'未知模式: "{new_mode}", 支持的模式: {list(MODE_FOLDER_MAP.keys())}'
            )
            return

        if new_mode == self.current_mode:
            return

        self.current_mode = new_mode
        self._load_random_image()
        self.get_logger().info(f'切换到模式: {self.current_mode}')

    def _publish_image(self):
        """发布当前图像"""
        if self.current_image is None:
            return
        msg = self.bridge.cv2_to_imgmsg(self.current_image, encoding='bgr8')
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    mock_camera = MockCameraNode()

    try:
        rclpy.spin(mock_camera)
    except KeyboardInterrupt:
        pass
    finally:
        mock_camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
