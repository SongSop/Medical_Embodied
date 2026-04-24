#!/usr/bin/env python3
"""
模拟相机发布节点 - 用于测试
整合版本: 用于Medical_Embodied项目的monitor包

功能:
  - 订阅 /camera/mode (std_msgs/String) 切换模式: area / bed / face
  - area 模式固定发布同一张图片（兼容 area_0/area1 等命名）
  - bed/face 模式从 test_images/{bed_pic,face_pic} 中随机抽取图片
  - 发布图像到 /camera/rgb/image_raw (10Hz)
  - 默认模式为 area
"""
import os
import random

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
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

        # 优先使用ROS安装后的share目录，兼容colcon install场景
        self.test_images_dir = self._resolve_test_images_dir()

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

        # area 模式固定使用一张图片（当仅剩一张时也能稳定工作）
        area_paths = self.mode_image_paths.get('area', [])
        self.fixed_area_image_path = area_paths[0] if area_paths else None

        # 当前图片缓存
        self.current_image = None
        self._load_image_for_mode()

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

    def _resolve_test_images_dir(self):
        """解析测试图片目录（优先share目录，回退源码目录）"""
        try:
            share_dir = get_package_share_directory('monitor')
            share_test_images = os.path.join(share_dir, 'test_images')
            if os.path.isdir(share_test_images):
                return share_test_images
            self.get_logger().warn(
                f'share目录不存在 test_images: {share_test_images}, 尝试源码目录'
            )
        except Exception as e:
            self.get_logger().warn(f'获取monitor share目录失败: {e}, 尝试源码目录')

        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        src_monitor_dir = os.path.dirname(current_file_dir)
        return os.path.join(src_monitor_dir, 'test_images')

    def _load_image_for_mode(self):
        """根据当前模式加载图片：area固定一张，bed/face随机"""
        paths = self.mode_image_paths.get(self.current_mode, [])
        if not paths:
            self.get_logger().warn(
                f'模式 [{self.current_mode}] 没有可用图片'
            )
            return

        if self.current_mode == 'area' and self.fixed_area_image_path:
            chosen = self.fixed_area_image_path
        else:
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

    def _normalize_mode(self, mode_raw: str) -> str:
        """兼容 area_0/area1/bed_2 等模式写法，统一归一到 area/bed/face"""
        if mode_raw.startswith('area'):
            return 'area'
        if mode_raw.startswith('bed'):
            return 'bed'
        if mode_raw.startswith('face'):
            return 'face'
        return mode_raw

    def _mode_callback(self, msg):
        """接收模式切换指令"""
        raw_mode = msg.data.strip().lower()
        new_mode = self._normalize_mode(raw_mode)
        if new_mode not in MODE_FOLDER_MAP:
            self.get_logger().warn(
                f'未知模式: "{raw_mode}", 支持的模式: {list(MODE_FOLDER_MAP.keys())}'
            )
            return

        if new_mode == self.current_mode:
            return

        self.current_mode = new_mode
        self._load_image_for_mode()
        self.get_logger().info(f'切换到模式: {self.current_mode} (raw="{raw_mode}")')

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
