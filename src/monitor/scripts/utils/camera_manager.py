#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通用相机管理器
提供统一的相机操作接口
支持 RGB 相机和 RGB-D 深度相机

使用方式:
    # 创建 RGB 相机 (Bed 模式)
    rgb_camera = CameraManager(camera_id=0, camera_type='RGB')

    # 创建 RGB-D 相机 (Area 模式)
    rgbd_camera = CameraManager(
        camera_id=0,
        camera_type='RGBD',
        frame_width=640,
        frame_height=480
    )
"""

import cv2
import rospy


class CameraManager:
    """
    通用相机管理器

    支持的相机类型:
    - RGB: 普通彩色相机 (适用于 Bed 模式)
    - RGBD: 深度相机 (适用于 Area 模式，需要深度信息进行坐标匹配)

    设计理念:
    - 提供统一的相机操作接口
    - Area 和 Bed 模式各自创建独立的相机实例
    - 避免资源冲突，支持不同的相机配置
    """

    # 相机类型常量
    CAMERA_RGB = 'RGB'
    CAMERA_RGBD = 'RGBD'

    def __init__(self, camera_id=0, camera_type='RGB',
                 frame_width=640, frame_height=480,
                 rgb_topic=None, depth_topic=None):
        """
        初始化相机管理器

        参数:
            camera_id: 相机ID (USB 相机)
            camera_type: 相机类型 ('RGB' 或 'RGBD')
            frame_width: 帧宽度 (像素)
            frame_height: 帧高度 (像素)
            rgb_topic: RGB 图像话题 (ROS 深度相机，可选)
            depth_topic: 深度图像话题 (ROS 深度相机，可选)
        """
        self.camera_id = camera_id
        self.camera_type = camera_type
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic

        self.cap = None
        self.is_rgbd = (camera_type == self.CAMERA_RGBD)

        rospy.loginfo('[CameraManager] 初始化相机管理器')
        rospy.loginfo('[CameraManager] 类型: %s, ID: %d, 分辨率: %dx%d',
                     camera_type, camera_id, frame_width, frame_height)

    def open(self):
        """
        打开相机

        返回:
            bool: 成功返回 True，失败返回 False
        """
        try:
            if self.is_rgbd:
                return self._open_rgbd_camera()
            else:
                return self._open_rgb_camera()
        except Exception as e:
            rospy.logerr('[CameraManager] 打开相机异常: %s', str(e))
            return False

    def _open_rgb_camera(self):
        """打开 RGB 相机"""
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if not self.cap.isOpened():
            rospy.logerr('[CameraManager] 无法打开 RGB 相机 %d', self.camera_id)
            return False

        # 测试读取一帧
        ret, frame = self.cap.read()
        if not ret or frame is None:
            rospy.logerr('[CameraManager] 无法从 RGB 相机读取图像')
            self.close()
            return False

        rospy.loginfo('[CameraManager] RGB 相机已打开')
        return True

    def _open_rgbd_camera(self):
        """
        打开 RGB-D 深度相机

        说明:
        目前支持 USB 深度相机，ROS 话题支持待实现
        """
        rospy.loginfo('[CameraManager] 尝试打开 RGB-D 深度相机')

        # 目前先按 RGB 相机处理，深度功能待实现
        # TODO: 实现深度相机支持 (RealSense / Azure Kinect 等)
        rospy.logwarn('[CameraManager] 深度相机支持待实现，当前使用 RGB 模式')
        return self._open_rgb_camera()

    def close(self):
        """关闭相机"""
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            rospy.loginfo('[CameraManager] 相机已关闭')

    def is_opened(self):
        """检查相机是否已打开"""
        return self.cap is not None and self.cap.isOpened()

    def capture_frame(self):
        """
        捕获 RGB 图像帧

        返回:
            numpy.ndarray: RGB 图像 (BGR 格式)，失败返回 None
        """
        if not self.is_opened():
            rospy.logerr('[CameraManager] 相机未打开')
            return None

        # 清空摄像头缓冲区，丢弃旧帧（最多丢弃5帧）
        for _ in range(5):
            self.cap.grab()

        ret, frame = self.cap.read()
        if not ret or frame is None:
            rospy.logerr('[CameraManager] 无法捕获图像')
            return None

        return frame

    def capture_depth_frame(self):
        """
        捕获深度图像帧 (仅 RGB-D 相机)

        返回:
            numpy.ndarray: 深度图像，失败返回 None
        """
        if not self.is_rgbd:
            rospy.logwarn('[CameraManager] 非深度相机，无法捕获深度图像')
            return None

        # TODO: 实现深度图像捕获
        rospy.logwarn('[CameraManager] 深度图像捕获未实现')
        return None

    def capture_rgbd_frames(self):
        """
        捕获 RGB-D 图像对 (仅 RGB-D 相机)

        返回:
            (rgb_frame, depth_frame): RGB 和深度图像
        """
        if not self.is_rgbd:
            rospy.logwarn('[CameraManager] 非深度相机，无法捕获 RGB-D')
            return None, None

        rgb_frame = self.capture_frame()
        depth_frame = self.capture_depth_frame()

        return rgb_frame, depth_frame

    def get_camera_info(self):
        """
        获取相机信息

        返回:
            dict: 相机信息字典
        """
        return {
            'camera_id': self.camera_id,
            'camera_type': self.camera_type,
            'frame_width': self.frame_width,
            'frame_height': self.frame_height,
            'is_opened': self.is_opened()
        }

    def __del__(self):
        """析构函数，自动关闭相机"""
        self.close()

    def __enter__(self):
        """支持上下文管理器"""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """支持上下文管理器"""
        self.close()
