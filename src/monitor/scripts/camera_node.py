#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立相机节点
持续打开摄像头，发布RGB图像到ROS话题

设计理念:
- 相机资源统一管理，避免多个服务冲突
- 持续发布图像，订阅者可以随时获取最新帧
- 符合ROS发布/订阅架构规范
"""

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class CameraNode:
    """
    相机节点类

    功能:
    1. 持续打开指定摄像头
    2. 定期捕获图像帧
    3. 发布到 ROS 话题供其他节点订阅
    """

    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)

        # 相机配置参数
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.frame_width = rospy.get_param('~frame_width', 640)
        self.frame_height = rospy.get_param('~frame_height', 480)
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)  # 发布频率 Hz

        # 话题名称
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/rgb/camera_info')

        # 摄像头对象
        self.cap = None
        self.bridge = CvBridge()

        # 发布者
        self.image_pub = rospy.Publisher(self.rgb_topic, Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=1, latch=True)

        # 初始化摄像头
        if not self.open_camera():
            rospy.logerr('[CameraNode] 摄像头初始化失败，节点退出')
            rospy.signal_shutdown('Camera initialization failed')
            return

        # 发布相机信息
        self.publish_camera_info()

        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)

        rospy.loginfo('[CameraNode] ========== 相机节点已启动 ==========')
        rospy.loginfo('[CameraNode] 摄像头ID: %d', self.camera_id)
        rospy.loginfo('[CameraNode] 分辨率: %dx%d', self.frame_width, self.frame_height)
        rospy.loginfo('[CameraNode] 发布频率: %.1f Hz', self.publish_rate)
        rospy.loginfo('[CameraNode] RGB话题: %s', self.rgb_topic)
        rospy.loginfo('[CameraNode] ======================================')

    def open_camera(self):
        """打开摄像头"""
        try:
            rospy.loginfo('[CameraNode] 正在打开摄像头 %d...', self.camera_id)
            self.cap = cv2.VideoCapture(self.camera_id)

            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

            # 设置帧率
            self.cap.set(cv2.CAP_PROP_FPS, self.publish_rate)

            if not self.cap.isOpened():
                rospy.logerr('[CameraNode] 无法打开摄像头 %d', self.camera_id)
                return False

            # 测试读取一帧
            ret, frame = self.cap.read()
            if not ret or frame is None:
                rospy.logerr('[CameraNode] 无法从摄像头读取图像')
                self.cap.release()
                return False

            rospy.loginfo('[CameraNode] 摄像头已成功打开')
            return True

        except Exception as e:
            rospy.logerr('[CameraNode] 打开摄像头异常: %s', str(e))
            return False

    def publish_camera_info(self):
        """发布相机标定信息"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera_optical_frame"
        camera_info.width = self.frame_width
        camera_info.height = self.frame_height

        # 默认相机内参（实际使用时应该使用标定数据）
        fx = self.frame_width  # 焦距
        fy = self.frame_height
        cx = self.frame_width / 2.0
        cy = self.frame_height / 2.0

        camera_info.K = [fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0]

        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # 畸变系数
        camera_info.P = [fx, 0.0, cx, 0.0,
                         0.0, fy, cy, 0.0,
                         0.0, 0.0, 1.0, 0.0]

        self.camera_info_pub.publish(camera_info)
        rospy.loginfo('[CameraNode] 相机信息已发布')

    def timer_callback(self, event):
        """定时器回调，捕获并发布图像"""
        if self.cap is None or not self.cap.isOpened():
            rospy.logwarn('[CameraNode] 摄像头未打开')
            return

        try:
            # 捕获一帧
            ret, frame = self.cap.read()
            if not ret or frame is None:
                rospy.logwarn('[CameraNode] 无法捕获图像')
                return

            # 转换为ROS消息
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = "camera_optical_frame"

                # 发布图像
                self.image_pub.publish(ros_image)

            except Exception as e:
                rospy.logerr('[CameraNode] 图像转换失败: %s', str(e))

        except Exception as e:
            rospy.logerr('[CameraNode] 捕获图像异常: %s', str(e))

    def close(self):
        """关闭摄像头"""
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            rospy.loginfo('[CameraNode] 摄像头已关闭')

    def __del__(self):
        """析构函数"""
        self.close()


if __name__ == '__main__':
    try:
        camera_node = CameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'camera_node' in locals():
            camera_node.close()
