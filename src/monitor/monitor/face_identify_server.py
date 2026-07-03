#!/usr/bin/env python3
"""
人脸识别服务节点
通过可配置的相机话题获取图像（默认RealSense L515: /camera/camera/color/image_raw）
"""
import os
import time

import cv2
import face_recognition
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from interfaces.srv import FaceIdentify
from std_msgs.msg import String


class FaceIdentifyServer(Node):
    def __init__(self):
        super().__init__('face_identify_server')

        self.declare_parameter('max_recognition_attempts', 5)
        self.declare_parameter('recognition_timeout', 10.0)
        self.declare_parameter('face_database_path', '')
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')

        self.max_recognition_attempts = int(self.get_parameter('max_recognition_attempts').value)
        self.recognition_timeout = float(self.get_parameter('recognition_timeout').value)
        self.camera_topic = self.get_parameter('camera_topic').value

        db_path = str(self.get_parameter('face_database_path').value).strip()
        if not db_path:
            db_path = os.path.join(os.path.dirname(__file__), '..', 'face_database')
        self.face_database_path = os.path.abspath(db_path)

        self.known_face_encodings = []
        self.known_face_ids = []
        self.bridge = CvBridge()

        # 订阅相机话题获取图像（通过参数配置，默认使用RealSense L515彩色图像话题）
        self.current_image = None
        self.image_received = False
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self._image_callback, 10
        )
        self.get_logger().info(f'已订阅相机话题: {self.camera_topic}')

        # 发布模式切换指令到模拟相机
        self.mode_pub = self.create_publisher(String, '/camera/mode', 10)

        self.debug_image_pub = self.create_publisher(Image, '/face_identify/debug_image', 10)
        self._srv = self.create_service(FaceIdentify, '/face_identify', self.handle_face_identify)

        self.load_face_database()
        self.get_logger().info('face_identify_server started')

    def _image_callback(self, msg):
        """接收相机图像"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')

    def load_face_database(self):
        if not os.path.exists(self.face_database_path):
            os.makedirs(self.face_database_path, exist_ok=True)
            self.get_logger().warning(f'face database not found: {self.face_database_path}')
            return

        for filename in os.listdir(self.face_database_path):
            if not filename.lower().endswith(('.jpg', '.jpeg', '.png')):
                continue
            path = os.path.join(self.face_database_path, filename)
            try:
                image = face_recognition.load_image_file(path)
                locs = face_recognition.face_locations(image)
                if not locs:
                    continue
                enc = face_recognition.face_encodings(image, locs)[0]
                self.known_face_encodings.append(enc)
                try:
                    person_id = int(os.path.splitext(filename)[0])
                except ValueError:
                    person_id = -1
                self.known_face_ids.append(person_id)
            except Exception as exc:
                self.get_logger().warning(f'failed to load face {filename}: {exc}')

        self.get_logger().info(f'已加载 {len(self.known_face_encodings)} 个人脸编码')

    def recognize_face(self, frame):
        if not self.known_face_encodings:
            return -1, 0.0
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        locs = face_recognition.face_locations(rgb)
        if not locs:
            return -1, 0.0
        encs = face_recognition.face_encodings(rgb, locs)
        if not encs:
            return -1, 0.0

        distances = face_recognition.face_distance(self.known_face_encodings, encs[0])
        if len(distances) == 0:
            return -1, 0.0
        idx = int(np.argmin(distances))
        min_distance = float(distances[idx])
        if min_distance < 0.6:
            confidence = (1.0 - min_distance) * 100.0
            return int(self.known_face_ids[idx]), confidence
        return -1, 0.0

    def handle_face_identify(self, _request, response):
        # 通知模拟相机切换到face模式
        mode_msg = String()
        mode_msg.data = 'face'
        self.mode_pub.publish(mode_msg)

        start_time = time.time()
        last_debug = None

        for _ in range(self.max_recognition_attempts):
            if time.time() - start_time > self.recognition_timeout:
                break

            if not self.image_received or self.current_image is None:
                time.sleep(0.2)
                continue

            frame = self.current_image.copy()
            last_debug = frame
            person_id, confidence = self.recognize_face(frame)

            if person_id > -1:
                response.success = True
                response.person_id = person_id
                response.confidence = float(confidence)
                response.message = f'recognized person_id={person_id}'
                self.get_logger().info(
                    f'人脸识别成功: person_id={person_id}, confidence={confidence:.1f}%'
                )
                return response
            time.sleep(0.2)

        if last_debug is not None:
            try:
                self.debug_image_pub.publish(
                    self.bridge.cv2_to_imgmsg(last_debug, encoding='bgr8')
                )
            except Exception:
                pass

        response.success = True
        response.person_id = -1
        response.confidence = 0.0
        response.message = 'unknown face'
        self.get_logger().info('人脸识别: 未识别到已知人脸')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FaceIdentifyServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
