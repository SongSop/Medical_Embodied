#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人脸识别服务节点
使用 face_recognition 包和 USB 摄像头实现真实的人脸识别
"""

import rospy
import face_recognition
import cv2
import numpy as np
import os
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces.srv import FaceIdentify, FaceIdentifyResponse


class FaceIdentifyServer:
    def __init__(self):
        rospy.init_node('face_identify_server', anonymous=True)

        # 摄像头配置
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.frame_width = rospy.get_param('~frame_width', 640)
        self.frame_height = rospy.get_param('~frame_height', 480)
        self.max_recognition_attempts = rospy.get_param('~max_recognition_attempts', 10)
        self.recognition_timeout = rospy.get_param('~recognition_timeout', 10.0)

        # 人脸数据库路径（相对路径，相对于 monitor 包）
        self.face_database_path = rospy.get_param('~face_database_path',
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '../face_database'))

        # 已知人脸编码数据库
        self.known_face_encodings = []
        self.known_face_ids = []

        # CV Bridge 用于图像转换
        self.bridge = CvBridge()

        # 发布调试图像
        self.debug_image_pub = rospy.Publisher('/face_identify/debug_image', Image, queue_size=10)

        # 加载人脸数据库
        self.load_face_database()

        # 打开摄像头
        self.cap = None
        self.open_camera()

        # 创建服务
        self.service = rospy.Service('face_identify', FaceIdentify, self.handle_face_identify)

        rospy.loginfo('[FaceIdentify] 人脸识别服务器已启动')
        rospy.loginfo('[FaceIdentify] 摄像头ID: %d, 分辨率: %dx%d', self.camera_id, self.frame_width, self.frame_height)
        rospy.loginfo('[FaceIdentify] 已加载 %d 张人脸', len(self.known_face_encodings))

    def load_face_database(self):
        """加载已知人脸数据库"""
        if not os.path.exists(self.face_database_path):
            rospy.logwarn('[FaceIdentify] 人脸数据库目录不存在: %s', self.face_database_path)
            os.makedirs(self.face_database_path)
            return

        # 遍历数据库目录，加载所有图像
        for filename in os.listdir(self.face_database_path):
            if filename.lower().endswith(('.jpg', '.jpeg', '.png')):
                filepath = os.path.join(self.face_database_path, filename)
                try:
                    # 读取图像
                    image = face_recognition.load_image_file(filepath)

                    # 检测人脸
                    face_locations = face_recognition.face_locations(image)

                    if len(face_locations) > 0:
                        # 只使用第一个检测到的人脸
                        face_encoding = face_recognition.face_encodings(image, face_locations)[0]
                        self.known_face_encodings.append(face_encoding)

                        # 从文件名提取 person_id (格式: person_id.jpg -> person_id)
                        person_id = -1
                        try:
                            person_id = int(os.path.splitext(filename)[0])
                        except ValueError:
                            rospy.logwarn('[FaceIdentify] 文件名格式错误，应为纯数字ID: %s', filename)

                        self.known_face_ids.append(person_id)

                        rospy.loginfo('[FaceIdentify] 加载人脸: %s (ID: %d)', filename, person_id)
                    else:
                        rospy.logwarn('[FaceIdentify] 图像中未检测到人脸: %s', filename)

                except Exception as e:
                    rospy.logerr('[FaceIdentify] 加载人脸失败 %s: %s', filename, str(e))

    def open_camera(self):
        """打开摄像头"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

            if not self.cap.isOpened():
                rospy.logerr('[FaceIdentify] 无法打开摄像头 %d', self.camera_id)
                return False

            # 测试读取一帧
            ret, frame = self.cap.read()
            if not ret or frame is None:
                rospy.logerr('[FaceIdentify] 无法从摄像头读取图像')
                self.cap.release()
                return False

            rospy.loginfo('[FaceIdentify] 摄像头已成功打开')
            return True

        except Exception as e:
            rospy.logerr('[FaceIdentify] 打开摄像头异常: %s', str(e))
            return False

    def capture_frame(self):
        """从摄像头捕获一帧图像"""
        if self.cap is None or not self.cap.isOpened():
            rospy.logerr('[FaceIdentify] 摄像头未打开')
            return None

        # 清空摄像头缓冲区，丢弃旧帧（最多丢弃5帧）
        for _ in range(5):
            self.cap.grab()

        ret, frame = self.cap.read()
        if not ret or frame is None:
            rospy.logerr('[FaceIdentify] 无法捕获图像')
            return None

        return frame

    def recognize_face(self, frame):
        """
        在图像中识别人脸
        返回: (person_id, confidence, debug_image)
        """
        if len(self.known_face_encodings) == 0:
            rospy.logwarn('[FaceIdentify] 人脸数据库为空，无法识别')
            return -1, 0.0, frame

        # 将 BGR 转换为 RGB (face_recognition 使用 RGB)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 检测人脸位置
        face_locations = face_recognition.face_locations(rgb_frame)

        if len(face_locations) == 0:
            rospy.loginfo('[FaceIdentify] 未检测到人脸')
            return -1, 0.0, frame

        # 获取人脸编码
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

        # 只识别第一个检测到的人脸
        if len(face_encodings) > 0:
            face_encoding = face_encodings[0]
            face_location = face_locations[0]

            # 与已知人脸比对
            distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)

            if len(distances) > 0:
                # 找到距离最小的（最相似）
                min_distance_idx = np.argmin(distances)
                min_distance = distances[min_distance_idx]

                # 距离阈值，小于 0.6 认为匹配成功
                if min_distance < 0.6:
                    person_id = self.known_face_ids[min_distance_idx]
                    confidence = (1.0 - min_distance) * 100

                    # 绘制人脸框和标签
                    label = f"ID {person_id}"
                    debug_image = self.draw_face_box(frame, face_location, label, confidence, True)

                    rospy.loginfo('[FaceIdentify] 识别成功: ID=%d (置信度: %.1f%%)', person_id, confidence)
                    return person_id, confidence, debug_image
                else:
                    rospy.loginfo('[FaceIdentify] 未匹配到已知人脸 (最小距离: %.3f)', min_distance)
                    debug_image = self.draw_face_box(frame, face_location, "Unknown", 0.0, False)
                    return -1, 0.0, debug_image

        return -1, 0.0, frame

    def draw_face_box(self, frame, face_location, name, confidence, matched):
        """在图像上绘制人脸识别结果"""
        top, right, bottom, left = face_location

        # 根据是否匹配选择颜色
        color = (0, 255, 0) if matched else (0, 0, 255)  # 绿色匹配，红色不匹配

        # 绘制人脸框
        cv2.rectangle(frame, (left, top), (right, bottom), color, 2)

        # 绘制标签
        label = name
        if matched:
            label = name

        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
        cv2.putText(frame, label, (left + 6, bottom - 6),
                   cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)

        return frame

    def handle_face_identify(self, req):
        """
        处理人脸识别请求
        关键：无论任何情况都返回 success=True，避免行为树卡死
        """
        rospy.loginfo('[FaceIdentify] ========== 收到新的人脸识别请求 ==========')

        # 如果摄像头未打开，尝试重新打开
        if self.cap is None or not self.cap.isOpened():
            rospy.logwarn('[FaceIdentify] 摄像头未打开，尝试重新打开')
            if not self.open_camera():
                # 摄像头打开失败，仍返回 success=True 但 person_id=-1
                rospy.logwarn('[FaceIdentify] 摄像头打开失败，返回默认值继续流程')
                return FaceIdentifyResponse(success=True, person_id=-1, confidence=0.0, message="摄像头打开失败")

        start_time = time.time()
        person_id = -1
        confidence = 0.0
        last_debug_image = None

        # 尝试多次识别
        for attempt in range(self.max_recognition_attempts):
            # 检查超时
            if time.time() - start_time > self.recognition_timeout:
                rospy.logwarn('[FaceIdentify] 识别超时 (%.1f秒)，返回默认值继续流程', self.recognition_timeout)
                break

            rospy.loginfo('[FaceIdentify] 识别尝试 %d/%d', attempt + 1, self.max_recognition_attempts)

            # 捕获图像
            frame = self.capture_frame()
            if frame is None:
                rospy.logwarn('[FaceIdentify] 无法捕获图像')
                time.sleep(0.3)
                continue

            rospy.loginfo('[FaceIdentify] 成功捕获图像帧，开始识别人脸')

            # 识别人脸
            person_id, confidence, last_debug_image = self.recognize_face(frame)

            # 如果识别成功（person_id > -1），返回结果
            if person_id > -1:
                rospy.loginfo('[FaceIdentify] 识别成功，返回 person_id=%d', person_id)
                return FaceIdentifyResponse(
                    success=True,
                    person_id=person_id,
                    confidence=confidence,
                    message=f"识别成功: person_id={person_id}"
                )

            # 等待一小段时间再试
            time.sleep(0.3)

        # 发布最后一张调试图像
        if last_debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(last_debug_image, encoding="bgr8")
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                rospy.logwarn('[FaceIdentify] 发布调试图像失败: %s', str(e))

        # 所有尝试都失败或超时，返回 success=True, person_id=-1 让行为树继续执行
        rospy.loginfo('[FaceIdentify] 未识别到已知人脸，返回 person_id=-1 继续流程')
        return FaceIdentifyResponse(
            success=True,
            person_id=-1,
            confidence=0.0,
            message="未识别到已知人脸"
        )

    def shutdown(self):
        """关闭节点，释放资源"""
        rospy.loginfo('[FaceIdentify] 正在关闭...')
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            rospy.loginfo('[FaceIdentify] 摄像头已关闭')


def main():
    try:
        server = FaceIdentifyServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'server' in locals():
            server.shutdown()


if __name__ == '__main__':
    main()
