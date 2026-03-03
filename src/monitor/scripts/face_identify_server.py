#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人脸识别服务节点
使用 face_recognition 包和订阅相机话题实现真实的人脸识别

架构改进:
- 不再直接打开摄像头
- 订阅独立相机节点发布的图像话题
- 符合ROS发布/订阅架构规范
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

        # 相机话题配置
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image_raw')

        # 识别配置
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

        # 存储最新的相机帧
        self.latest_frame = None

        # 加载人脸数据库
        self.load_face_database()

        # 订阅相机话题
        self.camera_sub = rospy.Subscriber(self.rgb_topic, Image, self.camera_callback)
        rospy.loginfo('[FaceIdentify] 订阅相机话题: %s', self.rgb_topic)

        # 创建服务
        self.service = rospy.Service('face_identify', FaceIdentify, self.handle_face_identify)

        rospy.loginfo('[FaceIdentify] 人脸识别服务器已启动 (订阅相机话题模式)')
        rospy.loginfo('[FaceIdentify] 已加载 %d 张人脸', len(self.known_face_encodings))

    def camera_callback(self, msg):
        """相机话题回调，存储最新帧"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr('[FaceIdentify] 图像转换失败: %s', str(e))

    def get_latest_frame(self, timeout=1.0):
        """
        获取最新的相机帧

        参数:
            timeout: 超时时间(秒)

        返回:
            numpy.ndarray: 图像帧，超时返回 None
        """
        if self.latest_frame is not None:
            return self.latest_frame

        # 等待新帧
        start_time = rospy.Time.now()
        rate = rospy.Rate(30)
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.latest_frame is not None:
                return self.latest_frame
            rate.sleep()

        rospy.logwarn('[FaceIdentify] 等待相机帧超时')
        return None

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
        从订阅的相机话题获取图像
        """
        rospy.loginfo('[FaceIdentify] ========== 收到新的人脸识别请求 ==========')

        start_time = time.time()
        person_id = -1
        confidence = 0.0
        last_debug_image = None

        # 尝试多次识别
        for attempt in range(self.max_recognition_attempts):
            # 检查超时
            if time.time() - start_time > self.recognition_timeout:
                rospy.logwarn('[FaceIdentify] 识别超时 (%.1f秒)', self.recognition_timeout)
                break

            rospy.loginfo('[FaceIdentify] 识别尝试 %d/%d', attempt + 1, self.max_recognition_attempts)

            # 从相机话题获取最新帧
            frame = self.get_latest_frame(timeout=1.0)
            if frame is None:
                rospy.logwarn('[FaceIdentify] 无法获取相机帧')
                time.sleep(0.3)
                continue

            rospy.loginfo('[FaceIdentify] 成功获取图像帧，开始识别人脸')

            # 识别人脸
            person_id, confidence, debug_image = self.recognize_face(frame)
            last_debug_image = debug_image

            # 发布调试图像
            if last_debug_image is not None:
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(last_debug_image, encoding='bgr8')
                    self.debug_image_pub.publish(ros_image)
                except Exception as e:
                    rospy.logwarn('[FaceIdentify] 发布调试图像失败: %s', str(e))

            # 如果识别成功（person_id != -1），立即返回
            if person_id != -1:
                message = f"识别成功: ID={person_id}, 置信度={confidence:.1f}%"
                rospy.loginfo('[FaceIdentify] %s', message)
                return FaceIdentifyResponse(success=True, person_id=person_id,
                                             confidence=confidence, message=message)

            # 如果未识别到人脸，等待一段时间再试
            time.sleep(0.3)

        # 所有尝试都失败
        message = f"识别失败 (尝试{self.max_recognition_attempts}次)"
        rospy.logwarn('[FaceIdentify] %s', message)

        # 发布最后一次调试图像
        if last_debug_image is not None:
            try:
                ros_image = self.bridge.cv2_to_imgmsg(last_debug_image, encoding='bgr8')
                self.debug_image_pub.publish(ros_image)
            except Exception as e:
                rospy.logwarn('[FaceIdentify] 发布调试图像失败: %s', str(e))

        return FaceIdentifyResponse(success=True, person_id=-1, confidence=0.0, message=message)


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
