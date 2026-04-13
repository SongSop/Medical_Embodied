#!/usr/bin/env python3
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


class FaceIdentifyServer(Node):
    def __init__(self):
        super().__init__('face_identify_server')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('max_recognition_attempts', 10)
        self.declare_parameter('recognition_timeout', 10.0)
        self.declare_parameter('face_database_path', '')

        self.camera_id = int(self.get_parameter('camera_id').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.max_recognition_attempts = int(self.get_parameter('max_recognition_attempts').value)
        self.recognition_timeout = float(self.get_parameter('recognition_timeout').value)

        db_path = str(self.get_parameter('face_database_path').value).strip()
        if not db_path:
            db_path = os.path.join(os.path.dirname(__file__), '..', 'face_database')
        self.face_database_path = os.path.abspath(db_path)

        self.known_face_encodings = []
        self.known_face_ids = []
        self.bridge = CvBridge()

        self.debug_image_pub = self.create_publisher(Image, '/face_identify/debug_image', 10)
        self._srv = self.create_service(FaceIdentify, 'face_identify', self.handle_face_identify)

        self.cap = None
        self.load_face_database()
        self.open_camera()

        self.get_logger().info('face_identify_server started')

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

    def open_camera(self):
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            if not self.cap.isOpened():
                self.get_logger().warning('camera open failed')
                return False
            return True
        except Exception as exc:
            self.get_logger().warning(f'camera exception: {exc}')
            return False

    def capture_frame(self):
        if self.cap is None or not self.cap.isOpened():
            return None
        for _ in range(5):
            self.cap.grab()
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def recognize_face(self, frame):
        if not self.known_face_encodings:
            return -1, 0.0, frame
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        locs = face_recognition.face_locations(rgb)
        if not locs:
            return -1, 0.0, frame
        encs = face_recognition.face_encodings(rgb, locs)
        if not encs:
            return -1, 0.0, frame

        distances = face_recognition.face_distance(self.known_face_encodings, encs[0])
        if len(distances) == 0:
            return -1, 0.0, frame
        idx = int(np.argmin(distances))
        min_distance = float(distances[idx])
        if min_distance < 0.6:
            confidence = (1.0 - min_distance) * 100.0
            return int(self.known_face_ids[idx]), confidence, frame
        return -1, 0.0, frame

    def handle_face_identify(self, _request, response):
        if self.cap is None or not self.cap.isOpened():
            self.open_camera()

        start_time = time.time()
        last_debug = None

        for _ in range(self.max_recognition_attempts):
            if time.time() - start_time > self.recognition_timeout:
                break
            frame = self.capture_frame()
            if frame is None:
                time.sleep(0.2)
                continue
            person_id, confidence, debug_image = self.recognize_face(frame)
            last_debug = debug_image
            if person_id > -1:
                response.success = True
                response.person_id = person_id
                response.confidence = float(confidence)
                response.message = f'recognized person_id={person_id}'
                return response
            time.sleep(0.2)

        if last_debug is not None:
            try:
                self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(last_debug, encoding='bgr8'))
            except Exception:
                pass

        response.success = True
        response.person_id = -1
        response.confidence = 0.0
        response.message = 'unknown face'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FaceIdentifyServer()
    try:
        rclpy.spin(node)
    finally:
        if node.cap is not None and node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
