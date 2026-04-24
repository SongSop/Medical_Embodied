#!/usr/bin/env python3
"""
床位检测节点 - Area/Bed模式实现
整合版本: 用于Medical_Embodied项目的monitor包

功能: 
  Area模式: 扫描指定区域,返回有人的床位ID列表
  Bed模式:  检测指定床位是否有人(异常),返回is_anomaly

服务接口: /detect_anomaly (继承自interfaces/DetectAnomaly.srv)
"""
import os
import json
import rclpy
import logging
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from interfaces.srv import DetectAnomaly

import cv2
import numpy as np
import torch
from PIL import Image as PILImage
from ultralytics import YOLO
import open_clip

# 模式枚举
MODE_AREA = 0  # 区域扫描模式
MODE_BED = 1   # 单床检测模式

class BedDetectionNode(Node):
    def __init__(self):
        super().__init__('bed_detection_node')

        # 创建CV桥
        self.bridge = CvBridge()

        # 声明参数用于配置模型路径
        self.declare_parameter('yolo_model_path', '')
        self.declare_parameter('clip_model_path', '')
        self.declare_parameter('max_beds', 10)

        # 获取参数值
        yolo_model_param = self.get_parameter('yolo_model_path').value
        clip_model_param = self.get_parameter('clip_model_path').value
        self.max_beds = self.get_parameter('max_beds').value

        # 确定源代码目录路径（简单直接的方法）
        # 使用当前Python文件位置找到src/monitor/models目录
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        src_monitor_dir = os.path.dirname(current_file_dir)  # monitor目录
        self.src_models_dir = os.path.join(src_monitor_dir, 'models')

        # 加载巡诊点-床位映射配置
        self.declare_parameter('patrol_bed_mapping_path', '')
        mapping_param = self.get_parameter('patrol_bed_mapping_path').value
        if mapping_param:
            mapping_path = mapping_param
        else:
            mapping_path = os.path.join(src_monitor_dir, 'config', 'patrol_bed_mapping.json')
        self.patrol_bed_map = self._load_patrol_bed_mapping(mapping_path)
        
        # 记录路径信息
        self.get_logger().info(f'Python文件目录: {current_file_dir}')
        self.get_logger().info(f'监控包源代码目录: {src_monitor_dir}')
        self.get_logger().info(f'模型目录: {self.src_models_dir}')

        # YOLO模型路径 - 优先使用参数配置，否则使用源代码目录路径
        if yolo_model_param:
            yolo_model_path = yolo_model_param
        else:
            # 直接使用源代码目录下的models
            yolo_model_path = os.path.join(self.src_models_dir, 'best.pt')
            self.get_logger().info(f'使用默认YOLO模型路径: {yolo_model_path}')
        
        try:
            self.yolo_model = YOLO(yolo_model_path)
            self.get_logger().info(f'YOLOv8模型加载成功: {yolo_model_path}')
        except Exception as e:
            self.get_logger().error(f'YOLOv8模型加载失败: {e}')
            # 尝试备用路径（同样是源代码目录）
            fallback_path = os.path.join(self.src_models_dir, 'bed_detector.pt')
            try:
                self.yolo_model = YOLO(fallback_path)
                self.get_logger().info(f'使用备用YOLOv8模型: {fallback_path}')
            except Exception as e2:
                self.get_logger().error(f'备用YOLOv8模型也失败: {e2}')
                raise

        # CLIP模型路径 - 优先使用参数配置，否则使用源代码目录路径
        if clip_model_param:
            clip_model_path = clip_model_param
        else:
            # 直接使用源代码目录下的models
            clip_model_path = os.path.join(self.src_models_dir, 'best_model.pt')
            self.get_logger().info(f'使用默认CLIP模型路径: {clip_model_path}')
        
        try:
            # 加载检查点
            checkpoint = torch.load(clip_model_path, map_location='cpu')
            state_dict = checkpoint['clip_model']
            
            # 基于模型检查结果,使用ViT-L-14架构
            model_name = 'ViT-L-14'
            
            # 首先加载模型架构,但不打印其内部警告
            clip_logger = logging.getLogger('root')
            original_level = clip_logger.level
            clip_logger.setLevel(logging.ERROR)
            
            try:
                self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms(
                    model_name, pretrained=None
                )
            finally:
                clip_logger.setLevel(original_level)
            
            # 安全地加载权重,确保键匹配
            model_state_dict = self.clip_model.state_dict()
            
            # 过滤掉不匹配的键
            filtered_state_dict = {}
            for key, value in state_dict.items():
                if key in model_state_dict and value.shape == model_state_dict[key].shape:
                    filtered_state_dict[key] = value
                else:
                    self.get_logger().warn(f"跳过不匹配的权重: {key}")
            
            # 加载权重
            missing_keys, unexpected_keys = self.clip_model.load_state_dict(filtered_state_dict, strict=False)
            
            if missing_keys:
                self.get_logger().warn(f"缺失的权重键: {missing_keys}")
            if unexpected_keys:
                self.get_logger().warn(f"意外的权重键: {unexpected_keys}")
            
            self.clip_tokenizer = open_clip.get_tokenizer(model_name)
            
            # 将模型设置为评估模式
            self.clip_model.eval()
            
            # 将模型移动到合适的设备
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.clip_model = self.clip_model.to(device)
            
            # 检查训练结果信息
            if 'accuracy' in checkpoint:
                self.get_logger().info(f'训练准确率: {checkpoint["accuracy"]:.4f}')
            if 'class_names' in checkpoint:
                self.get_logger().info(f'训练类别: {checkpoint["class_names"]}')
            
            self.get_logger().info(f'CLIP模型加载成功(设备: {device}): {clip_model_path}')
            self.get_logger().info(f'加载了 {len(filtered_state_dict)}/{len(state_dict)} 个权重参数')
            
        except Exception as e:
            self.get_logger().error(f'CLIP模型加载失败: {e}')
            raise

        # 存储最新图像
        self.current_image = None
        self.image_received = False

        # 订阅相机话题
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('已订阅相机话题: /camera/rgb/image_raw')

        # 发布模式切换指令到模拟相机
        self.mode_pub = self.create_publisher(String, '/camera/mode', 10)

        # 创建床位检测服务 - 使用与anomaly_detect_server相同的服务名称
        self.detect_service = self.create_service(
            DetectAnomaly,
            '/detect_anomaly',
            self.handle_detect_request
        )
        self.get_logger().info('床位检测服务已创建: /detect_anomaly')

    def _load_patrol_bed_mapping(self, path):
        """加载巡诊点-床位映射JSON"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            mapping = {}
            for item in data.get('patrol_points', []):
                pid = item['patrol_id']
                # 新格式: beds = [{"detection_index": 0, "bed_id": 1}, ...]
                # 旧格式兼容: bed_ids = [1, 2, 3, ...]
                if 'beds' in item:
                    bed_map = {}
                    for bed_info in item['beds']:
                        bed_map[bed_info['detection_index']] = bed_info['bed_id']
                    mapping[pid] = bed_map
                elif 'bed_ids' in item:
                    bed_map = {i: bid for i, bid in enumerate(item['bed_ids'])}
                    mapping[pid] = bed_map
            self.get_logger().info(f'加载巡诊点-床位映射: {path}, {len(mapping)} 个巡诊点')
            for pid, bed_map in mapping.items():
                self.get_logger().info(
                    f'  巡诊点 {pid} -> ' +
                    ', '.join(f'det[{k}]=bed{v}' for k, v in sorted(bed_map.items()))
                )
            return mapping
        except Exception as e:
            self.get_logger().warn(f'加载巡诊点-床位映射失败: {e}, 使用默认顺序编号')
            return {}

    def _get_bed_id(self, patrol_id, detection_index):
        """根据巡诊点ID和YOLO检测框索引获取对应的导航床位编号"""
        if patrol_id in self.patrol_bed_map:
            bed_map = self.patrol_bed_map[patrol_id]
            if detection_index in bed_map:
                return bed_map[detection_index]
        # 无映射时按顺序编号 (index 0 -> bed_id 1)
        return detection_index + 1

    def _get_detection_index_for_bed(self, bed_id):
        """根据导航床位编号反查YOLO检测框索引，遍历所有巡诊点映射"""
        for pid, bed_map in self.patrol_bed_map.items():
            for det_idx, bid in bed_map.items():
                if bid == bed_id:
                    self.get_logger().info(
                        f'床位 {bed_id} 在巡诊点 {pid} 映射中, 检测索引 {det_idx}'
                    )
                    return det_idx
        # 无映射时 bed_id-1 即索引
        return bed_id - 1

    def image_callback(self, msg):
        """接收相机图像"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')

    def detect_beds_with_yolo(self, image):
        """
        使用YOLOv8检测图像中的床位

        Returns:
            List[dict]: [{'bbox': [x1,y1,x2,y2], 'class_id': 0, 'confidence': 0.95}, ...]
        """
        results = self.yolo_model(image, verbose=False)

        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                class_id = int(box.cls[0].cpu().numpy())
                confidence = float(box.conf[0].cpu().numpy())

                detections.append({
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'class_id': class_id,
                    'confidence': confidence
                })

        return detections

    def detect_person_with_clip(self, bed_image):
        """
        使用CLIP判断床位上是否有人 (Area模式)

        Args:
            bed_image (np.ndarray): 床位区域图像

        Returns:
            tuple: (bool, float) - (是否有人, 置信度)
        """
        text = ["a patient lying on a hospital bed", "an empty hospital bed"]
        return self._clip_classify(bed_image, text)

    def detect_fall_risk_with_clip(self, bed_image):
        """
        使用CLIP判断病人是否有坠床风险 (Bed模式)

        Args:
            bed_image (np.ndarray): 床位区域图像

        Returns:
            tuple: (bool, float) - (是否有坠床风险, 置信度)
        """
        text = [
            "a patient about to fall off a hospital bed",
            "a patient safely lying in a hospital bed"
        ]
        return self._clip_classify(bed_image, text)

    def _clip_classify(self, bed_image, text_prompts):
        """
        CLIP通用分类方法

        Args:
            bed_image (np.ndarray): 床位区域图像
            text_prompts (list[str]): 两个文本提示, 第一个为"异常/正向", 第二个为"正常/负向"

        Returns:
            tuple: (bool, float) - (第一个提示得分更高, 第一个提示的置信度)
        """
        try:
            bed_image_rgb = cv2.cvtColor(bed_image, cv2.COLOR_BGR2RGB)
            bed_image_pil = PILImage.fromarray(bed_image_rgb)
            image_tensor = self.clip_preprocess(bed_image_pil).unsqueeze(0)

            device = next(self.clip_model.parameters()).device
            text_tokens = self.clip_tokenizer(text_prompts).to(device)
            image_tensor = image_tensor.to(device)

            with torch.no_grad():
                text_features = self.clip_model.encode_text(text_tokens)
                image_features = self.clip_model.encode_image(image_tensor)

            similarity = (image_features @ text_features.T).softmax(dim=-1)
            positive_score = similarity[0][0].item()
            negative_score = similarity[0][1].item()

            is_positive = positive_score > negative_score
            return is_positive, positive_score

        except Exception as e:
            self.get_logger().error(f'CLIP检测失败: {e}')
            return False, 0.0

    def handle_detect_request(self, request, response):
        """
        处理床位检测服务请求 - Area和Bed模式
        """
        if not self.image_received or self.current_image is None:
            self.get_logger().warn('未收到相机图像')
            response.is_anomaly = False
            response.details = "No image available"
            response.bed_ids = []
            response.urgencies = []
            return response

        if request.mode == MODE_AREA:
            return self._handle_area_mode(request, response)
        elif request.mode == MODE_BED:
            return self._handle_bed_mode(request, response)
        else:
            self.get_logger().warn(f'未知模式: {request.mode}, 支持Area(0)/Bed(1)')
            response.is_anomaly = False
            response.details = f"Unknown mode: {request.mode}"
            response.bed_ids = []
            response.urgencies = []
            return response

    def _switch_camera_mode(self, mode_name):
        """通知模拟相机切换图片模式"""
        msg = String()
        msg.data = mode_name
        self.mode_pub.publish(msg)
        self.get_logger().info(f'已发送相机模式切换: {mode_name}')

    def _handle_area_mode(self, request, response):
        """
        Area模式处理逻辑
        根据巡诊点ID扫描对应床位,返回有人床位的ID列表
        """
        self._switch_camera_mode('area')
        patrol_id = request.area_bed_id
        self.get_logger().info(f'开始Area模式检测, 巡诊点ID: {patrol_id}')

        # 使用YOLOv8检测所有床位
        bed_detections = self.detect_beds_with_yolo(self.current_image)

        if not bed_detections:
            self.get_logger().warn('未检测到任何床位')
            response.is_anomaly = False
            response.details = "No beds detected"
            response.bed_ids = []
            response.urgencies = []
            return response

        num_detected = len(bed_detections)
        self.get_logger().info(f'检测到 {num_detected} 个床位')

        occupied_beds = []
        urgencies = []

        # 对每个检测到的床位进行CLIP判断
        for i, detection in enumerate(bed_detections):
            bbox = detection['bbox']
            confidence = detection['confidence']

            # 裁剪床位区域
            x1, y1, x2, y2 = bbox
            bed_image = self.current_image[y1:y2, x1:x2]

            # 跳过过小的区域
            if bed_image.size == 0:
                continue

            # 使用CLIP判断是否有人
            is_person, person_score = self.detect_person_with_clip(bed_image)

            # 从映射表获取床位导航编号
            bed_id = self._get_bed_id(patrol_id, i)

            if is_person:
                occupied_beds.append(bed_id)

                # 紧急程度:根据CLIP置信度设置
                urgency = 1 if person_score > 0.7 else 0
                urgencies.append(urgency)

                self.get_logger().info(
                    f'床位 {bed_id} 有人, CLIP置信度: {person_score:.3f}, '
                    f'YOLO置信度: {confidence:.3f}'
                )

        response.is_anomaly = len(occupied_beds) > 0
        response.details = f"Detected {len(occupied_beds)} occupied beds out of {len(bed_detections)} total beds"
        response.bed_ids = occupied_beds
        response.urgencies = urgencies

        self.get_logger().info(
            f'Area模式检测完成: {len(occupied_beds)} 个有人床位 -> {occupied_beds}'
        )

        return response

    def _handle_bed_mode(self, request, response):
        """
        Bed模式处理逻辑
        检测指定床位是否有人(异常),返回is_anomaly
        
        Args:
            request.area_bed_id: 目标床位ID (来自映射表)
        """
        self._switch_camera_mode('bed')
        target_bed_id = request.area_bed_id
        self.get_logger().info(f'开始Bed模式检测, 目标床位ID: {target_bed_id}')

        # 使用YOLOv8检测所有床位
        bed_detections = self.detect_beds_with_yolo(self.current_image)

        if not bed_detections:
            self.get_logger().warn('未检测到任何床位')
            response.is_anomaly = False
            response.details = "No beds detected"
            response.bed_ids = []
            response.urgencies = []
            return response

        num_detected = len(bed_detections)
        self.get_logger().info(f'检测到 {num_detected} 个床位')

        # 通过映射表将床位ID反查为检测索引
        det_idx = self._get_detection_index_for_bed(target_bed_id)

        if det_idx < 0 or det_idx >= num_detected:
            self.get_logger().warn(
                f'目标床位ID {target_bed_id} 对应检测索引 {det_idx} 超出范围 (0-{num_detected - 1})'
            )
            response.is_anomaly = False
            response.details = f"Bed ID {target_bed_id} (index {det_idx}) out of range"
            response.bed_ids = []
            response.urgencies = []
            return response

        # 获取目标床位的检测结果
        detection = bed_detections[det_idx]
        bbox = detection['bbox']
        yolo_confidence = detection['confidence']

        # 裁剪目标床位区域
        x1, y1, x2, y2 = bbox
        bed_image = self.current_image[y1:y2, x1:x2]

        if bed_image.size == 0:
            self.get_logger().warn(f'床位 {target_bed_id} 裁剪区域为空')
            response.is_anomaly = False
            response.details = f"Bed {target_bed_id} crop region is empty"
            response.bed_ids = []
            response.urgencies = []
            return response

        # 使用CLIP判断是否有坠床风险
        is_risk, risk_score = self.detect_fall_risk_with_clip(bed_image)

        # Bed模式: 有坠床风险即为异常
        is_anomaly = is_risk
        response.is_anomaly = is_anomaly
        response.details = (
            f"Bed {target_bed_id}: {'fall risk detected' if is_anomaly else 'patient safe'}, "
            f"CLIP confidence: {risk_score:.3f}, YOLO confidence: {yolo_confidence:.3f}"
        )
        response.bed_ids = [target_bed_id] if is_anomaly else []
        response.urgencies = [1 if risk_score > 0.7 else 0] if is_anomaly else []

        self.get_logger().info(
            f'Bed模式检测完成: 床位 {target_bed_id} '
            f'{"有坠床风险(异常)" if is_anomaly else "病人安全(正常)"}, '
            f'CLIP置信度: {risk_score:.3f}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = BedDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()