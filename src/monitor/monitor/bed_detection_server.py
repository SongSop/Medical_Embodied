#!/usr/bin/env python3
"""
床位检测节点 - Area/Bed模式实现
整合版本: 用于Medical_Embodied项目的monitor包

功能:
  Area模式: 扫描指定区域,返回有人的床位ID列表
  Bed模式:  检测指定床位是否有人(异常),返回is_anomaly

服务接口: /detect_anomaly (继承自interfaces/DetectAnomaly.srv)
相机话题: 通过camera_topic参数配置 (默认RealSense D455: /camera/camera/color/image_raw)
"""
import os
import json
import yaml
import rclpy
import logging
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
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

        # 声明参数用于配置模型路径和相机话题
        self.declare_parameter('yolo_model_path', '')
        self.declare_parameter('clip_model_path', '')
        self.declare_parameter('max_beds', 10)
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('yolo_conf_threshold', 0.5)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('use_depth_matching', True)
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')

        # 获取参数值
        yolo_model_param = self.get_parameter('yolo_model_path').value
        clip_model_param = self.get_parameter('clip_model_path').value
        self.max_beds = self.get_parameter('max_beds').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.yolo_conf_threshold = self.get_parameter('yolo_conf_threshold').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.use_depth_matching = self.get_parameter('use_depth_matching').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        # 解析资源目录（优先ROS安装share目录，回退源码目录）
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        src_monitor_dir = os.path.dirname(current_file_dir)
        self.share_dir, self.src_root_dir = self._resolve_resource_roots(current_file_dir)
        self.src_models_dir = os.path.join(self.share_dir, 'models')

        # 加载巡诊点-床位映射配置
        self.declare_parameter('patrol_bed_mapping_path', '')
        mapping_param = self.get_parameter('patrol_bed_mapping_path').value
        if mapping_param:
            mapping_path = mapping_param
        else:
            mapping_path = os.path.join(self.share_dir, 'config', 'patrol_bed_mapping.json')
        self.patrol_bed_map = self._load_patrol_bed_mapping(mapping_path)

        # 加载深度相机空间匹配配置（waypoint→map坐标映射）
        self.declare_parameter('patrol_waypoint_mapping_path', '')
        waypoint_mapping_param = self.get_parameter('patrol_waypoint_mapping_path').value
        if waypoint_mapping_param:
            waypoint_mapping_path = waypoint_mapping_param
        else:
            waypoint_mapping_path = os.path.join(self.share_dir, 'config', 'patrol_waypoint_mapping.json')
        self.waypoint_config = self._load_patrol_waypoint_mapping(waypoint_mapping_path)
        self.bed_waypoints = self._load_waypoints(self.waypoint_config.get('waypoints_config_path', ''))

        # 记录路径信息
        self.get_logger().info(f'Python文件目录: {current_file_dir}')
        self.get_logger().info(f'监控包资源目录(share/src): {self.share_dir}')
        self.get_logger().info(f'监控包源码根目录回退路径: {self.src_root_dir}')
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
            checkpoint = torch.load(clip_model_path, map_location='cpu', weights_only=False)
            self.clip_class_names = checkpoint.get('class_names', ['bed_empty', 'bed_with_person'])

            # open_clip 加载 ViT-L-14 架构
            model_name = 'ViT-L-14'
            self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms(
                model_name, pretrained=None
            )

            # 加载 CLIP 编码器权重（兼容 open_clip / openai clip 键名差异）
            state_dict = checkpoint['clip_model']
            model_state_dict = self.clip_model.state_dict()
            filtered_state_dict = {}
            for key, value in state_dict.items():
                if key in model_state_dict and value.shape == model_state_dict[key].shape:
                    filtered_state_dict[key] = value
            self.clip_model.load_state_dict(filtered_state_dict, strict=False)
            self.clip_model.eval()

            # 加载 Linear Probe 分类器头
            feature_dim = checkpoint.get('feature_dim', 768)
            self.clip_classifier = torch.nn.Linear(feature_dim, len(self.clip_class_names))
            if 'classifier' in checkpoint:
                classifier_sd = checkpoint['classifier']
                sd = {}
                for key, value in classifier_sd.items():
                    sd[key.replace('linear.', '')] = value
                self.clip_classifier.load_state_dict(sd)
            self.clip_classifier.eval()

            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.clip_model = self.clip_model.to(device)
            self.clip_classifier = self.clip_classifier.to(device)

            if 'accuracy' in checkpoint:
                self.get_logger().info(f'训练准确率: {checkpoint["accuracy"]:.4f}')
            self.get_logger().info(f'训练类别: {self.clip_class_names}')
            self.get_logger().info(f'CLIP+Linear分类器加载成功(设备: {device}): {clip_model_path}')

        except Exception as e:
            self.get_logger().error(f'CLIP模型加载失败: {e}')
            raise

        # 存储最新图像和深度数据
        self.current_image = None
        self.image_received = False
        self.current_depth = None
        self.depth_received = False
        self.camera_intrinsics = None  # (fx, fy, cx, cy)

        # 订阅相机话题（通过参数配置，默认使用RealSense D455彩色图像话题）
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'已订阅相机话题: {self.camera_topic}')

        # 订阅对齐深度图（用于空间匹配）
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        self.get_logger().info(f'已订阅深度话题: {self.depth_topic}')

        # 订阅相机内参
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        self.get_logger().info(f'已订阅相机内参话题: {self.camera_info_topic}')

        # 发布模式切换指令到模拟相机（真实相机下无人监听，无害）
        self.mode_pub = self.create_publisher(String, '/camera/mode', 10)

        # 调试用：发布带检测框标注的图像
        self.debug_image_pub = self.create_publisher(Image, '/bed_detection/debug_image', 10)

        # 调试用：发布RViz Marker（巡诊点、床位waypoint、检测点）
        self.marker_pub = self.create_publisher(MarkerArray, '/bed_detection/markers', 10)

        # 创建床位检测服务 - 使用与anomaly_detect_server相同的服务名称
        self.detect_service = self.create_service(
            DetectAnomaly,
            '/detect_anomaly',
            self.handle_detect_request
        )
        self.get_logger().info('床位检测服务已创建: /detect_anomaly')

    def _resolve_resource_roots(self, current_file_dir):
        """返回 (resource_root, source_root_fallback)"""
        source_root = os.path.dirname(current_file_dir)
        try:
            share_dir = get_package_share_directory('monitor')
            return share_dir, source_root
        except Exception as e:
            self.get_logger().warn(f'获取monitor share目录失败: {e}, 使用源码目录')
            return source_root, source_root

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

    def _load_patrol_waypoint_mapping(self, path):
        """加载深度相机巡诊点→waypoint映射JSON"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            mapping = {}
            for item in data.get('patrol_points', []):
                pid = item['patrol_id']
                mapping[pid] = item.get('bed_waypoints', [])
            self.get_logger().info(
                f'加载巡诊点-waypoint映射: {path}, '
                f'{len(mapping)} 个巡诊点, '
                f'匹配距离阈值: {data.get("match_distance_threshold", 1.5)}m'
            )
            for pid, waypoints in mapping.items():
                self.get_logger().info(f'  巡诊点 {pid} -> {waypoints}')
            return data
        except Exception as e:
            self.get_logger().warn(f'加载巡诊点-waypoint映射失败: {e}, 深度匹配不可用')
            return {}

    def _find_workspace_root(self):
        """找到ROS2 workspace根目录（包含src/子目录）"""
        current = os.path.dirname(os.path.abspath(__file__))
        for _ in range(10):
            if os.path.isdir(os.path.join(current, 'src')):
                return current
            parent = os.path.dirname(current)
            if parent == current:
                break
            current = parent
        return None

    def _load_waypoints(self, waypoints_config_path):
        """从waypoints.yaml加载bed_*和patrol_*的map坐标"""
        bed_waypoints = {}
        patrol_points = {}
        if not waypoints_config_path:
            workspace_root = self._find_workspace_root()
            if workspace_root:
                waypoints_config_path = os.path.join(
                    workspace_root, 'src', 'nav', 'xjrobot_bridge', 'config', 'waypoints.yaml'
                )
            else:
                self.get_logger().warn('无法找到workspace根目录，跳过waypoints加载')
                return bed_waypoints
        try:
            with open(waypoints_config_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            waypoints = data.get('xjrobot_bridge_node', {}).get(
                'ros__parameters', {}
            ).get('waypoints', {})

            for key, value in waypoints.items():
                if not isinstance(value, dict):
                    continue
                entry = {
                    'index': value.get('index', -1),
                    'name': value.get('name', key),
                    'frame_id': value.get('frame_id', 'map'),
                    'x': value.get('x', 0.0),
                    'y': value.get('y', 0.0),
                    'yaw': value.get('yaw', 0.0),
                }
                if key.startswith('bed_'):
                    bed_waypoints[key] = entry
                elif key.startswith('patrol_'):
                    patrol_points[key] = entry

            self.get_logger().info(
                f'加载waypoints: {waypoints_config_path}, '
                f'{len(bed_waypoints)} 个床位, {len(patrol_points)} 个巡诊点'
            )
            for name, info in sorted({**bed_waypoints, **patrol_points}.items()):
                prefix = 'bed' if name.startswith('bed_') else 'patrol'
                self.get_logger().info(
                    f'  {name} (index={info["index"]}): '
                    f'({info["x"]:.3f}, {info["y"]:.3f}, yaw={info["yaw"]:.3f}) [{info["frame_id"]}]'
                )
        except Exception as e:
            self.get_logger().warn(f'加载waypoints失败: {e}, 深度匹配不可用')

        # 保存patrol坐标供_pixel_to_map_point使用
        self._patrol_points = patrol_points
        return bed_waypoints

    def _get_bed_id(self, patrol_id, detection_index):
        """根据巡诊点ID和YOLO检测框索引获取对应的导航床位编号"""
        if patrol_id in self.patrol_bed_map:
            bed_map = self.patrol_bed_map[patrol_id]
            if detection_index in bed_map:
                return bed_map[detection_index]
        # 无映射时按顺序编号 (index 0 -> bed_id 1)
        return detection_index + 1

    def image_callback(self, msg):
        """接收相机图像"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')

    def depth_callback(self, msg):
        """接收对齐深度图像（单位: mm, 16UC1）"""
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
            self.depth_received = True
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {e}')

    def camera_info_callback(self, msg):
        """接收相机内参"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.get_logger().info(
                f'相机内参: fx={self.camera_intrinsics[0]:.1f}, '
                f'fy={self.camera_intrinsics[1]:.1f}, '
                f'cx={self.camera_intrinsics[2]:.1f}, '
                f'cy={self.camera_intrinsics[3]:.1f}'
            )

    def detect_beds_with_yolo(self, image):
        """
        使用YOLOv8检测图像中的床位，低于置信度阈值的检测结果被过滤

        Returns:
            List[dict]: [{'bbox': [x1,y1,x2,y2], 'class_id': 0, 'confidence': 0.95}, ...]
        """
        results = self.yolo_model(image, verbose=False)

        detections = []
        filtered_count = 0
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                class_id = int(box.cls[0].cpu().numpy())
                confidence = float(box.conf[0].cpu().numpy())

                if confidence < self.yolo_conf_threshold:
                    filtered_count += 1
                    continue

                detections.append({
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'class_id': class_id,
                    'confidence': confidence
                })

        if filtered_count > 0:
            self.get_logger().info(
                f'YOLO过滤 {filtered_count} 个低置信度检测 (阈值: {self.yolo_conf_threshold})'
            )

        return detections

    def _classify_crop(self, crop_bgr):
        """
        使用 CLIP + Linear Probe 分类器对裁剪图分类。
        与训练推理代码完全一致。

        Returns:
            (class_id, confidence): class_id 0=bed_empty, 1=bed_with_person
        """
        device = next(self.clip_model.parameters()).device
        rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(rgb)
        img_tensor = self.clip_preprocess(pil_img).unsqueeze(0).to(device)

        with torch.no_grad():
            features = self.clip_model.encode_image(img_tensor)
            features = features / features.norm(dim=-1, keepdim=True)
            logits = self.clip_classifier(features.float())
            probs = torch.softmax(logits, dim=1)
            pred = logits.argmax(dim=1).item()

        return pred, probs[0][pred].item()

    def detect_person_with_clip(self, bed_image):
        """
        使用 CLIP+Linear分类器 判断床位上是否有人 (Area模式)

        Returns:
            tuple: (bool, float) - (是否有人, 置信度)
        """
        pred, conf = self._classify_crop(bed_image)
        # 根据训练类别: 0=bed_empty, 1=bed_with_person
        # 但class_names顺序从checkpoint读取，因此以实际类别名称为准
        is_person = False
        person_confidence = 0.0
        for i, name in enumerate(self.clip_class_names):
            if 'person' in name.lower() or 'with_person' in name.lower():
                is_person = (pred == i)
                if is_person:
                    person_confidence = conf
                break
        # 回退：如果类别名解析失败，直接检查pred
        if not is_person and pred == 1:
            is_person = True
            person_confidence = conf

        return is_person, person_confidence

    def detect_fall_risk_with_clip(self, bed_image):
        """
        使用CLIP判断病人是否有坠床风险 (Bed模式)
        TODO: 坠床检测模型训练完成后替换

        Returns:
            tuple: (bool, float) - (是否有坠床风险, 置信度)
        """
        # 当前先用有人/无人分类器代替
        return self.detect_person_with_clip(bed_image)

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

    def _pixel_to_map_point(self, u, v, patrol_id):
        """
        将像素坐标(u,v)转换为map坐标系下的(x,y)。
        使用对齐深度图获取深度，相机内参反投影为相机光学坐标，
        再用巡诊点pose近似变换到map（相机位置 ≈ 巡诊点位置）。

        坐标系转换：
          相机光学: X_right, Y_down, Z_forward
          机器人:   X_forward, Y_left, Z_up
          → x_robot = z_cam, y_robot = -x_cam
          → x_map = patrol_x + z_cam*cos(yaw) + x_cam*sin(yaw)
          → y_map = patrol_y + z_cam*sin(yaw) - x_cam*cos(yaw)

        Returns:
            (x, y) in map frame, or None if conversion fails
        """
        if self.current_depth is None or self.camera_intrinsics is None:
            return None

        # 查找巡诊点pose
        patrol_name = f'patrol_{patrol_id}'
        patrol_pose = self._patrol_points.get(patrol_name)
        if patrol_pose is None:
            self.get_logger().warn(f'找不到巡诊点: {patrol_name}')
            return None

        u = int(np.clip(u, 0, self.current_depth.shape[1] - 1))
        v = int(np.clip(v, 0, self.current_depth.shape[0] - 1))

        # 取中心附近5x5区域的中值深度，避免单像素空洞（反光、低纹理导致）
        half = 3
        u0 = max(0, u - half); u1 = min(self.current_depth.shape[1], u + half + 1)
        v0 = max(0, v - half); v1 = min(self.current_depth.shape[0], v + half + 1)
        patch = self.current_depth[v0:v1, u0:u1]
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None
        depth_mm = float(np.median(valid))

        fx, fy, cx, cy = self.camera_intrinsics
        z_cam = depth_mm / 1000.0  # mm → m, 前方距离
        x_cam = (u - cx) * z_cam / fx  # 相机光学右方向
        y_cam = (v - cy) * z_cam / fy  # 相机光学下方向（这里实际用不到）

        # 相机光学 → 机器人base → map
        # 近似：相机在机器人中心，无安装偏移
        x_robot = z_cam     # 前方
        y_robot = -x_cam    # 左方

        px, py, yaw = patrol_pose['x'], patrol_pose['y'], patrol_pose['yaw']
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        x_map = px + x_robot * cos_yaw - y_robot * sin_yaw
        y_map = py + x_robot * sin_yaw + y_robot * cos_yaw

        return (x_map, y_map)

    def _match_detections_to_waypoints(self, detection_map_points, visible_waypoint_names):
        """
        象限匹配：以可见waypoint的几何中心为原点做十字分割，
        每个检测点和waypoint按象限归属一对一匹配。
        不受深度误差和床小幅移动影响，精度远高于绝对距离最近邻。

        Args:
            detection_map_points: [(x, y), ...] 检测框中心的map坐标列表
            visible_waypoint_names: [str, ...] 该巡诊点可见的bed waypoint名称

        Returns:
            [(waypoint_name, waypoint_index, wp_x, wp_y), ...]
        """
        # 收集waypoint坐标，计算十字中心
        wp_coords = []
        for name in visible_waypoint_names:
            wp = self.bed_waypoints.get(name)
            if wp:
                wp_coords.append((name, wp['x'], wp['y'], wp['index']))

        if not wp_coords:
            return [None] * len(detection_map_points)

        cx = sum(w[1] for w in wp_coords) / len(wp_coords)
        cy = sum(w[2] for w in wp_coords) / len(wp_coords)
        self.get_logger().info(f'象限匹配: 十字中心=({cx:.3f},{cy:.3f}), {len(wp_coords)}个waypoint')

        # waypoint按象限分组: key=(qx,qy), 0=左/下, 1=右/上
        wp_quadrants = {}
        for name, wx, wy, wp_idx in wp_coords:
            key = (1 if wx >= cx else 0, 1 if wy >= cy else 0)
            if key not in wp_quadrants:
                wp_quadrants[key] = []
            wp_quadrants[key].append((name, wp_idx, wx, wy))

        results = []
        used_quadrants = set()

        for entry in detection_map_points:
            if entry is None:
                results.append(None)
                self.get_logger().info('象限匹配: 深度无效，跳过此检测框')
                continue

            det_x, det_y = entry
            det_key = (1 if det_x >= cx else 0, 1 if det_y >= cy else 0)

            # 同象限匹配
            candidates = wp_quadrants.get(det_key, [])
            available = [w for w in candidates if det_key not in used_quadrants]

            if available:
                name, wp_idx, wx, wy = available[0]
                used_quadrants.add(det_key)
                results.append((name, wp_idx, wx, wy))
                qname = {(1, 1): '右上', (0, 1): '左上', (0, 0): '左下', (1, 0): '右下'}
                dist = np.sqrt((det_x - wx) ** 2 + (det_y - wy) ** 2)
                self.get_logger().info(
                    f'象限匹配: det({det_x:.3f},{det_y:.3f}) [{qname.get(det_key, "?")}] '
                    f'→ {name} (dist={dist:.3f}m)'
                )
            else:
                # 回退：跨象限最近邻
                best_name = best_idx = None
                best_wx = best_wy = 0.0
                best_dist = float('inf')
                best_key = None
                for qkey, wlist in wp_quadrants.items():
                    if qkey in used_quadrants:
                        continue
                    for name, wp_idx, wx, wy in wlist:
                        d = np.sqrt((det_x - wx) ** 2 + (det_y - wy) ** 2)
                        if d < best_dist:
                            best_dist = d
                            best_name, best_idx = name, wp_idx
                            best_wx, best_wy = wx, wy
                            best_key = qkey
                if best_name is not None:
                    used_quadrants.add(best_key)
                    results.append((best_name, best_idx, best_wx, best_wy))
                    self.get_logger().info(
                        f'象限匹配(回退): det({det_x:.3f},{det_y:.3f}) '
                        f'→ {best_name} (dist={best_dist:.3f}m)'
                    )
                else:
                    results.append(None)
                    self.get_logger().warn(
                        f'象限匹配: det({det_x:.3f},{det_y:.3f}) 无可用的waypoint'
                    )

        return results

    def _publish_debug_markers(self, patrol_id, detection_map_points, waypoint_matches, visible_names):
        """发布RViz Marker：巡诊点(绿色)、床位waypoint(蓝色)、检测点(红色=未匹配/绿色=已匹配)"""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # ---- 巡诊点：大绿色球 ----
        patrol_name = f'patrol_{patrol_id}'
        patrol_pose = self._patrol_points.get(patrol_name)
        if patrol_pose:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'patrol'
            m.id = patrol_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = patrol_pose['x']
            m.pose.position.y = patrol_pose['y']
            m.pose.position.z = 0.3
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.a = 1.0
            m.color.g = 1.0
            marker_array.markers.append(m)

            # 巡诊点文字
            m2 = Marker()
            m2.header.frame_id = 'map'; m2.header.stamp = now
            m2.ns = 'patrol_label'; m2.id = patrol_id
            m2.type = Marker.TEXT_VIEW_FACING; m2.action = Marker.ADD
            m2.pose.position.x = patrol_pose['x']; m2.pose.position.y = patrol_pose['y']
            m2.pose.position.z = 0.6
            m2.scale.z = 0.2
            m2.color.a = 1.0; m2.color.g = 1.0
            m2.text = f'patrol_{patrol_id}'
            marker_array.markers.append(m2)

        # ---- 床位waypoint：蓝色方块 ----
        for i, name in enumerate(visible_names):
            wp = self.bed_waypoints.get(name)
            if wp is None:
                continue
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = now
            m.ns = 'bed_wp'; m.id = i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = wp['x']; m.pose.position.y = wp['y']
            m.pose.position.z = 0.15
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.a = 1.0; m.color.b = 1.0
            marker_array.markers.append(m)

            # 文字
            m2 = Marker()
            m2.header.frame_id = 'map'; m2.header.stamp = now
            m2.ns = 'bed_label'; m2.id = i
            m2.type = Marker.TEXT_VIEW_FACING; m2.action = Marker.ADD
            m2.pose.position.x = wp['x']; m2.pose.position.y = wp['y']
            m2.pose.position.z = 0.4
            m2.scale.z = 0.2
            m2.color.a = 1.0; m2.color.b = 1.0
            m2.text = f'{name} (idx={wp["index"]})'
            marker_array.markers.append(m2)

        # ---- 检测到的点 ----
        if detection_map_points:
            for i, (det_point, match) in enumerate(zip(detection_map_points, waypoint_matches or [])):
                if det_point is None:
                    continue
                det_x, det_y = det_point
                matched = match is not None

                m = Marker()
                m.header.frame_id = 'map'; m.header.stamp = now
                m.ns = 'detection'; m.id = i
                m.type = Marker.SPHERE; m.action = Marker.ADD
                m.pose.position.x = det_x; m.pose.position.y = det_y
                m.pose.position.z = 0.1
                m.scale.x = m.scale.y = m.scale.z = 0.2
                m.color.a = 1.0
                if matched:
                    m.color.g = 1.0  # 绿色=匹配成功
                else:
                    m.color.r = 1.0  # 红色=未匹配
                marker_array.markers.append(m)

                # 连到匹配waypoint的线
                if matched:
                    wp_name, wp_index, wp_x, wp_y = match
                    line = Marker()
                    line.header.frame_id = 'map'; line.header.stamp = now
                    line.ns = 'match_line'; line.id = i
                    line.type = Marker.LINE_STRIP; line.action = Marker.ADD
                    line.pose.orientation.w = 1.0
                    line.scale.x = 0.03
                    line.color.a = 0.7; line.color.g = 0.8; line.color.b = 0.3
                    line.points = [
                        Point(x=det_x, y=det_y, z=0.05),
                        Point(x=wp_x, y=wp_y, z=0.05),
                    ]
                    marker_array.markers.append(line)

        self.marker_pub.publish(marker_array)

    def _handle_area_mode(self, request, response):
        """
        Area模式处理逻辑
        深度可用时：使用空间匹配（像素→3D→TF→map→最近邻waypoint）
        深度不可用时：回退到patrol_bed_mapping.json索引映射
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

        # 按置信度降序排序
        bed_detections.sort(key=lambda d: d['confidence'], reverse=True)

        # 确定该巡诊点期望的床位数N
        visible_waypoints = self.waypoint_config.get('patrol_points', [])
        visible_names = []
        for pt in visible_waypoints:
            if pt.get('patrol_id') == patrol_id:
                visible_names = pt.get('bed_waypoints', [])
                break
        max_expected = len(visible_names) if visible_names else len(
            self.patrol_bed_map.get(patrol_id, {})
        )

        if max_expected > 0:
            bed_detections = bed_detections[:max_expected]

        self.get_logger().info(f'处理 {len(bed_detections)} 个床位 (期望 {max_expected})')

        # ==================== 深度空间匹配 ====================
        depth_matching_used = False
        waypoint_matches = None

        if (self.use_depth_matching and self.depth_received
                and self.camera_intrinsics is not None and visible_names
                and self.bed_waypoints):
            self.get_logger().info('使用深度空间匹配模式')
            depth_matching_used = True
            detection_map_points = []

            for detection in bed_detections:
                x1, y1, x2, y2 = detection['bbox']
                cx_pixel = int((x1 + x2) / 2)
                cy_pixel = int((y1 + y2) / 2)
                map_point = self._pixel_to_map_point(cx_pixel, cy_pixel, patrol_id)
                if map_point is not None:
                    detection_map_points.append(map_point)
                else:
                    detection_map_points.append(None)

            waypoint_matches = self._match_detections_to_waypoints(
                detection_map_points, visible_names
            )
        else:
            self.get_logger().info('使用索引映射模式（无深度/模拟相机）')
            detection_map_points = []
            waypoint_matches = []

        # 发布RViz标记（不管哪种模式都发送巡诊点和床位waypoint）
        self._publish_debug_markers(patrol_id, detection_map_points,
                                    waypoint_matches, visible_names)

        # ==================== CLIP判断 ====================
        occupied_beds = []
        urgencies = []

        for i, detection in enumerate(bed_detections):
            bbox = detection['bbox']
            confidence = detection['confidence']
            x1, y1, x2, y2 = bbox
            bed_image = self.current_image[y1:y2, x1:x2]

            if bed_image.size == 0:
                continue

            is_person, person_score = self.detect_person_with_clip(bed_image)

            if not is_person:
                continue

            if depth_matching_used and waypoint_matches is not None:
                match = waypoint_matches[i] if i < len(waypoint_matches) else None
                if match is None:
                    self.get_logger().info(
                        f'检测框{i}(conf={confidence:.3f})CLIP判定有人，但空间匹配失败，丢弃'
                    )
                    continue
                wp_name, wp_index, wp_x, wp_y = match
                bed_id = wp_index
                self.get_logger().info(
                    f'{wp_name}(index={bed_id}) 有人, CLIP置信度: {person_score:.3f}, '
                    f'YOLO置信度: {confidence:.3f}'
                )
            else:
                bed_id = self._get_bed_id(patrol_id, i)
                self.get_logger().info(
                    f'床位 {bed_id} 有人, CLIP置信度: {person_score:.3f}, '
                    f'YOLO置信度: {confidence:.3f}'
                )

            occupied_beds.append(bed_id)
            urgency = 1 if person_score > 0.7 else 0
            urgencies.append(urgency)

        response.is_anomaly = len(occupied_beds) > 0
        response.details = f"Detected {len(occupied_beds)} occupied beds out of {len(bed_detections)} total beds"
        response.bed_ids = occupied_beds
        response.urgencies = urgencies

        self.get_logger().info(
            f'Area模式检测完成: {len(occupied_beds)} 个有人床位 -> {occupied_beds}'
        )

        if self.publish_debug_image:
            self._publish_debug_image(self.current_image, bed_detections,
                                      occupied_beds, waypoint_matches, patrol_id)

        return response

    def _publish_debug_image(self, image, detections, occupied_beds, waypoint_matches, patrol_id):
        """绘制YOLO检测框和床位标签，深度模式下显示实际匹配的waypoint名称"""
        debug_img = image.copy()
        for i, detection in enumerate(detections):
            x1, y1, x2, y2 = detection['bbox']
            conf = detection['confidence']

            # 深度匹配模式：使用实际匹配的waypoint名称
            if waypoint_matches and i < len(waypoint_matches) and waypoint_matches[i] is not None:
                wp_name, wp_index, wp_x, wp_y = waypoint_matches[i]
                bed_label = f'{wp_name}(idx={wp_index})'
                is_occupied = wp_index in occupied_beds
            else:
                # 索引映射模式：回退到旧方法
                bed_id = self._get_bed_id(patrol_id, i)
                bed_label = f'Bed {bed_id}'
                is_occupied = bed_id in occupied_beds

            if is_occupied:
                color = (0, 255, 0)    # 绿色：有人
                label = f'{bed_label} (OCCUPIED) {conf:.2f}'
            else:
                color = (0, 0, 255)    # 红色：无人
                label = f'{bed_label} (empty) {conf:.2f}'

            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(debug_img, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f'发布调试图像失败: {e}')

    def _handle_bed_mode(self, request, response):
        """
        Bed模式处理逻辑
        机器人已停在目标床位前，直接用CLIP判断整张图像中病人是否在床上

        Args:
            request.area_bed_id: 目标床位ID
        """
        self._switch_camera_mode('bed')
        target_bed_id = request.area_bed_id
        self.get_logger().info(f'开始Bed模式检测, 目标床位ID: {target_bed_id}')

        # 直接对整个图像使用CLIP判断是否有人（坠床检测模型训练完成后替换为 detect_fall_risk_with_clip）
        is_person, person_score = self.detect_person_with_clip(self.current_image)

        # 有人在床上即为异常
        response.is_anomaly = is_person
        response.details = (
            f"Bed {target_bed_id}: {'person detected' if is_person else 'bed empty'}, "
            f"CLIP confidence: {person_score:.3f}"
        )
        response.bed_ids = [target_bed_id] if is_person else []
        response.urgencies = [1 if person_score > 0.7 else 0] if is_person else []

        self.get_logger().info(
            f'Bed模式检测完成: 床位 {target_bed_id} '
            f'{"有人(异常)" if is_person else "无人(正常)"}, '
            f'CLIP置信度: {person_score:.3f}'
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