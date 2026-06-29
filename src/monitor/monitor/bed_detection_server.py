#!/usr/bin/env python3
"""
床位检测节点 - Area/Bed模式实现
整合版本: 用于Medical_Embodied项目的monitor包

功能:
  Area模式: 扫描指定区域,返回有人的床位ID列表
  Bed模式:  检测指定床位是否有人(异常),返回is_anomaly

服务接口: /detect_anomaly (继承自interfaces/DetectAnomaly.srv)
相机话题: 通过camera_topic参数配置 (默认RealSense L515: /camera/camera/color/image_raw)
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

# RViz 床位配色（按槽位索引区分）
_BED_MARKER_COLORS = [
    (1.0, 0.25, 0.25), (0.25, 0.45, 1.0), (1.0, 0.65, 0.1), (0.65, 0.25, 0.85),
    (0.2, 0.85, 0.45), (1.0, 0.3, 0.65), (0.35, 0.8, 0.95), (0.9, 0.85, 0.2),
]

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
        self.invert_lateral_sign = bool(self.waypoint_config.get('invert_lateral_sign', False))
        self.match_distance_threshold = float(
            self.waypoint_config.get('match_distance_threshold', 1.5)
        )
        self.get_logger().info(
            f'深度匹配: 距离阈值={self.match_distance_threshold}m, '
            f'invert_lateral_sign={self.invert_lateral_sign}'
        )

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

        # 订阅相机话题（通过参数配置，默认使用RealSense L515彩色图像话题）
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

    def _get_patrol_bed_names(self, patrol_id):
        """返回当前巡诊点允许的床位 waypoint 名称（顺序即槽位）。"""
        for pt in self.waypoint_config.get('patrol_points', []):
            if pt.get('patrol_id') == patrol_id:
                return list(pt.get('bed_waypoints', []))
        return []

    def _get_patrol_bed_slots(self, patrol_id):
        """
        构建巡诊点床位槽位列表。床位编号与区域严格绑定，不会跨巡诊点。
        每个槽位: {slot, name, bed_id, x, y, yaw}
        """
        slots = []
        for slot, name in enumerate(self._get_patrol_bed_names(patrol_id)):
            wp = self.bed_waypoints.get(name)
            if wp is None:
                self.get_logger().warn(f'巡诊点{patrol_id}缺少waypoint: {name}')
                continue
            slots.append({
                'slot': slot,
                'name': name,
                'bed_id': wp['index'],
                'x': wp['x'],
                'y': wp['y'],
                'yaw': wp['yaw'],
            })
        return slots

    def _get_bed_id(self, patrol_id, detection_index):
        """根据巡诊点ID和YOLO检测框索引获取对应的导航床位编号"""
        if patrol_id in self.patrol_bed_map:
            bed_map = self.patrol_bed_map[patrol_id]
            if detection_index in bed_map:
                return bed_map[detection_index]
        # 无映射时按顺序编号 (index 0 -> bed_id 1)
        return detection_index + 1

    def _waypoint_to_bed_id(self, waypoint_name, waypoint_index):
        """返回导航床位编号（waypoints.yaml 中的 index 字段）。"""
        del waypoint_name
        return waypoint_index

    def _map_point_xy(self, map_point):
        """从 map_point 元组提取 map 坐标 (x, y)。"""
        if map_point is None:
            return None
        return map_point[0], map_point[1]

    def _detection_lateral_sign(self, y_robot):
        sign = np.sign(y_robot) if abs(y_robot) > 0.05 else 0
        return -sign if self.invert_lateral_sign and sign != 0 else sign

    def _bed_lateral_sign(self, bed_yaw, patrol_yaw):
        return np.sign(np.sin(bed_yaw - patrol_yaw))

    def _lateral_compatible(self, y_robot, bed_yaw, patrol_yaw):
        """同坐标双侧床位：用横向符号区分左右。"""
        det_sign = self._detection_lateral_sign(y_robot)
        bed_sign = self._bed_lateral_sign(bed_yaw, patrol_yaw)
        if det_sign == 0 or bed_sign == 0:
            return True
        return det_sign == bed_sign

    def _prepare_detections(self, bed_detections, patrol_id):
        """
        限制检测数量不超过巡诊点床位数，并附加像素/地图投影信息。
        返回按图像从左到右排序的列表，每项:
          {det, det_index, u, map_point}
        """
        slots = self._get_patrol_bed_slots(patrol_id)
        max_beds = len(slots)
        if max_beds == 0:
            return [], slots

        sorted_dets = sorted(bed_detections, key=lambda d: d['confidence'], reverse=True)
        sorted_dets = sorted_dets[:max_beds]

        prepared = []
        for det_index, det in enumerate(sorted_dets):
            x1, y1, x2, y2 = det['bbox']
            u = (x1 + x2) / 2.0
            v = (y1 + y2) / 2.0
            map_point = None
            if self.use_depth_matching and self.depth_received and self.camera_intrinsics:
                map_point = self._pixel_to_map_point(int(u), int(v), patrol_id)
            prepared.append({
                'det': det,
                'det_index': det_index,
                'u': u,
                'v': v,
                'map_point': map_point,
            })

        return prepared, slots

    def _match_cost(self, map_point, bed_slot, patrol_pose):
        """检测点到槽位的匹配代价；不可匹配返回 inf。"""
        if map_point is None:
            return float('inf')
        det_x, det_y, y_robot = map_point[0], map_point[1], map_point[2]
        dist = np.hypot(det_x - bed_slot['x'], det_y - bed_slot['y'])
        if dist > self.match_distance_threshold:
            return float('inf')
        if not self._lateral_compatible(y_robot, bed_slot['yaw'], patrol_pose['yaw']):
            return float('inf')
        return dist

    def _group_beds_by_side(self, bed_slots, patrol_pose):
        """左侧近→远 bed_1,2；右侧近→远 bed_3,4。"""
        pyaw = patrol_pose['yaw']
        left_beds, right_beds = [], []
        for bed in bed_slots:
            fwd = self._forward_distance(bed['x'], bed['y'], patrol_pose)
            entry = (fwd, bed)
            if self._bed_side(bed['yaw'], pyaw) > 0:
                left_beds.append(entry)
            else:
                right_beds.append(entry)
        left_beds.sort(key=lambda item: item[0])
        right_beds.sort(key=lambda item: item[0])
        return (
            [bed for _, bed in left_beds],
            [bed for _, bed in right_beds],
        )

    def _group_dets_by_side(self, prepared, patrol_pose):
        """按横向分左右，同侧内按前向距离由近到远排序。"""
        left_dets, right_dets, unknown_dets = [], [], []
        for det_i, item in enumerate(prepared):
            pt = item['map_point']
            if pt is not None:
                side = self._det_side(pt[2])
                fwd = self._det_forward(pt, patrol_pose)
                if side > 0:
                    left_dets.append((fwd, det_i))
                elif side < 0:
                    right_dets.append((fwd, det_i))
                else:
                    unknown_dets.append(det_i)
            else:
                unknown_dets.append(det_i)

        left_dets.sort(key=lambda item: item[0])
        right_dets.sort(key=lambda item: item[0])
        return (
            [det_i for _, det_i in left_dets],
            [det_i for _, det_i in right_dets],
            unknown_dets,
        )

    def _fallback_group_dets_by_image(self, prepared):
        """无深度时：按图像左右分区，同侧按 v 由大到小（近→远）。"""
        if self.camera_intrinsics is None:
            cx = 640.0
        else:
            cx = self.camera_intrinsics[2]
        left_dets, right_dets = [], []
        for det_i, item in enumerate(prepared):
            entry = (item['v'], det_i)
            if item['u'] < cx:
                left_dets.append(entry)
            else:
                right_dets.append(entry)
        left_dets.sort(key=lambda item: -item[0])
        right_dets.sort(key=lambda item: -item[0])
        return (
            [det_i for _, det_i in left_dets],
            [det_i for _, det_i in right_dets],
        )

    def _assign_side_pairs(self, matches, det_indices, bed_slots, prepared, patrol_pose):
        """同侧内按远近顺序分配；空间校验失败时仍强制分配床位编号。"""
        for det_i, bed in zip(det_indices, bed_slots):
            pt = prepared[det_i]['map_point']
            spatial_ok = (
                pt is None
                or np.isfinite(self._match_cost(pt, bed, patrol_pose))
            )
            matches[det_i] = bed
            fwd = self._det_forward(pt, patrol_pose)
            coord = (
                f'({pt[0]:.2f},{pt[1]:.2f},fwd={fwd:.2f})'
                if pt else f'(image u={prepared[det_i]["u"]:.0f}, v={prepared[det_i]["v"]:.0f})'
            )
            mode = '空间匹配' if spatial_ok else '远近强制分配'
            self.get_logger().info(
                f'{mode}: det#{prepared[det_i]["det_index"]} {coord} → '
                f'{bed["name"]}(bed_id={bed["bed_id"]})'
            )

    def _assign_unknown_dets(self, matches, unknown_dets, prepared, left_beds, right_beds):
        """无法分侧的检测：按图像左右 + 近→远分配到剩余槽位。"""
        if not unknown_dets:
            return

        used = {m['bed_id'] for m in matches if m}
        left_pool = [b for b in left_beds if b['bed_id'] not in used]
        right_pool = [b for b in right_beds if b['bed_id'] not in used]

        if self.camera_intrinsics is None:
            cx = 640.0
        else:
            cx = self.camera_intrinsics[2]

        left_unknown, right_unknown = [], []
        for det_i in unknown_dets:
            if matches[det_i] is not None:
                continue
            entry = (prepared[det_i]['v'], det_i)
            if prepared[det_i]['u'] < cx:
                left_unknown.append(entry)
            else:
                right_unknown.append(entry)

        left_unknown.sort(key=lambda item: -item[0])
        right_unknown.sort(key=lambda item: -item[0])

        for (_, det_i), bed in zip(left_unknown, left_pool):
            matches[det_i] = bed
            self.get_logger().info(
                f'远近强制分配: det#{prepared[det_i]["det_index"]} '
                f'(image) → {bed["name"]}(bed_id={bed["bed_id"]})'
            )
        for (_, det_i), bed in zip(right_unknown, right_pool):
            matches[det_i] = bed
            self.get_logger().info(
                f'远近强制分配: det#{prepared[det_i]["det_index"]} '
                f'(image) → {bed["name"]}(bed_id={bed["bed_id"]})'
            )

    def _assign_beds(self, prepared, bed_slots, patrol_id):
        """
        按设计规则匹配：左侧近→远 1,2；右侧近→远 3,4。
        YOLO 检测成功的床位即使空间未匹配，也按同侧远近强制分配编号。
        """
        if not prepared or not bed_slots:
            return []

        patrol_pose = self._patrol_points.get(f'patrol_{patrol_id}')
        if patrol_pose is None:
            self.get_logger().warn(f'找不到巡诊点 patrol_{patrol_id}，按槽位顺序强制分配')
            return [bed_slots[i] if i < len(bed_slots) else None for i in range(len(prepared))]

        left_beds, right_beds = self._group_beds_by_side(bed_slots, patrol_pose)
        left_dets, right_dets, unknown_dets = self._group_dets_by_side(prepared, patrol_pose)

        if not left_dets and not right_dets:
            self.get_logger().info('无有效深度侧向信息，使用图像左右分区 + 近→远排序')
            left_dets, right_dets = self._fallback_group_dets_by_image(prepared)
            unknown_dets = []

        matches = [None] * len(prepared)
        self._assign_side_pairs(matches, left_dets, left_beds, prepared, patrol_pose)
        self._assign_side_pairs(matches, right_dets, right_beds, prepared, patrol_pose)
        self._assign_unknown_dets(matches, unknown_dets, prepared, left_beds, right_beds)

        # 兜底：仍有未分配的检测，按全局远近（图像 v）占用剩余槽位
        unmatched = [i for i, m in enumerate(matches) if m is None]
        if unmatched:
            used = {m['bed_id'] for m in matches if m}
            remaining = [b for b in bed_slots if b['bed_id'] not in used]
            fallback_dets = sorted(
                unmatched, key=lambda i: -prepared[i]['v']
            )
            for det_i, bed in zip(fallback_dets, remaining):
                matches[det_i] = bed
                self.get_logger().info(
                    f'远近强制分配(兜底): det#{prepared[det_i]["det_index"]} → '
                    f'{bed["name"]}(bed_id={bed["bed_id"]})'
                )

        return matches

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
            (x, y, y_robot, x_robot) in map/robot frame, or None
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

        return (x_map, y_map, y_robot, x_robot)

    def _bed_side(self, bed_yaw, patrol_yaw):
        """左侧 +1，右侧 -1（由 waypoint yaw 相对巡诊朝向决定）。"""
        sign = self._bed_lateral_sign(bed_yaw, patrol_yaw)
        return 1 if sign >= 0 else -1

    def _det_side(self, y_robot):
        sign = self._detection_lateral_sign(y_robot)
        if sign > 0:
            return 1
        if sign < 0:
            return -1
        return 0

    def _forward_distance(self, x, y, patrol_pose):
        dx = x - patrol_pose['x']
        dy = y - patrol_pose['y']
        pyaw = patrol_pose['yaw']
        return dx * np.cos(pyaw) + dy * np.sin(pyaw)

    def _det_forward(self, map_point, patrol_pose):
        if map_point is None:
            return float('inf')
        if len(map_point) >= 4:
            return map_point[3]
        return self._forward_distance(map_point[0], map_point[1], patrol_pose)

    def _marker_color(self, marker, r, g, b, a=1.0):
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = float(a)

    def _publish_debug_markers(self, patrol_id, bed_slots, prepared, matches):
        """发布 RViz 标记：巡诊点、床位槽位、检测点（按槽位配色）。"""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        patrol_name = f'patrol_{patrol_id}'
        patrol_pose = self._patrol_points.get(patrol_name)
        if patrol_pose:
            px, py, pyaw = patrol_pose['x'], patrol_pose['y'], patrol_pose['yaw']
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'patrol'
            m.id = patrol_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = 0.12
            m.scale.x = m.scale.y = m.scale.z = 0.12
            self._marker_color(m, 0.1, 1.0, 0.1)
            marker_array.markers.append(m)

            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = now
            arrow.ns = 'patrol_heading'
            arrow.id = patrol_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = px
            arrow.pose.position.y = py
            arrow.pose.position.z = 0.12
            arrow.pose.orientation.z = np.sin(pyaw / 2.0)
            arrow.pose.orientation.w = np.cos(pyaw / 2.0)
            arrow.scale.x = 0.35
            arrow.scale.y = 0.06
            arrow.scale.z = 0.06
            self._marker_color(arrow, 0.1, 1.0, 0.1)
            marker_array.markers.append(arrow)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'patrol_label'
            label.id = patrol_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = px
            label.pose.position.y = py
            label.pose.position.z = 0.28
            label.scale.z = 0.09
            self._marker_color(label, 0.1, 1.0, 0.1)
            label.text = f'patrol_{patrol_id}'
            marker_array.markers.append(label)

        for bed in bed_slots:
            slot = bed['slot']
            r, g, b = _BED_MARKER_COLORS[slot % len(_BED_MARKER_COLORS)]
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'bed_slot'
            m.id = slot
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = bed['x']
            m.pose.position.y = bed['y']
            m.pose.position.z = 0.06
            m.scale.x = 0.14
            m.scale.y = 0.14
            m.scale.z = 0.03
            self._marker_color(m, r, g, b, 0.85)
            marker_array.markers.append(m)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'bed_label'
            label.id = slot
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = bed['x']
            label.pose.position.y = bed['y']
            label.pose.position.z = 0.18
            label.scale.z = 0.08
            self._marker_color(label, r, g, b)
            label.text = f'{bed["name"]} id={bed["bed_id"]}'
            marker_array.markers.append(label)

        for i, (item, bed) in enumerate(zip(prepared, matches)):
            pt = item['map_point']
            if pt is None:
                continue
            det_x, det_y = self._map_point_xy(pt)
            if bed is None:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = now
                m.ns = 'detection'
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = det_x
                m.pose.position.y = det_y
                m.pose.position.z = 0.08
                m.scale.x = m.scale.y = m.scale.z = 0.08
                self._marker_color(m, 1.0, 0.2, 0.2)
                marker_array.markers.append(m)
                continue

            slot = bed['slot']
            r, g, b = _BED_MARKER_COLORS[slot % len(_BED_MARKER_COLORS)]
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'detection'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = det_x
            m.pose.position.y = det_y
            m.pose.position.z = 0.08
            m.scale.x = m.scale.y = m.scale.z = 0.09
            self._marker_color(m, r, g, b)
            marker_array.markers.append(m)

            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = now
            line.ns = 'match_line'
            line.id = i
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.012
            self._marker_color(line, r, g, b, 0.7)
            line.points = [
                Point(x=det_x, y=det_y, z=0.04),
                Point(x=bed['x'], y=bed['y'], z=0.04),
            ]
            marker_array.markers.append(line)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'det_label'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = det_x
            label.pose.position.y = det_y
            label.pose.position.z = 0.14
            label.scale.z = 0.07
            self._marker_color(label, r, g, b)
            label.text = f'{bed["name"]}({bed["bed_id"]})'
            marker_array.markers.append(label)

        self.marker_pub.publish(marker_array)

    def _handle_area_mode(self, request, response):
        """Area 模式：在巡诊点槽位内匹配床位，漏检时仅输出已识别结果。"""
        self._switch_camera_mode('area')
        patrol_id = request.area_bed_id
        self.get_logger().info(f'开始Area模式检测, 巡诊点ID: {patrol_id}')

        bed_slots = self._get_patrol_bed_slots(patrol_id)
        if not bed_slots:
            self.get_logger().warn(f'巡诊点 {patrol_id} 未配置床位槽位')
            response.is_anomaly = False
            response.details = f'No bed slots for patrol {patrol_id}'
            response.bed_ids = []
            response.urgencies = []
            return response

        allowed_bed_ids = {slot['bed_id'] for slot in bed_slots}
        self.get_logger().info(
            f'巡诊点{patrol_id}床位槽位: ' +
            ', '.join(f'{s["name"]}(id={s["bed_id"]})' for s in bed_slots)
        )

        bed_detections = self.detect_beds_with_yolo(self.current_image)
        if not bed_detections:
            self.get_logger().warn('未检测到任何床位')
            response.is_anomaly = False
            response.details = "No beds detected"
            response.bed_ids = []
            response.urgencies = []
            self._publish_debug_markers(patrol_id, bed_slots, [], [])
            return response

        prepared, bed_slots = self._prepare_detections(bed_detections, patrol_id)
        self.get_logger().info(
            f'YOLO检测 {len(bed_detections)} 个, 巡诊点槽位 {len(bed_slots)} 个, '
            f'参与匹配 {len(prepared)} 个'
        )

        matches = self._assign_beds(prepared, bed_slots, patrol_id)
        self._publish_debug_markers(patrol_id, bed_slots, prepared, matches)

        occupied_beds = []
        urgencies = []
        for item, bed in zip(prepared, matches):
            if bed is None:
                continue
            bed_id = bed['bed_id']
            if bed_id not in allowed_bed_ids:
                self.get_logger().warn(f'忽略越界床位编号: {bed_id}')
                continue

            det = item['det']
            x1, y1, x2, y2 = det['bbox']
            bed_image = self.current_image[y1:y2, x1:x2]
            if bed_image.size == 0:
                continue

            is_person, person_score = self.detect_person_with_clip(bed_image)
            if not is_person:
                continue

            occupied_beds.append(bed_id)
            # TODO：urgencies等级现在只由检测人的置信度决定，后期可能需要扩展clip的分类功能
            urgencies.append(1 if person_score > 0.7 else 0)
            self.get_logger().info(
                f'{bed["name"]}(bed_id={bed_id}) 有人, CLIP={person_score:.3f}, '
                f'YOLO={det["confidence"]:.3f}'
            )

        response.is_anomaly = len(occupied_beds) > 0
        response.details = (
            f'Patrol {patrol_id}: {len(occupied_beds)} occupied / '
            f'{len(prepared)} detected / {len(bed_slots)} slots'
        )
        response.bed_ids = occupied_beds
        response.urgencies = urgencies

        self.get_logger().info(
            f'Area模式完成: 有人床位 {occupied_beds} (允许范围 {sorted(allowed_bed_ids)})'
        )

        if self.publish_debug_image:
            self._publish_debug_image(prepared, matches, occupied_beds)

        return response

    def _publish_debug_image(self, prepared, matches, occupied_beds):
        """绘制检测框，颜色与槽位一致。"""
        debug_img = self.current_image.copy()
        for item, bed in zip(prepared, matches):
            det = item['det']
            x1, y1, x2, y2 = det['bbox']
            conf = det['confidence']

            if bed is None:
                color = (0, 0, 255)
                label = f'unmatched {conf:.2f}'
            else:
                slot = bed['slot']
                r, g, b = _BED_MARKER_COLORS[slot % len(_BED_MARKER_COLORS)]
                color = (int(b * 255), int(g * 255), int(r * 255))
                bed_id = bed['bed_id']
                status = 'OCCUPIED' if bed_id in occupied_beds else 'empty'
                label = f'{bed["name"]}({bed_id}) {status} {conf:.2f}'

            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(debug_img, label, (x1, max(y1 - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        try:
            self.debug_image_pub.publish(
                self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            )
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