# 人脸识别模块

## 概述

人脸识别模块包括相机节点、人脸识别服务和模拟导航服务，用于医疗机器人的身份验证功能。

## 文件结构

```
src/monitor/
├── scripts/
│   ├── camera_node.py              # 相机节点，持续发布图像
│   ├── face_identify_server.py     # 人脸识别服务
│   └── mock_navigation_server.py   # 模拟导航服务（测试用）
├── launch/
│   ├── camera_node.launch          # 相机节点启动文件
│   ├── face_identify.launch        # 人脸识别服务启动文件
│   └── test_face_integration.launch # 集成测试启动文件
└── face_database/                  # 人脸数据库目录
    ├── 1.jpg                       # 示例：person_id=1 的照片
    └── README.md                   # 数据库使用说明
```

## 快速开始

### 1. 依赖安装
```bash
pip3 install face_recognition opencv-python numpy
sudo apt-get install ros-${ROS_DISTRO}-cv-bridge
```

### 2. 添加人脸数据
将照片放到 `face_database` 目录，文件名为 `person_id.jpg`（如 `1.jpg`）

### 3. 测试服务
```bash
# 启动相机和人脸识别
roslaunch monitor camera_node.launch
roslaunch monitor face_identify.launch

# 测试识别
rosservice call /face_identify "{}"
```

### 4. 行为树集成测试
```bash
# 一键启动所有服务
roslaunch monitor test_face_integration.launch

# 触发人脸识别
rostopic pub /call_signal std_msgs/Bool "data: true"
```

## 服务接口

### /face_identify
**类型**: `interfaces/FaceIdentify`

**请求**: 无参数

**响应**:
```
bool success           # 始终为 True
int32 person_id        # 识别到的人员 ID，-1 表示未识别
float32 confidence      # 置信度 0-100
string message         # 结果描述
```

## 架构说明

1. **相机节点** - 持续打开摄像头，以 30Hz 频率发布图像到 `/camera/rgb/image_raw`
2. **人脸识别服务** - 订阅相机图像，收到请求后进行识别，最多尝试 10 次，超时 10 秒
3. **模拟导航服务** - 用于测试完整流程，模拟导航成功后继续执行人脸识别

## 测试验证

### 独立测试
```bash
# 1. 启动服务
roslaunch monitor camera_node.launch
roslaunch monitor face_identify.launch

# 2. 查看图像
rosrun image_view image_view image:=/camera/rgb/image_raw

# 3. 调用服务
rosservice call /face_identify "{}"
```

### 行为树集成测试
```bash
# 1. 启动所有服务
roslaunch monitor test_face_integration.launch

# 2. 触发识别（快速）
rostopic pub /call_signal std_msgs/Bool "data: true"

# 3. 触发识别（完整流程，含导航）
rostopic pub /patrol_triggered std_msgs/Bool "data: true"
```

**预期结果**:
```
[DONE] NavgateTo
[INFO] [FaceIdentify] 识别成功: ID=1 (置信度: 74.6%)
[ACT ] FaceIdentify 识别成功: person_id=1, confidence=74.6
```

## 性能指标

| 指标 | 数值 |
|------|------|
| 识别延迟 | 0.5-1.0 秒 |
| 相机发布频率 | 30 Hz |
| 最大尝试次数 | 10 次 |
| 总超时时间 | 10 秒 |

## 故障排查

| 问题 | 解决方法 |
|------|---------|
| 摄像头无法打开 | 检查 `/dev/video*`，修改 `camera_id` 参数 |
| 人脸数据库为空 | 添加照片到 `face_database` 目录 |
| 服务未找到 | 确保已编译：`catkin build behavior_tree interfaces` |
| 权限错误 | 执行：`chmod +x src/monitor/scripts/*.py` |

## 更新日志

### 2026-03-03
- 初始版本
- 实现独立相机节点
- 实现人脸识别服务
- 实现模拟导航服务
- 添加集成测试支持
