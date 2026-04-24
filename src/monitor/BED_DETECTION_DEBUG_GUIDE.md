# 床位检测模块联调指南

## 1. 前置准备

### 1.1 模型文件

确保以下模型文件已放置到 `src/monitor/models/` 目录：

| 文件 | 用途 |
|------|------|
| `best.pt` | YOLOv8 床位检测模型 |
| `best_model.pt` | CLIP 有人/无人判断模型 |

如果没有，用 U 盘拷入。备用文件名 `bed_detector.pt` 也会被尝试加载。

### 1.2 测试图片

确保测试图片在对应目录：

```
src/monitor/test_images/
├── area_pic/    ← Area 模式图片（病房全景）
├── bed_pic/     ← Bed 模式图片（单床近景）
└── face_pic/    ← 人脸识别图片
```

模拟相机节点会从这些目录随机抽图发布。

### 1.3 映射配置

编辑 `src/monitor/config/patrol_bed_mapping.json`：

```json
{
  "patrol_points": [
    {
      "patrol_id": 1,
      "beds": [
        {"detection_index": 0, "bed_id": 1},
        {"detection_index": 1, "bed_id": 2},
        {"detection_index": 2, "bed_id": 3},
        {"detection_index": 3, "bed_id": 4},
        {"detection_index": 4, "bed_id": 5},
        {"detection_index": 5, "bed_id": 6}
      ]
    },
    {
      "patrol_id": 2,
      "beds": [
        {"detection_index": 0, "bed_id": 7},
        {"detection_index": 1, "bed_id": 8},
        {"detection_index": 2, "bed_id": 9},
        {"detection_index": 3, "bed_id": 10},
        {"detection_index": 4, "bed_id": 11},
        {"detection_index": 5, "bed_id": 12}
      ]
    }
  ]
}
```

**联调前必须确认**：
- `patrol_id`：必须与 C++ 行为树传来的 `area_bed_id` 值一致（可能是 `0/1` 也可能是 `1/2`，取决于师兄那边的实现）
- `detection_index`：YOLO 检测框从左到右的顺序编号，从 0 开始
- `bed_id`：导航节点需要的床位编号

**联调时如何调整**：观察日志中 YOLO 检测框的实际顺序，修改 `detection_index` 与 `bed_id` 的对应关系即可。改完重启节点生效。

---

## 2. 编译与启动

### 2.1 编译

```bash
cd ~/Desktop/Medical_Embodied

# 编译 interfaces 包（服务定义）
colcon build --packages-select interfaces

# 编译 monitor 包
colcon build --packages-select monitor

# 如果行为树也有改动
colcon build --packages-select behavior_tree

# 或者全量编译
colcon build

# 刷新环境
source install/setup.bash
```

### 2.2 单独启动（你的模块自测）

```bash
# 用模拟相机启动（默认 use_mock_camera=true）
ros2 launch monitor bed_detection.launch.py

# 指定模型路径启动
ros2 launch monitor bed_detection.launch.py \
    yolo_model_path:=/path/to/best.pt \
    clip_model_path:=/path/to/best_model.pt

# 不用模拟相机（接真实相机话题）
ros2 launch monitor bed_detection.launch.py use_mock_camera:=false
```

启动后应看到以下日志：
```
[bed_detection_server-X] YOLOv8模型加载成功: ...
[bed_detection_server-X] CLIP模型加载成功(设备: cuda/cpu): ...
[bed_detection_server-X] 加载巡诊点-床位映射: ..., 2 个巡诊点
[bed_detection_server-X]   巡诊点 1 -> det[0]=bed1, det[1]=bed2, ...
[bed_detection_server-X]   巡诊点 2 -> det[0]=bed7, det[1]=bed8, ...
[bed_detection_server-X] 床位检测服务已创建: /detect_anomaly
[mock_camera-X] 模拟相机节点已启动, 默认模式: area
```

### 2.3 全链路联调启动

需要按顺序启动所有模块：

```bash
# 终端1: 导航节点（师兄负责）
ros2 launch navigation xxx.launch.py

# 终端2: 行为树
ros2 launch behavior_tree xxx.launch.py

# 终端3: 你的床位检测（不用模拟相机，接真实相机）
ros2 launch monitor bed_detection.launch.py use_mock_camera:=false

# 终端4: 相机驱动（如果有独立节点）
ros2 launch camera_driver xxx.launch.py
```

---

## 3. 手动测试服务

### 3.1 Area 模式测试

```bash
# 请求: mode=0(AREA), area_bed_id=1(巡诊点1)
ros2 service call /detect_anomaly interfaces/srv/DetectAnomaly \
    "{mode: 0, area_bed_id: 1}"
```

预期返回：
```
waiting for service to become available...
requester: making request: interfaces.srv.DetectAnomaly_Request(mode=0, area_bed_id=1)

response:
  is_anomaly: True
  details: "Detected 2 occupied beds out of 6 total beds"
  bed_ids: [3, 5]
  urgencies: [1, 0]
```

### 3.2 Bed 模式测试

```bash
# 请求: mode=1(BED), area_bed_id=3(床位3)
ros2 service call /detect_anomaly interfaces/srv/DetectAnomaly \
    "{mode: 1, area_bed_id: 3}"
```

预期返回：
```
response:
  is_anomaly: False
  details: "Bed 3: patient safe, CLIP confidence: 0.230, YOLO confidence: 0.950"
  bed_ids: []
  urgencies: []
```

### 3.3 检查服务是否存在

```bash
# 查看服务列表
ros2 service list | grep detect

# 查看服务类型
ros2 service type /detect_anomaly
```

---

## 4. 问题排查

### 4.1 服务调用无响应

```bash
# 检查节点是否存活
ros2 node list | grep bed

# 检查服务是否注册
ros2 service list | grep detect

# 检查相机话题是否有数据
ros2 topic hz /camera/rgb/image_raw

# 查看节点日志
ros2 topic echo /rosout --filter "node_name == 'bed_detection_server'"
```

**常见原因**：
- 未 `source install/setup.bash`
- 模型文件路径不对，节点启动失败
- 相机话题名不匹配

### 4.2 area_bed_id 始终为 -1 或 0

说明 C++ 行为树传来的值与映射表 key 不匹配。

```bash
# 监控服务请求，看实际传来的值
ros2 service call /detect_anomaly interfaces/srv/DetectAnomaly \
    "{mode: 0, area_bed_id: 0}"   # 先手动测试 0
```

**排查步骤**：
1. 看启动日志中映射表打印的 key 是什么（`巡诊点 1` 还是 `巡诊点 0`）
2. 让师兄确认行为树 `convertFromString("p0")` 输出的是 `0` 还是 `1`
3. 修改 JSON 中 `patrol_id` 使之匹配

### 4.3 YOLO 检测不到床位

```bash
# 查看检测日志
# 正常: "检测到 6 个床位"
# 异常: "未检测到任何床位"
```

**可能原因**：
- 图片中没有床位（测试图片不对）
- 模型文件损坏或版本不对
- 检测置信度阈值过高

### 4.4 CLIP 判断全为无人/全为有人

查看日志中 CLIP 置信度：
```
床位 3 有人, CLIP置信度: 0.820   ← 正常区分
床位 3 有人, CLIP置信度: 0.501   ← 阈值附近，可能不准
```

**可能原因**：
- CLIP 模型不匹配（用了通用模型而非微调模型）
- 裁剪区域太小或偏移（YOLO 框不准）

### 4.5 映射编号与实际不对应

YOLO 检测框的顺序取决于画面中床位从左到右的位置。如果发现 `bed_id=3` 实际对应的是画面中第 5 张床：

1. 记住 YOLO 日志中检测框的顺序（index 0, 1, 2...）
2. 确认每个 index 实际对应的真实床位编号
3. 修改 JSON 中 `detection_index` → `bed_id` 的映射

---

## 5. 关键话题与服务速查

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/detect_anomaly` | `interfaces/srv/DetectAnomaly` | 服务 | 你的核心服务 |
| `/camera/rgb/image_raw` | `sensor_msgs/msg/Image` | 订阅 | 相机图像输入 |
| `/camera/mode` | `std_msgs/msg/String` | 发布 | 切换相机模式(area/bed) |

### 服务接口定义 (`DetectAnomaly.srv`)

```
# 请求
int32 mode          # 0=AREA(区域扫描), 1=BED(单床检测)
int32 area_bed_id   # AREA模式: 巡诊点ID | BED模式: 床位ID
---
# 响应
bool is_anomaly     # AREA: 有人则True | BED: 坠床风险则True
string details      # 检测结果描述
int32[] bed_ids     # AREA: 有人的床位ID列表 | BED: 异常床位ID
int32[] urgencies   # 对应紧急程度 (0=普通, 1=紧急)
```

---

## 6. 数据流全景

```
行为树 NextPatrolPoint → 输出 patrol_point="p0"/"p1"
        ↓
行为树 NavgateTo → 导航到目标点 → 导航完成
        ↓
行为树 Detect_BedProcess → 调用 /detect_anomaly 服务
        ↓                   mode=0, area_bed_id=0或1
        ↓
bed_detection_server → 查映射表 → YOLO检测 → CLIP判断
        ↓
返回 bed_ids=[3,5,9] urgencies=[1,0,0] → 行为树拿到有人床位列表
        ↓
行为树 BedProcessTree → 逐个床位调 /detect_anomaly
        ↓                   mode=1, area_bed_id=3(床位ID)
bed_detection_server → 反查映射 → 定位检测框 → CLIP坠床判断
        ↓
返回 is_anomaly=True/False → 行为树决定是否报警
```

---

## 7. 紧急调试命令

```bash
# 一键重启你的节点（改了JSON后）
# Ctrl+C 停掉当前节点后：
colcon build --packages-select monitor && source install/setup.bash && \
    ros2 launch monitor bed_detection.launch.py

# 快速验证服务是否正常响应
ros2 service call /detect_anomaly interfaces/srv/DetectAnomaly \
    "{mode: 0, area_bed_id: 1}" --timeout 5

# 监控所有日志
ros2 topic echo /rosout

# 检查话题连通性
ros2 topic info /camera/rgb/image_raw -v
```
