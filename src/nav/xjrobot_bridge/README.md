# xjrobot_bridge

## 包用途说明

`xjrobot_bridge` 用于在你的 ROS2 系统中提供一个导航桥接层，把上层行为树节点 `NavgateTo` 发出的自定义 action：

- action 类型：`interfaces::action::Navigate`
- action 名称：`navigate`

转换成 Nav2 原生的单点导航 action：

- action 类型：`nav2_msgs::action::NavigateToPose`
- action 名称：`navigate_to_pose`

当前版本只实现单点导航 `NavigateToPose` 桥接，并保留了后续扩展多点导航与 docking 的代码结构。

## 架构说明

桥接节点名：

- `xjrobot_bridge_node`

职责分层：

- 上层：作为 `interfaces::action::Navigate` 的 action server，对外提供 `navigate`
- 下层：作为 `nav2_msgs::action::NavigateToPose` 的 action client，对接 Nav2 的 `navigate_to_pose`

处理逻辑：

1. 接收上层 `Navigate::Goal`
2. 根据 `target_index` 查询 `waypoints.yaml` 中配置的点位
3. 转成 `geometry_msgs::msg::PoseStamped`
4. 当 `nav_type == 0` 时，转发给 Nav2 `NavigateToPose`
5. 当 `nav_type == 1` 时，取消当前正在执行的 Nav2 goal
6. 当 `nav_type == 2` 时，仅打印日志并返回未实现
7. 将 Nav2 执行结果转换回 `interfaces::action::Navigate::Result`

当前你提供的 `Navigate.action` 字段中只有 `target_index` 与 `nav_type`，并没有 `target_name`。因此本版本先按 `target_index` 查表。代码中已经预留了扩展结构，后续如果你的接口新增 `target_name`，可以直接补充按名称查表逻辑。

## 点位参数示例

参数文件：`config/waypoints.yaml`

```yaml
xjrobot_bridge_node:
  ros__parameters:
    action_name: navigate
    nav2_action_name: navigate_to_pose
    default_frame_id: map
    waypoint_names: [nurse_station, bed_1, charging_pile]

    waypoints.nurse_station.index: 0
    waypoints.nurse_station.frame_id: map
    waypoints.nurse_station.x: 1.0
    waypoints.nurse_station.y: 2.0
    waypoints.nurse_station.yaw: 0.0
```

建议约定：

- `target_index` 由上层 BT 节点传入
- `waypoints.<name>.index` 用于建立索引到点位名的映射
- `yaw` 单位为弧度
- 坐标系默认使用 `map`

## 编译方法

由于 `xjrobot_bridge` 依赖你另一个工作空间中的 `interfaces` 包，编译前需要先 source 对应环境。例如：

```bash
source /home/wu/demo0422/Medical_Embodied/install/setup.bash
cd /home/wu/demo0422/medical_embodied_ros2_ws
colcon build --packages-select xjrobot_bridge
```

编译完成后：

```bash
source /home/wu/demo0422/medical_embodied_ros2_ws/install/setup.bash
```

## 运行方法

启动桥接节点：

```bash
ros2 launch xjrobot_bridge xjrobot_bridge.launch.py
```

如果要替换自定义点位文件：

```bash
ros2 launch xjrobot_bridge xjrobot_bridge.launch.py \
  waypoints_file:=/absolute/path/to/waypoints.yaml
```

## 如何与现有 `NavgateTo` 节点对接

你不需要修改现有 `NavgateTo` 节点。

只要保证以下几点：

- `NavgateTo` 继续向 `navigate` action 发送 `interfaces::action::Navigate` goal
- `target_index` 与 `waypoints.yaml` 中配置的 `waypoints.<name>.index` 一致
- Nav2 已正常启动，并对外提供 `navigate_to_pose` action server

这样上层 BT 看到的仍然是你原来的自定义导航 action，而底层实际执行已经切换为 Nav2。

## 后续扩展

后续可以在当前结构上继续扩展：

- `NavigateThroughPoses`
  - 在桥接节点中增加多点目标解析
  - 新增到 `nav2_msgs::action::NavigateThroughPoses` 的 action client
  - 根据上层自定义 action 字段决定走单点还是多点

- docking
  - 在 `execute_dock_navigation()` 中接入真实 docking action 或 service
  - 当前版本仅明确返回“未实现”，避免误接假逻辑

## 结果状态映射说明

当前版本的结果映射策略：

- Nav2 成功：映射为 `ActionStatus::OK`
- Nav2 被取消：映射为 `ActionStatus::PREEMPTED`
- Nav2 拒绝、失败或 docking 未实现：映射为 `ActionStatus::ABORTED`
- 等待 Nav2 server 或响应超时：映射为 `ActionStatus::TIMEOUT`

如果你后续希望进一步区分 `NO_PATH` 等细粒度错误，可以在 Nav2 result error code 扩展后继续补充映射表。
