# xjrobot 导航功能包说明

本文档用于说明 `src/nav` 目录下的 ROS 2 导航相关功能包。当前系统面向实机导航，整体链路为：底盘与传感器驱动 -> 轮式/IMU/FastLIO 里程计融合 -> 3D 地图重定位与 `map->odom` 发布 -> Nav2 路径规划与控制 -> 上层业务 action 桥接。

## 1. 目录结构与职责

| 功能包 | 主要职责 |
| --- | --- |
| `xjrobot_bringup` | 实机总启动入口，统一装配底盘、定位、导航和业务桥接。 |
| `xjrobot_base` | 底盘 CAN 驱动、速度控制、手柄输入、轮速里程计、Livox MID360 数据旋转补偿、robot_state_publisher 和 EKF。 |
| `xjrobot_localization` | FastLIO 定位、全局重定位、PCD 地图发布、`map->odom` TF 发布、FastLIO 相对里程计转换。 |
| `xjrobot_navigation` | Nav2 启动与参数配置，支持 RPP/MPPI 控制器；将地面分割后的 3D 障碍点云转为 `/scan` 给 Nav2 costmap 使用。 |
| `xjrobot_bridge` | 将上层自定义 `navigate` action 转换为 Nav2 `navigate_to_pose` action，并维护点位表。 |
| `xjrobot_description` | 仿真和可视化使用的机器人 URDF/Xacro、传感器和 Gazebo 插件描述。 |
| `xjrobot_gazebo` | Gazebo Sim 仿真启动、world/model、ROS-Gazebo bridge 和仿真 EKF。 |
| `xjrobot_thirdparty` | 工程内维护的第三方依赖，包括 `ground_segmentation`、`ground_segmentation_ros2`、`pointcloud_to_laserscan` 和 Navigation2 相关源码。 |

## 2. 推荐启动方式

实机完整导航推荐从总入口启动：

```bash
ros2 launch xjrobot_bringup robot.launch.py
```

常用参数：

```bash
ros2 launch xjrobot_bringup robot.launch.py \
  controller:=rpp \
  map:=/home/medical/maps/0423.yaml \
  pcd_map:=/home/medical/maps/0423.pcd \
  initial_pose_x:=0.0 \
  initial_pose_y:=0.0 \
  initial_pose_yaw:=0.0
```

参数说明：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `controller` | `rpp` | Nav2 局部控制器配置，支持 `rpp` 和 `mppi`。 |
| `use_sim_time` | `false` | 实机为 `false`，仿真或 rosbag 回放时设为 `true`。 |
| `map` | `/home/medical/maps/0423.yaml` | Nav2 使用的 2D 栅格地图。 |
| `pcd_map` | `/home/medical/maps/0423.pcd` | FastLIO/重定位使用的 3D PCD 地图。 |
| `localization_config` | `mid360.yaml` | `xjrobot_localization/config` 下的定位参数文件。 |
| `waypoints_file` | `xjrobot_bridge/config/waypoints.yaml` | 上层业务点位配置文件。 |
| `launch_base` | `true` | 是否启动底盘、Livox、地面分割、EKF 等基础链路。 |
| `launch_localization` | `true` | 是否启动 FastLIO、全局重定位和 `map->odom`。 |
| `launch_navigation` | `true` | 是否启动 Nav2。 |
| `launch_bridge` | `true` | 是否启动业务 action 桥接节点。 |
| `navigation_rviz` | `true` | 是否启动导航 RViz。 |
| `localization_rviz` | `false` | 是否启动定位 RViz。 |

分段调试示例：

```bash
# 只启动底盘和传感器链路
ros2 launch xjrobot_bringup robot.launch.py launch_localization:=false launch_navigation:=false launch_bridge:=false

# 调定位，不启动 Nav2 和桥接
ros2 launch xjrobot_bringup robot.launch.py launch_navigation:=false launch_bridge:=false localization_rviz:=true

# 调 Nav2，不启动上层桥接
ros2 launch xjrobot_bringup robot.launch.py launch_bridge:=false
```

## 3. 系统运行逻辑

### 3.1 底盘与传感器链路

`xjrobot_base/launch/base.launch.py` 负责实机底层 bringup：

1. 启动 Livox MID360 驱动，默认读取原始 `/livox/lidar` 和 `/livox/imu`。
2. `mid360_rotation_node` 根据安装角补偿点云和 IMU，输出 `/livox/lidar_rotated`、`/livox/lidar_rotated/points`、`/livox/imu_rotated`。
3. `ground_segmentation_ros2` 对旋转后的点云做地面分割，输出障碍点云 `/ground_segmentation/obstacle_points`。
4. `base_controller_node` 订阅 `/cmd_vel_safe`，通过 `/realcmd_ctrl` 下发底盘控制。
5. `can_hardware_node` 与 CAN 设备通信，发布 `/can_msg_fb` 等反馈。
6. `wheel_odom_fusion_node` 读取 CAN 轮速，输出原始轮式里程计 `/odom/unfiltered`。
7. `robot_localization/ekf_node` 融合 `/odom/unfiltered`、`/odom_fastlio_relative`、`/livox/imu_rotated`，输出最终 `/odom` 和 `odom->base_footprint` TF。
8. `robot_state_publisher` 发布机器人固定 TF。

底盘关键参数在 `xjrobot_base/config/base.yaml`。需要重点关注：

| 参数 | 作用 |
| --- | --- |
| `cmd_vel_topic: /cmd_vel_safe` | 底盘只执行安全速度口，通常来自 Nav2 smoother/collision monitor 后级。 |
| `output_odom_topic: /odom/unfiltered` | 轮速原始里程计输出，供 EKF 使用。 |
| `mount_roll_deg/mount_pitch_deg/mount_yaw_deg` | MID360 安装角补偿。 |
| `imu_angular_velocity_bias` | IMU 静态角速度偏置补偿，单位必须为 rad/s。 |
| `publish_odom_tf: false` | 轮式节点不发布 TF，最终 `odom->base_footprint` 由 EKF 发布。 |

### 3.2 定位与 TF 链路

`xjrobot_localization/launch/localization.launch.py` 负责定位：

1. `fastlio_mapping` 使用 `/livox/lidar_rotated` 和 `/livox/imu_rotated` 运行 FastLIO，输出 `/odom_fastlio` 和 FastLIO TF 树。
2. `pcd_to_pointcloud` 将 PCD 地图发布到 `/pcd_map`。
3. `global_localization.py` 根据 PCD 地图和当前扫描做全局重定位。
4. `transform_fusion.py` 维护 FastLIO 内部定位结果。
5. `fastlio_relative_odom_node` 将 `/odom_fastlio` 的连续位姿转换为平面相对里程计 `/odom_fastlio_relative`，供 EKF 融合。
6. `map_odom_tf_publisher` 将 FastLIO 定位结果和主导航 TF 树对齐，发布对外使用的 `map->odom`。
7. `manual_initial_pose_publisher.py` 可在启动后延迟发布一次初始位姿先验。

主导航 TF 约定：

```text
map -> odom -> base_footprint -> base_link -> lidar_3d_link / imu_link
```

FastLIO 内部 TF 约定：

```text
map_fastlio -> odom_fastlio -> base_link_fastlio -> lidar_3d_frame_fastlio
```

`map_odom_tf_publisher` 会把这两棵树对齐，对 Nav2 输出标准 `map->odom`。因此 Nav2 不启动 AMCL，定位来源由 3D FastLIO/重定位链路提供。

定位关键参数在 `xjrobot_localization/config/mid360.yaml`。换地图时必须同步检查：

| 项目 | 文件/参数 |
| --- | --- |
| 3D 地图 | `laser_mapping.ros__parameters.map_file_path` 或总 launch 的 `pcd_map:=...` |
| 2D 地图 | 总 launch 的 `map:=...` |
| FastLIO 输入 | `common.lid_topic`、`common.imu_topic` |
| 全局定位阈值 | `global_localization.localization_threshold` |
| PCD 地图话题 | `global_localization.pcd_map_topic`、`map_publisher.cloud_topic` |

### 3.3 Nav2 导航链路

`xjrobot_navigation` 提供多个导航启动入口：

| launch | 用途 |
| --- | --- |
| `navigation_rpp.launch.py` | 实机 RPP 控制器配置，默认推荐。 |
| `navigation_mppi.launch.py` | 实机 MPPI 控制器配置，适合进一步调优。 |
| `navigation.launch.py` | 通用 Nav2 bringup，带官方 localization，更多用于早期/仿真验证。 |
| `navigation_rpp_smac.launch.py` | RPP + SMAC 规划器配置。 |
| `pointcloud_to_laserscan.launch.py` | 将 3D 障碍点云投影为 `/scan`。 |

实机 RPP/MPPI 启动逻辑：

1. `pointcloud_to_laserscan` 订阅 `/ground_segmentation/obstacle_points`，输出 `/scan`。
2. 单独启动 `nav2_map_server` 读取 2D 地图。
3. 启动 Nav2 bringup，传入 `RPP.yaml` 或 `MPPI.yaml`。
4. 传参 `use_localization:=False`，避免 AMCL 与 3D 定位链路冲突。
5. 可选启动 `xjrobot_navigation/rviz/xjrobot_navigation.rviz`。

Nav2 主要配置文件：

| 文件 | 说明 |
| --- | --- |
| `config/RPP.yaml` | RPP 控制器实机配置。 |
| `config/MPPI.yaml` | MPPI 控制器实机配置。 |
| `config/RPP_SMAC.yaml` | SMAC planner 相关配置。 |
| `behavior_trees/*.xml` | 去掉倒车或只等待等定制行为树。 |

### 3.4 上层业务桥接

`xjrobot_bridge` 负责把上层自定义 action `navigate` 转成 Nav2 `navigate_to_pose`：

| 输入/输出 | 说明 |
| --- | --- |
| action server `navigate` | 上层调用入口。 |
| action client `navigate_to_pose` | 下发到 Nav2 的标准 action。 |
| `waypoints.yaml` | 点位表，使用 `target_index` 查找目标点。 |

当前 `nav_type` 语义：

| `nav_type` | 说明 |
| --- | --- |
| `0` | 普通导航：按 `target_index` 查点位，并发送到 Nav2。 |
| `1` | 停止导航：取消当前 Nav2 目标。 |
| `2` | 回桩：预留接口，当前未实现，会返回失败。 |

点位配置位于 `xjrobot_bridge/config/waypoints.yaml`。新增或修改点位时需要：

1. 在 `waypoint_ids` 中加入点位条目 id。
2. 在 `waypoints.<id>` 下配置 `index/name/frame_id/x/y/yaw`。
3. 确认 `index` 与上层 `Navigate.goal.target_index` 一致。
4. 确认坐标系通常为 `map`，坐标来自当前 2D/3D 地图对齐后的地图系。

## 4. 仿真链路

Gazebo 仿真入口：

```bash
ros2 launch xjrobot_gazebo gazebo.launch.py
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 是否启动 Gazebo GUI。 |
| `use_sim_time` | `true` | 仿真使用 `/clock`。 |
| `world_name` | `xjrobot/myworld2.sdf` | world 相对路径。 |
| `spawn_x/spawn_y/spawn_z/spawn_yaw` | `0/0/0.12/0` | 机器人初始位姿。 |
| `odom_topic` | `/odom` | 仿真 EKF 输出里程计话题。 |

仿真中 `ros_gz_bridge` 会桥接 `/clock`、`/cmd_vel`、`/odom/unfiltered`、`/imu/data`、`/scan`、`/livox/scan3d/points` 等话题，并由仿真 EKF 输出 `/odom`。

## 5. 地图与点位交接

当前实机默认地图路径写在 `xjrobot_bringup/launch/robot.launch.py`：

```text
2D 栅格地图: /home/medical/maps/0423.yaml
3D PCD 地图: /home/medical/maps/0423.pcd
```

交接或换场地时建议同时交付：

| 文件 | 用途 |
| --- | --- |
| `*.yaml` + 对应 `*.pgm/*.png` | Nav2 map_server 使用的 2D 栅格地图。 |
| `*.pcd` | FastLIO/global localization 使用的 3D 点云地图。 |
| `waypoints.yaml` | 上层业务点位表。 |
| RViz 配置截图或说明 | 用于接手人确认地图、TF、点云和 costmap 是否正常。 |

换图流程建议：

1. 将新的 2D/3D 地图放入统一目录，例如 `/home/medical/maps`。
2. 启动时通过 `map:=...` 和 `pcd_map:=...` 显式传入。
3. 在 RViz 中检查 `/pcd_map`、2D map、机器人当前位置是否对齐。
4. 重新采集或修正 `xjrobot_bridge/config/waypoints.yaml` 中的点位。
5. 用桥接 action 或 RViz 逐点验证导航。

## 6. 常用检查命令

构建：

```bash
colcon build --packages-up-to \
  xjrobot_bringup \
  xjrobot_base \
  xjrobot_localization \
  xjrobot_navigation \
  xjrobot_bridge
source install/setup.bash
```

检查关键话题：

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic echo /livox/imu_rotated
ros2 topic echo /ground_segmentation/obstacle_points
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link lidar_3d_link
```

检查 Nav2 action：

```bash
ros2 action list
ros2 action info /navigate_to_pose
ros2 action info /navigate
```

检查节点：

```bash
ros2 node list
ros2 node info /ekf_filter_node
ros2 node info /xjrobot_bridge_node
```

## 7. 常见问题排查

| 现象 | 优先检查 |
| --- | --- |
| Nav2 costmap 无障碍物 | `/ground_segmentation/obstacle_points` 是否有数据，`pointcloud_to_laserscan` 是否输出 `/scan`，`target_frame` 是否为 `lidar_3d_link`。 |
| RViz 中机器人不在地图上 | `map->odom` 是否发布，`pcd_map` 与 2D map 是否同场景同坐标，初始位姿是否合理。 |
| `map->odom` 不发布 | FastLIO TF 是否存在，`odom->lidar_3d_link` 是否完整，检查 `map_odom_tf_publisher` 日志。 |
| EKF 输出抖动或漂移 | 检查 `/odom/unfiltered`、`/odom_fastlio_relative`、`/livox/imu_rotated` 时间戳和协方差，确认 IMU 单位为 rad/s。 |
| 底盘不动 | 检查 `/cmd_vel_safe` 是否有速度，CAN 设备是否正常，`can_hardware_node` 是否启动，急停/安全层是否拦截。 |
| 桥接 action 返回未找到点位 | 检查 `waypoints.yaml` 的 `waypoint_ids` 是否包含该条目，`index` 是否与上层 `target_index` 一致。 |
| STOP 无效 | 只有当前桥接节点维护着活动 Nav2 goal 时 STOP 才能取消；如果目标不是通过 `navigate` action 下发，桥接层可能没有活动上下文。 |

## 8. 维护注意事项

1. 实机默认不使用 AMCL。不要在实机 RPP/MPPI 链路中同时启动 AMCL，否则可能与 3D 定位发布的 `map->odom` 冲突。
2. `/odom` 和 `odom->base_footprint` 由 EKF 统一发布，轮式里程计节点保持 `publish_odom_tf: false`。
3. MID360 安装角、IMU bias、轮径、轮距和速度补偿都属于实机标定参数，换车或改安装位置后需要重新校准。
4. `xjrobot_bridge` 当前只实现单点导航和停止，`nav_type=2` 回桩只是预留接口。
5. 地图、点位、RViz 显示必须成套验证。只换 2D 地图或只换 PCD 地图都可能造成定位与导航坐标不一致。
6. `xjrobot_base/lib` 下包含 CAN 相关动态库，部署到新机器时要确认库文件、设备权限和 CAN/USB 连接都可用。
7. 总 launch 中 Nav2 延迟 5 秒、桥接延迟 8 秒启动，是为了等待底盘/TF/定位链路先起来；如果现场机器启动慢，可适当增大延迟。

## 9. 关键文件索引

| 文件 | 说明 |
| --- | --- |
| `xjrobot_bringup/launch/robot.launch.py` | 实机完整导航总入口。 |
| `xjrobot_base/launch/base.launch.py` | 底盘、传感器、地面分割、EKF 启动入口。 |
| `xjrobot_base/config/base.yaml` | 底盘、轮速、MID360 旋转和 IMU 补偿参数。 |
| `xjrobot_base/config/ekf.yaml` | 实机 EKF 融合配置。 |
| `xjrobot_localization/launch/localization.launch.py` | FastLIO、重定位、PCD 地图和 `map->odom` 启动入口。 |
| `xjrobot_localization/config/mid360.yaml` | 实机 MID360 定位配置。 |
| `xjrobot_navigation/launch/navigation_rpp.launch.py` | RPP 实机导航入口。 |
| `xjrobot_navigation/launch/navigation_mppi.launch.py` | MPPI 实机导航入口。 |
| `xjrobot_navigation/config/RPP.yaml` | RPP Nav2 参数。 |
| `xjrobot_navigation/config/MPPI.yaml` | MPPI Nav2 参数。 |
| `xjrobot_bridge/config/waypoints.yaml` | 业务点位表。 |
| `xjrobot_bridge/src/xjrobot_bridge_node.cpp` | 自定义导航 action 到 Nav2 action 的桥接实现。 |
| `xjrobot_gazebo/launch/gazebo.launch.py` | Gazebo 仿真入口。 |

