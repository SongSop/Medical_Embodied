#ifndef XJROBOT_BRIDGE__XJROBOT_BRIDGE_NODE_HPP_
#define XJROBOT_BRIDGE__XJROBOT_BRIDGE_NODE_HPP_

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/action/navigate.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace xjrobot_bridge
{

// 桥接节点：
// 1. 对上作为 interfaces::action::Navigate 的 action server
// 2. 对下作为 nav2_msgs::action::NavigateToPose 的 action client
// 3. 负责把上层抽象导航指令转换为 Nav2 可执行的单点导航目标
class XjrobotBridgeNode : public rclcpp::Node
{
public:
  using Navigate = interfaces::action::Navigate;
  using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<Navigate>;
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using Nav2GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  explicit XjrobotBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 点位表中的单个点位定义。
  // 当前版本主要通过 index 建立映射，同时保留 name 字段，方便后续扩展
  // “按 target_name 查表”或“多点序列导航”等能力。
  struct Waypoint
  {
    int index{0};
    std::string name;
    std::string frame_id;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  // 当前正在执行的一次桥接导航上下文。
  // custom_goal_handle: 上层自定义 action 的 goal handle
  // nav2_goal_handle:   下层 Nav2 action 的 goal handle
  // waypoint_name:      便于日志定位当前目标点
  // initial_distance:   用于把 Nav2 的剩余距离换算成 0~1 进度
  struct ActiveNavigation
  {
    std::shared_ptr<NavigateGoalHandle> custom_goal_handle;
    std::shared_ptr<Nav2GoalHandle> nav2_goal_handle;
    std::string waypoint_name;
    double initial_distance{-1.0};
  };

  // 上层发来新 goal 时的入口。
  // 这里只做“是否接收”的决策，不做真正耗时执行。
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigate::Goal> goal);

  // 上层 action client 主动 cancel 时的入口。
  // 这里会把取消请求继续向下传递到 Nav2。
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<NavigateGoalHandle> goal_handle);

  // Goal 被接受后，在独立线程中启动真正执行逻辑。
  void handle_accepted(const std::shared_ptr<NavigateGoalHandle> goal_handle);

  // 根据 nav_type 分发到 GOAL / STOP / DOCK 三条处理路径。
  void execute(const std::shared_ptr<NavigateGoalHandle> goal_handle);

  // 处理普通导航：
  // 自定义 Navigate(goal) -> 查点位 -> Nav2 NavigateToPose。
  void execute_goal_navigation(
    const std::shared_ptr<NavigateGoalHandle> goal_handle,
    const Navigate::Goal & goal);

  // 处理停止导航：
  // 取消当前正在运行的 Nav2 目标。
  void execute_stop_navigation(const std::shared_ptr<NavigateGoalHandle> goal_handle);

  // 处理回桩导航：
  // 当前版本只保留结构，不接入真实 docking 逻辑。
  void execute_dock_navigation(const std::shared_ptr<NavigateGoalHandle> goal_handle);

  // 根据上层 goal 查找点位并构造 PoseStamped。
  // 当前版本按 target_index 查表，后续可扩展为按名称优先查找。
  bool resolve_goal_pose(
    const Navigate::Goal & goal,
    geometry_msgs::msg::PoseStamped & pose,
    std::string & waypoint_name,
    std::string & error_message) const;

  // 从 ROS 参数加载 waypoints 配置。
  // 参数结构约定为：
  // waypoint_names: [name1, name2, ...]
  // waypoints.<name>.index/frame_id/x/y/yaw
  void load_waypoints();

  // 统一构造 ActionStatus，避免各处分散填写状态字段。
  interfaces::msg::ActionStatus build_status(uint8_t code) const;

  // 统一封装上层 action 的成功/失败/取消结果返回。
  void finish_goal_succeeded(
    const std::shared_ptr<NavigateGoalHandle> & goal_handle,
    const std::string & message) const;
  void finish_goal_aborted(
    const std::shared_ptr<NavigateGoalHandle> & goal_handle,
    uint8_t status_code,
    const std::string & message) const;
  void finish_goal_canceled(
    const std::shared_ptr<NavigateGoalHandle> & goal_handle,
    const std::string & message) const;

  // 尝试取消当前活跃的 Nav2 目标。
  // wait_for_result=false 适用于上层 cancel 回调，避免在回调线程里阻塞太久。
  bool cancel_active_nav2_goal(const std::string & reason, bool wait_for_result = true);

  // 仅在当前 active_navigation_ 仍属于这个上层 goal 时才清理，
  // 避免并发情况下误清掉更新后的上下文。
  void clear_active_navigation_locked(const std::shared_ptr<NavigateGoalHandle> & custom_goal_handle);

  // 将 Nav2 的 distance_remaining 转成上层可理解的 progress。
  void publish_progress_feedback(
    const std::shared_ptr<NavigateGoalHandle> & goal_handle,
    double initial_distance,
    double remaining_distance) const;

  // 上层自定义导航 action server。
  rclcpp_action::Server<Navigate>::SharedPtr navigate_server_;

  // 下层 Nav2 单点导航 action client。
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav2_client_;

  // 两份索引：
  // 1. index -> Waypoint：服务当前 target_index 查表
  // 2. name  -> Waypoint：为后续 target_name/调试日志预留
  std::unordered_map<int, Waypoint> waypoint_by_index_;
  std::unordered_map<std::string, Waypoint> waypoint_by_name_;

  // active_navigation_ 可能同时被执行线程、反馈回调、cancel 路径访问，
  // 因此必须通过互斥锁保护。
  mutable std::mutex active_nav_mutex_;
  std::optional<ActiveNavigation> active_navigation_;

  // 运行参数：
  // action_name_                上层 action server 名称，默认 navigate
  // nav2_action_name_           下层 Nav2 action 名称，默认 navigate_to_pose
  // default_frame_id_           点位未单独指定 frame_id 时使用的默认坐标系
  // nav2_server_wait_timeout    等待 Nav2 action server 出现的超时时间
  // nav2_request_timeout        等待 Nav2 接收 goal / cancel 响应的超时时间
  std::string action_name_;
  std::string nav2_action_name_;
  std::string default_frame_id_;
  double nav2_server_wait_timeout_sec_{5.0};
  double nav2_request_timeout_sec_{5.0};
};

}  // namespace xjrobot_bridge

#endif  // XJROBOT_BRIDGE__XJROBOT_BRIDGE_NODE_HPP_
