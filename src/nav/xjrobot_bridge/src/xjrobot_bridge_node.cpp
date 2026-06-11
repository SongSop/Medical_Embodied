#include "xjrobot_bridge/xjrobot_bridge_node.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <memory>
#include <sstream>
#include <thread>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"

namespace xjrobot_bridge
{

namespace
{

// 把 ROS2 action UUID 转成十六进制字符串，便于日志追踪同一条请求。
std::string goal_uuid_to_string(const rclcpp_action::GoalUUID & uuid)
{
  std::ostringstream stream;
  stream << std::hex;
  for (const auto byte : uuid) {
    stream.width(2);
    stream.fill('0');
    stream << static_cast<int>(byte);
  }
  return stream.str();
}

}  // namespace

XjrobotBridgeNode::XjrobotBridgeNode(const rclcpp::NodeOptions & options)
: Node("xjrobot_bridge_node", options)
{
  // 基础参数：
  // action_name_ / nav2_action_name_ 用于适配现有系统话题命名；
  // timeout 参数用于控制“等待 server 出现”和“等待请求响应”的最长时间。
  action_name_ = this->declare_parameter<std::string>("action_name", "navigate");
  nav2_action_name_ = this->declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
  default_frame_id_ = this->declare_parameter<std::string>("default_frame_id", "map");
  nav2_server_wait_timeout_sec_ =
    this->declare_parameter<double>("nav2_server_wait_timeout_sec", 5.0);
  nav2_request_timeout_sec_ =
    this->declare_parameter<double>("nav2_request_timeout_sec", 5.0);
  dock_staging_index_ = this->declare_parameter<int>("dock_staging_index", -1);
  dock_action_name_ = this->declare_parameter<std::string>("dock_action_name", "dock");
  dock_action_wait_timeout_sec_ =
    this->declare_parameter<double>("dock_action_wait_timeout_sec", 130.0);
  dock_coarse_progress_ratio_ =
    static_cast<float>(this->declare_parameter<double>("dock_coarse_progress_ratio", 0.5));

  // 启动时一次性加载点位表，后续 goal 执行时直接查内存映射。
  load_waypoints();

  // 建立下层 Nav2 action client 和上层自定义 action server。
  nav2_client_ = rclcpp_action::create_client<Nav2Navigate>(this, nav2_action_name_);
  dock_action_client_ = rclcpp_action::create_client<DockingAction>(this, dock_action_name_);
  navigate_server_ = rclcpp_action::create_server<Navigate>(
    this,
    action_name_,
    std::bind(&XjrobotBridgeNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&XjrobotBridgeNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&XjrobotBridgeNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "xjrobot_bridge_node ready. custom_action=%s, nav2_action=%s, waypoint_count=%zu, "
    "dock_staging_index=%d, dock_action=%s",
    action_name_.c_str(),
    nav2_action_name_.c_str(),
    waypoint_by_name_.size(),
    dock_staging_index_,
    dock_action_name_.c_str());
}

void XjrobotBridgeNode::load_waypoints()
{
  // 重新加载前先清空，避免热重启或重复构造时残留旧数据。
  waypoint_by_index_.clear();
  waypoint_by_name_.clear();

  // 点位参数结构：
  // waypoint_ids: [patrol_0, bed_1, ...]
  // waypoints.<id>.index/name/frame_id/x/y/yaw
  // id 是配置条目的业务化 key，真正用于查表的是每个条目里的 index 字段。
  const auto waypoint_ids = this->declare_parameter<std::vector<std::string>>(
    "waypoint_ids", std::vector<std::string>{});
  for (const auto & waypoint_id : waypoint_ids) {
    Waypoint waypoint;
    waypoint.index = this->declare_parameter<int>("waypoints." + waypoint_id + ".index", -1);
    if (waypoint.index < 0) {
      RCLCPP_WARN(
        get_logger(),
        "Skip waypoint id '%s': waypoints.%s.index must be >= 0.",
        waypoint_id.c_str(),
        waypoint_id.c_str());
      continue;
    }
    waypoint.name = this->declare_parameter<std::string>(
      "waypoints." + waypoint_id + ".name", waypoint_id);
    waypoint.frame_id = this->declare_parameter<std::string>(
      "waypoints." + waypoint_id + ".frame_id", default_frame_id_);
    waypoint.x = this->declare_parameter<double>("waypoints." + waypoint_id + ".x", 0.0);
    waypoint.y = this->declare_parameter<double>("waypoints." + waypoint_id + ".y", 0.0);
    waypoint.yaw = this->declare_parameter<double>("waypoints." + waypoint_id + ".yaw", 0.0);

    // 姓名索引总是保留，方便调试与后续扩展。
    waypoint_by_name_[waypoint.name] = waypoint;
    // 当前业务真正使用的是 index -> waypoint 映射。
    waypoint_by_index_[waypoint.index] = waypoint;

    RCLCPP_INFO(
      get_logger(),
      "Loaded waypoint name=%s index=%d frame=%s pose=(%.3f, %.3f, %.3f)",
      waypoint.name.c_str(),
      waypoint.index,
      waypoint.frame_id.c_str(),
      waypoint.x,
      waypoint.y,
      waypoint.yaw);
  }
}

rclcpp_action::GoalResponse XjrobotBridgeNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Navigate::Goal> goal)
{
  // 当前策略是全部先接收，再在执行阶段根据 nav_type / 点位合法性决定成功或失败。
  // 这样可以把失败原因明确返回给上层，而不是在握手阶段直接拒绝掉。
  RCLCPP_INFO(
    get_logger(),
    "Received custom navigate goal: uuid=%s target_index=%d nav_type=%d",
    goal_uuid_to_string(uuid).c_str(),
    goal->target_index,
    goal->nav_type);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse XjrobotBridgeNode::handle_cancel(
  const std::shared_ptr<NavigateGoalHandle> goal_handle)
{
  // 上层主动 cancel 时，桥接层应该尽快把取消信号向下游 Nav2 传递。
  // 这里不等待 Nav2 完整返回，避免占住 action cancel 回调线程。
  RCLCPP_WARN(get_logger(), "Received cancel request from upper layer.");
  cancel_active_nav2_goal("upper action client requested cancel", false);
  {
    std::lock_guard<std::mutex> lock(active_dock_mutex_);
    if (active_docking_.has_value() &&
      active_docking_->custom_goal_handle == goal_handle)
    {
      stop_fine_docking();
    }
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void XjrobotBridgeNode::handle_accepted(const std::shared_ptr<NavigateGoalHandle> goal_handle)
{
  // action execute 通常包含等待 Nav2 返回、取消、反馈转发等耗时操作，
  // 因此放到独立线程，避免阻塞 executor。
  std::thread{std::bind(&XjrobotBridgeNode::execute, this, goal_handle)}.detach();
}

void XjrobotBridgeNode::execute(const std::shared_ptr<NavigateGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  // 自定义接口里 nav_type 直接代表三种业务语义：
  // 0=普通导航，1=停止当前导航，2=回桩（粗导航 + 精对接）。
  switch (goal->nav_type) {
    case 0:
      execute_goal_navigation(goal_handle, *goal);
      break;
    case 1:
      execute_stop_navigation(goal_handle);
      break;
    case 2:
      execute_dock_navigation(goal_handle);
      break;
    default:
      finish_goal_aborted(
        goal_handle,
        interfaces::msg::ActionStatus::ABORTED,
        "未知 nav_type，当前仅支持 GOAL(0) / STOP(1) / DOCK(2)");
      break;
  }
}

void XjrobotBridgeNode::execute_goal_navigation(
  const std::shared_ptr<NavigateGoalHandle> goal_handle,
  const Navigate::Goal & goal)
{
  geometry_msgs::msg::PoseStamped target_pose;
  std::string waypoint_name;
  std::string error_message;
  if (!resolve_goal_pose(goal, target_pose, waypoint_name, error_message)) {
    finish_goal_aborted(goal_handle, interfaces::msg::ActionStatus::ABORTED, error_message);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Resolved waypoint: index=%d name=%s frame=%s pose=(%.3f, %.3f, %.3f)",
    goal.target_index,
    waypoint_name.c_str(),
    target_pose.header.frame_id.c_str(),
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    waypoint_by_index_.at(goal.target_index).yaw);

  cancel_active_nav2_goal("new GOAL preempts current navigation");

  const auto outcome = run_nav2_navigation(
    goal_handle, target_pose, waypoint_name, error_message);
  switch (outcome) {
    case Nav2RunOutcome::SUCCEEDED:
      finish_goal_succeeded(goal_handle, "导航执行成功");
      return;
    case Nav2RunOutcome::CANCELED:
      finish_goal_canceled(goal_handle, "导航已取消");
      return;
    case Nav2RunOutcome::TIMEOUT:
      finish_goal_aborted(goal_handle, interfaces::msg::ActionStatus::TIMEOUT, error_message);
      return;
    case Nav2RunOutcome::ABORTED:
      finish_goal_aborted(goal_handle, interfaces::msg::ActionStatus::ABORTED, error_message);
      return;
  }
}

void XjrobotBridgeNode::execute_stop_navigation(const std::shared_ptr<NavigateGoalHandle> goal_handle)
{
  // STOP 不需要解析点位，也不会下发新目标，只负责取消当前 Nav2 目标。
  const bool canceled = cancel_active_nav2_goal("received STOP command");
  if (canceled) {
    finish_goal_succeeded(goal_handle, "已取消当前 Nav2 导航目标");
  } else {
    finish_goal_aborted(
      goal_handle,
      interfaces::msg::ActionStatus::ABORTED,
      "当前没有可取消的 Nav2 导航目标");
  }
}

XjrobotBridgeNode::Nav2RunOutcome XjrobotBridgeNode::run_nav2_navigation(
  const std::shared_ptr<NavigateGoalHandle> & goal_handle,
  const geometry_msgs::msg::PoseStamped & target_pose,
  const std::string & waypoint_name,
  std::string & error_message,
  const float progress_scale,
  const float progress_offset)
{
  if (!nav2_client_->wait_for_action_server(
      std::chrono::duration<double>(nav2_server_wait_timeout_sec_)))
  {
    error_message = "等待 Nav2 NavigateToPose action server 超时";
    return Nav2RunOutcome::TIMEOUT;
  }

  Nav2Navigate::Goal nav2_goal;
  nav2_goal.pose = target_pose;

  RCLCPP_INFO(
    get_logger(),
    "Forward goal to Nav2: waypoint=%s frame=%s pose=(%.3f, %.3f, %.3f)",
    waypoint_name.c_str(),
    nav2_goal.pose.header.frame_id.c_str(),
    nav2_goal.pose.pose.position.x,
    nav2_goal.pose.pose.position.y,
    nav2_goal.pose.pose.position.z);

  auto nav2_feedback_options =
    typename rclcpp_action::Client<Nav2Navigate>::SendGoalOptions();
  nav2_feedback_options.feedback_callback =
    [this, goal_handle, progress_scale, progress_offset](
    Nav2GoalHandle::SharedPtr,
    const std::shared_ptr<const Nav2Navigate::Feedback> feedback)
    {
      double initial_distance = -1.0;
      {
        std::lock_guard<std::mutex> lock(active_nav_mutex_);
        if (active_navigation_.has_value() &&
          active_navigation_->custom_goal_handle == goal_handle)
        {
          if (active_navigation_->initial_distance < 0.0 &&
            feedback->distance_remaining > 0.0)
          {
            active_navigation_->initial_distance = feedback->distance_remaining;
          }
          initial_distance = active_navigation_->initial_distance;
        }
      }
      publish_progress_feedback(
        goal_handle,
        initial_distance,
        feedback->distance_remaining,
        progress_scale,
        progress_offset);
    };

  auto goal_future = nav2_client_->async_send_goal(nav2_goal, nav2_feedback_options);
  if (goal_future.wait_for(std::chrono::duration<double>(nav2_request_timeout_sec_)) !=
    std::future_status::ready)
  {
    error_message = "发送目标到 Nav2 时等待响应超时";
    return Nav2RunOutcome::TIMEOUT;
  }

  auto nav2_goal_handle = goal_future.get();
  if (!nav2_goal_handle) {
    error_message = "Nav2 拒绝了导航目标";
    return Nav2RunOutcome::ABORTED;
  }

  {
    std::lock_guard<std::mutex> lock(active_nav_mutex_);
    active_navigation_ = ActiveNavigation{goal_handle, nav2_goal_handle, waypoint_name, -1.0};
  }

  auto result_future = nav2_client_->async_get_result(nav2_goal_handle);
  while (rclcpp::ok()) {
    if (result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
      break;
    }
  }
  if (!rclcpp::ok()) {
    clear_active_navigation_locked(goal_handle);
    error_message = "ROS 上下文已关闭";
    return Nav2RunOutcome::ABORTED;
  }

  const auto wrapped_result = result_future.get();
  clear_active_navigation_locked(goal_handle);

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return Nav2RunOutcome::SUCCEEDED;
    case rclcpp_action::ResultCode::CANCELED:
      error_message = "导航已取消";
      return Nav2RunOutcome::CANCELED;
    case rclcpp_action::ResultCode::ABORTED:
      error_message = wrapped_result.result->error_msg.empty() ?
        "Nav2 导航失败" : wrapped_result.result->error_msg;
      return Nav2RunOutcome::ABORTED;
    default:
      error_message = "Nav2 返回了未知结果状态";
      return Nav2RunOutcome::ABORTED;
  }
}

void XjrobotBridgeNode::execute_dock_navigation(const std::shared_ptr<NavigateGoalHandle> goal_handle)
{
  if (dock_staging_index_ < 0 ||
    waypoint_by_index_.find(dock_staging_index_) == waypoint_by_index_.end())
  {
    finish_goal_aborted(
      goal_handle,
      interfaces::msg::ActionStatus::ABORTED,
      "未配置有效的 dock_staging_index，请在 waypoints.yaml 中设置充电桩候位点");
    return;
  }

  geometry_msgs::msg::PoseStamped staging_pose;
  std::string staging_name;
  std::string error_message;
  if (!resolve_waypoint_pose(dock_staging_index_, staging_pose, staging_name, error_message)) {
    finish_goal_aborted(goal_handle, interfaces::msg::ActionStatus::ABORTED, error_message);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "DOCK phase 1/2: coarse navigation to staging waypoint index=%d name=%s",
    dock_staging_index_,
    staging_name.c_str());

  cancel_active_nav2_goal("DOCK starting");

  const auto coarse_outcome = run_nav2_navigation(
    goal_handle,
    staging_pose,
    staging_name,
    error_message,
    dock_coarse_progress_ratio_,
    0.0F);
  if (coarse_outcome != Nav2RunOutcome::SUCCEEDED) {
    switch (coarse_outcome) {
      case Nav2RunOutcome::CANCELED:
        finish_goal_canceled(goal_handle, "回桩粗导航已取消");
        return;
      case Nav2RunOutcome::TIMEOUT:
        finish_goal_aborted(
          goal_handle,
          interfaces::msg::ActionStatus::TIMEOUT,
          "回桩粗导航超时: " + error_message);
        return;
      case Nav2RunOutcome::ABORTED:
        finish_goal_aborted(
          goal_handle,
          interfaces::msg::ActionStatus::ABORTED,
          "回桩粗导航失败: " + error_message);
        return;
      default:
        break;
    }
  }

  if (goal_handle->is_canceling()) {
    finish_goal_canceled(goal_handle, "回桩在粗导航完成后被取消");
    return;
  }

  RCLCPP_INFO(
    get_logger(), "DOCK phase 2/2: fine docking via %s action", dock_action_name_.c_str());

  const bool docking_succeeded = run_fine_docking(goal_handle, error_message);

  if (goal_handle->is_canceling()) {
    stop_fine_docking();
    finish_goal_canceled(goal_handle, "回桩精对接已取消");
    return;
  }

  if (docking_succeeded) {
    publish_dock_progress_feedback(goal_handle, "DOCKING_COMPLETED", 1.0F);
    RCLCPP_INFO(get_logger(), "DOCK fine phase completed, finishing Navigate action");
    finish_goal_succeeded(goal_handle, "回桩完成");
    return;
  }

  stop_fine_docking();
  finish_goal_aborted(
    goal_handle,
    interfaces::msg::ActionStatus::ABORTED,
    "精对接失败: " + error_message);
}

void XjrobotBridgeNode::publish_dock_progress_feedback(
  const std::shared_ptr<NavigateGoalHandle> & goal_handle,
  const std::string & docking_state,
  const float progress) const
{
  auto feedback = std::make_shared<Navigate::Feedback>();
  feedback->progress = progress;
  feedback->docking_state = docking_state;
  goal_handle->publish_feedback(feedback);
}

bool XjrobotBridgeNode::run_fine_docking(
  const std::shared_ptr<NavigateGoalHandle> & navigate_goal_handle,
  std::string & error_message)
{
  const auto action_timeout = std::chrono::duration<double>(dock_action_wait_timeout_sec_);
  if (!dock_action_client_->wait_for_action_server(action_timeout)) {
    error_message = "等待 dock action server 超时，请确认 charge 栈已启动";
    return false;
  }

  DockingAction::Goal goal;
  goal.start = true;

  auto send_goal_options = rclcpp_action::Client<DockingAction>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [this, navigate_goal_handle](
    DockGoalHandle::SharedPtr,
    const std::shared_ptr<const DockingAction::Feedback> feedback)
    {
      const float fine_progress = feedback->progress;
      const float overall_progress = dock_coarse_progress_ratio_ +
        fine_progress * (1.0F - dock_coarse_progress_ratio_);
      publish_dock_progress_feedback(
        navigate_goal_handle,
        feedback->docking_state,
        std::clamp(overall_progress, 0.0F, 1.0F));
    };

  RCLCPP_INFO(
    get_logger(),
    "Sending dock action goal (timeout=%.1fs)",
    dock_action_wait_timeout_sec_);
  auto goal_future = dock_action_client_->async_send_goal(goal, send_goal_options);
  if (goal_future.wait_for(action_timeout) != std::future_status::ready) {
    error_message = "发送 dock action goal 超时";
    return false;
  }

  const auto dock_goal_handle = goal_future.get();
  if (!dock_goal_handle) {
    error_message = "dock action 拒绝了 goal";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(active_dock_mutex_);
    active_docking_ = ActiveDocking{navigate_goal_handle, dock_goal_handle};
  }

  auto result_future = dock_action_client_->async_get_result(dock_goal_handle);
  while (rclcpp::ok()) {
    if (navigate_goal_handle->is_canceling()) {
      error_message = "上层取消了回桩请求";
      {
        std::lock_guard<std::mutex> lock(active_dock_mutex_);
        active_docking_.reset();
      }
      return false;
    }
    if (result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
      break;
    }
  }

  {
    std::lock_guard<std::mutex> lock(active_dock_mutex_);
    active_docking_.reset();
  }

  if (!rclcpp::ok()) {
    error_message = "ROS 上下文已关闭";
    return false;
  }

  const auto wrapped_result = result_future.get();
  if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped_result.result &&
    wrapped_result.result->ok)
  {
    return true;
  }

  if (wrapped_result.result && !wrapped_result.result->message.empty()) {
    error_message = wrapped_result.result->message;
  } else if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
    error_message = "dock action 已取消";
  } else {
    error_message = "dock action 失败";
  }
  return false;
}

void XjrobotBridgeNode::stop_fine_docking()
{
  std::shared_ptr<DockGoalHandle> dock_goal_handle;
  {
    std::lock_guard<std::mutex> lock(active_dock_mutex_);
    if (active_docking_.has_value() && active_docking_->dock_goal_handle) {
      dock_goal_handle = active_docking_->dock_goal_handle;
    }
  }
  if (dock_goal_handle) {
    dock_action_client_->async_cancel_goal(dock_goal_handle);
  }
}

bool XjrobotBridgeNode::resolve_waypoint_pose(
  const int target_index,
  geometry_msgs::msg::PoseStamped & pose,
  std::string & waypoint_name,
  std::string & error_message) const
{
  const auto waypoint_it = waypoint_by_index_.find(target_index);
  if (waypoint_it == waypoint_by_index_.end()) {
    std::ostringstream stream;
    stream << "未找到 target_index=" << target_index << " 对应的点位，请检查 waypoints.yaml";
    error_message = stream.str();
    return false;
  }

  const auto & waypoint = waypoint_it->second;
  waypoint_name = waypoint.name;

  pose.header.stamp = this->now();
  pose.header.frame_id = waypoint.frame_id.empty() ? default_frame_id_ : waypoint.frame_id;
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, waypoint.yaw);
  pose.pose.orientation.x = quaternion.x();
  pose.pose.orientation.y = quaternion.y();
  pose.pose.orientation.z = quaternion.z();
  pose.pose.orientation.w = quaternion.w();
  return true;
}

bool XjrobotBridgeNode::resolve_goal_pose(
  const Navigate::Goal & goal,
  geometry_msgs::msg::PoseStamped & pose,
  std::string & waypoint_name,
  std::string & error_message) const
{
  return resolve_waypoint_pose(goal.target_index, pose, waypoint_name, error_message);
}

interfaces::msg::ActionStatus XjrobotBridgeNode::build_status(uint8_t code) const
{
  interfaces::msg::ActionStatus status;
  status.status = code;
  return status;
}

void XjrobotBridgeNode::finish_goal_succeeded(
  const std::shared_ptr<NavigateGoalHandle> & goal_handle,
  const std::string & message) const
{
  if (!goal_handle->is_active()) {
    RCLCPP_WARN(
      get_logger(),
      "Navigate goal is no longer active, skip succeed(): %s",
      message.c_str());
    return;
  }
  auto result = std::make_shared<Navigate::Result>();
  result->status = build_status(interfaces::msg::ActionStatus::OK);
  result->message = message;
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "Navigate action succeeded: %s", message.c_str());
}

void XjrobotBridgeNode::finish_goal_aborted(
  const std::shared_ptr<NavigateGoalHandle> & goal_handle,
  uint8_t status_code,
  const std::string & message) const
{
  // abort 分支允许携带更细粒度的业务状态，例如 TIMEOUT / ABORTED。
  auto result = std::make_shared<Navigate::Result>();
  result->status = build_status(status_code);
  result->message = message;
  goal_handle->abort(result);
}

void XjrobotBridgeNode::finish_goal_canceled(
  const std::shared_ptr<NavigateGoalHandle> & goal_handle,
  const std::string & message) const
{
  // 自定义接口里把“被取消”映射成 PREEMPTED，更贴近上层 BT 的理解。
  auto result = std::make_shared<Navigate::Result>();
  result->status = build_status(interfaces::msg::ActionStatus::PREEMPTED);
  result->message = message;
  goal_handle->canceled(result);
}

bool XjrobotBridgeNode::cancel_active_nav2_goal(const std::string & reason, bool wait_for_result)
{
  std::shared_ptr<Nav2GoalHandle> nav2_goal_handle;
  {
    std::lock_guard<std::mutex> lock(active_nav_mutex_);
    // 没有活动导航时，STOP 或 cancel 请求可以直接返回失败。
    if (!active_navigation_.has_value() || !active_navigation_->nav2_goal_handle) {
      return false;
    }
    nav2_goal_handle = active_navigation_->nav2_goal_handle;
  }

  RCLCPP_WARN(get_logger(), "Cancel active Nav2 goal: %s", reason.c_str());
  auto cancel_future = nav2_client_->async_cancel_goal(nav2_goal_handle);
  if (!wait_for_result) {
    // 某些调用路径只负责“发起取消”，不强求同步等待结果。
    return true;
  }

  if (cancel_future.wait_for(std::chrono::duration<double>(nav2_request_timeout_sec_)) !=
    std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "等待 Nav2 cancel 响应超时。");
    return false;
  }

  const auto cancel_response = cancel_future.get();
  return !cancel_response->goals_canceling.empty();
}

void XjrobotBridgeNode::clear_active_navigation_locked(
  const std::shared_ptr<NavigateGoalHandle> & custom_goal_handle)
{
  std::lock_guard<std::mutex> lock(active_nav_mutex_);
  // 只有当 active_navigation_ 仍然对应当前 goal 时才清理。
  // 这能避免旧 goal 的收尾逻辑把新 goal 的上下文清空。
  if (active_navigation_.has_value() &&
    active_navigation_->custom_goal_handle == custom_goal_handle)
  {
    active_navigation_.reset();
  }
}

void XjrobotBridgeNode::publish_progress_feedback(
  const std::shared_ptr<NavigateGoalHandle> & goal_handle,
  const double initial_distance,
  const double remaining_distance,
  const float progress_scale,
  const float progress_offset) const
{
  auto feedback = std::make_shared<Navigate::Feedback>();
  float normalized = 0.0F;
  if (initial_distance > 1e-6) {
    normalized = static_cast<float>(
      std::clamp(1.0 - (remaining_distance / initial_distance), 0.0, 1.0));
  }
  feedback->progress = progress_offset + normalized * progress_scale;
  feedback->docking_state = "";
  goal_handle->publish_feedback(feedback);
}

}  // namespace xjrobot_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xjrobot_bridge::XjrobotBridgeNode>();
  // MultiThreadedExecutor 可以更从容地处理：
  // 1. action server 回调
  // 2. Nav2 action client 反馈/结果
  // 3. 独立执行线程中的状态流转
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
