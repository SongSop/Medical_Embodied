#ifndef XJROBOT_BRIDGE__XJROBOT_BRIDGE_NODE_HPP_
#define XJROBOT_BRIDGE__XJROBOT_BRIDGE_NODE_HPP_

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/action/docking.hpp"
#include "interfaces/action/navigate.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace xjrobot_bridge
{

class XjrobotBridgeNode : public rclcpp::Node
{
public:
  using Navigate = interfaces::action::Navigate;
  using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<Navigate>;
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using Nav2GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;
  using DockingAction = interfaces::action::Docking;
  using DockGoalHandle = rclcpp_action::ClientGoalHandle<DockingAction>;

  explicit XjrobotBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Waypoint
  {
    int index{0};
    std::string name;
    std::string frame_id;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct ActiveNavigation
  {
    std::shared_ptr<NavigateGoalHandle> custom_goal_handle;
    std::shared_ptr<Nav2GoalHandle> nav2_goal_handle;
    std::string waypoint_name;
    double initial_distance{-1.0};
  };

  enum class Nav2RunOutcome
  {
    SUCCEEDED,
    CANCELED,
    ABORTED,
    TIMEOUT,
  };

  struct ActiveDocking
  {
    std::shared_ptr<NavigateGoalHandle> custom_goal_handle;
    std::shared_ptr<DockGoalHandle> dock_goal_handle;
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigate::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<NavigateGoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<NavigateGoalHandle> goal_handle);

  void execute(const std::shared_ptr<NavigateGoalHandle> goal_handle);

  void execute_goal_navigation(
    const std::shared_ptr<NavigateGoalHandle> goal_handle,
    const Navigate::Goal & goal);

  void execute_stop_navigation(const std::shared_ptr<NavigateGoalHandle> goal_handle);

  void execute_dock_navigation(const std::shared_ptr<NavigateGoalHandle> goal_handle);

  Nav2RunOutcome run_nav2_navigation(
    const std::shared_ptr<NavigateGoalHandle> & goal_handle,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & waypoint_name,
    std::string & error_message,
    float progress_scale = 1.0F,
    float progress_offset = 0.0F);

  bool run_fine_docking(
    const std::shared_ptr<NavigateGoalHandle> & goal_handle,
    std::string & error_message);

  void stop_fine_docking();

  void publish_dock_progress_feedback(
    const std::shared_ptr<NavigateGoalHandle> & goal_handle,
    const std::string & docking_state,
    float progress) const;

  bool resolve_waypoint_pose(
    int target_index,
    geometry_msgs::msg::PoseStamped & pose,
    std::string & waypoint_name,
    std::string & error_message) const;

  bool resolve_goal_pose(
    const Navigate::Goal & goal,
    geometry_msgs::msg::PoseStamped & pose,
    std::string & waypoint_name,
    std::string & error_message) const;

  void load_waypoints();

  interfaces::msg::ActionStatus build_status(uint8_t code) const;

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

  bool cancel_active_nav2_goal(const std::string & reason, bool wait_for_result = true);

  void clear_active_navigation_locked(
    const std::shared_ptr<NavigateGoalHandle> & custom_goal_handle);

  void publish_progress_feedback(
    const std::shared_ptr<NavigateGoalHandle> & goal_handle,
    double initial_distance,
    double remaining_distance,
    float progress_scale = 1.0F,
    float progress_offset = 0.0F) const;

  rclcpp_action::Server<Navigate>::SharedPtr navigate_server_;
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav2_client_;
  rclcpp_action::Client<DockingAction>::SharedPtr dock_action_client_;

  std::unordered_map<int, Waypoint> waypoint_by_index_;
  std::unordered_map<std::string, Waypoint> waypoint_by_name_;

  mutable std::mutex active_nav_mutex_;
  std::optional<ActiveNavigation> active_navigation_;

  mutable std::mutex active_dock_mutex_;
  std::optional<ActiveDocking> active_docking_;

  std::string action_name_;
  std::string nav2_action_name_;
  std::string default_frame_id_;
  double nav2_server_wait_timeout_sec_{5.0};
  double nav2_request_timeout_sec_{5.0};

  int dock_staging_index_{-1};
  std::string dock_action_name_{"dock"};
  double dock_action_wait_timeout_sec_{130.0};
  float dock_coarse_progress_ratio_{0.5F};
};

}  // namespace xjrobot_bridge

#endif  // XJROBOT_BRIDGE__XJROBOT_BRIDGE_NODE_HPP_
