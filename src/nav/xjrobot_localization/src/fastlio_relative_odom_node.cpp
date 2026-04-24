#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class FastlioRelativeOdomNode : public rclcpp::Node
{
public:
  FastlioRelativeOdomNode()
  : Node("fastlio_relative_odom_node")
  {
    input_topic_ = declare_parameter<std::string>("fastlio_odom_topic", "/odom_fastlio");
    output_topic_ = declare_parameter<std::string>(
      "fastlio_relative_odom_topic", "/odom_fastlio_relative");
    output_frame_ = declare_parameter<std::string>("fastlio_relative_odom_frame", "odom");
    output_base_frame_ = declare_parameter<std::string>(
      "fastlio_relative_base_frame", "base_footprint");
    pose_covariance_diagonal_ = loadCovarianceDiagonal(
      "fastlio_pose_covariance_diagonal",
      {0.04, 0.04, 1e6, 1e6, 1e6, 0.06});
    twist_covariance_diagonal_ = loadCovarianceDiagonal(
      "fastlio_twist_covariance_diagonal",
      {0.09, 0.09, 1e6, 1e6, 1e6, 0.12});

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FastlioRelativeOdomNode::odomCb, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 20);

    RCLCPP_INFO(
      get_logger(),
      "FastLIO relative odom enabled: input=%s, output=%s, frame=%s, child=%s",
      input_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str(),
      output_base_frame_.c_str());
  }

private:
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  std::array<double, 6> loadCovarianceDiagonal(
    const std::string & param_name,
    const std::array<double, 6> & defaults)
  {
    const auto values = declare_parameter<std::vector<double>>(
      param_name, std::vector<double>(defaults.begin(), defaults.end()));
    if (values.size() != 6U) {
      RCLCPP_WARN(
        get_logger(), "%s expects 6 values, got %zu. Falling back to defaults.",
        param_name.c_str(), values.size());
      return defaults;
    }

    std::array<double, 6> result{};
    for (size_t i = 0; i < result.size(); ++i) {
      result[i] = values[i];
    }
    return result;
  }

  static void fillCovariance(
    const std::array<double, 6> & diagonal,
    std::array<double, 36> & covariance)
  {
    covariance.fill(0.0);
    for (size_t i = 0; i < diagonal.size(); ++i) {
      covariance[i * 6 + i] = diagonal[i];
    }
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double current_x = msg->pose.pose.position.x;
    const double current_y = msg->pose.pose.position.y;
    const double current_yaw = tf2::getYaw(msg->pose.pose.orientation);

    if (!has_prev_sample_) {
      prev_x_ = current_x;
      prev_y_ = current_y;
      prev_yaw_ = current_yaw;
      prev_stamp_ = msg->header.stamp;
      has_prev_sample_ = true;
      publishCurrentState(msg->header.stamp, 0.0, 0.0, 0.0);
      return;
    }

    const rclcpp::Time current_stamp(msg->header.stamp);
    const rclcpp::Time previous_stamp(prev_stamp_);
    const double dt = (current_stamp - previous_stamp).seconds();
    if (dt <= 0.0) {
      prev_x_ = current_x;
      prev_y_ = current_y;
      prev_yaw_ = current_yaw;
      prev_stamp_ = msg->header.stamp;
      return;
    }

    const double delta_world_x = current_x - prev_x_;
    const double delta_world_y = current_y - prev_y_;
    const double delta_yaw = normalizeAngle(current_yaw - prev_yaw_);

    // 将 FastLIO 连续两帧之间的位移转成“上一时刻机体系下”的相对位移，
    // 再在纯平面里重新积分，得到适合 EKF 融合的局部相对里程计。
    const double cos_yaw = std::cos(prev_yaw_);
    const double sin_yaw = std::sin(prev_yaw_);
    const double delta_body_x = cos_yaw * delta_world_x + sin_yaw * delta_world_y;
    const double delta_body_y = -sin_yaw * delta_world_x + cos_yaw * delta_world_y;

    accumulated_x_ += std::cos(accumulated_yaw_) * delta_body_x -
      std::sin(accumulated_yaw_) * delta_body_y;
    accumulated_y_ += std::sin(accumulated_yaw_) * delta_body_x +
      std::cos(accumulated_yaw_) * delta_body_y;
    accumulated_yaw_ = normalizeAngle(accumulated_yaw_ + delta_yaw);

    publishCurrentState(
      msg->header.stamp,
      delta_body_x / dt,
      delta_body_y / dt,
      delta_yaw / dt);

    prev_x_ = current_x;
    prev_y_ = current_y;
    prev_yaw_ = current_yaw;
    prev_stamp_ = msg->header.stamp;
  }

  void publishCurrentState(
    const builtin_interfaces::msg::Time & stamp,
    double linear_x,
    double linear_y,
    double angular_z)
  {
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = output_frame_;
    out.child_frame_id = output_base_frame_;
    out.pose.pose.position.x = accumulated_x_;
    out.pose.pose.position.y = accumulated_y_;
    out.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, accumulated_yaw_);
    out.pose.pose.orientation = tf2::toMsg(q);

    out.twist.twist.linear.x = linear_x;
    out.twist.twist.linear.y = linear_y;
    out.twist.twist.angular.z = angular_z;

    fillCovariance(pose_covariance_diagonal_, out.pose.covariance);
    fillCovariance(twist_covariance_diagonal_, out.twist.covariance);

    odom_pub_->publish(out);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  std::string output_base_frame_;
  std::array<double, 6> pose_covariance_diagonal_{};
  std::array<double, 6> twist_covariance_diagonal_{};

  bool has_prev_sample_{false};
  double prev_x_{0.0};
  double prev_y_{0.0};
  double prev_yaw_{0.0};
  builtin_interfaces::msg::Time prev_stamp_;

  double accumulated_x_{0.0};
  double accumulated_y_{0.0};
  double accumulated_yaw_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlioRelativeOdomNode>());
  rclcpp::shutdown();
  return 0;
}
