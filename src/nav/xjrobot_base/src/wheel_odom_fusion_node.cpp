#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <array>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "xjrobot_base/msg/can_frame.hpp"

class WheelOdomFusionNode : public rclcpp::Node
{
public:
  WheelOdomFusionNode()
  : Node("wheel_odom_fusion_node")
  {
    can_topic_ = declare_parameter<std::string>("can_feedback_topic", "/can_msg_fb");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/wheel_odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", true);
    wheel_diameter_inch_ = declare_parameter<double>("wheel_diameter_inch", 8.0);
    wheel_distance_ = declare_parameter<double>("wheel_distance", 0.562);
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    speed_compensation_ratio_ =
      declare_parameter<double>("speed_compensation_ratio", 100.0 / 105.0);
    reverse_heading_ = declare_parameter<bool>("reverse_heading", false);
    reverse_drive_direction_ =
      declare_parameter<bool>("reverse_drive_direction", false);
    wheel_pose_covariance_diagonal_ = loadCovarianceDiagonal(
      "wheel_odom_pose_covariance_diagonal",
      {10.0, 10.0, 1e6, 1e6, 1e6, 2.0});
    wheel_twist_covariance_diagonal_ = loadCovarianceDiagonal(
      "wheel_odom_twist_covariance_diagonal",
      {0.35, 0.50, 1e6, 1e6, 1e6, 0.45});

    can_sub_ = create_subscription<xjrobot_base::msg::CanFrame>(
      can_topic_, 50, std::bind(&WheelOdomFusionNode::canFbCb, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 20);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  static int16_t parseInt16LE(uint8_t low, uint8_t high)
  {
    const uint16_t raw = (static_cast<uint16_t>(high) << 8) | low;
    return static_cast<int16_t>(raw);
  }

  static uint16_t parseUInt16LE(uint8_t low, uint8_t high)
  {
    return (static_cast<uint16_t>(high) << 8) | low;
  }

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

  static double radiansToDegrees360(double angle_rad)
  {
    double angle_deg = angle_rad * 180.0 / M_PI;
    while (angle_deg < 0.0) {
      angle_deg += 360.0;
    }
    while (angle_deg >= 360.0) {
      angle_deg -= 360.0;
    }
    return angle_deg;
  }

  double convertRpmToMs(double inch, double rpm) const
  {
    const double circumference_inch = inch * M_PI;
    const double circumference_meter = circumference_inch * 0.0254;
    const double meters_per_minute = circumference_meter * rpm;
    return meters_per_minute / 60.0 * speed_compensation_ratio_;
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

  void canFbCb(const xjrobot_base::msg::CanFrame::SharedPtr msg)
  {
    if ((msg->id & 0x180U) != 0x180U || msg->dlc < 4U) {
      return;
    }
    const int16_t left_raw_rpm = parseInt16LE(msg->data[0], msg->data[1]);
    const int16_t right_raw_rpm = parseInt16LE(msg->data[2], msg->data[3]);
    left_rpm_fd_ = static_cast<double>(left_raw_rpm) / 10.0;
    right_rpm_fd_ = static_cast<double>(right_raw_rpm) / 10.0;

    updateBotOdom();
  }

  void updateBotOdom()
  {
    const double left_speed = convertRpmToMs(wheel_diameter_inch_, left_rpm_fd_);
    const double right_speed = -convertRpmToMs(wheel_diameter_inch_, right_rpm_fd_);

    const rclcpp::Time current_time = now();
    if (!has_last_time_) {
      last_time_ = current_time;
      has_last_time_ = true;
      return;
    }

    const double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0 || dt < (0.5 / control_rate_)) {
      return;
    }

    double linear_speed = (right_speed + left_speed) / 2.0;
    if (reverse_drive_direction_) {
      linear_speed = -linear_speed;
    }
    const double angular_speed_wheel = (right_speed - left_speed) / wheel_distance_;
    const double delta_theta_wheel = angular_speed_wheel * dt;
    const double delta_dis = linear_speed * dt;

    wheel_yaw_ = normalizeAngle(wheel_yaw_ + delta_theta_wheel);
    double selected_yaw = wheel_yaw_;
    if (reverse_heading_) {
      selected_yaw = normalizeAngle(selected_yaw + M_PI);
    }

    accumulation_x_ += std::cos(selected_yaw) * delta_dis;
    accumulation_y_ += std::sin(selected_yaw) * delta_dis;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, selected_yaw);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = accumulation_x_;
    odom_msg.pose.pose.position.y = accumulation_y_;
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    odom_msg.twist.twist.linear.x = linear_speed;
    odom_msg.twist.twist.angular.z = angular_speed_wheel;
    fillCovariance(wheel_pose_covariance_diagonal_, odom_msg.pose.covariance);
    fillCovariance(wheel_twist_covariance_diagonal_, odom_msg.twist.covariance);
    odom_pub_->publish(odom_msg);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "wheel odom x=%.3f y=%.3f yaw_deg=%.1f vx=%.3f wz=%.3f",
      accumulation_x_, accumulation_y_, radiansToDegrees360(selected_yaw), linear_speed,
      angular_speed_wheel);

    if (publish_odom_tf_) {
      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.header.stamp = current_time;
      odom_tf.header.frame_id = odom_frame_;
      odom_tf.child_frame_id = base_frame_;
      odom_tf.transform.translation.x = accumulation_x_;
      odom_tf.transform.translation.y = accumulation_y_;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(odom_tf);
    }

    last_time_ = current_time;
  }

  rclcpp::Subscription<xjrobot_base::msg::CanFrame>::SharedPtr can_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string can_topic_;
  std::string output_odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_odom_tf_{true};

  double wheel_diameter_inch_{8.0};
  double wheel_distance_{0.562};
  double control_rate_{50.0};
  double speed_compensation_ratio_{100.0 / 105.0};
  bool reverse_heading_{false};
  bool reverse_drive_direction_{false};
  std::array<double, 6> wheel_pose_covariance_diagonal_{};
  std::array<double, 6> wheel_twist_covariance_diagonal_{};

  double left_rpm_fd_{0.0};
  double right_rpm_fd_{0.0};
  double accumulation_x_{0.0};
  double accumulation_y_{0.0};
  double wheel_yaw_{0.0};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  bool has_last_time_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdomFusionNode>());
  rclcpp::shutdown();
  return 0;
}
