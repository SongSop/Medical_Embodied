#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "xjrobot_base/msg/real_cmd.hpp"

namespace
{

uint16_t encode_signed_rpm(int rpm)
{
  const int clamped = std::clamp(rpm, -32768, 32767);
  return static_cast<uint16_t>(static_cast<int16_t>(clamped));
}

double read_axis(const sensor_msgs::msg::Joy & joy, std::size_t index)
{
  return index < joy.axes.size() ? joy.axes[index] : 0.0;
}

int read_button(const sensor_msgs::msg::Joy & joy, std::size_t index)
{
  return index < joy.buttons.size() ? joy.buttons[index] : 0;
}

}  // namespace

class BaseControllerNode : public rclcpp::Node
{
public:
  BaseControllerNode()
  : Node("base_controller_node")
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    real_cmd_topic_ = declare_parameter<std::string>("real_cmd_topic", "/realcmd_ctrl");
    joy_topic_default_ = declare_parameter<std::string>("joy_topic_default", "/joy");
    joy_topic_backup_ =
      declare_parameter<std::string>("joy_topic_backup", "/joy_node_usb/joy");
    joy_liner_k_ = declare_parameter<int>("joy_liner_k", 40);
    joy_angular_k_ = declare_parameter<int>("joy_angular_k", 17);
    wheel_distance_ = declare_parameter<double>("wheel_distance", 0.477);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.2032);
    reverse_drive_direction_ =
      declare_parameter<bool>("reverse_drive_direction", false);
    const double loop_hz = declare_parameter<double>("command_loop_hz", 100.0);

    real_cmd_pub_ = create_publisher<xjrobot_base::msg::RealCMD>(real_cmd_topic_, 10);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&BaseControllerNode::nav_vel_cb, this, std::placeholders::_1));
    joy_default_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_default_, 10,
      std::bind(&BaseControllerNode::joy_default_cb, this, std::placeholders::_1));
    joy_backup_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_backup_, 10,
      std::bind(&BaseControllerNode::joy_backup_cb, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(loop_hz, 1.0)),
      std::bind(&BaseControllerNode::publish_command, this));
  }

private:
  enum class CarState : uint8_t
  {
    Init = 0,
    Origin,
    Manual,
    AutoNavigating,
    Pause,
    MotorDisable,
    MotorReEnable,
    MotorStop,
  };

  struct ButtonEdgeGuard
  {
    double group0{0.0};
    double group1{0.0};
    double group2{0.0};
    double group3{0.0};
  };

  void nav_vel_cb(const geometry_msgs::msg::Twist::SharedPtr vel_msg)
  {
    const double linear_vel =
      reverse_drive_direction_ ? -vel_msg->linear.x : vel_msg->linear.x;
    const double angular_vel = vel_msg->angular.z;
    nav_to_left_rpm_ = static_cast<int>(
      std::lround((linear_vel - angular_vel * wheel_distance_ / 2.0) / wheel_radius_ * 60.0 /
      M_PI));
    nav_to_right_rpm_ = static_cast<int>(
      std::lround(-(linear_vel + angular_vel * wheel_distance_ / 2.0) / wheel_radius_ * 60.0 /
      M_PI));
  }

  void joy_backup_cb(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    process_joy(*joy, false);
  }

  void joy_default_cb(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    process_joy(*joy, true);
  }

  void process_joy(const sensor_msgs::msg::Joy & joy, bool default_layout)
  {
    const double left_rocker_raw = read_axis(joy, 1);
    const double left_rocker =
      reverse_drive_direction_ ? -left_rocker_raw : left_rocker_raw;
    const double right_rocker = read_axis(joy, default_layout ? 3 : 3);
    const int a_button = read_button(joy, 0);
    const int b_button = read_button(joy, 1);
    const int x_button = read_button(joy, default_layout ? 2 : 2);
    const int y_button = read_button(joy, default_layout ? 3 : 3);
    const double left_bumper = read_axis(joy, 6);
    const int m_button1 = read_button(joy, 7);
    const int m_button3 = read_button(joy, 8);
    const int l_button = read_button(joy, default_layout ? 4 : 4);
    const int left_roc_mid = read_button(joy, default_layout ? 8 : 9);
    const int right_roc_mid = read_button(joy, default_layout ? 9 : 10);
    ButtonEdgeGuard & edge_guard = default_layout ? joy_edge_default_ : joy_edge_backup_;

    if (
      edge_guard.group0 != 0.0 || edge_guard.group1 != 0.0 ||
      edge_guard.group2 != 0.0 || edge_guard.group3 != 0.0)
    {
      edge_guard = ButtonEdgeGuard{};
      return;
    }

    edge_guard.group0 = a_button * 1000.0 + b_button * 100.0 + x_button * 10.0 + y_button;
    edge_guard.group1 = m_button1 * 10.0 + m_button3;
    edge_guard.group2 = l_button * 10.0;
    edge_guard.group3 = left_roc_mid * 10.0 + right_roc_mid;

    if (left_roc_mid) {
      car_state_ = CarState::MotorDisable;
      return;
    }
    if (right_roc_mid) {
      car_state_ = CarState::MotorReEnable;
      return;
    }
    if (l_button) {
      car_state_ = CarState::MotorStop;
      return;
    }
    if (car_state_ == CarState::MotorDisable || car_state_ == CarState::MotorStop) {
      return;
    }
    if (a_button) {
      car_state_ = CarState::AutoNavigating;
      return;
    }

    if (std::abs(left_rocker) > 1e-4 || std::abs(right_rocker) > 1e-4) {
      car_state_ = CarState::Manual;
      const double normal_left =
        left_rocker * joy_liner_k_ - right_rocker * joy_angular_k_;
      const double normal_right =
        -(left_rocker * joy_liner_k_ + right_rocker * joy_angular_k_);
      if (left_bumper != 0.0 && left_bumper != 1.0) {
        const double speed_k = (-left_bumper + 1.0) * 2.5 + 1.0;
        manual_left_rpm_ = static_cast<int>(std::lround(
          left_rocker * joy_liner_k_ * speed_k -
          right_rocker * joy_angular_k_ * speed_k / 1.5));
        manual_right_rpm_ = static_cast<int>(std::lround(
          -(left_rocker * joy_liner_k_ * speed_k +
          right_rocker * joy_angular_k_ * speed_k / 1.5)));
      } else {
        manual_left_rpm_ = static_cast<int>(std::lround(normal_left));
        manual_right_rpm_ = static_cast<int>(std::lround(normal_right));
      }
    } else {
      car_state_ = CarState::Pause;
    }
  }

  void choose_wheel_speed(int & left_rpm, int & right_rpm) const
  {
    left_rpm = 0;
    right_rpm = 0;
    if (
      car_state_ == CarState::AutoNavigating)
    {
      left_rpm = nav_to_left_rpm_;
      right_rpm = nav_to_right_rpm_;
    } else if (car_state_ == CarState::Manual) {
      left_rpm = manual_left_rpm_;
      right_rpm = manual_right_rpm_;
    }
  }

  void publish_command()
  {
    xjrobot_base::msg::RealCMD real_cmd;
    int left_rpm = 0;
    int right_rpm = 0;
    choose_wheel_speed(left_rpm, right_rpm);

    switch (car_state_) {
      case CarState::Origin:
        break;
      case CarState::Init:
      case CarState::Pause:
      case CarState::AutoNavigating:
      case CarState::Manual:
        real_cmd.wheel_left_v = encode_signed_rpm(left_rpm);
        real_cmd.wheel_right_v = encode_signed_rpm(right_rpm);
        break;
      case CarState::MotorDisable:
        real_cmd.disabled = true;
        break;
      case CarState::MotorReEnable:
        real_cmd.re_enabled = true;
        car_state_ = CarState::Origin;
        break;
      case CarState::MotorStop:
        real_cmd.stop_flag = true;
        break;
    }

    real_cmd.auto_mode = (car_state_ == CarState::AutoNavigating);
    real_cmd_pub_->publish(real_cmd);
  }

  rclcpp::Publisher<xjrobot_base::msg::RealCMD>::SharedPtr real_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_default_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_backup_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string cmd_vel_topic_;
  std::string real_cmd_topic_;
  std::string joy_topic_default_;
  std::string joy_topic_backup_;

  int joy_liner_k_{40};
  int joy_angular_k_{17};
  double wheel_distance_{0.477};
  double wheel_radius_{0.2032};
  bool reverse_drive_direction_{false};

  CarState car_state_{CarState::Init};
  int nav_to_left_rpm_{0};
  int nav_to_right_rpm_{0};
  int manual_left_rpm_{0};
  int manual_right_rpm_{0};
  ButtonEdgeGuard joy_edge_default_;
  ButtonEdgeGuard joy_edge_backup_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseControllerNode>());
  rclcpp::shutdown();
  return 0;
}
