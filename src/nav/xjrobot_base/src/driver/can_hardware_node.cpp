#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "xjrobot_base/controlcan.h"
#include "xjrobot_base/msg/can_frame.hpp"
#include "xjrobot_base/msg/real_cmd.hpp"
#include "xjrobot_base/printf_utils.hpp"
#include "xjrobot_base/wheel_motor_driver.hpp"

namespace
{

uint16_t clamp_raw_rpm(uint16_t raw_value, int max_wheel_rpm)
{
  if (raw_value >= static_cast<uint16_t>(max_wheel_rpm) && raw_value <= 30000U) {
    return static_cast<uint16_t>(max_wheel_rpm);
  }
  if (
    raw_value <= static_cast<uint16_t>(65535 - max_wheel_rpm) &&
    raw_value >= 30000U)
  {
    return static_cast<uint16_t>(65535 - max_wheel_rpm);
  }
  return raw_value;
}

}  // namespace

class CanHardwareNode : public rclcpp::Node
{
public:
  CanHardwareNode()
  : Node("can_hardware_node")
  {
    real_cmd_topic_ = declare_parameter<std::string>("real_cmd_topic", "/realcmd_ctrl");
    can_feedback_topic_ = declare_parameter<std::string>("can_feedback_topic", "/can_msg_fb");
    const double receive_loop_hz = declare_parameter<double>("receive_loop_hz", 500.0);
    max_wheel_rpm_ = declare_parameter<int>("max_wheel_rpm", 150);
    can1_baud_t0_ = declare_parameter<int>("can1_baud_t0", 0x00);
    can1_baud_t1_ = declare_parameter<int>("can1_baud_t1", 0x1C);

    real_ctrl_sub_ = create_subscription<xjrobot_base::msg::RealCMD>(
      real_cmd_topic_, 10,
      std::bind(&CanHardwareNode::real_ctrl_cb, this, std::placeholders::_1));
    can_msg_fb_pub_ = create_publisher<xjrobot_base::msg::CanFrame>(can_feedback_topic_, 10);

    init_can1();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    motor_ctrl_.ZLAC8015D_Init_Velocity_Mode();

    receive_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(receive_loop_hz, 1.0)),
      std::bind(&CanHardwareNode::receive_func, this));
  }

  ~CanHardwareNode() override
  {
    dev_close();
  }

private:
  void real_ctrl_cb(const xjrobot_base::msg::RealCMD::SharedPtr real_cmd)
  {
    if (real_cmd->stop_flag) {
      motor_ctrl_.Clear_Error_Code(0x01);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      motor_ctrl_.Re_Enabled(0x01);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (motor_ctrl_.Quick_Stop(0x01)) {
        RCLCPP_WARN(get_logger(), "Wheel quick stop triggered");
      }
      return;
    }

    if (last_re_enabled_ != real_cmd->re_enabled) {
      last_re_enabled_ = real_cmd->re_enabled;
      if (real_cmd->re_enabled) {
        motor_ctrl_.Clear_Error_Code(0x01);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (motor_ctrl_.Re_Enabled(0x01)) {
          RCLCPP_INFO(get_logger(), "Wheel re-enabled");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return;
      }
    }

    if (last_disabled_ != real_cmd->disabled) {
      last_disabled_ = real_cmd->disabled;
      if (real_cmd->disabled) {
        if (motor_ctrl_.Driver_Disabled(0x01)) {
          RCLCPP_WARN(get_logger(), "Wheel disabled");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return;
      }
    }

    const uint16_t wh_vel_left = clamp_raw_rpm(real_cmd->wheel_left_v, max_wheel_rpm_);
    const uint16_t wh_vel_right = clamp_raw_rpm(real_cmd->wheel_right_v, max_wheel_rpm_);
    motor_ctrl_.Velocity_Joy_Control(0x01, wh_vel_left, wh_vel_right);
  }

  void receive_func()
  {
    int reclen = 0;
    VCI_CAN_OBJ rec[3000];
    if ((reclen = VCI_Receive(VCI_USBCAN2, 0, 0, rec, 3000, 100)) <= 0) {
      return;
    }
    for (int idx = 0; idx < reclen; ++idx) {
      const VCI_CAN_OBJ & frame = rec[idx];
      xjrobot_base::msg::CanFrame can_msg;
      can_msg.header.stamp = now();
      can_msg.id = frame.ID;
      can_msg.dlc = frame.DataLen;
      can_msg.is_extended = frame.ExternFlag != 0;
      can_msg.is_rtr = frame.RemoteFlag != 0;
      can_msg.is_error = false;
      for (uint8_t i = 0; i < frame.DataLen && i < can_msg.data.size(); ++i) {
        can_msg.data[i] = frame.Data[i];
      }
      can_msg_fb_pub_->publish(can_msg);
    }
  }

  void init_can1()
  {
    VCI_BOARD_INFO pinfo[50];
    (void)VCI_FindUsbDevice2(pinfo);
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != 1) {
      throw std::runtime_error("Failed to open CarCanalyst device");
    }

    VCI_INIT_CONFIG config{};
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;
    config.Timing0 = static_cast<unsigned char>(can1_baud_t0_);
    config.Timing1 = static_cast<unsigned char>(can1_baud_t1_);
    config.Mode = 0;

    if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1) {
      VCI_CloseDevice(VCI_USBCAN2, 0);
      throw std::runtime_error("Failed to initialize CAN1");
    }
    if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1) {
      VCI_CloseDevice(VCI_USBCAN2, 0);
      throw std::runtime_error("Failed to start CAN1");
    }
  }

  void dev_close()
  {
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  xjrobot_base::WheelMotorCanCtrl motor_ctrl_;
  rclcpp::Subscription<xjrobot_base::msg::RealCMD>::SharedPtr real_ctrl_sub_;
  rclcpp::Publisher<xjrobot_base::msg::CanFrame>::SharedPtr can_msg_fb_pub_;
  rclcpp::TimerBase::SharedPtr receive_timer_;

  std::string real_cmd_topic_;
  std::string can_feedback_topic_;
  int max_wheel_rpm_{150};
  int can1_baud_t0_{0x00};
  int can1_baud_t1_{0x1C};
  bool last_re_enabled_{false};
  bool last_disabled_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanHardwareNode>());
  rclcpp::shutdown();
  return 0;
}
