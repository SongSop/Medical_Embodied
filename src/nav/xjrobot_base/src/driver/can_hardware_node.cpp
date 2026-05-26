#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "xjrobot_base/controlcan.h"
#include "xjrobot_base/msg/can_frame.hpp"
#include "xjrobot_base/msg/real_cmd.hpp"
#include "xjrobot_base/printf_utils.hpp"
#include "xjrobot_base/wheel_motor_driver.hpp"

namespace
{

// 反馈帧中提取到的底盘左右轮故障信息。
struct WheelFaultFrame
{
  uint16_t left_fault{0U};
  uint16_t right_fault{0U};
  bool valid{false};
};

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

// 按小端格式解析 16 位无符号数。
uint16_t parse_uint16_le(uint8_t low, uint8_t high)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8U) | low);
}

// 将单侧车轮故障码转换为可读字符串，便于日志与状态发布。
std::string wheel_fault_to_string(uint16_t fault_code, const std::string & wheel_name)
{
  if (fault_code == 0x0000U) {
    return wheel_name + ":none";
  }

  const std::vector<std::pair<uint16_t, std::string>> wheel_fault_bits = {
    {0x0004, "overcurrent"},  // 电机过流
    {0x0008, "overload"},     // 电机过载
    {0x0020, "deviation"},    // 电机编码器超差
    {0x0080, "reference"},    // 电机参考电压异常
    {0x0200, "hall"},         // 电机霍尔故障
    {0x0400, "overheat"},     // 电机超温
    {0x0800, "encoder"},      // 电机编码器错误
    {0x2000, "setpoint"}};    // 电机速度给定错误
const std::vector<std::pair<uint16_t, std::string>> shared_fault_bits = {
    {0x0001, "overvoltage"},   // 过压
    {0x0002, "undervoltage"},  // 欠压
    {0x0100, "eeprom"}};       // EEPROM读写错误

  std::vector<std::string> faults;
  for (const auto & entry : wheel_fault_bits) {
    if ((fault_code & entry.first) != 0U) {
      faults.push_back(entry.second);
    }
  }
  for (const auto & entry : shared_fault_bits) {
    if ((fault_code & entry.first) != 0U) {
      faults.push_back(entry.second);
    }
  }

  uint16_t known_mask = 0U;
  for (const auto & entry : wheel_fault_bits) {
    known_mask = static_cast<uint16_t>(known_mask | entry.first);
  }
  for (const auto & entry : shared_fault_bits) {
    known_mask = static_cast<uint16_t>(known_mask | entry.first);
  }

  const uint16_t unknown_bits = static_cast<uint16_t>(fault_code & ~known_mask);
  if (unknown_bits != 0U) {
    std::ostringstream oss;
    oss << "unknown(0x" << std::hex << std::uppercase << unknown_bits << ")";
    faults.push_back(oss.str());
  }

  std::ostringstream result;
  result << wheel_name << ":";
  for (size_t i = 0; i < faults.size(); ++i) {
    if (i != 0U) {
      result << "|";
    }
    result << faults[i];
  }
  return result.str();
}

// 组合左右轮故障描述为统一状态文本。
std::string compose_fault_status(uint16_t left_fault, uint16_t right_fault)
{
  return wheel_fault_to_string(left_fault, "left") + "; " +
         wheel_fault_to_string(right_fault, "right");
}

// 从 CAN 帧中提取底盘故障字段，不符合格式时返回 invalid。
WheelFaultFrame parse_wheel_fault_frame(const VCI_CAN_OBJ & frame)
{
  WheelFaultFrame result{};
  // 底盘反馈帧格式：
  // data[4:5] -> 左轮故障码, data[6:7] -> 右轮故障码
  if (((frame.ID & 0x180U) != 0x180U) || frame.DataLen < 8U) {
    return result;
  }
  result.left_fault = parse_uint16_le(frame.Data[4], frame.Data[5]);
  result.right_fault = parse_uint16_le(frame.Data[6], frame.Data[7]);
  result.valid = true;
  return result;
}

}  // namespace

class CanHardwareNode : public rclcpp::Node
{
public:
  // 构造函数：完成参数加载、订阅/发布器创建、CAN 初始化和接收定时器注册。
  CanHardwareNode()
  : Node("can_hardware_node")
  {
    real_cmd_topic_ = declare_parameter<std::string>("real_cmd_topic", "/realcmd_ctrl");
    can_feedback_topic_ = declare_parameter<std::string>("can_feedback_topic", "/can_msg_fb");
    const double receive_loop_hz = declare_parameter<double>("receive_loop_hz", 500.0);
    max_wheel_rpm_ = declare_parameter<int>("max_wheel_rpm", 150);
    can1_baud_t0_ = declare_parameter<int>("can1_baud_t0", 0x00);
    can1_baud_t1_ = declare_parameter<int>("can1_baud_t1", 0x1C);
    chassis_fault_max_retries_ = std::max(
      0, static_cast<int>(declare_parameter<int>("chassis_fault_max_retries", 3)));
    chassis_fault_status_topic_ = declare_parameter<std::string>(
      "chassis_fault_status_topic", "/chassis_fault_status");

    real_ctrl_sub_ = create_subscription<xjrobot_base::msg::RealCMD>(
      real_cmd_topic_, 10,
      std::bind(&CanHardwareNode::real_ctrl_cb, this, std::placeholders::_1));
    can_msg_fb_pub_ = create_publisher<xjrobot_base::msg::CanFrame>(can_feedback_topic_, 10);
    chassis_fault_pub_ =
      create_publisher<std_msgs::msg::String>(chassis_fault_status_topic_, 10);

    init_can1();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    motor_ctrl_.ZLAC8015D_Init_Velocity_Mode();

    receive_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(receive_loop_hz, 1.0)),
      std::bind(&CanHardwareNode::receive_func, this));
  }

  ~CanHardwareNode() override
  {
    // 节点退出时确保 CAN 设备被复位并关闭。
    dev_close();
  }

private:
  // 底盘控制指令入口：处理停机、重使能、去使能及轮速下发。
  void real_ctrl_cb(const xjrobot_base::msg::RealCMD::SharedPtr real_cmd)
  {
    // 进入 fatal 锁存后，屏蔽常规速度/重使能指令，避免反复拉起故障底盘。
    if (chassis_fault_latched_ && !real_cmd->stop_flag) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Chassis is latched in fatal fault state, ignoring motion command");
      return;
    }

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
      // 分层设计：
      // 1) 先保持原始反馈发布，保证现有节点接口不变；
      // 2) 再在本节点内直接做故障处理，避免额外订阅链路引入延迟。
      handle_wheel_fault(frame);
    }
  }

  // 故障状态机核心：检测故障、重试恢复、超限锁存并触发去使能。
  void handle_wheel_fault(const VCI_CAN_OBJ & frame)
  {
    const WheelFaultFrame fault_frame = parse_wheel_fault_frame(frame);
    if (!fault_frame.valid) {
      return;
    }

    const bool has_fault = (fault_frame.left_fault != 0U || fault_frame.right_fault != 0U);
    if (!has_fault) {
      // 故障消失后立即复位状态机，允许后续新一轮自动恢复。
      if (fault_active_) {
        publish_fault_status(
          "recovered", fault_retry_count_, fault_frame.left_fault, fault_frame.right_fault,
          "chassis fault cleared");
        RCLCPP_INFO(get_logger(), "[ChassisFault] recovered, reset retry counter");
      }
      fault_active_ = false;
      fault_retry_count_ = 0;
      chassis_fault_latched_ = false;
      disable_sent_ = false;
      return;
    }

    fault_active_ = true;
    const std::string detail =
      compose_fault_status(fault_frame.left_fault, fault_frame.right_fault);
    if (chassis_fault_latched_) {
      publish_fault_status(
        "latched", fault_retry_count_, fault_frame.left_fault, fault_frame.right_fault, detail);
      return;
    }

    if (fault_retry_count_ < chassis_fault_max_retries_) {
      // 故障存在且未超重试上限：先尝试自动清错并重新使能。
      ++fault_retry_count_;
      publish_fault_status(
        "retrying", fault_retry_count_, fault_frame.left_fault, fault_frame.right_fault, detail);
      RCLCPP_WARN(
        get_logger(),
        "[ChassisFault] detected (%s), retry %d/%d",
        detail.c_str(), fault_retry_count_, chassis_fault_max_retries_);
      try_recover_chassis();
      return;
    }

    // 超过最大重试次数：去使能并发布异常锁存状态，阻止继续运动。
    latch_and_disable_chassis(detail, fault_frame.left_fault, fault_frame.right_fault);
  }

  // 故障恢复动作：执行清错并重使能。
  void try_recover_chassis()
  {
    motor_ctrl_.Clear_Error_Code(0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const bool re_enabled = motor_ctrl_.Re_Enabled(0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (re_enabled) {
      RCLCPP_WARN(get_logger(), "[ChassisFault] clear+re-enable command sent");
      return;
    }
    RCLCPP_ERROR(get_logger(), "[ChassisFault] failed to re-enable after clear");
  }

  // 超过最大重试后执行去使能，并将节点状态锁存为 fatal。
  void latch_and_disable_chassis(
    const std::string & detail, uint16_t left_fault, uint16_t right_fault)
  {
    if (!disable_sent_) {
      const bool disabled = motor_ctrl_.Driver_Disabled(0x01);
      disable_sent_ = true;
      chassis_fault_latched_ = true;
      publish_fault_status(
        "fatal", fault_retry_count_, left_fault, right_fault,
        detail + (disabled ? "; driver_disabled" : "; driver_disable_failed"));
      RCLCPP_ERROR(
        get_logger(),
        "[ChassisFault] retry exceeded(%d), chassis disabled. detail=%s",
        chassis_fault_max_retries_, detail.c_str());
      return;
    }

    chassis_fault_latched_ = true;
    publish_fault_status("fatal", fault_retry_count_, left_fault, right_fault, detail);
  }

  // 发布统一格式的底盘故障状态，供上层监控和联动逻辑使用。
  void publish_fault_status(
    const std::string & stage, int retry_count, uint16_t left_fault, uint16_t right_fault,
    const std::string & detail)
  {
    // 统一文本协议，便于上层监控/日志系统直接解析关键字段。
    std_msgs::msg::String msg;
    std::ostringstream oss;
    oss << "stage=" << stage
        << ";retry=" << retry_count
        << ";max_retry=" << chassis_fault_max_retries_
        << ";left=0x" << std::hex << std::uppercase << left_fault
        << ";right=0x" << std::hex << std::uppercase << right_fault
        << std::dec
        << ";detail=" << detail;
    msg.data = oss.str();
    chassis_fault_pub_->publish(msg);
  }

  // 初始化 USB-CAN 通道参数并启动 CAN1。
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

  // 关闭节点前重置并关闭 CAN 设备。
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chassis_fault_pub_;
  rclcpp::TimerBase::SharedPtr receive_timer_;

  std::string real_cmd_topic_;
  std::string can_feedback_topic_;
  std::string chassis_fault_status_topic_;
  int max_wheel_rpm_{150};
  int can1_baud_t0_{0x00};
  int can1_baud_t1_{0x1C};
  int chassis_fault_max_retries_{3};
  bool last_re_enabled_{false};
  bool last_disabled_{false};
  int fault_retry_count_{0};
  bool fault_active_{false};
  bool chassis_fault_latched_{false};
  bool disable_sent_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanHardwareNode>());
  rclcpp::shutdown();
  return 0;
}
