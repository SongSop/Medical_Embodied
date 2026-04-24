#include "xjrobot_base/wheel_motor_driver.hpp"

#include <array>
#include <cstring>
#include <iostream>
#include <thread>

#include "xjrobot_base/printf_utils.hpp"

namespace
{

void sleep_ms(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

bool transmit_frame(uint32_t id, const uint8_t * data, uint8_t len)
{
  VCI_CAN_OBJ frame{};
  frame.ID = id;
  frame.ExternFlag = false;
  frame.RemoteFlag = false;
  frame.DataLen = len;
  std::memcpy(frame.Data, data, len);
  return VCI_Transmit(VCI_USBCAN2, 0, 0, &frame, 1) == 1;
}

bool transmit_and_log(
  uint32_t id, const uint8_t * data, uint8_t len,
  xjrobot_base::WheelMotorCanCtrl & driver)
{
  VCI_CAN_OBJ frame{};
  frame.ID = id;
  frame.ExternFlag = false;
  frame.RemoteFlag = false;
  frame.DataLen = len;
  std::memcpy(frame.Data, data, len);
  const bool ok = VCI_Transmit(VCI_USBCAN2, 0, 0, &frame, 1) == 1;
  if (ok) {
    driver.print_send_info(&frame);
  } else {
    std::cout << RED << "--->  CAN Transmit ERROR!  <---" << TAIL << std::endl << std::endl;
  }
  return ok;
}

}  // namespace

namespace xjrobot_base
{

void WheelMotorCanCtrl::print_send_info(const VCI_CAN_OBJ * can_obj)
{
  std::cout << GREEN << std::setfill('0') << std::setw(4) << std::dec << count_num_ << "  " <<
    TAIL << std::flush;
  ++count_num_;
  std::cout << GREEN << "[CAN1] ID:0x" << std::setfill('0') << std::setw(3) << std::hex <<
    can_obj[0].ID << TAIL << std::flush;
  std::cout << GREEN << (can_obj[0].ExternFlag == 0 ? " Standard" : " Extend  ") << TAIL <<
    std::flush;
  std::cout << GREEN << (can_obj[0].RemoteFlag == 0 ? " Data  " : " Remote") << TAIL <<
    std::flush;
  std::cout << GREEN << "DLC:0x" << std::setfill('0') << std::setw(2) << std::hex <<
    static_cast<int>(can_obj[0].DataLen) << TAIL << std::flush;
  std::cout << GREEN << " [DATA:0x" << TAIL << std::flush;
  for (int i = 0; i < can_obj[0].DataLen; ++i) {
    std::cout << GREEN << " " << std::setfill('0') << std::setw(2) <<
      static_cast<int>(can_obj[0].Data[i]) << TAIL << std::flush;
  }
  std::cout << GREEN << "]" << TAIL << std::endl;
}

void WheelMotorCanCtrl::ZLAC8015D_Init_Velocity_Mode()
{
  RPDO0_Config(0x01);
  sleep_ms(1);
  RPDO1_Config(0x01);
  sleep_ms(1);
  RPDO2_Config(0x01);
  sleep_ms(1);
  TPDO0_Config(0x01);
  sleep_ms(1);
  TPDO1_Config(0x01);
  sleep_ms(1);
  Profile_Velocity_Init(0x01);
  sleep_ms(1);
  NMT_Control(0x01, 0x01);
  sleep_ms(1);
  Clear_Error_Code(0x01);
  sleep_ms(1);
  Set_Overload_param(0x01);
  sleep_ms(1);
  Enable_Error_PWM(0x01);
  sleep_ms(1);
  Set_max_current(0x01);
  sleep_ms(1);
  Driver_Enable(0x01);
  sleep_ms(1);
  Set_Overload_Time(0x01);
  sleep_ms(1);
}

uint8_t WheelMotorCanCtrl::RPDO0_Config(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: RPDO0_Config " << TAIL <<
    std::endl;
  std::array<std::array<uint8_t, 8>, 3> frames = {{
    {{0x2F, 0x00, 0x14, 0x02, 0xFE, 0x00, 0x00, 0x00}},
    {{0x23, 0x00, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60}},
    {{0x2F, 0x00, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00}},
  }};
  for (const auto & frame : frames) {
    transmit_and_log(0x600 + id, frame.data(), 8, *this);
  }
  return 0x00;
}

uint8_t WheelMotorCanCtrl::RPDO1_Config(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: RPDO1_Config" << TAIL <<
    std::endl;
  std::array<std::array<uint8_t, 8>, 3> frames = {{
    {{0x2F, 0x01, 0x14, 0x02, 0xFE, 0x00, 0x00, 0x00}},
    {{0x23, 0x01, 0x16, 0x01, 0x20, 0x03, 0xFF, 0x60}},
    {{0x2F, 0x01, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00}},
  }};
  for (const auto & frame : frames) {
    transmit_and_log(0x600 + id, frame.data(), 8, *this);
  }
  return 0x00;
}

uint8_t WheelMotorCanCtrl::RPDO2_Config(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: RPDO2_Config" << TAIL <<
    std::endl;
  std::array<std::array<uint8_t, 8>, 3> frames = {{
    {{0x2F, 0x02, 0x14, 0x02, 0xFE, 0x00, 0x00, 0x00}},
    {{0x23, 0x02, 0x16, 0x01, 0x00, 0x00, 0x5A, 0x60}},
    {{0x2F, 0x02, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00}},
  }};
  for (const auto & frame : frames) {
    transmit_and_log(0x600 + id, frame.data(), 8, *this);
  }
  return 0x00;
}

uint8_t WheelMotorCanCtrl::TPDO0_Config(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: TPDO0_Config" << TAIL <<
    std::endl;
  std::array<std::array<uint8_t, 8>, 5> frames = {{
    {{0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0x2F, 0x00, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00}},
    {{0x2B, 0x00, 0x18, 0x05, 0x28, 0x00, 0x00, 0x00}},
    {{0x23, 0x00, 0x1A, 0x01, 0x20, 0x03, 0x6C, 0x60}},
    {{0x2F, 0x00, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00}},
  }};
  for (const auto & frame : frames) {
    transmit_and_log(0x600 + id, frame.data(), 8, *this);
  }
  return 0x00;
}

uint8_t WheelMotorCanCtrl::TPDO1_Config(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: TPDO1_Config" << TAIL <<
    std::endl;
  std::array<std::array<uint8_t, 8>, 6> frames = {{
    {{0x2F, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0x23, 0x01, 0x1A, 0x01, 0x20, 0x03, 0x77, 0x60}},
    {{0x23, 0x01, 0x1A, 0x02, 0x10, 0x01, 0x32, 0x20}},
    {{0x23, 0x01, 0x1A, 0x03, 0x10, 0x02, 0x32, 0x20}},
    {{0x2F, 0x01, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00}},
    {{0x2B, 0x01, 0x18, 0x05, 0x28, 0x00, 0x00, 0x00}},
  }};
  for (const auto & frame : frames) {
    transmit_and_log(0x600 + id, frame.data(), 8, *this);
  }
  const std::array<uint8_t, 8> enable_map{{0x2F, 0x01, 0x1A, 0x00, 0x04, 0x00, 0x00, 0x00}};
  transmit_and_log(0x600 + id, enable_map.data(), 8, *this);
  return 0x00;
}

uint8_t WheelMotorCanCtrl::Profile_Velocity_Init(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: Profile_Velocity_Init " << TAIL <<
    std::endl;
  std::array<std::array<uint8_t, 8>, 5> frames = {{
    {{0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00}},
    {{0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00}},
    {{0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00}},
    {{0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00}},
    {{0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00}},
  }};
  for (const auto & frame : frames) {
    transmit_and_log(0x600 + id, frame.data(), 8, *this);
  }
  return 0x00;
}

uint8_t WheelMotorCanCtrl::NMT_Control(uint8_t data0, uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: NMT_Control" << TAIL <<
    std::endl;
  uint8_t data[2] = {data0, id};
  transmit_and_log(0x000, data, 2, *this);
  return 0x00;
}

uint8_t WheelMotorCanCtrl::Driver_Enable(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: Driver_Enable " << TAIL <<
    std::endl;
  const uint8_t data_1[2] = {0x06, 0x00};
  const uint8_t data_2[2] = {0x07, 0x00};
  const uint8_t data_3[2] = {0x0F, 0x00};
  transmit_and_log(0x200 + id, data_1, 2, *this);
  transmit_and_log(0x200 + id, data_2, 2, *this);
  transmit_and_log(0x200 + id, data_3, 2, *this);
  return 0x00;
}

void WheelMotorCanCtrl::Set_max_current(uint8_t id)
{
  std::cout << std::endl << RED1 << "--->  [CAN Msg] Transmit: Set motor max current !!! " <<
    TAIL << std::endl;
  uint8_t data[8] = {0x2B, 0x15, 0x20, 0x01, 0x2C, 0x01, 0x00, 0x00};
  transmit_and_log(0x600 + id, data, 8, *this);
  data[3] = 0x02;
  transmit_and_log(0x600 + id, data, 8, *this);
}

bool WheelMotorCanCtrl::Driver_Disabled(uint8_t id)
{
  std::cout << std::endl << RED1 << "--->  [CAN Msg] Transmit: Driver_Disabled !!! " << TAIL <<
    std::endl;
  const uint8_t data[2] = {0x06, 0x00};
  return transmit_frame(0x200 + id, data, 2);
}

bool WheelMotorCanCtrl::Quick_Stop(uint8_t id)
{
  std::cout << std::endl << RED1 << "--->  [CAN Msg] Transmit: Quick_Stop !!! " << TAIL <<
    std::endl;
  const uint8_t data[2] = {0x02, 0x00};
  return transmit_frame(0x200 + id, data, 2);
}

bool WheelMotorCanCtrl::Re_Enabled(uint8_t id)
{
  std::cout << std::endl << RED1 << "--->  [CAN Msg] Transmit: Re_Enabled !!! " << TAIL <<
    std::endl;
  const uint8_t data_1[2] = {0x06, 0x00};
  const uint8_t data_2[2] = {0x07, 0x00};
  const uint8_t data_3[2] = {0x0F, 0x00};
  transmit_and_log(0x200 + id, data_1, 2, *this);
  transmit_and_log(0x200 + id, data_2, 2, *this);
  return transmit_frame(0x200 + id, data_3, 2);
}

bool WheelMotorCanCtrl::Velocity_Joy_Control(uint8_t id, uint16_t left_v, uint16_t right_v)
{
  const uint8_t data[4] = {
    static_cast<uint8_t>(left_v & 0xFF),
    static_cast<uint8_t>(left_v >> 8),
    static_cast<uint8_t>(right_v & 0xFF),
    static_cast<uint8_t>(right_v >> 8)
  };
  return transmit_frame(0x300 + id, data, 4);
}

void WheelMotorCanCtrl::Clear_Error_Code(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: Clear_Error_Code " << TAIL <<
    std::endl;
  const uint8_t data[8] = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
  transmit_and_log(0x600 + id, data, 8, *this);
}

void WheelMotorCanCtrl::Enable_Error_PWM(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: Enable_Error_PWM " << TAIL <<
    std::endl;
  const uint8_t data[8] = {0x2B, 0x26, 0x20, 0x01, 0x01, 0x00, 0x00, 0x00};
  transmit_and_log(0x600 + id, data, 8, *this);
}

void WheelMotorCanCtrl::Set_Overload_param(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: Set_Overload_param " << TAIL <<
    std::endl;
  sleep_ms(10);
  uint8_t data[8] = {0x2B, 0x12, 0x20, 0x01, 0x2C, 0x01, 0x00, 0x00};
  transmit_and_log(0x600 + id, data, 8, *this);
  sleep_ms(10);
  data[3] = 0x02;
  transmit_and_log(0x600 + id, data, 8, *this);
}

void WheelMotorCanCtrl::Set_Overload_Time(uint8_t id)
{
  std::cout << std::endl << GREEN << "--->  [CAN Msg] Transmit: Set_Overload_param " << TAIL <<
    std::endl;
  sleep_ms(10);
  uint8_t data[8] = {0x2B, 0x16, 0x20, 0x01, 0x20, 0x03, 0x00, 0x00};
  transmit_and_log(0x600 + id, data, 8, *this);
  sleep_ms(10);
  data[3] = 0x02;
  transmit_and_log(0x600 + id, data, 8, *this);
}

}  // namespace xjrobot_base
