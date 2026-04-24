#pragma once

#include <cstdint>

#include "xjrobot_base/controlcan.h"

namespace xjrobot_base
{

class WheelMotorCanCtrl
{
public:
  void print_send_info(const VCI_CAN_OBJ * can_obj);
  uint8_t RPDO0_Config(uint8_t id);
  uint8_t RPDO1_Config(uint8_t id);
  uint8_t RPDO2_Config(uint8_t id);
  uint8_t TPDO0_Config(uint8_t id);
  uint8_t TPDO1_Config(uint8_t id);
  uint8_t Profile_Velocity_Init(uint8_t id);
  uint8_t NMT_Control(uint8_t data0, uint8_t id);
  uint8_t Driver_Enable(uint8_t id);
  void ZLAC8015D_Init_Velocity_Mode();
  bool Velocity_Joy_Control(uint8_t id, uint16_t left_v, uint16_t right_v);
  void Clear_Error_Code(uint8_t id);
  bool Quick_Stop(uint8_t id);
  bool Re_Enabled(uint8_t id);
  bool Driver_Disabled(uint8_t id);
  void Set_max_current(uint8_t id);
  void Enable_Error_PWM(uint8_t id);
  void Set_Overload_param(uint8_t id);
  void Set_Overload_Time(uint8_t id);

private:
  int count_num_ = 0;
};

}  // namespace xjrobot_base
