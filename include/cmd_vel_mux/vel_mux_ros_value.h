#ifndef VEL_MUX_ROS_Value
#define VEL_MUX_ROS_Value

#include "cmd_vel_mux/vel_mux_type.h"
#include "ros/ros.h"

class VelMuxROSValue
{
public:
  void set_McuKeyMode(const int& state);
  int get_McuKeyMode();
  void set_handlePtr(std::shared_ptr<VelMember> ptr);
  std::shared_ptr<VelMember> get_handlePtr();
  void clear_handlePtr();

private:
  int mcu_key_mode_;
  bool emergency_on_ = false;
  std::shared_ptr<VelMember> handle_ptr_ = nullptr;
};

#endif  // VEL_MUX_ROS_Value
