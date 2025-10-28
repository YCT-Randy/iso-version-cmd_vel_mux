#include "cmd_vel_mux/vel_mux_ros_value.h"

//*** Member value set&get from Member function ***//
void VelMuxROSValue::set_McuKeyMode(const int& state)
{
  mcu_key_mode_ = state;
}

int VelMuxROSValue::get_McuKeyMode()
{
  return mcu_key_mode_;
}

void VelMuxROSValue::set_handlePtr(std::shared_ptr<VelMember> ptr)
{
  handle_ptr_ = ptr;
}

std::shared_ptr<VelMember> VelMuxROSValue::get_handlePtr()
{
  return handle_ptr_;
}

void VelMuxROSValue::clear_handlePtr()
{
  ROS_ERROR_STREAM("Release Current Velocity " << handle_ptr_->topic_name);
  handle_ptr_ = nullptr;
}
