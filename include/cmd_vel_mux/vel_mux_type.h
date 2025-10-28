#ifndef VEL_MUX_TYPE
#define VEL_MUX_TYPE

#include "ros/ros.h"

struct VelMember
{
  std::string topic_name;
  float timeout_time;
  int key_mode;
  int priority_level;

  ros::Subscriber sub;
  ros::Timer timeout_check_timer;
};

#endif  // VEL_MUX_TYPE
