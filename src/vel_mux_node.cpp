#include <cmd_vel_mux/vel_mux_ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_mux_node");
  ros::NodeHandle nh;

  ros::NodeHandle nh_private("~");
  VelMuxROS VelMuxROS(nh, nh_private);

  ros::spin();

  return 0;
}