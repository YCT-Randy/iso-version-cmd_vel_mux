#ifndef VEL_MUX_ROS
#define VEL_MUX_ROS

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include "cmd_vel_mux/vel_mux_ros_value.h"
#include "cmd_vel_mux/vel_mux_type.h"

#include <yaml-cpp/yaml.h>
#include <vector>

class VelMuxROS
{
public:
  VelMuxROS(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  virtual ~VelMuxROS();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher vel_pub_;
  ros::Subscriber key_mode_sub_;
  ros::Subscriber emergency_subscriber_;

  VelMuxROSValue values_;

  bool show_debug_info_;
  std::string config_path_;

  std::vector<std::shared_ptr<VelMember>> vel_member_list_;
  void loadConfigure();
  void createVelMember();
  void publishZeroVel();
  void handlePtr_switchTo(const std::shared_ptr<VelMember> &target_ptr);
  void restart_TimeOut(const std::shared_ptr<VelMember> &target_ptr);

  void checkTimeOut_Timer(const ros::TimerEvent &event, const std::shared_ptr<VelMember> &member_ptr);
  void cmdVel_Callback(const geometry_msgs::Twist::ConstPtr &msg, const std::shared_ptr<VelMember> &member_ptr);
  void key_Callback(const std_msgs::Int8::ConstPtr &msg);
  void emergency_Callback(const std_msgs::Bool::ConstPtr &msg);
};

#endif // VEL_MUX_ROS
