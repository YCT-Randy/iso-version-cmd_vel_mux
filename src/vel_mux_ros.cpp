#include "cmd_vel_mux/vel_mux_ros.h"

VelMuxROS::VelMuxROS(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
  key_mode_sub_ = nh_.subscribe("key_switch_mode", 1, &VelMuxROS::key_Callback, this);
  nh_private_.param<bool>("show_debug_info", show_debug_info_, true);
  nh_private_.param<std::string>("config_path", config_path_, "cfg/config.yaml");

  loadConfigure();
}

VelMuxROS::~VelMuxROS()
{
  std::cout << "Destroying VelMuxROS()" << std::endl;
}

void VelMuxROS::key_Callback(const std_msgs::Int8::ConstPtr &msg)
{
  // Key switch
  if (values_.get_McuKeyMode() != msg->data)
  {
    if (show_debug_info_)
      ROS_WARN_STREAM("MCU Key Mode " << values_.get_McuKeyMode() << " switch to " << msg->data);
    publishZeroVel();
    values_.set_McuKeyMode(msg->data);

    // Stop the handlePtr timer and release handlePtr
    if (values_.get_handlePtr())
    {
      values_.get_handlePtr()->timeout_check_timer.stop();
      values_.clear_handlePtr();
    }
  }
  if (show_debug_info_)
    ROS_WARN_STREAM("Current Key Mode is " << values_.get_McuKeyMode());
}

void VelMuxROS::loadConfigure()
{
  YAML::Node config = YAML::LoadFile(config_path_);
  for (const auto &node : config["subscribers"])
  {
    auto ptr = std::make_shared<VelMember>();
    ptr->topic_name = node["name"].as<std::string>();
    ptr->timeout_time = node["timeout"].as<float>();
    ptr->key_mode = node["key_mode"].as<int>();
    ptr->priority_level = node["priority_level"].as<int>();
    if (show_debug_info_)
      ROS_INFO_STREAM("Subscriber: " << ptr->topic_name << " timeout=" << ptr->timeout_time
                                     << " key_mode=" << ptr->key_mode << " priority_level=" << ptr->priority_level);
    vel_member_list_.push_back(ptr);
  }

  createVelMember();
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(config["publisher"][0]["name"].as<std::string>(), 1);
}

void VelMuxROS::createVelMember()
{
  if (vel_member_list_.size() != 0)
  {
    for (const auto &member_ptr : vel_member_list_)
    {
      if (member_ptr)
      {
        // Create subscribe
        member_ptr->sub = nh_.subscribe<geometry_msgs::Twist>(
            member_ptr->topic_name, 1,
            [this, member_ptr](const geometry_msgs::Twist::ConstPtr &msg)
            { this->cmdVel_Callback(msg, member_ptr); });

        // Create timeout oneshot timer
        member_ptr->timeout_check_timer = nh_.createTimer(
            ros::Duration(member_ptr->timeout_time),
            [this, member_ptr](const ros::TimerEvent &ev)
            { this->checkTimeOut_Timer(ev, member_ptr); },
            /*oneshot=*/true,
            /*autostart=*/false);
      }
    }
  }
  else
  {
    if (show_debug_info_)
      ROS_INFO_STREAM("Yaml Empty or Invalid format");
  }
}

void VelMuxROS::cmdVel_Callback(const geometry_msgs::Twist::ConstPtr &msg, const std::shared_ptr<VelMember> &member_ptr)
{
  if (show_debug_info_)
    ROS_INFO_STREAM("Get " << member_ptr->topic_name << " Velocity");

  // Init Handle ptr
  if (!values_.get_handlePtr())
  {
    if (member_ptr->key_mode != values_.get_McuKeyMode() && member_ptr->key_mode != -1)
    {
      // Because switch key mode will release handlePtr.
      // Defence handle ptr not match the key mode which get from vel topic.
      // cmd_vel_emergency key mode is -1
      if (show_debug_info_)
        ROS_ERROR_STREAM("Current Velocity is Null because Key Mode not match. "
                         << member_ptr->topic_name << " Key Mode is " << member_ptr->key_mode
                         << ", Current Key Mode is " << values_.get_McuKeyMode());
      return;
    }
    else
    {
      // Set the handle ptr
      if (show_debug_info_)
        ROS_WARN_STREAM("Current Velocity reset to: " << member_ptr->topic_name);
      values_.set_handlePtr(member_ptr);
    }
  }
  if (show_debug_info_)
    ROS_INFO_STREAM("Current Velocity is " << values_.get_handlePtr()->topic_name);

  if (member_ptr->key_mode == -1) // Get Topic: cmd_vel_emergency
  {
    if (values_.get_handlePtr() != member_ptr)
    {
      handlePtr_switchTo(member_ptr);
    }
    restart_TimeOut(member_ptr);
    if (show_debug_info_)
      ROS_WARN_STREAM("Get Twist from " << member_ptr->topic_name << " and publish");
    vel_pub_.publish(msg);
  }
  else if (member_ptr->key_mode != values_.get_McuKeyMode()) // Key mode not match, refuse publish
  {
    if (show_debug_info_)
      ROS_ERROR_STREAM("Key Mode not match, " << member_ptr->topic_name << " Key Mode is " << member_ptr->key_mode
                                              << ", Current Key Mode is " << values_.get_McuKeyMode());
  }
  else if (member_ptr->priority_level >= values_.get_handlePtr()->priority_level) // Same key mode but high priority
                                                                                  // level, switch handle ptr than
                                                                                  // publish
  {
    // Get priority_level more then now we handle
    if (values_.get_handlePtr() != member_ptr)
    {
      handlePtr_switchTo(member_ptr);
    }
    restart_TimeOut(member_ptr);
    if (show_debug_info_)
      ROS_WARN_STREAM("Get Twist from " << member_ptr->topic_name << " and send");
    vel_pub_.publish(msg);
  }
  else // Same key mode but low priority level, refuse publish
  {
    if (show_debug_info_)
      ROS_ERROR_STREAM("Refuse velocity: " << member_ptr->topic_name);
  }
}

// Timeout timer,  timeout will clear handle ptr
void VelMuxROS::checkTimeOut_Timer(const ros::TimerEvent &event, const std::shared_ptr<VelMember> &member_ptr)
{
  if (show_debug_info_)
    ROS_ERROR_STREAM("Current Velocity" << member_ptr->topic_name << " Time Out, timeout time is "
                                        << member_ptr->timeout_time << " sec");
  // Let same key mode but priority_level low can handle in.
  if (values_.get_handlePtr())
    values_.clear_handlePtr();
  publishZeroVel();
}

void VelMuxROS::publishZeroVel()
{
  geometry_msgs::Twist msg;
  vel_pub_.publish(msg);
}

// Stop old handle ptr timeout timer and switch handle ptr to target pptr
void VelMuxROS::handlePtr_switchTo(const std::shared_ptr<VelMember> &target_ptr)
{
  if (show_debug_info_)
    ROS_WARN_STREAM("Current Velocity switch to " << target_ptr->topic_name);
  values_.get_handlePtr()->timeout_check_timer.stop(); // Stop old handle ptr timeout timer
  values_.set_handlePtr(target_ptr);
}

void VelMuxROS::restart_TimeOut(const std::shared_ptr<VelMember> &target_ptr)
{
  target_ptr->timeout_check_timer.stop();
  target_ptr->timeout_check_timer.start();
}
