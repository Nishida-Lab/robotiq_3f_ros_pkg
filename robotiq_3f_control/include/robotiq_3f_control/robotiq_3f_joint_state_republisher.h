#ifndef ROBOTIQ_3F_JOINT_STATE_REPUBLISHER_H
#define ROBOTIQ_3F_JOINT_STATE_REPUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <map>
#include <vector>

namespace robotiq_3f_joint_state_republisher
{
class Robotiq3FJointStateRePublisher
{
public:
  Robotiq3FJointStateRePublisher(ros::NodeHandle& nh);

private:
  void jointstateCallback(const sensor_msgs::JointState::ConstPtr& js);
  void jointstateRobotiqCallback(const sensor_msgs::JointState::ConstPtr& js);

private:
  ros::NodeHandle nh_;
  ros::Subscriber js_sub_;
  ros::Subscriber js_robotiq_sub_;
  ros::Publisher js_pub_;
  std::map<std::string, double> robotiq_joint_state_;
};
}  // robotiq_3f_joint_state_republisher

#endif  // ROBOTIQ_3F_JOINT_STATE_REPUBLISHER_H
