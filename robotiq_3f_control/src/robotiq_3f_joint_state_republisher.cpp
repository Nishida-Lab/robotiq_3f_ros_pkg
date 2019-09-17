#include <robotiq_3f_control/robotiq_3f_joint_state_republisher.h>

using robotiq_3f_joint_state_republisher::Robotiq3FJointStateRePublisher;

Robotiq3FJointStateRePublisher::Robotiq3FJointStateRePublisher(ros::NodeHandle& nh) : nh_(nh)
{
  js_pub_ = nh.advertise<sensor_msgs::JointState>(
      nh.param<std::string>("joint_state_republish_topic_name", "/joint_states_republish"), 1);
  js_sub_ =
      nh.subscribe<sensor_msgs::JointState>(nh.param<std::string>("joint_state_subscribe_topic_name", "/joint_states"),
                                            10, &Robotiq3FJointStateRePublisher::jointstateCallback, this);
  js_robotiq_sub_ = nh.subscribe<sensor_msgs::JointState>(
      "/robotiq/joint_states", 10, &Robotiq3FJointStateRePublisher::jointstateRobotiqCallback, this);
  robotiq_joint_state_.clear();
}

void Robotiq3FJointStateRePublisher::jointstateRobotiqCallback(const sensor_msgs::JointState::ConstPtr& js)
{
  for (int i = 0; i < js->name.size(); i++)
  {
    robotiq_joint_state_[js->name[i]] = js->position[i];
  }
}

void Robotiq3FJointStateRePublisher::jointstateCallback(const sensor_msgs::JointState::ConstPtr& js)
{
  sensor_msgs::JointState joint_state;
  joint_state.header = js->header;
  joint_state.name = js->name;
  joint_state.position = js->position;
  joint_state.velocity = js->velocity;
  joint_state.effort = js->effort;

  if (robotiq_joint_state_.size() != 0)
  {
    for (int i = 0; i < joint_state.name.size(); i++)
    {
      if (robotiq_joint_state_.find(joint_state.name[i]) != robotiq_joint_state_.end())
      {
        joint_state.position[i] = robotiq_joint_state_[joint_state.name[i]];
      }
      else
      {
        continue;
      }
    }
  }

  js_pub_.publish(joint_state);
}
