#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>

class RobotiqHandJointStateRePublisher
{
public:
  RobotiqHandJointStateRePublisher(ros::NodeHandle &nh);

private:
  void jointstateCallback(const sensor_msgs::JointState::ConstPtr &js);
  float joint_position[11] = {0.2814, 1.0443, -1.2217, 0.2814, 1.0443, -1.2217, 0.2814, 1.0443, -1.2217, 0.0, 0.0};

  ros::Subscriber js_sub;
  ros::Publisher js_pub;

  std::vector<std::string> robotiq_hand_joint_name;
};

RobotiqHandJointStateRePublisher::RobotiqHandJointStateRePublisher(ros::NodeHandle &nh)
{
  robotiq_hand_joint_name.push_back("finger_1_joint_1");
  robotiq_hand_joint_name.push_back("finger_1_joint_2");
  robotiq_hand_joint_name.push_back("finger_1_joint_3");
  robotiq_hand_joint_name.push_back("finger_2_joint_1");
  robotiq_hand_joint_name.push_back("finger_2_joint_2");
  robotiq_hand_joint_name.push_back("finger_2_joint_3");
  robotiq_hand_joint_name.push_back("finger_middle_joint_1");
  robotiq_hand_joint_name.push_back("finger_middle_joint_2");
  robotiq_hand_joint_name.push_back("finger_middle_joint_3");
  robotiq_hand_joint_name.push_back("palm_finger_1_joint");
  robotiq_hand_joint_name.push_back("palm_finger_2_joint");

  ros::NodeHandle n("~");
  js_pub = nh.advertise<sensor_msgs::JointState>(n.param<std::string>("joint_state_republish_topic_name", "/joint_states_republish"), 1);
  js_sub = nh.subscribe<sensor_msgs::JointState>(n.param<std::string>("joint_state_subscribe_topic_name", "/joint_states"), 10, &RobotiqHandJointStateRePublisher::jointstateCallback, this);
}

void RobotiqHandJointStateRePublisher::jointstateCallback(const sensor_msgs::JointState::ConstPtr &js)
{
  sensor_msgs::JointState joint_state;
  joint_state.header = js->header;
  joint_state.name = js->name;
  joint_state.position = js->position;
  joint_state.velocity = js->velocity;
  joint_state.effort = js->effort;

  for (int i = 0; i < robotiq_hand_joint_name.size(); i++)
  {
    joint_state.name.push_back(robotiq_hand_joint_name[i]);
    joint_state.position.push_back(joint_position[i]);
    joint_state.velocity.push_back(0.0);
    joint_state.effort.push_back(0.0);
  }

  js_pub.publish(joint_state);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robotiq_3f_joint_state_republisher");
  ros::NodeHandle n;
  RobotiqHandJointStateRePublisher robotiq_hand_joint_state_republisher(n);
  ros::spin();
  return 0;
}
