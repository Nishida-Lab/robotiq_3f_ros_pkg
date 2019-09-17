#include <robotiq_3f_control/robotiq_3f_joint_state_republisher.h>

using robotiq_3f_joint_state_republisher::Robotiq3FJointStateRePublisher;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "robotiq_3f_joint_state_republisher");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Robotiq3FJointStateRePublisher robotiq_3f_joint_state_republisher(nh);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  spinner.stop();

  return 0;
}
