cmake_minimum_required(VERSION 2.8.3)
project(robotiq_3f_srvs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Activate.srv
  Reset.srv
  Move.srv
  SetMode.srv
  SetPosition.srv
  SetSpeed.srv
  SetTorque.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

