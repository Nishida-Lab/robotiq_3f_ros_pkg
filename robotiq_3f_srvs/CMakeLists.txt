cmake_minimum_required(VERSION 2.8.3)
project(robotiq_3f_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
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
  GetMode.srv
  GetPosition.srv
  GetSpeed.srv
  GetTorque.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_generation
  message_runtime
  std_msgs
  message_runtime
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

