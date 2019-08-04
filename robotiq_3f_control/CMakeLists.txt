cmake_minimum_required(VERSION 2.8.3)
project(robotiq_3f_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  roscpp
  sensor_msgs
)
catkin_package(
 CATKIN_DEPENDS
 control_msgs
 roscpp
 sensor_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robotiq_3f_joint_state_republisher src/robotiq_3f_joint_state_republisher.cpp)
target_link_libraries(robotiq_3f_joint_state_republisher
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

set(EXEC_FILES
  robotiq_3f_joint_state_republisher
  )

foreach(nodes ${EXEC_FILES})
  install(TARGETS robotiq_3f_joint_state_republisher
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
endforeach(nodes)

set(DIRECTORIES
  ./launch/
  )
# other files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  set(DIRECTORIES
    ./launch/
    )

  foreach(dirs ${DIRECTORIES})
    roslaunch_add_file_check(${dirs})
  endforeach()
endif()
