cmake_minimum_required(VERSION 2.8.3)
project(robotiq_3f_gazebo)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

set(DIRECTORIES
  config
  launch
  robots
  urdf)

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
  roslaunch_add_file_check(launch)
endif()
