cmake_minimum_required(VERSION 2.8.3)
project(robotiq_3f_gazebo)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

# other files
foreach(dir urdf robots config launch)
   install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
