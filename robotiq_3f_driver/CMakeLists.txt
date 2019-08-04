cmake_minimum_required(VERSION 2.8.3)
project(robotiq_3f_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  robotiq_3f_gripper_control
  )

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_srvs
  robotiq_3f_gripper_control
  )

include_directories(${catkin_INCLUDE_DIRS})

###########
# Install #
###########

# Python scripts
set(PYTHON_SCRIPTS
  src/activate.py
  src/driver.py
  src/listener.py
)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(scripts)

set(DIRECTORIES
  ./launch/
  )
# Install all files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

########
# Test #
########

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  set(DIRECTORIES
    ./launch/
    )

  foreach(dirs ${DIRECTORIES})
    roslaunch_add_file_check(${dirs})
  endforeach()
endif()

