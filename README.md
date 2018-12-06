# robotiq_3f_ros_pkg

ROS wrapper package that provides functions related to 3-Finger Gripper control of robotiq package(https://github.com/ros-industrial/robotiq) as ROS service

## Dependency
- ROS kinetic
- [Nishida-Lab/robotiq](https://github.com/Nishida-Lab/robotiq)

## Install
```
$ cd your_ws/src
$ git clone https://github.com/Nishida-Lab/robotiq.git
$ git clone https://github.com/Nishida-Lab/robotiq_3f_ros_pkg.git
$ cd ..
$ rosdep install -iry --from-paths src
$ catkin build
```

## Available ROS services
```
/robotiq_3f_gripper/activate
/robotiq_3f_gripper/reset
/robotiq_3f_gripper/open_hand
/robotiq_3f_gripper/close_hand
/robotiq_3f_gripper/set_mode
/robotiq_3f_gripper/set_position
/robotiq_3f_gripper/set_speed
/robotiq_3f_gripper/set_torque
/robotiq_3f_gripper/get_mode
/robotiq_3f_gripper/get_position
/robotiq_3f_gripper/get_speed
/robotiq_3f_gripper/get_torque
```

## Available gripper mode
- basic
- pinch
- wide
- scissor

## Usage
```
$ roslaunch robotiq_3f_driver listener.launch ip_address:=192.168.1.11
```
then, ROS services and gripper will be activated.
