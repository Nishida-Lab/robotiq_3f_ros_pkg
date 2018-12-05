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

## Usage
```
$ rosrun robotiq_3f_gripper_control Robotiq3FGripperTcpNode.py <gripper_ip_address>
$ rosrun robotiq_3f_driver listener.py
```
then, ROS services will be activated.
