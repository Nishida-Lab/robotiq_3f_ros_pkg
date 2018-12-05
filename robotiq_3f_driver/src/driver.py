#!/usr/bin/env python

import rospy

from robotiq_3f_gripper_control.msg import _Robotiq3FGripper_robot_input as inputMsg
from robotiq_3f_gripper_control.msg import _Robotiq3FGripper_robot_output as outputMsg

#################################################
# NOTE:This program assumes simple control mode #
#################################################

class Robotic3fGripperDriver(object):
    def __init__(self):
        self._gripper_status = inputMsg.Robotiq3FGripper_robot_input()

    def _gripper_status_callback(self, msg):
        self._gripper_status = msg

    def reset(self):
        pass

    def activate(self):
        pass

    def open_hand(self):
        pass

    def close_hand(self):
        pass

    def set_mode(self):
        pass

    def set_position(self):
        pass

    def set_speed(self):
        pass

    def set_torque(self):
        pass

    def get_mode(self):
        pass

    def get_position(self):
        pass

    def get_speed(self):
        pass

    def get_torque(self):
        pass

