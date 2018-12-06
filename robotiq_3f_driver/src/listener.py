#!/usr/bin/env python

import rospy

from robotiq_3f_srvs.srv import Activate, ActivateResponse
from robotiq_3f_srvs.srv import Reset, ResetResponse
from robotiq_3f_srvs.srv import Move, MoveResponse
from robotiq_3f_srvs.srv import SetMode, SetModeResponse
from robotiq_3f_srvs.srv import SetPosition, SetPositionResponse
from robotiq_3f_srvs.srv import SetSpeed, SetSpeedResponse
from robotiq_3f_srvs.srv import SetTorque, SetTorqueResponse
from robotiq_3f_srvs.srv import GetMode, GetModeResponse
from robotiq_3f_srvs.srv import GetPosition, GetPositionResponse
from robotiq_3f_srvs.srv import GetSpeed, GetSpeedResponse
from robotiq_3f_srvs.srv import GetTorque, GetTorqueResponse

from driver import Robotic3fGripperDriver

from time import sleep

class Robotic3fGripperListener(object):
    def __init__(self):
        self.gripper = Robotic3fGripperDriver()
        self.gripper.activate()
        sleep(0.01)
        while self.gripper._gripper_status.gIMC != 3:
            pass
        self._activate_service = rospy.Service('/robotiq_3f_gripper/activate', Activate, self.activate)
        self._open_hand_service = rospy.Service('/robotiq_3f_gripper/open_hand', Move, self.open_hand)
        self._close_hand_service = rospy.Service('/robotiq_3f_gripper/close_hand', Move, self.close_hand)
        self._set_mode_service = rospy.Service('/robotiq_3f_gripper/set_mode', SetMode, self.set_mode)
        self._set_position_service = rospy.Service('/robotiq_3f_gripper/set_position', SetPosition, self.set_position)
        self._set_speed_service = rospy.Service('/robotiq_3f_gripper/set_speed', SetSpeed, self.set_speed)
        self._set_torque_service = rospy.Service('/robotiq_3f_gripper/set_torque', SetTorque, self.set_torque)
        self._get_mode_service = rospy.Service('/robotiq_3f_gripper/get_mode', GetMode, self.get_mode)
        self._get_position_service = rospy.Service('/robotiq_3f_gripper/get_position', GetPosition, self.get_position)
        self._get_speed_service = rospy.Service('/robotiq_3f_gripper/get_speed', GetSpeed, self.get_speed)
        self._get_torque_service = rospy.Service('/robotiq_3f_gripper/get_torque', GetTorque, self.get_torque)
        rospy.loginfo("Ready to Robotic 3f Gripper services")

    def activate(self, req):
        ret = ActivateResponse()
        self.gripper.activate()
        sleep(0.01)
        while self.gripper._gripper_status.gIMC != 3:
            pass
        ret.success = True
        return ret

    def open_hand(self, req):
        ret = MoveResponse()
        if not self.gripper._is_activated():
            ret.success = False
            return ret
        self.gripper.open_hand()
        sleep(0.01)
        while True:
            if self.gripper._gripper_status.gGTO == 1 and \
               self.gripper._gripper_status.gSTA != 0:
                break
        ret.success = True
        return ret

    def close_hand(self, req):
        ret = MoveResponse()
        if not self.gripper._is_activated():
            ret.success = False
            return ret
        self.gripper.close_hand()
        sleep(0.01)
        while True:
            if self.gripper._gripper_status.gGTO == 1 and \
               self.gripper._gripper_status.gSTA != 0:
                break
        ret.success = True
        return ret

    def set_mode(self, req):
        ret = SetModeResponse()
        if not self.gripper._is_activated():
            ret.success = False
            return ret
        self.gripper.set_mode(req.mode)
        sleep(0.01)
        while self.gripper._gripper_status.gIMC != 3:
            pass
        ret.success = True
        return ret

    def set_position(self, req):
        ret = SetPositionResponse()
        if not self.gripper._is_activated():
            ret.success = False
            return ret
        self.gripper.set_position(req.position)
        sleep(0.01)
        while True:
            if self.gripper._gripper_status.gGTO == 1 and \
               self.gripper._gripper_status.gSTA != 0:
                break
        ret.success = True
        return ret

    def set_speed(self, req):
        ret = SetSpeedResponse()
        if not self.gripper._is_activated():
            ret.success = False
            return ret
        self.gripper.set_speed(req.speed)
        ret.success = True
        return ret

    def set_torque(self, req):
        ret = SetTorqueResponse()
        if not self.gripper._is_activated():
            ret.success = False
            return ret
        self.gripper.set_torque(req.torque)
        ret.success = True
        return ret

    def get_mode(self, req):
        ret = GetModeResponse()
        if not self.gripper._is_activated():
            ret.success = False
            return ret
        ret.mode = self.gripper.get_mode()
        if ret.mode:
            ret.success = True
        else:
            ret.success = False
        return ret

    def get_position(self, req):
        ret = GetPositionResponse()
        if not self.gripper._is_activated():
            ret.success = False
            return ret
        ret.target_position = self.gripper.get_position_target()
        ret.finger_a_position = self.gripper.get_position_a()
        ret.finger_b_position = self.gripper.get_position_b()
        ret.finger_c_position = self.gripper.get_position_c()
        if ret.target_position and ret.finger_a_position and \
           ret.finger_b_position and ret.finger_c_position:
            ret.success = True
        else:
            ret.success = False
        return ret

    def get_speed(self, req):
        ret = GetSpeedResponse()
        if not self.gripper._is_activated():
            ret.success = False
            return ret
        ret.speed = self.gripper.get_speed()
        ret.success = True
        return ret

    def get_torque(self, req):
        ret = GetTorqueResponse()
        if not self.gripper._is_activated():
            ret.success = False
            return ret
        ret.torque = self.gripper.get_torque()
        ret.success = True
        return ret

if __name__ == '__main__':
    rospy.init_node('robotiq_3f_gripper_listener')
    robotiq_3f_gripper_listener = Robotic3fGripperListener()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down.")
