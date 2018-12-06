#!/usr/bin/env python

import rospy
from robotiq_3f_srvs.srv import Activate

if __name__ == '__main__':
    rospy.init_node('robotiq_activate')
    rospy.loginfo('waiting for robotiq 3f gripper activate.')
    rospy.wait_for_service('/robotiq_3f_gripper/activate')
    try:
        activate = rospy.ServiceProxy('/robotiq_3f_gripper/activate', Activate)
        ret = activate()
        rospy.loginfo('robotiq 3f gripper activation complete.')
    except rospy.ServiceException as e:
        rospy.logerr('Activation service failed:{}'.format(e))

