#!/usr/bin/python3

import rospy
from arm_msgs.msg import arm_master_comm

def cb_func(data: arm_master_comm):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.L1))


if __name__ == '__main__':
    rospy.init_node('cmd_input_listener')
    rospy.Subscriber("cmd_input", arm_master_comm, cb_func)

    rospy.spin()