#!/usr/bin/env python

import rospy
from joint_publisher import JointPub

if __name__ == "__main__":
    rospy.init_node('jump_control_node', log_level=rospy.INFO)
    joint_publisher = JointPub()
    rate_value = 8.0
    joint_publisher.move_joints_jump(rate_value)
