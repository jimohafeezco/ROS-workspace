#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

class JointPub(object):
    def __init__(self):

        self.publishers_array = []
        self._pub_upperlegM1_joint_position = rospy.Publisher(
            '/spotmini/head_upperlegM1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self._pub_upperlegM2_joint_position = rospy.Publisher(
            '/spotmini/head_upperlegM2_joint_position_controller/command',
            Float64,
            queue_size=1)
        self._pub_upperlegM3_joint_position = rospy.Publisher(
            '/spotmini/head_upperlegM3_joint_position_controller/command',
            Float64,
            queue_size=1)
        self._pub_upperlegM4_joint_position = rospy.Publisher(
            '/spotmini/head_upperlegM3_joint_position_controller/command',
            Float64,
            queue_size=1)
        self._pub_lowerlegM1_joint_position = rospy.Publisher(
            '/spotmini/upperlegM1_lowerlegM1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self._pub_lowerlegM2_joint_position = rospy.Publisher(
            '/spotmini/upperlegM2_lowerlegM2_joint_position_controller/command',
            Float64,
            queue_size=1)
        self._pub_lowerlegM3_joint_position = rospy.Publisher(
            '/spotmini/upperlegM3_lowerlegM3_joint_position_controller/command',
            Float64,
            queue_size=1)
        self._pub_lowerlegM4_joint_position = rospy.Publisher(
            '/spotmini/upperlegM3_lowerlegM3_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.publishers_array.append(self._pub_upperlegM1_joint_position)
        self.publishers_array.append(self._pub_upperlegM2_joint_position)
        self.publishers_array.append(self._pub_upperlegM3_joint_position)
        self.publishers_array.append(self._pub_upperlegM4_joint_position)
        self.publishers_array.append(self._pub_lowerlegM1_joint_position)
        self.publishers_array.append(self._pub_lowerlegM2_joint_position)
        self.publishers_array.append(self._pub_lowerlegM3_joint_position)
        self.publishers_array.append(self._pub_lowerlegM4_joint_position)




    def move_joints(self, joints_array):

        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.loginfo(str(joint_value))
          publisher_object.publish(joint_value)
          i += 1


    def start_loop(self, rate_value = 2.0):
        rospy.loginfo("Start Loop")
        pos1 = [-0.3,-0.0,-0.3, 0.0,-0.2,-0.0, -0.2,0.0]
        pos2 = [0.0,-0.3,-0.0, -0.3,0.0, -0.2, -0.0,-0.2]
        position = "pos1"
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
          if position == "pos1":
            self.move_joints(pos1)
            position = "pos2"
          else:
            self.move_joints(pos2)
            position = "pos1"
          rate.sleep()


if __name__=="__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 0.5
    joint_publisher.start_loop(rate_value)



