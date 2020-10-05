#!/usr/bin/python
from sensor_msgs.msg import JointState
import rospy
import math

rospy.init_node("joint_state_publisher", anonymous=True)

joint_pub = rospy.Publisher("/joint_states", JointState,  queue_size=1) 
rate = rospy.Rate(20)
def joint_callback():
    #print j1, j2
    i=0
    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name += ["base_to_second_joint"]
        joint_state.effort+= []
        joint_state.position+= [math.sin(i)]
        joint_state.velocity+= [0]
        # if 
            # joint_state.position+= [0]

        joint_pub.publish(joint_state)

        rate.sleep()


if __name__ == '__main__':
    try:
        joint_callback()
    except rospy.ROSInterruptException:
        pass