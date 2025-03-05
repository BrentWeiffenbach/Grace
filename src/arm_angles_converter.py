#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import math
def callback(data):
    # Convert degrees to radians
    angles_in_radians = [math.radians(angle) for angle in data.position]

    # Create a new JointState message
    joint_state_msg = JointState()
    joint_state_msg.header = data.header
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    joint_state_msg.position = angles_in_radians
    joint_state_msg.velocity = data.velocity
    joint_state_msg.effort = data.effort

    # Publish the new JointState message
    pub.publish(joint_state_msg)

def listener():
    rospy.init_node('arm_angles_converter', anonymous=True)
    rospy.Subscriber('/grace/arm_joint_angles', JointState, callback)
    global pub
    pub = rospy.Publisher('/grace/arm_joint_states', JointState, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()