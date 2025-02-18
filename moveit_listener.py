#!/usr/bin/env python

import rospy
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Twist
import time

def callback(data):
    rospy.loginfo("Received DisplayTrajectory message")
    for trajectory in data.trajectory:
        rospy.loginfo("Processing a trajectory")
        rospy.loginfo("Trajectory type: {}".format(type(trajectory)))
        rospy.loginfo("Joint names: {}".format(trajectory.multi_dof_joint_trajectory.joint_names))
        points = trajectory.multi_dof_joint_trajectory.points
        for i in range(len(points)):
            point = points[i]
            rospy.loginfo("Point time_from_start: {}".format(point.time_from_start))
            twist_msg = Twist()
            if point.velocities:
                twist_msg.linear.x = point.velocities[0].linear.x
                twist_msg.linear.y = point.velocities[0].linear.y
                twist_msg.linear.z = point.velocities[0].linear.z
                twist_msg.angular.x = point.velocities[0].angular.x
                twist_msg.angular.y = point.velocities[0].angular.y
                twist_msg.angular.z = point.velocities[0].angular.z
                rospy.loginfo("Publishing Twist message: linear=({}, {}, {}), angular=({}, {}, {})".format(
                    twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
                    twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z))
                
                start_time = time.time()
                if i < len(points) - 1:
                    duration = (points[i + 1].time_from_start - point.time_from_start).to_sec()
                else:
                    duration = point.time_from_start.to_sec()  # Last point, use its own time_from_start
                
                while time.time() - start_time < duration:
                    cmd_vel_pub.publish(twist_msg)
                    rospy.sleep(0.1)  # Adjust the sleep duration as needed
                
                # Stop the robot after the duration
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = 0.0
                rospy.loginfo("Publishing Twist message to stop")
                cmd_vel_pub.publish(twist_msg)
            else:
                rospy.logwarn("No velocities found in the trajectory point")


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('moveit_listener', anonymous=True)
    rospy.loginfo("moveit_listener node initialized")
    # Subscribe to the /move_group/display_planned_path topic
    rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, callback)
    rospy.loginfo("Subscribed to /move_group/display_planned_path topic")
    # Create a publisher for the /cmd_vel_mux/input/navi topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    rospy.loginfo("Publisher for /cmd_vel_mux/input/navi topic created")
    # Keep the node running
    rospy.spin()