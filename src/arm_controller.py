#!/usr/bin/env python
import rospy
from moveit_msgs.msg import MoveGroupActionResult
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        
        self.arm_status = None
        self.trajectory_points = []
        self.current_point_index = 0
        self.publish_timer = None
        self.final_point_sent = False

        rospy.Subscriber('/move_group/result', MoveGroupActionResult, self.move_group_result_callback)
        rospy.Subscriber('/grace/arm_status', String, self.arm_status_callback)
        self.arm_goal_pub = rospy.Publisher('/grace/arm_goal', JointState, queue_size=10)

        # Dummy topics for MoveIt! to recognize the controllers
        self.dummy_arm_controller_pub = rospy.Publisher('grace/arm_controller/follow_joint_trajectory', JointTrajectory, queue_size=10)
        self.dummy_gripper_controller_pub = rospy.Publisher('grace/gripper_controller/follow_joint_trajectory', JointTrajectory, queue_size=10)

    def move_group_result_callback(self, msg):
        rospy.loginfo("Received move group result")
        move_group_action = msg.result
        planned_trajectory = move_group_action.planned_trajectory
        joint_trajectory = planned_trajectory.joint_trajectory
        self.trajectory_points = joint_trajectory.points
        self.current_point_index = 0
        self.final_point_sent = False
        rospy.loginfo("Number of points in the trajectory: {}".format(len(self.trajectory_points)))
        for i, point in enumerate(self.trajectory_points):
            rospy.loginfo("Trajectory point {}: positions (radians) = {}, positions (degrees) = {}".format(
                i, point.positions, [math.degrees(pos) for pos in point.positions]))
        self.send_next_trajectory_point()

    def arm_status_callback(self, msg):
        self.arm_status = msg.data
        rospy.loginfo("Arm status: {}".format(self.arm_status))
        if self.arm_status == "waiting":
            self.send_next_trajectory_point()
        elif self.arm_status == "executing" or self.arm_status == "completed":
            rospy.loginfo("Stopping timer so no more publishes should occur")
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
                self.publish_timer = None
            self.final_point_sent = True
        elif self.arm_status == "lost_sync":
            rospy.logwarn("Lost sync with device, attempting to reconnect...")
            self.reconnect()

    def send_next_trajectory_point(self):
        if self.current_point_index < len(self.trajectory_points):
            point = self.trajectory_points[self.current_point_index]
            joint_state = JointState()
            joint_state.position = [math.degrees(pos) for pos in point.positions]
            if self.current_point_index == len(self.trajectory_points) - 1:
                rospy.loginfo("Publishing final trajectory point")
                joint_state.header.frame_id = "Goal"
            rospy.loginfo("Publishing joint state for trajectory point {}: positions (radians) = {}, positions (degrees) = {}".format(
                self.current_point_index, point.positions, joint_state.position))
            self.arm_goal_pub.publish(joint_state)
            rospy.loginfo("Sent joint state for trajectory point {}: {}".format(self.current_point_index, joint_state.position))
            self.current_point_index += 1
            if self.publish_timer is None:
                self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.republish_current_point)
        else:
            rospy.loginfo("All trajectory points have been sent")
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
                self.publish_timer = None

    def republish_current_point(self, event):
        if self.arm_status != "executing" and not self.final_point_sent:
            point = self.trajectory_points[self.current_point_index - 1]
            joint_state = JointState()
            joint_state.position = [math.degrees(pos) for pos in point.positions]
            if self.current_point_index == len(self.trajectory_points):
                rospy.loginfo("Publishing final trajectory point")
                joint_state.header.frame_id = "Goal"
            rospy.loginfo("Re-publishing joint state for trajectory point {}: positions (radians) = {}, positions (degrees) = {}".format(
                self.current_point_index - 1, point.positions, joint_state.position))
            self.arm_goal_pub.publish(joint_state)
            rospy.loginfo("Re-sent joint state for trajectory point {}: {}".format(self.current_point_index - 1, joint_state.position))

    def reconnect(self):
        # Reinitialize the necessary components
        rospy.loginfo("Reinitializing connection...")
        self.arm_status = None
        self.current_point_index = 0
        self.final_point_sent = False
        if self.publish_timer is not None:
            self.publish_timer.shutdown()
            self.publish_timer = None
        rospy.Subscriber('/move_group/result', MoveGroupActionResult, self.move_group_result_callback)
        rospy.Subscriber('/grace/arm_status', String, self.arm_status_callback)
        self.arm_goal_pub = rospy.Publisher('/grace/arm_goal', JointState, queue_size=10)
        rospy.loginfo("Reconnection complete. Resending trajectory points...")
        self.send_next_trajectory_point()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        arm_controller = ArmController()
        arm_controller.run()
    except rospy.ROSInterruptException:
        pass