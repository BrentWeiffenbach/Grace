#!/usr/bin/env python2.7
import rospy
from moveit_msgs.msg import MoveGroupActionResult
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
from grace_navigation.msg import RobotState
from moveit_commander import MoveGroupCommander # type: ignore

class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        self.state = -1
        self.arm_status = None
        self.trajectory_points = []
        self.current_point_index = 0
        self.publish_timer = None
        self.final_point_sent = False
        self.home_sent = False
        self.zero_sent = False
        self.group = MoveGroupCommander("arm_group")  # Use your specific planning group name

        rospy.Subscriber('/move_group/result', MoveGroupActionResult, self.move_group_result_callback)
        rospy.Subscriber('/grace/arm_status', String, self.arm_status_callback)
        rospy.Subscriber('/grace/state', RobotState, self.state_callback)
        self.arm_goal_pub = rospy.Publisher('/grace/arm_goal', JointState, queue_size=10, latch=True)
        self.gripper_pub = rospy.Publisher('/grace/gripper', String, queue_size=10)
        self.arm_control_status_pub = rospy.Publisher('/grace/arm_control_status', Bool, queue_size=10)

        # Dummy topics for MoveIt! to recognize the controllers
        self.dummy_arm_controller_pub = rospy.Publisher('grace/arm_controller/follow_joint_trajectory', JointTrajectory, queue_size=10)
        self.dummy_gripper_controller_pub = rospy.Publisher('grace/gripper_controller/follow_joint_trajectory', JointTrajectory, queue_size=10)

    def state_callback(self, msg):
        self.state = msg.state
        rospy.loginfo("State is now {}".format(self.state))
        if self.state == RobotState.WAITING:
            self.homing()
        if self.state == RobotState.ZEROING:
            self.zeroing()
    
    def move_group_result_callback(self, msg):
        rospy.loginfo("Received move group result")
        move_group_action = msg.result
        planned_trajectory = move_group_action.planned_trajectory
        joint_trajectory = planned_trajectory.joint_trajectory
        self.trajectory_points = joint_trajectory.points
        self.current_point_index = 0
        self.final_point_sent = False
        self.home_sent = False
        # rospy.loginfo("Number of points in the trajectory: {}".format(len(self.trajectory_points)))
        # for i, point in enumerate(self.trajectory_points):
            # rospy.loginfo("Trajectory point {}: positions (radians) = {}, positions (degrees) = {}".format(
                # i, point.positions, [math.degrees(pos) for pos in point.positions]))
        self.send_next_trajectory_point()

    def homing(self):
        # rospy.loginfo("Homing signal received, publishing blank trajectory point with header 'Homing'")
        self.home_sent = True
        blank_trajectory_point = JointTrajectoryPoint()
        blank_trajectory_point.positions = [0, 0, 0, 0, 0, 0]
        self.trajectory_points = [blank_trajectory_point]
        self.current_point_index = 0
        self.final_point_sent = False
        self.send_next_trajectory_point()
    
    def zeroing(self):
        # rospy.loginfo("State is returning to zero pose")
        self.arm_control_status_pub.publish(Bool(False))
    
        joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.loginfo("Sending zero request to moveit...")
        self.group.set_joint_value_target(joint_values)
        success, plan, _, _ = self.group.plan()
        self.state = RobotState.EXPLORING
        self.current_point_index = 0
        self.final_point_sent = False
        self.home_sent = False
        self.zero_sent = True

    def arm_status_callback(self, msg):
        self.arm_status = msg.data
        if not self.arm_status == "waiting":
             rospy.loginfo("Arm status: {}".format(self.arm_status))
        if self.arm_status == "waiting":
            self.send_next_trajectory_point()
        elif self.arm_status == "executing" or self.arm_status == "homing":
            rospy.loginfo("Stopping timer so no more publishes should occur")
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
                self.publish_timer = None
            self.final_point_sent = True
        elif self.arm_status == "completed":
            if self.state == RobotState.PICKING:
                rospy.loginfo("State is PICKING. Closing the gripper.")
                self.gripper_pub.publish("close")
                rospy.sleep(1)
                self.arm_control_status_pub.publish(Bool(True))
            elif self.state == RobotState.PLACING:
                rospy.loginfo("State is PLACING. Opening the gripper.")
                self.gripper_pub.publish("open")
                rospy.sleep(1)
                self.arm_control_status_pub.publish(Bool(True))
            elif self.state == RobotState.ZEROING:
                self.zeroing()
            elif self.state == RobotState.WAITING:
                self.arm_control_status_pub.publish(Bool(True))
            elif self.state == RobotState.EXPLORING:
                self.arm_control_status_pub.publish(Bool(True))
            else:
                rospy.loginfo("Unkown state: {}".format(self.state))
        elif self.arm_status == "lost_sync":
            rospy.logwarn("Lost sync with device, attempting to reconnect...")
            self.reconnect()

    def send_next_trajectory_point(self):
        if self.trajectory_points and self.current_point_index < len(self.trajectory_points) or (self.arm_status == "waiting" and self.zero_sent):
            point = self.trajectory_points[self.current_point_index]
            joint_state = JointState()
            joint_state.position = [math.degrees(pos) for pos in point.positions]
            if self.current_point_index == len(self.trajectory_points) - 1:
                # rospy.loginfo("Publishing final trajectory point")
                joint_state.header.frame_id = "Goal"
                if self.home_sent:
                    joint_state.header.frame_id = "Homing"
                self.current_point_index -= 1
            # rospy.loginfo("Publishing joint state for trajectory point {}: positions (radians) = {}, positions (degrees) = {}, header = {}".format(
                # self.current_point_index, point.positions, joint_state.position, joint_state.header))
            self.arm_goal_pub.publish(joint_state)
            # rospy.loginfo("Sent joint state for trajectory point {}: {}".format(self.current_point_index, joint_state.position))
            self.current_point_index += 1
            if self.publish_timer is None:
                self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.republish_current_point) # type: ignore
        else:
            rospy.loginfo("All trajectory points have been sent")
            if self.zero_sent:
                self.zero_sent = False
            if self.publish_timer is not None:
                self.publish_timer.shutdown()
                self.publish_timer = None

    def republish_current_point(self, event):
        if self.arm_status != "executing" or self.arm_status != "homing" and not self.final_point_sent and self.trajectory_points:
            point = self.trajectory_points[self.current_point_index - 1]
            joint_state = JointState()
            joint_state.position = [math.degrees(pos) for pos in point.positions]
            if self.current_point_index == len(self.trajectory_points):
                rospy.loginfo("Publishing final trajectory point")
                joint_state.header.frame_id = "Goal"
                if self.home_sent:
                    joint_state.header.frame_id = "Homing"
            # rospy.loginfo("Re-publishing joint state for trajectory point {}: positions (radians) = {}, positions (degrees) = {}".format(
                # self.current_point_index - 1, point.positions, joint_state.position))
            self.arm_goal_pub.publish(joint_state)
            # rospy.loginfo("Re-sent joint state for trajectory point {}: {}".format(self.current_point_index - 1, joint_state.position))

    def reconnect(self):
        # Reinitialize the necessary components
        rospy.loginfo("Reinitializing connection...")
        self.arm_status = None
        self.current_point_index = 0
        self.final_point_sent = False
        self.home_sent = False
        if self.publish_timer is not None:
            self.publish_timer.shutdown()
            self.publish_timer = None
        rospy.Subscriber('/move_group/result', MoveGroupActionResult, self.move_group_result_callback)
        rospy.Subscriber('/grace/arm_status', String, self.arm_status_callback)
        self.arm_goal_pub = rospy.Publisher('/grace/arm_goal', JointState, queue_size=10, latch=True)
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