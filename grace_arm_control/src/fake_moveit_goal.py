#!/usr/bin/env python2.7
from moveit_commander import MoveGroupCommander  # type: ignore
from grace_navigation.msg import RobotState

import rospy

class FakeMoveItGoal:
    def __init__(self):
        rospy.init_node("fake_moveit_goal")
        self.group = MoveGroupCommander("arm_group")  # Use your specific planning group name
        rospy.Subscriber("/grace/state", RobotState, self.state_callback)

    def publish_goal(self):
        # Define the goal joint values
        goal_joint_values = [0.08059417813617288, 0.8414356555608558, -0.3932249476604554, 
                             0.18421584162174223, 0.45491917923620506, 0.16590019448519736]
        self.group.set_joint_value_target(goal_joint_values)

        # Plan to the goal
        success, plan, _, _ = self.group.plan()
        if success:
            rospy.loginfo("Planning successful!")
        else:
            rospy.logerr("Planning failed!")

    def state_callback(self, msg):
        if msg.state in [RobotState.PLACING, RobotState.PICKING]:
            self.publish_goal()

if __name__ == "__main__":
    while True:
        state_msg = rospy.wait_for_message(topic="/grace/state", topic_type=RobotState)
        if state_msg.state == RobotState.EXPLORING:  # type: ignore
            break
    fake_moveit_goal = FakeMoveItGoal()
    rospy.spin()