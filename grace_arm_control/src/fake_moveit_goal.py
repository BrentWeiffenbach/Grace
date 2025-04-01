#!/usr/bin/env python2.7
from moveit_commander import MoveGroupCommander  # type: ignore
from grace_navigation.msg import RobotState

import rospy

def publish_goal():
    group = MoveGroupCommander("arm_group")  # Use your specific planning group name

    # Get the current joint values as the start state
    # current_joint_values = group.get_current_joint_values()
    group.set_start_state_to_current_state()

    # Define the goal joint values
    goal_joint_values = [0.08059417813617288, 0.8414356555608558, -0.3932249476604554, 
                         0.18421584162174223, 0.45491917923620506, 0.16590019448519736]
    group.set_joint_value_target(goal_joint_values)

    # Plan to the goal
    success, plan, _, _ = group.plan()
    if success:
        rospy.loginfo("Planning successful!")
    else:
        rospy.logerr("Planning failed!")

def state_callback(msg):
    if msg.state in [RobotState.PLACING, RobotState.PICKING]:
        publish_goal()

if __name__ == "__main__":
    rospy.init_node("fake_moveit_goal")
    rospy.Subscriber("/grace/state", RobotState, state_callback)
    rospy.spin()