#!/usr/bin/env python2.7
from moveit_commander import MoveGroupCommander # type: ignore
from grace_navigation.msg import RobotState

import rospy

class FakeMoveItGoal:
    def __init__(self):
        rospy.sleep(20)  # Wait for move_group to initialize
        rospy.Subscriber("/grace/state", RobotState, self.state_callback)
        self.group = MoveGroupCommander("arm_group", wait_for_servers=10)
        self.state = None

    def publish_goal(self):
        assert self.group is not None
        # Define the goal joint values
        goal_joint_values = [0.08059417813617288, 0.95, -0.3932249476604554, 
                             0.18421584162174223, 0.45491917923620506, 0.16590019448519736]
        self.group.set_joint_value_target(goal_joint_values)

        # Plan to the goal
        success, plan, _, _ = self.group.plan()
        if success:
            rospy.loginfo("Planning successful!")
        else:
            rospy.logerr("Planning failed!")

    def state_callback(self, msg):
        # if self.state is None:
        #     self.group = MoveGroupCommander("arm_group")
        #     self.state = msg.state

        if msg.state in [RobotState.PLACING, RobotState.PICKING]:
            # if self.group is None:
            #     self.group = MoveGroupCommander("arm_group")
            self.publish_goal()

if __name__ == "__main__":
    rospy.init_node("fake_moveit_goal")        
    rospy.loginfo("Fake MoveIt Goal Node Initalized")
    fake_moveit_goal = FakeMoveItGoal()
    rospy.spin()