#!/usr/bin/env python2.7
from moveit_commander import MoveGroupCommander, RobotCommander # type: ignore
from grace_navigation.msg import RobotState

import rospy

def do_stuff():
    group = MoveGroupCommander("arm_group")  # Use your specific planning group name
    joint_values = [0.08059417813617288, 0.8414356555608558, -0.3932249476604554, 
                    0.18421584162174223, 0.45491917923620506, 0.16590019448519736]
    def publish_goal():
        group.set_joint_value_target(joint_values)

        # robot = RobotCommander()
        # Get the corresponding Cartesian pose
        # pose = robot.get_current_state()

        # rospy.loginfo("Target Pose: {}".format(pose))

        success, plan, _, _ = group.plan()
        # rospy.loginfo(success)
  
    def state_callback(msg):
        if msg.state in [RobotState.PLACING, RobotState.PICKING]:
            publish_goal()
    rospy.Subscriber("/grace/state", RobotState, state_callback)


            

if __name__ == "__main__":
    rospy.init_node("fake_moveit_goal")
    do_stuff()
    rospy.spin()