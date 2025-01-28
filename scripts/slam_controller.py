#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python


from typing import Union

import actionlib
import rospy
from geometry_msgs.msg import Point, Quaternion
from grace.msg import RobotState
from grace_node import RobotGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class SlamController:
    verbose: bool = False

    def __init__(self, verbose: bool = False) -> None:
        SlamController.verbose = verbose
        self.goal: RobotGoal

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()  # Wait until move_base server starts

    def explore(self, goal: Union[RobotGoal, None]) -> bool:
        """Have GRACE start exploring. Tries to find the provided goal."""
        if goal is None:
            rospy.logerr("SlamController cannot start exploring with no goal!")
            return False
        if SlamController.verbose:
            rospy.loginfo(f"SlamController recieved new goal! {goal}")

        # Check if object is in the semantic map
        # Go towards object
        # https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-action-client-py.md
        dest = MoveBaseGoal()
        dest.target_pose.header.frame_id = "map"
        dest.target_pose.header.stamp = rospy.Time.now()
        # TODO: Add the target pose
        dest.target_pose.pose.position = Point(2, 1, 1)
        dest.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.move_base.send_goal(dest)
        wait = self.move_base.wait_for_result()
        if not wait:
            rospy.logerr("Action server not availible!")
            return False
        else:
            self.move_base.get_result()
            return (
                True  # BUG: This is WRONG. The result could be that it can't reach it.
            )

    def run(self) -> None:
        rospy.spin()

    def shutdown(self) -> None:
        if SlamController.verbose:
            rospy.loginfo("Shutting down SlamController.")


if __name__ == "__main__":
    rospy.init_node("slam_controller")
    slam_controller = SlamController()
    rospy.on_shutdown(slam_controller.shutdown)
    try:
        slam_controller.run()
    except rospy.ROSInterruptException:
        slam_controller.shutdown()
        rospy.loginfo("SlamController shut down.")
