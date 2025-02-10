#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python


from typing import Union

import actionlib
import rospy
from geometry_msgs.msg import Point, Quaternion
from grace.msg import RobotState, Object2DArray, Object2D
from grace_node import RobotGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class SlamController:
    verbose: bool = False

    @staticmethod
    def verbose_log(log: str) -> None:
        """Logs the input to the console if `SlamController.verbose` is True. Also uses `rospy.logdebug()`.

        Args:
            log (str): The log to print conditionally.
        """
        if SlamController.verbose:
            rospy.loginfo(log)
        rospy.logdebug(log)

    def __init__(self, verbose: bool = False) -> None:
        SlamController.verbose = verbose
        self.goal: RobotGoal
        self.semantic_map: Object2DArray
        self.lock: bool = False

        self.semantic_map_sub = rospy.Subscriber(
            name="/semantic_map",
            data_class=Object2DArray,
            callback=self.semantic_map_callback,
        )
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        SlamController.verbose_log("Starting move_base action server...")
        self.move_base.wait_for_server()  # Wait until move_base server starts
        SlamController.verbose_log("move_base action server started!")

        rospy.set_param('/move_base/base_global_planner', 'navfn/NavfnROS')  # Use a simpler global planner
        rospy.set_param('/move_base/base_local_planner', 'dwa_local_planner/DWAPlannerROS')

    def semantic_map_callback(self, msg: Object2DArray) -> None:
        if not self.lock:
            self.semantic_map = msg

    def explore(self, goal: Union[RobotGoal, None], timeout: Union[int, None]) -> bool:
        """Have GRACE start exploring. Tries to find the provided goal."""
        if goal is None:
            rospy.logerr("SlamController cannot start exploring with no goal!")
            return False
        SlamController.verbose_log(f"SlamController recieved new goal! {goal}")

        # Give semantic map time to sleep
        is_semantic_map_loaded: bool = self.wait_for_semantic_map(sleep_time_s=10)
        if not is_semantic_map_loaded:
            return False

        start_time: rospy.Time = rospy.Time.now()
        end_time: rospy.Time = start_time + rospy.Duration(
            timeout or 0
        )  # or 0 since it will not be used if there is no timeout
        # Check if object is in the semantic map
        while timeout is None or end_time.__gt__(rospy.Time.now()):
            if self.lock:
                continue

            goal_coords: Union[Point, None] = self.find_object_in_map(goal.child_object)
            if goal_coords is None:
                # Explore frontier here
                continue

            result: bool = self.goto(goal_coords)
            return result
        rospy.loginfo("Slam Controller ran out of time to find object!")
        return False

    def wait_for_semantic_map(self, sleep_time_s: Union[int, rospy.Duration]) -> bool:
        """Checks if self.semantic_map is initialized. If it is not, sleep for `sleep_time_s` seconds and try again.

        Args:
            sleep_time_s (int | rospy.Duration): The time, in seconds, to wait before checking if semantic_map is initialized again.

        Returns:
            bool: True if it is initialized, False otherwise.
        """
        result = rospy.wait_for_message(
            "/semantic_map", Object2DArray, timeout=sleep_time_s
        )
        if not result:
            rospy.logerr(
                f"Slam Controller could not receive a map after waiting for {sleep_time_s} seconds."
            )
            return False
        return True

    def find_object_in_map(self, obj: str) -> Union[Point, None]:
        assert self.semantic_map  # I do not want to deal with this not being initialized # fmt: skip
        assert self.semantic_map.objects
        for map_obj in self.semantic_map.objects:
            map_obj: Object2D
            if map_obj.cls == obj:
                return Point(map_obj.x, map_obj.y, 0)
        return None


    def goto(self, obj: Point) -> bool:
        if not self.lock:
            self.lock = True
        SlamController.verbose_log(f"Going to position {obj}")
        # Go towards object
        # https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-action-client-py.md
        dest = MoveBaseGoal()
        dest.target_pose.header.frame_id = "map"
        dest.target_pose.header.stamp = rospy.Time.now()
        # TODO: Add the target pose
        dest.target_pose.pose.position = obj
        dest.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.move_base.send_goal(dest)
        wait = self.move_base.wait_for_result()
        if not wait:
            rospy.logerr("Action server not availible!")
            self.lock = False
            return False
        else:
            self.move_base.get_result()
            self.lock = False
            return self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED

    def run(self) -> None:
        rospy.spin()

    def shutdown(self) -> None:
        SlamController.verbose_log("Shutting down SlamController.")


if __name__ == "__main__":
    rospy.init_node("slam_controller")
    slam_controller = SlamController(verbose=True)
    rospy.on_shutdown(slam_controller.shutdown)
    try:
        slam_controller.run()
    except rospy.ROSInterruptException:
        slam_controller.shutdown()
        rospy.loginfo("SlamController shut down.")
