#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python


import threading
from math import atan2, radians
from typing import Dict, Union

import actionlib
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from grace.msg import Object2D, Object2DArray
from grace_node import RobotGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseGoal
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation


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

    def semantic_map_callback(self, msg: Object2DArray) -> None:
        # if not self.lock:
        #     self.semantic_map = msg
        self.semantic_map = msg

    def explore(self, goal: Union[RobotGoal, None], timeout: Union[int, None]) -> bool:
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

        while timeout is None or end_time.__gt__(rospy.Time.now()):
            # BUG: This loop is kinda really bad. It doesn't loop much at all...
            if self.lock:
                continue

            goal_coords: Union[Point, None] = self.find_object_in_map(goal.child_object)
            if goal_coords is None:
                # TODO: Explore frontier here
                SlamController.verbose_log("Object not initially found. Exploring...")
                SlamController.verbose_log("Explore not implemented yet!")
                continue

            goal_pose: Pose = self.calculate_offset(goal_coords)
            goto_thread: threading.Thread = self.goto(
                goal_pose, timeout=rospy.Duration(timeout or 0)
            )

            while goto_thread.is_alive():
                if rospy.Time.now() >= end_time:
                    # Run out of time
                    rospy.loginfo("Slam Controller ran out of time to find object!")
                    self.move_base.cancel_goal()
                    goto_thread.join()
                    return False

                cb_result: Union[None, bool] = self.goto_cb(
                    tick=0.1, goto_thread=goto_thread
                )
                if cb_result is not None:
                    # Pass up the chain if it returned something
                    return cb_result
            # BUG: This should return based off goal, rather than just returning True every time
            return True
        return False

    def wait_for_semantic_map(self, sleep_time_s: Union[int, rospy.Duration]) -> bool:
        """Checks if self.semantic_map is initialized. If it is not, sleep for `sleep_time_s` seconds and try again.

        Args:
            sleep_time_s (int | rospy.Duration): The time, in seconds, to wait before checking if semantic_map is initialized again.

        Returns:
            bool: True if it is initialized, False otherwise.
        """
        try:
            self.semantic_map
        except AttributeError:
            result = False
        else:
            result = True
        # timeout=sleep_time_s
        result = result or rospy.wait_for_message("/semantic_map", Object2DArray)
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

    def calculate_offset(self, point: Point) -> Pose:
        current_pose: Pose = self.get_current_pose()
        obj_angle: float = self.calculate_object_direction(current_pose, point)
        inverse_direction = np.array(
            [np.cos(obj_angle + np.pi), np.sin(obj_angle + np.pi)]
        )
        offset_position = np.array([point.x, point.y]) + inverse_direction * 0.5
        new_quaternion = Rotation.from_euler(
            "xyz", [0, 0, radians(obj_angle)]
        ).as_quat()

        new_pose = Pose()
        new_pose.position = Point(*offset_position, 0)
        new_pose.orientation = Quaternion(*new_quaternion)
        return new_pose

    def get_current_pose(self) -> Pose:
        """Returns the current pose of the turtlebot by subscribing to the `/odom`.

        Returns:
            Pose: The turtlebot's current Pose.
        """

        TIMEOUT_SECONDS: Union[int, rospy.Duration] = 2
        odom_msg = rospy.wait_for_message(
            topic="/odom", topic_type=Odometry, timeout=TIMEOUT_SECONDS
        )
        odom: Odometry = odom_msg or Odometry()  # Does nothing but make pylance happy
        # The Odometry() call would never run since odom_msg has to be True (or an exception is raised)
        pose: Pose = odom.pose.pose
        return pose

    def calculate_object_direction(self, pose: Pose, obj: Point) -> float:
        angle: float = atan2(obj.y - pose.position.y, obj.x - pose.position.x)
        return angle

    def feedback_cb(self, feedback: MoveBaseFeedback) -> None:
        """The callback when the robot moves.

        Args:
            feedback (MoveBaseFeedback): The MoveBaseFeedback msg. It contains a `base_position` attribute.
        """
        pass

    def goto(self, obj: Pose, timeout: rospy.Duration) -> threading.Thread:
        def execute_goto() -> bool:
            SlamController.verbose_log(f"Going to position {obj}")
            # Go towards object
            # https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-action-client-py.md
            dest = MoveBaseGoal()
            dest.target_pose.header.frame_id = "map"
            dest.target_pose.header.stamp = rospy.Time.now()
            dest.target_pose.pose = obj
            self.move_base.send_goal(goal=dest, feedback_cb=self.feedback_cb)

            # TODO: Add callbacks to this to allow it to use more accurate maps
            wait = self.move_base.wait_for_result(timeout=timeout)
            if not wait:
                rospy.logerr("Action server not availible!")
                self.lock = False
                return False

            # Can be any of the actionlib.GoalStatus constants
            state: int = self.move_base.get_state()
            self.lock = False
            if SlamController.verbose:
                # Get all the constants
                _states: Dict[int, str] = {
                    value: key
                    for key, value in actionlib.GoalStatus.__dict__.items()
                    if not key.startswith("_")
                    and not callable(value)
                    and isinstance(value, int)
                }
                SlamController.verbose_log(
                    f"The move_base state at the end of goto was {_states.get(state)}({state})."
                )

            return state == actionlib.GoalStatus.SUCCEEDED

        thread = threading.Thread(target=execute_goto)
        thread.start()
        return thread

    def goto_cb(
        self, tick: Union[rospy.Duration, int, float], goto_thread: threading.Thread
    ) -> Union[None, bool]:
        """Run every `tick` seconds.

        Args:
            tick (rospy.Duration | int | float): The amount of time the function should sleep.
            goto_thread (threading.Thread): The goto thread. See `threading` documentation for uses.

        Returns:
            Union[None, bool]: It either does not return anything, or returns a bool that should be passed up the chain.
        """
        # TODO: Update goal position here
        rospy.sleep(tick)

    def run(self) -> None:
        rospy.spin()

    def shutdown(self) -> None:
        SlamController.verbose_log("Shutting down SlamController.")
        self.move_base.cancel_all_goals()


if __name__ == "__main__":
    rospy.init_node("slam_controller")
    slam_controller = SlamController(verbose=True)
    rospy.on_shutdown(slam_controller.shutdown)
    try:
        slam_controller.run()
    except rospy.ROSInterruptException:
        slam_controller.shutdown()
        rospy.loginfo("SlamController shut down.")
