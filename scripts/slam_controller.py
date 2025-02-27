#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python


import random
import threading
from math import atan2, radians, sqrt
from typing import Callable, Dict, Tuple, Union

import actionlib
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from grace.msg import Object2D, Object2DArray
from grace_node import RobotGoal, get_constants_from_msg
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseFeedback,
    MoveBaseGoal,
    MoveBaseResult,
)
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

goal_statuses: Dict[int, str] = get_constants_from_msg(actionlib.GoalStatus)
"""Gets all of the non-callable integer constants from actionlib.GoalStatus msg. """


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

    def __init__(
        self, done_cb: Union[Callable, None] = None, verbose: bool = False
    ) -> None:
        SlamController.verbose = verbose
        self.goal: RobotGoal
        self.semantic_map: Object2DArray
        self.lock: bool = False
        self.goal_lock = threading.Lock()

        self.yield_to_master: Callable = done_cb or (
            lambda: SlamController.verbose_log(
                "done_cb not provided to SlamController!"
            )
        )

        self.semantic_map_sub = rospy.Subscriber(
            name="/semantic_map",
            data_class=Object2DArray,
            callback=self.semantic_map_callback,
        )

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        SlamController.verbose_log("Starting move_base action server...")
        self.move_base.wait_for_server()  # Wait until move_base server starts
        SlamController.verbose_log("move_base action server started!")

        # Get initial pose of turtlebot for homing
        self.initial_pose: Pose = self.get_current_pose()

    def semantic_map_callback(self, msg: Object2DArray) -> None:
        self.semantic_map = msg

    def explore_until_found(
        self,
        goal: RobotGoal,
        find_child: bool = True,
        exploration_timeout: rospy.Duration = rospy.Duration(60),
    ) -> Union[Pose, None]:
        # Give semantic map time to sleep
        is_semantic_map_loaded: bool = self.wait_for_semantic_map(sleep_time_s=10)
        if not is_semantic_map_loaded:
            return None

        start_time: rospy.Time = rospy.Time.now()
        exploration_timeout = max(exploration_timeout, rospy.Duration(0))

        while not rospy.is_shutdown():
            if rospy.Time.now() - start_time > exploration_timeout:
                return None

            # See if object is in semantic map
            if (
                self.find_object_in_map(
                    goal.child_object if find_child else goal.parent_object
                )
                is not None
            ):
                _, goal_pose = self.calculate_goal_pose(goal, find_child=find_child)
                SlamController.verbose_log("Found object while exploring!")
                return goal_pose

            # Explore map
            frontier_pose = self.compute_frontier_goal_pose()
            if frontier_pose is None:
                SLEEP_SECONDS = 1
                SlamController.verbose_log(
                    f"No frontiers found. Waiting for {SLEEP_SECONDS} {'seconds' if SLEEP_SECONDS != 1 else 'second'}."
                )
                rospy.sleep(SLEEP_SECONDS)
                continue

            # Frontier found
            goto_thread: threading.Thread = self.goto(
                frontier_pose, timeout=rospy.Duration(30)
            )
            goto_thread.join()
            rospy.sleep(2)
        return None

    def compute_frontier_goal_pose(self) -> Union[Pose, None]:
        random_point = Point(random.randint(0, 4), random.random(), random.random()) 
        return self.calculate_offset(random_point)

    def navigate_to_pose(
        self, goal_pose: Pose, timeout: rospy.Duration = rospy.Duration(60)
    ) -> bool:
        """A wrapper for goto. Will go to the given pose and return if it was successful.

        Args:
            goal_pose (Pose): The pose to go to.
            timeout (rospy.Duration, optional): The amount of time given to the Turtlebot to go to the pose. Defaults to rospy.Duration(60).

        Returns:
            bool: If the TurtleBot was successful in reaching the pose (according to move_base).
        """
        goto_thread: threading.Thread = self.goto(obj=goal_pose, timeout=timeout)
        goto_thread.join()
        return self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED

    def calculate_goal_pose(
        self, goal: RobotGoal, find_child: bool = True
    ) -> Tuple[bool, Pose]:
        """Calculates the goal pose and explores if necessary.

        Args:
            goal (RobotGoal): The RobotGoal.
            find_child (bool): A flag representing if the child should be found or not. Defaults to True.

        Returns:
            Tuple (explore_flag: bool, goal_pose: Pose): explore_flag is a flag that represents if the object was not found, and exploring is needed.

        Example:
            ```
            explore_flag: bool
            goal_pose: Pose
            explore_flag, goal_pose = calculate_goal_pose(goal)
            if explore_flag:
                # Implement exploration logic here
            # goal_pose is defined at this point. Keep going...
            ```
        """
        goal_coords: Union[Point, None] = self.find_object_in_map(
            goal.child_object if find_child else goal.parent_object
        )
        if goal_coords is None:
            SlamController.verbose_log("Object not initially found. Exploring...")
            return True, Pose()  # Empty Pose object to make it not be None

        goal_pose: Pose = self.calculate_offset(goal_coords)
        return False, goal_pose

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
        obj_angle: float = atan2(
            point.y - current_pose.position.y, point.x - current_pose.position.x
        )

        inverse_direction = np.array(
            [np.cos(obj_angle + np.pi), np.sin(obj_angle + np.pi)]
        )
        # Magic number that works sorta in rviz but isn't reliable
        offset_position = np.array([point.x, point.y]) + inverse_direction * 0.5
        new_quaternion = Rotation.from_euler(
            "xyz", [0, 0, radians(obj_angle)]
        ).as_quat()

        new_pose = Pose()
        new_pose.position = Point(*offset_position, 0)
        new_pose.orientation = Quaternion(*new_quaternion)
        return new_pose

    def feedback_cb(self, feedback: MoveBaseFeedback) -> None:
        """The callback when the robot moves.

        Args:
            feedback (MoveBaseFeedback): The MoveBaseFeedback msg. It contains a `base_position` attribute.
        """
        pass

    def done_cb(self, status: int, result: MoveBaseResult) -> None:
        """The callback for when the robot has finished with it's goal

        Args:
            status (int): The status of the robot at the end. Use the `goal_statuses` dict to get a user-friendly string.
            result (MoveBaseResult): The result of the move base.
        """
        self.yield_to_master(status, result, goal_statuses)

    def goto(self, obj: Pose, timeout: rospy.Duration) -> threading.Thread:
        """Takes in a pose and a timeout time (buggy as of 2/19/2025) and attempts to go to that pose.

        Args:
            obj (Pose): The pose move_base should attempt to go to.
            timeout (rospy.Duration): The timeout for the move_base wait.

        Returns:
            threading.Thread: A thread containing the goto call.

        Example:
        ```
            goto_thread: threading.Thread = self.goto(goal_pose, timeout=rospy.Duration(timeout or 0))
            while goto_thread.is_alive():
                # Do stuff
                if you_want_to_cancel:
                    self.move_base.cancel_all_goals()
                    goto_thread.join()
        ```
        """

        def execute_goto() -> None:
            SlamController.verbose_log(f"Going to position {obj}")
            # Go towards object
            # https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-action-client-py.md
            dest = MoveBaseGoal()
            dest.target_pose.header.frame_id = "map"
            dest.target_pose.header.stamp = rospy.Time.now()
            dest.target_pose.pose = obj
            self.move_base.send_goal(
                goal=dest, feedback_cb=self.feedback_cb, done_cb=self.done_cb
            )

            start_time: rospy.Time = rospy.Time.now()
            while not rospy.is_shutdown():
                # Can be any of the actionlib.GoalStatus constants
                state: int = self.move_base.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    break

                if rospy.Time.now() - start_time > timeout and timeout != rospy.Duration(0):
                    rospy.logwarn("Slam Controller ran out of time to find object!")
                    self.move_base.cancel_all_goals()
                    break
                rospy.sleep(0.1)

            self.lock = False
            # if SlamController.verbose:
            #     SlamController.verbose_log(
            #         f"The move_base state at the end of goto was {goal_statuses.get(state)}({state})."
            #     )

        thread = threading.Thread(target=execute_goto)
        thread.start()
        return thread

    def goal_needs_update(
        self, current_goal: RobotGoal, goal_pose: Pose, going_to_child: bool = True
    ) -> bool:
        # BUG: This heuristic is very bad and simply does not work
        # find_object_in_map will return None even when it has seen an object before
        # Is this a problem with semantic_map?
        # return False # Keeping this return False will return the code to being stable
        return self.move_base.get_state() == actionlib.GoalStatus.ABORTED
        new_point = self.find_object_in_map(
            current_goal.child_object if going_to_child else current_goal.parent_object
        )

        if new_point is None:
            return False

        distance: float = sqrt(
            (goal_pose.position.x - new_point.x) ** 2
            + (goal_pose.position.y - new_point.y) ** 2
            + (goal_pose.position.z - new_point.z) ** 2
        )

        # Check if the points are not equal (aka the position has changed)
        # do_update: bool = not SlamController.are_points_equal(
        #     p1=goal_pose.position, p2=new_point, epsilon=1e-2
        # )
        threshold = 0.5
        do_update: bool = distance > threshold
        if do_update:
            rospy.sleep(1)
        return do_update

    def home(self) -> None:
        """Homes the TurtleBot back to its starting position."""
        SlamController.verbose_log("Homing to starting location...")
        HOMING_TIMEOUT_SEC: int = 60 * 1  # 1 minute
        self.move_base.cancel_all_goals()
        self.goto(self.initial_pose, timeout=rospy.Duration(secs=HOMING_TIMEOUT_SEC)).join()
        self.dummy_done_with_task()

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

    def dummy_done_with_task(self) -> None:
        """Used as a stub function for emulating that a task has finished."""
        self.yield_to_master(3, None, goal_statuses)

    @staticmethod
    def are_points_equal(p1: Point, p2: Point, epsilon: float = 1e-6) -> bool:
        """Compares if two points are equal based on their x, y, and z fields within a tolerence of epsilon.

        Args:
            p1 (Point): The first point.
            p2 (Point): The second point.
            epsilon (float, optional): The tolerance to compare the two points to. Defaults to 1e-6.

        Returns:
            bool: If the two points are equal within a tolerance
        """
        # abs(a, b) < epsilon will compare if a, b are equal within a tolerence
        return (
            abs(p1.x - p2.x) < epsilon
            and abs(p1.y - p2.y) < epsilon
            and abs(p1.z - p2.z) < epsilon
        )

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
