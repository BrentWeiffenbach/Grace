#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python


import random
from math import atan2, radians
from typing import Callable, Dict, Tuple, Union

import actionlib
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from grace.msg import Object2D, Object2DArray, RobotGoalMsg, RobotState
from grace_node import GraceNode, RobotGoal, get_constants_from_msg
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseResult,
)
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool

goal_statuses: Dict[int, str] = get_constants_from_msg(actionlib.GoalStatus)
"""Gets all of the non-callable integer constants from actionlib.GoalStatus msg. """


class GraceNavigation:
    verbose: bool = False

    @staticmethod
    def verbose_log(log: str) -> None:
        """Logs the input to the console if `GraceNavigation.verbose` is True. Also uses `rospy.logdebug()`.

        Args:
            log (str): The log to print conditionally.
        """
        if GraceNavigation.verbose:
            rospy.loginfo(log)
        rospy.logdebug(log)

    def __init__(self, verbose: bool = False) -> None:
        GraceNavigation.verbose = verbose
        self.goal: RobotGoal
        self.state: int
        self.semantic_map: Object2DArray

        self.yield_to_master: Callable = (
            lambda: rospy.logfatal("yield_to_master is not actually implemented!")
        )  # TODO: Remove this and make it use ROS topics instead of trying to go to yield to grace_node

        self.semantic_map_sub = rospy.Subscriber(
            name="/semantic_map",
            data_class=Object2DArray,
            callback=self.semantic_map_callback,
        )

        self.goal_sub = rospy.Subscriber(
            name=GraceNode.GOAL_TOPIC,
            data_class=RobotGoalMsg,
            callback=self.goal_callback,
        )

        self.state_sub = rospy.Subscriber(
            name=GraceNode.STATE_TOPIC,
            data_class=RobotState,
            callback=self.state_callback,
        )

        # self.has_object_sub = rospy.Subscriber(name=has_object_TOPIC, data_class=Bool, callback=self.has_object_)

        self.est_goal_pub = rospy.Publisher(
            name="/semantic_map/est_goal_pose", data_class=Pose, queue_size=10
        )

        self.status_pub = rospy.Publisher(
            name=GraceNode.NAV_STATUS_TOPIC,
            data_class=actionlib.GoalStatus,
            queue_size=10,
        )

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        GraceNavigation.verbose_log("Starting move_base action server...")
        self.move_base.wait_for_server()  # Wait until move_base server starts
        GraceNavigation.verbose_log("move_base action server started!")

    def semantic_map_callback(self, msg: Object2DArray) -> None:
        self.semantic_map = msg

    def goal_callback(self, msg: RobotGoalMsg) -> None:
        goal = RobotGoal(place_location=msg.place_location, pick_object=msg.pick_object)
        self.goal = goal

    def state_callback(self, msg: RobotState) -> None:
        self.state = msg.state
        if msg.state == RobotState.EXPLORING:
            self.explore()

    def publish_status(self, status: int) -> None:
        # Publish LOST as the state since it will never naturally be published
        dummy_goal_status = actionlib.GoalStatus(None, status, None)
        self.status_pub.publish(dummy_goal_status)

    def publish_timeout(self) -> None:
        # Publish LOST as the state since it will never naturally be published
        self.publish_status(actionlib.GoalStatus.LOST)

    def explore(self) -> None:
        assert self.goal
        EXPLORE_SECONDS = 60
        rospy.loginfo("Exploring")
        found_pose = self.explore_until_found(
            exploration_timeout=rospy.Duration(EXPLORE_SECONDS),
        )
        if found_pose is None:
            rospy.logwarn("Exploration timed out without finding the object!")
            self.publish_timeout()
            return

        success: bool = self.navigate_to_pose(found_pose, timeout=rospy.Duration(100))
        if success:
            self.publish_status(actionlib.GoalStatus.SUCCEEDED)
            return

        rospy.logwarn("Failed to navigate to the object!")
        self.publish_timeout()

    def explore_until_found(
        self,
        exploration_timeout: rospy.Duration = rospy.Duration(60),
    ) -> Union[Pose, None]:
        assert self.goal
        # Give semantic map time to sleep
        is_semantic_map_loaded: bool = self.wait_for_semantic_map(sleep_time_s=10)
        if not is_semantic_map_loaded:
            return None

        start_time: rospy.Time = rospy.Time.now()
        exploration_timeout = max(exploration_timeout, rospy.Duration(0))

        has_object = rospy.wait_for_message(
            topic=GraceNode.HAS_OBJECT_TOPIC,
            topic_type=Bool,
            timeout=rospy.Duration(10),
        )

        assert has_object

        has_object = has_object.data

        while not rospy.is_shutdown():
            if self.state != RobotState.EXPLORING:
                self.move_base.cancel_all_goals()
                break

            if rospy.Time.now() - start_time > exploration_timeout:
                break

            # See if object is in semantic map
            if (
                self.find_object_in_map(
                    self.goal.place_location if has_object else self.goal.pick_object
                )
                is not None
            ):
                rospy.loginfo(f"Has object: {has_object}")
                _, goal_pose = self.calculate_goal_pose(has_object)
                GraceNavigation.verbose_log(
                    f"Found {self.goal.place_location if has_object else self.goal.pick_object} while exploring!"
                )
                return goal_pose

            # Explore map
            frontier_pose = self.compute_frontier_goal_pose()
            if frontier_pose is None:
                SLEEP_SECONDS = 1
                GraceNavigation.verbose_log(
                    f"No frontiers found. Waiting for {SLEEP_SECONDS} {'seconds' if SLEEP_SECONDS != 1 else 'second'}."
                )
                rospy.sleep(SLEEP_SECONDS)
                continue

            # Frontier found
            self.goto(frontier_pose, timeout=rospy.Duration(30), yield_when_done=False)
            rospy.sleep(2)
        return None

    def compute_frontier_goal_pose(self) -> Union[Pose, None]:
        return None
        random_point = Point(random.randint(0, 4), random.random(), random.random())
        rospy.loginfo("Using random point as substitute for frontier")
        return self.calculate_offset(random_point)

    def frontier_explore_to_object(self, obj_pose: Pose) -> Pose:
        # Tells frontier_search to go in the direction of the goal pose
        # Stops frontier_search when the goal pose is within the costmap
        # TODO: Decide whether to implement this or not
        # It might be better to implement this somewhere else
        ...

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
        self.est_goal_pub.publish(goal_pose)
        self.goto(obj=goal_pose, timeout=timeout)
        # BUG: move_base.get_state here can still return the previous goal's get_state
        return self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED

    def calculate_goal_pose(self, has_object: bool) -> Tuple[bool, Pose]:
        """Calculates the goal pose and explores if necessary.

        Args:
            has_object (bool): A flag representing if the child should be found or not.

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
            self.goal.place_location if has_object else self.goal.pick_object
        )
        if goal_coords is None:
            GraceNavigation.verbose_log("Object not initially found. Exploring...")
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

    def done_cb(self, status: int, result: MoveBaseResult) -> None:
        """The callback for when the robot has finished with it's goal

        Args:
            status (int): The status of the robot at the end. Use the `goal_statuses` dict to get a user-friendly string.
            result (MoveBaseResult): The result of the move base.
        """
        self.publish_status(status)

    def goto(
        self, obj: Pose, timeout: rospy.Duration, yield_when_done: bool = True
    ) -> None:
        """Takes in a pose and a timeout time (buggy as of 2/19/2025) and attempts to go to that pose.

        Args:
            obj (Pose): The pose move_base should attempt to go to.
            timeout (rospy.Duration): The timeout for the move_base wait.
            yield_when_done (bool, optional): Whether goto should yield to grace_node (call done_cb) when it is done. Defaults to True.
        """
        # Go towards object
        dest = MoveBaseGoal()
        dest.target_pose.header.frame_id = "map"
        dest.target_pose.header.stamp = rospy.Time.now()
        dest.target_pose.pose = obj
        self.move_base.send_goal(
            goal=dest,
            done_cb=self.done_cb if yield_when_done else None,
        )

        start_time: rospy.Time = rospy.Time.now()
        while not rospy.is_shutdown():
            # Can be any of the actionlib.GoalStatus constants
            state: int = self.move_base.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                break

            if rospy.Time.now() - start_time > timeout and timeout != rospy.Duration(0):
                rospy.logwarn("Grace Navigator ran out of time to find object!")
                self.move_base.cancel_all_goals()
                break
            rospy.sleep(0.1)

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
                f"Grace Navigation could not receive a map after waiting for {sleep_time_s} seconds."
            )
            return False
        return True

    def dummy_done_with_task(self) -> None:
        """Used as a stub function for emulating that a task has finished."""
        self.publish_status(3)

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
        GraceNavigation.verbose_log("Shutting down GraceNavigation.")
        self.move_base.cancel_all_goals()


if __name__ == "__main__":
    rospy.init_node("grace_navigation")
    grace_navigation = GraceNavigation(verbose=True)
    rospy.on_shutdown(grace_navigation.shutdown)
    try:
        grace_navigation.run()
    except rospy.ROSInterruptException:
        grace_navigation.shutdown()
        rospy.loginfo("GraceNavigation shut down.")
