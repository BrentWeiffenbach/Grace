#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python
from math import atan2
from typing import Dict, List, Tuple, Union

import actionlib
import numpy as np
import rospy
from frontier_search import FrontierSearch
from geometry_msgs.msg import Point, Pose, Quaternion
from grace.msg import Object2D, Object2DArray, RobotGoalMsg, RobotState
from grace_node import GraceNode, RobotGoal, get_constants_from_msg
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseFeedback,
    MoveBaseGoal,
    MoveBaseResult,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

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

    # region GraceNavigation Init
    def __init__(self, verbose: bool = False) -> None:
        GraceNavigation.verbose = verbose
        self.goal: RobotGoal
        self.state: int
        self.semantic_map: Object2DArray

        # init FrontierSearch class to keep track of frontiers and map properties
        self.frontier_search = FrontierSearch()

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

        self.odom_sub = rospy.Subscriber(
            name="/odom", data_class=Odometry, callback=self.odom_callback
        )
        self.odom: Odometry = Odometry()

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self._map_size: float = 0.0

        self.centroid_marker_pub = rospy.Publisher(
            "/frontier/centroids", MarkerArray, queue_size=10
        )

        self.status_pub = rospy.Publisher(
            name=GraceNode.NAV_STATUS_TOPIC,
            data_class=actionlib.GoalStatus,
            queue_size=10,
        )

        self.should_update_goal: bool = True

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        GraceNavigation.verbose_log("Starting move_base action server...")
        self.move_base.wait_for_server()  # Wait until move_base server starts
        GraceNavigation.verbose_log("move_base action server started!")

    # region Callbacks
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.frontier_search.map = msg
        new_size: float = self.frontier_search.map.info.resolution
        if self._map_size != new_size:
            self._map_size = self.frontier_search.map.info.resolution
            self.should_update_goal = True

    def semantic_map_callback(self, msg: Object2DArray) -> None:
        self.semantic_map = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def goal_callback(self, msg: RobotGoalMsg) -> None:
        goal = RobotGoal(place_location=msg.place_location, pick_object=msg.pick_object)
        self.goal = goal

    def state_callback(self, msg: RobotState) -> None:
        self.state = msg.state
        if msg.state == RobotState.EXPLORING:
            self.explore()

    def done_cb(self, status: int, result: MoveBaseResult) -> None:
        """The callback for when the robot has finished with it's goal

        Args:
            status (int): The status of the robot at the end. Use the `goal_statuses` dict to get a user-friendly string.
            result (MoveBaseResult): The result of the move base.
        """
        self.publish_status(status)
        if status == actionlib.GoalStatus.ABORTED:
            ...

    def no_aborted_cb(self, status: int, result: MoveBaseResult) -> None:
        # Does not publish the state unlike done_cb
        # Will ensure that the state is not aborted
        if status == actionlib.GoalStatus.ABORTED:
            ...

    def feedback_cb(self, feedback: MoveBaseFeedback) -> None:
        """The callback when the robot moves.

        Args:
            feedback (MoveBaseFeedback): The MoveBaseFeedback msg. It contains a `base_position` attribute.
        """
        pass

    # region Publishers
    def publish_status(self, status: int) -> None:
        dummy_goal_status = actionlib.GoalStatus(None, status, None)
        self.status_pub.publish(dummy_goal_status)

    def publish_timeout(self) -> None:
        # Publish LOST as the state since it will never naturally be published
        self.publish_status(actionlib.GoalStatus.LOST)

    def publish_markers(
        self,
        keypoints: List[Point],
        namespace: str = "frontiers",
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
    ):
        marker_array = MarkerArray()
        marker_array.markers = []
        marker_id = 0

        for k in keypoints:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = marker_id

            if namespace == "Object_Goal":
                marker.type = Marker.TEXT_VIEW_FACING
                has_object = rospy.wait_for_message(
                    topic=GraceNode.HAS_OBJECT_TOPIC,
                    topic_type=Bool,
                    timeout=rospy.Duration(10),
                )

                assert has_object
                has_object = has_object.data
                target_obj_name: str = (
                    self.goal.place_location if has_object else self.goal.pick_object
                )

                marker.text = target_obj_name
            else:
                marker.type = Marker.SPHERE

            marker.action = Marker.ADD

            marker.pose.position = k

            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.lifetime = rospy.Duration(0)

            marker_array.markers.append(marker)
            marker_id += 1
        self.centroid_marker_pub.publish(marker_array)

    # region Exploration
    def explore(self) -> None:
        assert self.goal

        EXPLORE_SECONDS = 180
        rospy.loginfo("Exploring")
        at_goal = self.explore_until_found(
            exploration_timeout=rospy.Duration(EXPLORE_SECONDS),
        )
        if at_goal is None:
            return

        if not at_goal:
            rospy.logwarn("Failed to navigate to the object!")
            self.publish_status(actionlib.GoalStatus.ABORTED)
            return

        rospy.loginfo("\033[92mReached object!\033[0m")
        self.move_base.cancel_all_goals()
        self.publish_status(actionlib.GoalStatus.SUCCEEDED)
        return

    def explore_until_found(
        self,
        exploration_timeout: rospy.Duration = rospy.Duration(60),
    ) -> Union[bool, None]:
        assert self.goal

        # Give semantic map time to load
        is_semantic_map_loaded: bool = self.wait_for_semantic_map(sleep_time_s=10)
        if not is_semantic_map_loaded:
            rospy.logfatal("SEMANTIC MAP WAS NOT LOADED")
            return False

        start_time: rospy.Time = rospy.Time.now()
        exploration_timeout = max(exploration_timeout, rospy.Duration(0))
        has_object = rospy.wait_for_message(
            topic=GraceNode.HAS_OBJECT_TOPIC,
            topic_type=Bool,
            timeout=rospy.Duration(10),
        )

        assert has_object
        has_object = has_object.data
        target_obj_name: str = (
            self.goal.place_location if has_object else self.goal.pick_object
        )

        navigating_to_object = False
        last_check_time: rospy.Time = rospy.Time.now()
        check_interval = rospy.Duration(1)

        while not rospy.is_shutdown() and self.state == RobotState.EXPLORING:
            current_time: rospy.Time = rospy.Time.now()
            if current_time - start_time > exploration_timeout:
                self.publish_timeout()
                return False

            if self.state != RobotState.EXPLORING:
                GraceNavigation.verbose_log("Canceling all goals")
                self.move_base.cancel_all_goals()
                return False

            if current_time - last_check_time > check_interval:
                last_check_time = current_time
                obj_point = self.find_object_in_semantic_map(target_obj_name)

                if obj_point is not None:
                    # Found object
                    GraceNavigation.verbose_log(
                        f"Found {target_obj_name} in semantic map"
                    )

                    goal_pose: Pose = self.calculate_offset(obj_point)
                    self.publish_markers(
                        [goal_pose.position], "Object_Goal", color=(0, 0, 1)
                    )

                    # Goal is in occupancy grid
                    if self.frontier_search.is_pose_in_occupancy_grid(goal_pose):
                        GraceNavigation.verbose_log(
                            f"Goal for {target_obj_name} is accessible, navigating to it"
                        )
                        navigating_to_object = True

                        self.move_base.cancel_all_goals()

                        self.goto(
                            goal_pose, timeout=rospy.Duration(30), yield_when_done=True
                        )

                        current_pose = self.get_current_pose()
                        if GraceNavigation.are_points_equal(
                            goal_pose.position, current_pose.position, epsilon=0.2
                        ):
                            GraceNavigation.verbose_log(
                                "Successfully reached the object!"
                            )
                            return True

                        distance = np.linalg.norm(
                            [
                                goal_pose.position.x - current_pose.position.x,
                                goal_pose.position.y - current_pose.position.y,
                            ]
                        )
                        if distance < 0.25:
                            GraceNavigation.verbose_log("Close enough to the object!")
                            return True

                        rospy.sleep(1)
                        continue
                    else:
                        GraceNavigation.verbose_log(
                            f"Goal for {target_obj_name} is not accessible yet"
                        )
                        navigating_to_object = False

            if not navigating_to_object:
                frontier_pose = self.compute_frontier_goal_pose(
                    heuristic_pose=self.get_current_pose(), publish_markers=True
                )

                if frontier_pose is None:
                    GraceNavigation.verbose_log("No frontiers found, waiting...")
                    rospy.sleep(2)
                    continue

                if self.should_update_goal or self.move_base.get_state() not in [
                    actionlib.GoalStatus.ACTIVE,
                    actionlib.GoalStatus.PENDING,
                ]:
                    GraceNavigation.verbose_log(
                        "Navigating to a frontier for exploration"
                    )
                    self.should_update_goal = False
                    self.goto(
                        self.create_navigable_goal(frontier_pose),
                        timeout=rospy.Duration(30),
                        yield_when_done=False,
                    )

            rospy.sleep(0.5)

        return None

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
            done_cb=self.done_cb if yield_when_done else self.no_aborted_cb,
            feedback_cb=self.feedback_cb,
        )

    def find_object_in_semantic_map(self, obj: str) -> Union[Point, None]:
        assert self.semantic_map  # I do not want to deal with this not being initialized # fmt: skip
        assert self.semantic_map.objects
        reversed_objects = self.semantic_map.objects[::-1]
        for map_obj in reversed_objects:
            map_obj: Object2D
            if map_obj.cls == obj:
                return Point(map_obj.x, map_obj.y, 0)
        return None

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
        result = result or rospy.wait_for_message(
            "/semantic_map", Object2DArray, timeout=sleep_time_s
        )
        if not result:
            rospy.logerr(
                f"Grace Navigation could not receive a map after waiting for {sleep_time_s} seconds."
            )
            return False
        return True

    # region Frontiers
    def score_frontiers(
        self, frontiers: List[Point], target_pose: Union[Pose, None] = None
    ) -> Union[Point, None]:
        if not frontiers:
            return None

        current_pose: Pose = self.get_current_pose()
        current_position = np.array([current_pose.position.x, current_pose.position.y])

        MIN_DISTANCE = 1.0
        MAX_DISTANCE = 15.0

        scored_frontiers: List[Tuple[Point, Union[np.floating, float]]] = []

        for frontier in frontiers:
            frontier_pos = np.array([frontier.x, frontier.y])
            distance: np.floating = np.linalg.norm(frontier_pos - current_position)

            if distance < MIN_DISTANCE:
                continue

            if distance > MAX_DISTANCE:
                continue

            # Basic score is inverse of distance
            score: Union[np.floating, float] = 1.0 / max(distance, 0.1)

            if target_pose is not None:
                target_position = np.array(
                    [target_pose.position.x, target_pose.position.y]
                )
                robot_to_target = target_position - current_position
                robot_to_frontier = frontier_pos - current_position

                if (
                    np.linalg.norm(robot_to_target) > 0
                    and np.linalg.norm(robot_to_frontier) > 0
                ):
                    cos_angle = np.dot(robot_to_target, robot_to_frontier) / (
                        np.linalg.norm(robot_to_target)
                        * np.linalg.norm(robot_to_frontier)
                    )
                    direction_factor = (cos_angle + 1) / 2
                    score *= 1 + 2 * direction_factor

            scored_frontiers.append((frontier, score))

        # Sort by score (highest first)
        scored_frontiers.sort(key=lambda x: x[1], reverse=True)

        if scored_frontiers:
            return scored_frontiers[0][0]
        return None

    def compute_frontier_goal_pose(
        self, heuristic_pose: Pose, publish_markers: bool = False
    ) -> Union[Pose, None]:
        keypoints: List[Point] = self.frontier_search.compute_centroids()
        if not keypoints:
            return None

        best_frontier = self.score_frontiers(keypoints, heuristic_pose)
        self.publish_markers(
            keypoints=keypoints,
            namespace="frontiers",
            color=(0.5, 0.1, 0.1),
        )
        if best_frontier is None:
            return None
        self.publish_markers(
            keypoints=[best_frontier],
            namespace="best frontier",
            color=(0.0, 1.0, 0.0),
        )
        best_frontier_pose = Pose()
        best_frontier_pose.position = best_frontier

        # Calculate orientation to face the object
        direction_to_object: Quaternion = self.calculate_direction_towards_object(
            best_frontier
        )
        best_frontier_pose.orientation = direction_to_object

        return best_frontier_pose

    # region Calculations

    def calculate_offset(self, point: Point) -> Pose:
        # point is in map coordinates
        # get current pose is from /odom
        offset_position = np.array([point.x, point.y]) * 0.8

        new_pose = Pose()
        new_pose.position = Point(*offset_position, 0)
        new_pose.orientation = self.calculate_direction_towards_object(point)

        return new_pose

    def calculate_direction_towards_object(self, point: Point) -> Quaternion:
        current_pose: Pose = self.get_current_pose()
        obj_angle: float = atan2(
            point.y - current_pose.position.y, point.x - current_pose.position.x
        )
        new_quaternion = Rotation.from_euler("xyz", [0, 0, obj_angle]).as_quat()
        return Quaternion(*new_quaternion)

    def create_navigable_goal(self, goal: Pose, safety_margin: int = 4) -> Pose:
        resolution = self.frontier_search.map.info.resolution
        origin = self.frontier_search.map.info.origin.position

        grid = np.array(self.frontier_search.map.data).reshape(
            (self.frontier_search.map.info.height, self.frontier_search.map.info.width)
        )

        goal_x = int((goal.position.x - origin.x) / resolution)
        goal_y = int((goal.position.y - origin.y) / resolution)

        local_size = safety_margin * 2 + 1
        local_map = np.ones((local_size, local_size)) * 100

        # Fill in the local map from the global map
        for dy in range(-safety_margin, safety_margin + 1):
            for dx in range(-safety_margin, safety_margin + 1):
                map_x = goal_x + dx
                map_y = goal_y + dy

                if 0 <= map_x < grid.shape[1] and 0 <= map_y < grid.shape[0]:
                    local_map[dy + safety_margin, dx + safety_margin] = grid[
                        map_y, map_x
                    ]

        best_cost = float("inf")
        best_point = (goal_x, goal_y)

        for dy in range(-safety_margin, safety_margin + 1):
            for dx in range(-safety_margin, safety_margin + 1):
                map_x: int = goal_x + dx
                map_y: int = goal_y + dy

                if 0 <= map_x < grid.shape[1] and 0 <= map_y < grid.shape[0]:
                    # Check if there's free space
                    if grid[map_y, map_x] == 0:
                        # Calculate distance from original goal
                        cost: int = dx * dx + dy * dy

                        # Check if it's safe
                        is_safe = True
                        for sy in range(-2, 3):
                            for sx in range(-2, 3):
                                test_x: int = map_x + sx
                                test_y: int = map_y + sy
                                if (
                                    0 <= test_x < grid.shape[1]
                                    and 0 <= test_y < grid.shape[0]
                                ):
                                    if grid[test_y, test_x] > 0:
                                        is_safe = False
                                        break
                            if not is_safe:
                                break

                        if is_safe and cost < best_cost:
                            best_cost = cost
                            best_point = (map_x, map_y)

        # Create the adjusted goal
        adjusted_goal = Pose()
        adjusted_goal.position.x = best_point[0] * resolution + origin.x
        adjusted_goal.position.y = best_point[1] * resolution + origin.y
        adjusted_goal.position.z = goal.position.z
        adjusted_goal.orientation = goal.orientation

        self.publish_markers([adjusted_goal.position], "adjusted", (0, 1, 0))
        return adjusted_goal

    def get_current_pose(self) -> Pose:
        """Returns the current pose of the turtlebot by subscribing to the `/odom`.

        Returns:
            Pose: The turtlebot's current Pose.
        """

        assert self.odom
        pose: Pose = self.odom.pose.pose
        return pose

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

    # region Run Node
    def run(self) -> None:
        rospy.spin()

    def shutdown(self) -> None:
        GraceNavigation.verbose_log("Shutting down GraceNavigation.")
        self.move_base.cancel_all_goals()


if __name__ == "__main__":
    rospy.init_node("grace_navigation")
    verbose = rospy.get_param("~verbose", False)
    assert type(verbose) is bool
    grace_navigation = GraceNavigation(verbose=verbose)
    rospy.on_shutdown(grace_navigation.shutdown)
    rospy.sleep(5)
    try:
        grace_navigation.run()
    except rospy.ROSInterruptException:
        grace_navigation.shutdown()
        rospy.loginfo("GraceNavigation shut down.")
