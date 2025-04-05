import queue
from math import atan2
from typing import Dict, List, Tuple, Union, Callable

import actionlib
import numpy as np
import rospy
from frontier_search import FrontierSearch
from geometry_msgs.msg import Point, Pose, Quaternion
from grace_navigation.msg import RangeBearing, RangeBearingArray, Object2D
from grace_node import GraceNode, RobotGoal, get_constants_from_msg, rotate_360
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseResult,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Int16
from visualization_msgs.msg import Marker, MarkerArray

from grace_navigation.msg import Object2DArray, RobotGoalMsg, RobotState

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
        self.goal_pose: Union[Pose, None] = None
        self.state: int
        self.semantic_map: Object2DArray = Object2DArray()
        self.should_update_goal: bool = True
        self.done_flag: bool = False
        self.odom: Odometry = Odometry()  # Initialize to empty odom

        # init FrontierSearch class to keep track of frontiers and map properties
        self.frontier_search = FrontierSearch()

        self.remove_pub = rospy.Publisher("/semantic_map/remove", Int16, queue_size=10)

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

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self._map_size: float = 0.0
        """The size of the map. Used for determining if the goal should be updated."""
        self.global_costmap_sub = rospy.Subscriber(
            "/move_base/global_costmap/costmap",
            OccupancyGrid,
            self.global_costmap_initial_cb,
        )
        self.global_sub = rospy.Subscriber(
            "/move_base/global_costmap/costmap_updates",
            OccupancyGridUpdate,
            self.global_cb,
            tcp_nodelay=True,
        )

        self.centroid_marker_pub = rospy.Publisher(
            "/frontier/centroids", MarkerArray, queue_size=10
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

        self.last_goal: Pose = self.get_current_pose()

    # region Callbacks
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.frontier_search.map = msg
        new_size: float = self.frontier_search.map.info.resolution
        if self._map_size != new_size:
            self._map_size = self.frontier_search.map.info.resolution
            self.should_update_goal = True

    def global_costmap_initial_cb(self, msg: OccupancyGrid) -> None:
        rospy.loginfo(
            f"Initial Global Costmap: width={msg.info.width}, height={msg.info.height}, "
            f"resolution={msg.info.resolution}"
        )
        self.frontier_search._global_costmap = msg
        # Unsubscribe immediately after receiving the first message
        self.global_costmap_sub.unregister()

    def global_cb(self, msg: OccupancyGridUpdate) -> None:
        self.frontier_search.global_map_cb(msg)

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
            try:
                self.goal
            except AttributeError:
                goal_msg = rospy.wait_for_message(
                    GraceNode.GOAL_TOPIC, RobotGoalMsg, timeout=10
                )
                self.goal_callback(goal_msg)  # type: ignore
                self.explore()
                return

            self.explore()

    # region Publishers
    def publish_status(self, status: int) -> None:
        """Publishes the current status to the status topic.

        Args:
            status (int): The status to be published.
        """
        dummy_goal_status = actionlib.GoalStatus()
        dummy_goal_status.status = status
        self.status_pub.publish(dummy_goal_status)

    def publish_timeout(self) -> None:
        # Publish LOST as the state since it will never naturally be published
        self.publish_status(actionlib.GoalStatus.LOST)

    def publish_markers(
        self,
        keypoints: List[Point],
        namespace: str = "frontiers",
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
        scale: Tuple[float, float, float] = (0.15, 0.15, 0.15),
    ):
        marker_array = MarkerArray()
        marker_array.markers = []
        marker_id = 0
        has_object = rospy.wait_for_message(
            topic=GraceNode.HAS_OBJECT_TOPIC,
            topic_type=Bool,
            timeout=rospy.Duration(10),
        )
        if has_object is None:
            if GraceNavigation.verbose:
                rospy.logwarn(
                    "Failed to receive message for has_object. Defaulting to False."
                )
            has_object = False
        else:
            has_object = has_object.data

        for k in keypoints:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = marker_id

            if namespace == "Object_Goal" or namespace == "Object_Goal_Offset":
                marker.type = Marker.TEXT_VIEW_FACING
                target_obj_name: str = (
                    self.goal.place_location if has_object else self.goal.pick_object
                )

                marker.text = target_obj_name
            else:
                marker.type = Marker.SPHERE

            marker.action = Marker.ADD

            marker.pose.position = k

            marker.scale.x = scale[0]
            marker.scale.y = scale[1]
            marker.scale.z = scale[2]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.lifetime = rospy.Duration(0)

            marker_array.markers.append(marker)
            marker_id += 1
        self.centroid_marker_pub.publish(marker_array)

    def publish_labled_markers(
        self,
        keypoints: List[Tuple[Point, Union[np.floating, float]]],
        namespace: str = "frontiers",
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
    ):
        marker_array = MarkerArray()
        marker_array.markers = []
        marker_id = 0

        for k in keypoints:
            pos, score = k
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.text = f"Score: {round(score, 2)}"
            marker.action = Marker.ADD

            marker.pose.position = pos

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
        # rospy.Publisher(
        #     "/frontier/centroids", MarkerArray, queue_size=10
        # ).publish(marker_array)

    # region Exploration
    def explore(self) -> None:
        # BUG (?): self.goal can possibly be undefined. It is difficult to reproduce how it happened.

        EXPLORE_SECONDS = 20 * 60  # 20 minutes
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

        rospy.loginfo("Reached object!")
        self.move_base.cancel_all_goals()
        self.publish_status(actionlib.GoalStatus.SUCCEEDED)
        return

    def explore_until_found(
        self,
        exploration_timeout: rospy.Duration = rospy.Duration(60),
    ) -> Union[bool, None]:
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
        assert self.goal
        has_object = has_object.data
        target_obj_name: str = (
            self.goal.place_location if has_object else self.goal.pick_object
        )

        navigating_to_object = False

        while not rospy.is_shutdown() and self.state == RobotState.EXPLORING:
            current_time: rospy.Time = rospy.Time.now()
            if current_time - start_time > exploration_timeout:
                self.publish_timeout()
                return False

            if self.state != RobotState.EXPLORING:
                GraceNavigation.verbose_log("Canceling all goals")
                self.move_base.cancel_all_goals()
                return False

            if self.done_flag:
                self.done_flag = False
                self.goal_pose = None
                return True

            obj_point = self.find_object_in_semantic_map(target_obj_name)
            if obj_point is not None:
                self.goal_pose: Union[Pose, None] = self.calculate_offset(obj_point)

            if self.goal_pose is not None:
                # self.publish_markers(
                #     [self.goal_pose.position], "Object_Goal_Offset", color=(0, 0, 1)
                # )
                # Found object
                # Goal is in occupancy grid
                navigating_to_object = True

                if self.done_flag:
                    self.done_flag = False
                    self.goal_pose = None
                    return True

                if self.last_goal != self.goal_pose:
                    if GraceNavigation.verbose:
                        rospy.loginfo_throttle(
                            1,
                            f"Goal for {target_obj_name} is accessible, navigating to it",
                        )

                self.goto(
                    self.goal_pose,
                    yield_when_done=True,
                )

                self.goal_pose = None

                continue
            else:
                GraceNavigation.verbose_log("Goal pose is none!")
                navigating_to_object = False

            if not navigating_to_object:
                # Go to frontier
                frontier_pose = self.compute_frontier_goal_pose(
                    heuristic_pose=self.goal_pose
                    if self.goal_pose is not None
                    else self.get_current_pose()
                )

                if frontier_pose is None:
                    GraceNavigation.verbose_log("No frontiers found, waiting...")
                    rotate_360()
                    rospy.sleep(2)
                    continue

                if self.should_update_goal or self.move_base.get_state() not in [
                    actionlib.GoalStatus.PENDING
                ]:
                    GraceNavigation.verbose_log(
                        "Navigating to a frontier for exploration"
                    )
                    self.should_update_goal = False

                    def navigable_heuristic(
                        poses_list, current_position, original_goal
                    ):
                        # return min(
                        #     poses_list,
                        #     key=lambda pose: np.linalg.norm(
                        #         np.array([pose.position.x, pose.position.y]) - current_position
                        #     ),
                        # )
                        return poses_list[0]

                    offset_pose = self.calculate_offset(
                        frontier_pose.position, navigable_heuristic
                    )
                    if offset_pose is not None:
                        offset_pose.orientation = (
                            self.calculate_direction_towards_object(
                                offset_pose.position
                            )
                        )
                    self.goto(
                        offset_pose if offset_pose is not None else frontier_pose,
                        yield_when_done=False,
                    )
                    # Commented because in sim it does not go to the object immediately but instead continues going to a frontier
                    # rospy.sleep(
                    #     2
                    # )  # Increase to increase time between updating which frontier should be navigated to

            rospy.sleep(1)  # Originally 0.5

        return None

    def goto(self, obj: Pose, yield_when_done: bool = True) -> None:
        """Takes in a pose and a timeout time (buggy as of 2/19/2025) and attempts to go to that pose.

        Args:
            obj (Pose): The pose move_base should attempt to go to.
            yield_when_done (bool, optional): Whether goto should yield to grace_node (call done_cb) when it is done. Defaults to True.
        """
        # Get distacne from robot to goal
        current_pose = self.get_current_pose()
        pose_distance = np.linalg.norm(
            np.array([current_pose.position.x, current_pose.position.y])
            - np.array([obj.position.x, obj.position.y])
        )
        # distance from last to current goal
        last_goal_distance = np.linalg.norm(
            np.array([self.last_goal.position.x, self.last_goal.position.y])
            - np.array([obj.position.x, obj.position.y])
        )
        # if either are too close, don't goto
        # # BUG: If two goal semantic objects for pick and palce are right next to each other this doesnt work
        THRESHOLD = 0.3  # TODO: Tune this
        # if pose_distance <= THRESHOLD:
        #     if GraceNavigation.verbose:
        #         rospy.logwarn_throttle(
        #             1, "Robot is already within the threshold distance to the goal."
        #         )
        #     return
        if (
            last_goal_distance <= THRESHOLD
            and self.move_base.get_state() != actionlib.GoalStatus.SUCCEEDED
        ):  # tries to prevent not publishing a goal ever again if a frontier succeeds (or 2d nav goal)
            if GraceNavigation.verbose:
                rospy.logwarn_throttle(1, "Robot tried to goto the same goal.")
            return

        self.last_goal = obj
        GraceNavigation.verbose_log("Publishing goal to MoveBase")
        dest = MoveBaseGoal()
        dest.target_pose.header.frame_id = "map"
        dest.target_pose.header.stamp = rospy.Time.now()
        dest.target_pose.pose = obj
        self.move_base.send_goal(
            goal=dest,
            done_cb=self.done_cb if yield_when_done else self.no_yield_done_cb,
        )
        # if yield_when_done:
        rospy.sleep(2)
        # rememebr last goto goal

    def yolo_final_check(self) -> bool:
        def exit_final_check():
            self.should_update_goal = True
            self.done_flag = False
            # self.last_goal = self.get_current_pose()
            self.remove_pub.publish(self.goal_obj_id)
        # Only be done if the yolo object is visible at the goal
        # Subscribe to /range_bearing
        try:
            detections = rospy.wait_for_message(
                "/range_bearing", RangeBearingArray, timeout=3
            )
        except rospy.ROSException:
            rospy.logwarn("Timeout waiting for YOLO detections in done_cb")
            exit_final_check()
            return False

        has_object = rospy.wait_for_message(
            topic=GraceNode.HAS_OBJECT_TOPIC,
            topic_type=Bool,
            timeout=rospy.Duration(10),
        )
        if has_object is None:
            rospy.logwarn(
                "Failed to receive message for has_object. Defaulting to False."
            )
            has_object = False
        else:
            has_object = has_object.data

        target_obj_name: str = (
            self.goal.place_location if has_object else self.goal.pick_object
        )

        assert isinstance(detections, RangeBearingArray)  # type: ignore

        # check if any detections are the target class
        if detections.range_bearings is None:
            rospy.logwarn("No detections received in done_cb.")
            exit_final_check()
            return False

        for detection in detections.range_bearings:
            detection: RangeBearing
            if detection.obj_class == target_obj_name:
                rospy.loginfo("YOLO final check worked!")
                return True
            else:
                rospy.logwarn("No detections with goal class at goal.")
                exit_final_check()
                return False
        return False

    def done_cb(self, status: int, _: MoveBaseResult) -> None:
        """The callback for when the robot has finished with it's goal

        Args:
            status (int): The status of the robot at the end. Use the `goal_statuses` dict to get a user-friendly string.
            _ (MoveBaseResult): The result of the move base. Ignored
        """
        # TODO: If final check fails, do something to not be here anymore
        if not self.yolo_final_check():
            rospy.loginfo("Final check failed, returning")
            return
        self.done_flag = False
        self.goal_pose = None
        if goal_statuses[status] not in ["PREEMPTED"]:
            self.publish_status(status)
        if status == actionlib.GoalStatus.ABORTED:
            self.should_update_goal = True
        if goal_statuses[status] in ["SUCCEEDED"]:
            self.done_flag = True

    def no_yield_done_cb(self, status: int, _: MoveBaseResult) -> None:
        self.goal_pose = None
        if goal_statuses[status] in ["SUCCEEDED"]:
            self.should_update_goal = True

    def find_object_in_semantic_map(self, obj: str) -> Union[Point, None]:
        """
        Returns:
            Union (Point, None): The object found's coordinates, or None if it was not found. In map coordinates.
        """
        assert self.semantic_map  # I do not want to deal with this not being initialized # fmt: skip
        if self.semantic_map.objects is None or len(self.semantic_map.objects) == 0:
            GraceNavigation.verbose_log(
                "self.semantic_map.object has no objects. Returning none"
            )
            return None

        cls_objects: List[Object2D] = [
            map_obj for map_obj in self.semantic_map.objects if map_obj.cls == obj
        ]

        # DEBUG:
        self.publish_markers(
            [Point(class_obj.x, class_obj.y, 0) for class_obj in cls_objects],
            namespace="semantic_objects",
            color=(0.0, 1.0, 0.7),
        )

        if not cls_objects or len(cls_objects) == 0:
            if GraceNavigation.verbose:
                rospy.logwarn(f"No objects with class: {obj}")
            return None

        in_occupancy_grid_objects = []
        # too_close_objects # Too close to object could potentially continuially remove the goal pose when approaching making it impossible to actually arrive

        # loop through objs to check if they are closest and 'NOT navigable' / in occupancy grid
        for class_obj in cls_objects:
            # closest_obj = min(
            #     cls_objects,
            #     key=lambda obj: np.linalg.norm(np.array([obj.x, obj.y]) - current_position),
            # )
            obj_point = Point(class_obj.x, class_obj.y, 0)  # type: ignore
            self.goal_obj_id = class_obj.id
            rospy.loginfo(f"Setting goal_obj_id to {self.goal_obj_id}")
            # YOU COULD maybe approximate the closest object location after seeing it is in occupancy grid, to effectivly calculate_offset inversely to find a good semantic location
            # What two back-to-back all nighters does to a guy ^

            # Commented from cherry picked commit because it makes the robot not go to reasonable goals sometimes
            if self.frontier_search.is_point_in_occupancy_grid(obj_point, True):
                # most likely a haclucination if not inside the inflation layer
                # is_point_in_occupancy_grid returns True if it is reachable by turtlebot
                GraceNavigation.verbose_log(
                    "DEBUG: This semantic object was in occupancy grid, most likely a hallucination."
                )
                # remove object from cls_objects
                cls_objects.remove(class_obj)
                in_occupancy_grid_objects.append(obj_point)
                self.remove_pub.publish(class_obj.id)
                continue
            else:
                # self.goal_pose: Union[Pose, None] = self.calculate_offset(obj_point)
                # self.publish_markers([obj_point], "Object_Goal", color=(0, 1, 0))
                # if len(in_occupancy_grid_objects) > 0:
                #     self.publish_markers(
                #         in_occupancy_grid_objects, "IN_GRID", color=(1, 0, 1)
                #     )
                return obj_point
        # if len(in_occupancy_grid_objects) > 0:
        #     self.publish_markers(in_occupancy_grid_objects, "IN_GRID", color=(1, 0, 1))
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
        self,
        frontiers: List[Point],
        sizes: List[float],
        target_pose: Union[Pose, None] = None,
    ) -> Union[List[Tuple[Point, Union[np.floating, float]]], None]:
        if not frontiers or not sizes or len(frontiers) != len(sizes):
            rospy.logwarn(
                "Frontiers and sizes must be non-empty and of the same length."
            )
            return None

        current_pose: Pose = self.get_current_pose()
        current_position = np.array([current_pose.position.x, current_pose.position.y])

        MIN_DISTANCE = 0.72  # TODO: Tune this
        MAX_DISTANCE = 30.0  # TODO: Tune this
        MIN_SIZE = max(40.0, sum(sizes) / len(sizes))  # TODO: Tune this

        scored_frontiers: List[Tuple[Point, Union[np.floating, float]]] = []

        for frontier, size in zip(frontiers, sizes):
            frontier_position = np.array([frontier.x, frontier.y])
            distance: np.floating = np.linalg.norm(frontier_position - current_position)

            if size <= MIN_SIZE:
                continue

            if distance < MIN_DISTANCE:
                continue

            if distance > MAX_DISTANCE:
                continue

            # Basic score is inverse distance
            score: Union[np.floating, float] = 1 / max(distance, 0.1)

            # Adjust score based on size
            score += size / 5

            if target_pose is not None:
                target_position = np.array(
                    [target_pose.position.x, target_pose.position.y]
                )
                frontier_to_target = frontier_position - target_position
                frontier_to_position = frontier_position - current_position

                if (
                    np.linalg.norm(frontier_to_target) > 0
                    and np.linalg.norm(frontier_to_position) > 0
                ):
                    cos_angle = np.dot(frontier_to_target, frontier_to_position) / (
                        np.linalg.norm(frontier_to_target)
                        * np.linalg.norm(frontier_to_position)
                    )
                    # Boost score if frontier is in direction of target object
                    direction_factor = (cos_angle + 1) / 2
                    score *= 1 + 3 * direction_factor

            scored_frontiers.append((frontier, score))

        # Sort by score (highest first)
        scored_frontiers.sort(key=lambda x: x[1], reverse=True)

        if scored_frontiers:
            best_frontier_position = scored_frontiers[0][0]
            distance_from_current_pose = np.linalg.norm(
                np.array([best_frontier_position.x, best_frontier_position.y])
                - np.array([current_position[0], current_position[1]])
            )
            if GraceNavigation.verbose:
                rospy.loginfo(
                    f"Best Frontier Position: ({best_frontier_position.x:.2f}, {best_frontier_position.y:.2f}, {best_frontier_position.z:.2f}), "
                    f"Distance from Current Pose: {distance_from_current_pose:.2f} meters, "
                    f"Score: {scored_frontiers[0][1]:.2f}"
                )
            return scored_frontiers
        return None

    def compute_frontier_goal_pose(self, heuristic_pose: Pose) -> Union[Pose, None]:
        keypoints: List[Point]
        sizes: List[float]
        keypoints, sizes = self.frontier_search.compute_centroids()

        if not keypoints or len(keypoints) == 0:
            return None

        scored_frontiers = self.score_frontiers(keypoints, sizes, heuristic_pose)
        if scored_frontiers is None:
            return None
        self.publish_labled_markers(
            keypoints=scored_frontiers,
            namespace="frontiers",
            color=(0.5, 0.1, 0.1),
        )

        best_frontier = scored_frontiers[0][0]
        best_frontier_pose = Pose()
        best_frontier_pose.position = best_frontier

        # Calculate orientation to face the object
        direction_to_object: Quaternion = self.calculate_direction_towards_object(
            best_frontier
        )
        best_frontier_pose.orientation = direction_to_object

        return best_frontier_pose

    # region Calculations
    def calculate_offset(
        self, point: Point, heuristic_func: Union[Callable, None] = None
    ) -> Union[Pose, None]:
        """
        Calculate an offset point using BFS with a customizable heuristic function.

        Args:
            point (Point): A goal in map coordinates
            heuristic_func (Callable, optional): A function that takes (pose, current_position, original_goal) and returns a score
                (lower is better). If None, uses distance from current position as the default. Defaults to None.

        Returns:
            A valid Pose or None if no valid offset could be found
        """
        if heuristic_func is None:

            def default_heuristic_func(poses_list, current_position, original_goal):
                return min(
                    poses_list,
                    key=lambda pose: np.linalg.norm(
                        np.array([pose.position.x, pose.position.y]) - current_position
                    ),
                )

            heuristic_func = default_heuristic_func

        # point is a goal in map coordinates
        map_image = np.array(self.frontier_search.global_costmap.data).reshape(
            (
                self.frontier_search.global_costmap.info.height,
                self.frontier_search.global_costmap.info.width,
            )
        )
        max_offset_distance = 800.0  # Maximum depth for BFS TODO: Tune this
        visited = set()
        bfs_queue = queue.Queue()
        img_point = self.frontier_search.convert_map_coords_to_img_coords(
            (point.x, point.y)
        )
        bfs_queue.put(img_point)

        depth: float = 0.0

        best_poses = []

        while not bfs_queue.empty():
            img_x, img_y = bfs_queue.get()
            map_x, map_y = self.frontier_search.convert_img_coords_to_map_coords(
                (img_x, img_y)
            )
            if (map_x, map_y) in visited:
                continue

            visited.add((map_x, map_y))
            depth += 1.0
            # uncomment to debug bfs
            # self.publish_markers([Point(map_x, map_y, 0)], "BFS", color=(1, 1, 0))
            # rospy.sleep(0.1)

            # Check if the point is not in the occupancy grid
            if self.frontier_search.is_point_in_occupancy_grid(
                Point(map_x, map_y, 0), True
            ):
                new_pose = Pose()
                new_pose.position = Point(map_x, map_y, 0)
                new_pose.orientation = self.calculate_direction_between_points(
                    point, new_pose.position
                )
                best_poses.append(new_pose)
                if (
                    len(best_poses) >= max_offset_distance / 10
                    and depth >= 0.6 * max_offset_distance
                ) or len(best_poses) >= 100:  # TODO: Tune this
                    current_pose = self.get_current_pose()
                    current_position = np.array(
                        [current_pose.position.x, current_pose.position.y]
                    )
                    best_pose = heuristic_func(best_poses, current_position, point)
                    return best_pose

            if depth >= max_offset_distance:
                if GraceNavigation.verbose:
                    rospy.logwarn(
                        "Could not find a valid offset point using BFS. Depth Limited"
                    )
                return None

            # Add neighbors to the queue
            for dx, dy in [(-2, 0), (2, 0), (0, -2), (0, 2)]:
                nx, ny = img_x + dx, img_y + dy
                if (
                    0 <= nx < map_image.shape[1]
                    and 0 <= ny < map_image.shape[0]
                    and (nx, ny) not in visited
                ):
                    bfs_queue.put((nx, ny))

        if GraceNavigation.verbose:
            rospy.logwarn("Could not find a valid offset point using BFS.")
        return None

    def calculate_direction_towards_object(self, point: Point) -> Quaternion:
        return self.calculate_direction_between_points(
            point, self.get_current_pose().position
        )

    def calculate_direction_between_points(
        self, point: Point, point2: Point
    ) -> Quaternion:
        # Calculate angle to point
        dx = point.x - point2.x
        dy = point.y - point2.y
        target_angle = atan2(dy, dx)

        # Convert to quaternion
        qz = np.sin(target_angle / 2)
        qw = np.cos(target_angle / 2)

        return Quaternion(x=0, y=0, z=qz, w=qw)

    def get_current_pose(self) -> Pose:
        """Returns the current pose of the turtlebot by subscribing to the `/odom`.

        Returns:
            Pose: The turtlebot's current Pose.
        """

        assert self.odom
        pose: Pose = self.odom.pose.pose
        return pose

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
