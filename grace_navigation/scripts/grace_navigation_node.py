from math import atan2
from typing import Dict, List, Tuple, Union

import actionlib
import numpy as np
import rospy
from frontier_search import FrontierSearch
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from grace_node import GraceNode, ManualControl, RobotGoal, get_constants_from_msg
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseResult,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Int16
from utils import MarkerPublisher
from visualization_msgs.msg import MarkerArray

from grace_grasping.srv import GetObjectPose, GetObjectPoseResponse
from grace_navigation.msg import (
    Object2D,
    Object2DArray,
    RobotGoalMsg,
    RobotState,
)
from grace_navigation.srv import Transform, TransformResponse

goal_statuses: Dict[int, str] = get_constants_from_msg(actionlib.GoalStatus)
"""Gets all of the non-callable integer constants from actionlib.GoalStatus msg. """


# region GraceNavigation Init
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

    def __init__(self, verbose: bool = False, is_sim: bool = False) -> None:
        ### FLAGS ###
        GraceNavigation.verbose = verbose
        self.is_sim: bool = is_sim
        self.semantic_map: Object2DArray = Object2DArray()
        self.should_update_goal: bool = True
        self.done_flag: bool = False
        self.final_checking: bool = False
        self.last_goal: Pose = Pose()
        self.clear_last_goal()
        """if yolo final checking is running its true to prevent double cb"""
        self.has_object: bool = False

        ### ROBOT INFORMATION ###
        self.goal: RobotGoal
        self.goal_pose: Union[Pose, None] = None
        self.state: int
        self.odom: Odometry = Odometry()  # Initialize to empty odom
        self.transform_srv: Union[rospy.ServiceProxy, None] = None
        """The transform service proxy. Use `self.connect_to_transform_service()` to connect to the service."""

        # init FrontierSearch class to keep track of frontiers and map properties
        self.frontier_search = FrontierSearch()

        ### SEMANTIC SLAM ###
        self.semantic_map_sub = rospy.Subscriber(
            name="/semantic_map",
            data_class=Object2DArray,
            callback=self.semantic_map_callback,
        )
        self.remove_pub = rospy.Publisher("/semantic_map/remove", Int16, queue_size=10)

        ### EXPLORATION ###
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

        ### ENVIRONMENT SUBSCRIBERS ###
        self.connect_to_transform_service()
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

        self.has_object_sub = rospy.Subscriber(
            "/grace/has_object",
            Bool,
            self.has_object_callback,
        )

        ### PUBLISHERS ###
        self.centroid_marker_pub = rospy.Publisher(
            "/frontier/centroids", MarkerArray, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            name=GraceNode.NAV_STATUS_TOPIC,
            data_class=actionlib.GoalStatus,
            queue_size=10,
        )

        ### MOVE_BASE ###
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        GraceNavigation.verbose_log("Starting move_base action server...")
        self.move_base.wait_for_server()  # Wait until move_base server starts
        GraceNavigation.verbose_log("move_base action server started!")

        ### OTHER ###
        self.marker_utils = MarkerPublisher(
            self.centroid_marker_pub, verbose=self.verbose
        )

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

    def has_object_callback(self, msg: Bool) -> None:
        self.has_object = msg.data

    def goal_callback(self, msg: RobotGoalMsg) -> None:
        goal = RobotGoal(place_location=msg.place_location, pick_location=msg.pick_location, pick_object=msg.pick_object)
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

    def connect_to_transform_service(self) -> None:
        """Connects to the transform service"""
        try:
            rospy.wait_for_service("transform", timeout=5)
            self.transform_srv = rospy.ServiceProxy("transform", Transform)
            GraceNavigation.verbose_log("Connected to Transform service!")
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to connect to Transform service! {e}")
            self.transform_srv = None

    # region Exploration
    def explore(self) -> None:
        EXPLORE_SECONDS = 20 * 60  # 20 minutes
        rospy.loginfo("Exploring")
        # Explore the env until the goal is completed and done_cb finishes
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
        self.done_flag = False
        self.final_checking = False
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

        assert self.goal
        target_obj_name: str = (
            self.goal.place_location if self.has_object else self.goal.pick_location
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
                self.goal_pose: Union[Pose, None] = self.calculate_offset(
                    obj_point, is_goal=True
                )

            if self.goal_pose is not None:
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
                        
                if not self.final_checking:
                    self.goto(
                        self.goal_pose,
                        yield_when_done=True,
                    )
                rospy.sleep(1)

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
                    ManualControl.rotate_360()
                    rospy.sleep(2)
                    continue

                if self.should_update_goal or self.move_base.get_state() not in [
                    actionlib.GoalStatus.PENDING
                ]:
                    GraceNavigation.verbose_log(
                        "Navigating to a frontier for exploration"
                    )
                    self.should_update_goal = False

                    offset_pose = self.calculate_offset(frontier_pose.position)
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
            rospy.sleep(1)  # Sleep for node to catch up

        return None

    def goto(self, target_pose: Pose, yield_when_done: bool = True) -> None:
        """Takes in a pose and a timeout time (buggy as of 2/19/2025) and attempts to go to that pose.

        Args:
            target_pose (Pose): The pose move_base should attempt to go to.
            yield_when_done (bool, optional): Whether goto should yield to grace_node (call done_cb) when it is done. Defaults to True.
        """
        if target_pose is None:
            rospy.logerr("Obj pose None was passed to goto")
            return
        if self.final_checking:
            rospy.logerr("Tried to navigate to object while final checking")
            return

        # Find distance between prev and cur goal to determine if it should be published again
        last_goal_distance = np.linalg.norm(
            np.array([self.last_goal.position.x, self.last_goal.position.y])
            - np.array([target_pose.position.x, target_pose.position.y])
        )

        GOAL_REACH_THRESHOLD = 0.3  # Tunable

        if (
            last_goal_distance <= GOAL_REACH_THRESHOLD
            and self.move_base.get_state() != actionlib.GoalStatus.SUCCEEDED
        ):  # tries to prevent not publishing a goal ever again if a frontier succeeds (or 2d nav goal)
            if GraceNavigation.verbose:
                rospy.logwarn_throttle(1, "Robot tried to goto the same goal.")
            return

        self.last_goal = target_pose
        GraceNavigation.verbose_log("Publishing goal to MoveBase")
        dest = MoveBaseGoal()
        dest.target_pose.header.frame_id = "map"
        dest.target_pose.header.stamp = rospy.Time.now()
        dest.target_pose.pose = target_pose
        self.move_base.send_goal(
            goal=dest,
            done_cb=self.done_cb if yield_when_done else self.no_yield_done_cb,
        )
        self.goal_pose = None

    def yolo_final_check(self) -> bool:
        if self.is_sim:
            return True # Bypass final check in sim
        
        if self.final_checking:
            return False
        self.final_checking = True
        # Stop the robot
        twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        twist = Twist()
        twist_pub.publish(twist)
        target_location_name: str = (
            self.goal.place_location if self.has_object else self.goal.pick_location
        )

        # Only be done if the yolo object is visible at the goal
        try:
            rospy.wait_for_service("get_object_pose", timeout=5)
            obj_pose_srv = rospy.ServiceProxy("get_object_pose", GetObjectPose)
        except rospy.ROSException as e:
            rospy.logerr("YOLO_final_check failed to connect to GetObjectPose service! {}".format(e))
            obj_pose_srv = None

        if obj_pose_srv is not None:
            obj_pose_srv_result: GetObjectPoseResponse = obj_pose_srv(target_location_name, True)
            if not self.has_object and not self.is_sim:
                obj_pose_srv_result: GetObjectPoseResponse = obj_pose_srv(self.goal.pick_object, True)
            if obj_pose_srv_result.success:
                    rospy.loginfo("YOLO final check worked!")
                    self.final_checking = False
                    self.done_flag = True
                    return True
            else: 
                rospy.logwarn("Pose is unknown")        

        self.should_update_goal = True
        self.done_flag = False
        self.clear_last_goal()

        if hasattr(self, "goal_obj_id"):
            self.remove_pub.publish(self.goal_obj_id)

        self.final_checking = False
        return False

    def done_cb(self, status: int, _: MoveBaseResult) -> None:
        """The callback for when the robot has finished with it's goal

        Args:
            status (int): The status of the robot at the end. Use the `goal_statuses` dict to get a user-friendly string.
            _ (MoveBaseResult): The result of the move base. Ignored
        """
        if self.final_checking:
            return
        self.goal_pose = None
        self.done_flag = False
        if status == actionlib.GoalStatus.ABORTED:
            self.should_update_goal = True
        if goal_statuses[status] in ["SUCCEEDED"]:
            if not self.yolo_final_check():
                rospy.loginfo("Final check failed, returning")
                return
            self.done_flag = True
        elif goal_statuses[status] not in ["PREEMPTED"]:
            self.publish_status(status)

    def no_yield_done_cb(self, status: int, _: MoveBaseResult) -> None:
        self.goal_pose = None
        if status == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("NO YEILD ABORTED")
            self.should_update_goal = True
        if goal_statuses[status] in ["SUCCEEDED"]:
            self.should_update_goal = True

    def find_object_in_semantic_map(self, obj: str) -> Union[Point, None]:
        """Finds the provided object in the semantic map, and returns its location, or None if it was not found.
        Args:
            obj (str): The object to find in the semantic map.
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
        self.marker_utils.publish_point_markers(
            points=[Point(class_obj.x, class_obj.y, 0) for class_obj in cls_objects],
            namespace="semantic_objects",
            color=(0.0, 1.0, 0.7),
        )

        if not cls_objects or len(cls_objects) == 0:
            if GraceNavigation.verbose:
                rospy.logwarn(f"No objects with class: {obj}")
            return None

        if cls_objects:
            best_object = cls_objects[0]
            self.goal_obj_id = best_object.id
            return Point(best_object.x, best_object.y, 0)
        return None

    def clear_last_goal(self) -> None:
        """Clears self.last goal by setting everything to garbage values."""
        self.last_goal = Pose()
        self.last_goal.position.x = float("inf")
        self.last_goal.position.y = float("inf")
        self.last_goal.position.z = float("inf")
        self.last_goal.orientation.x = 0.0
        self.last_goal.orientation.y = 0.0
        self.last_goal.orientation.z = 0.0
        self.last_goal.orientation.w = 1.0

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
        """Scores the frontiers based on the distance, size of frontier, and direction the robot is facing
        
        Args:
            frontiers (List[Point]): The frontiers.
            sizes (List[float]): The size of the frontiers.
            target_pose (Pose, optional): The pose to use as a hueristic for scoring frontiers. Defaults to None.
        """
        if not frontiers or not sizes or len(frontiers) != len(sizes):
            rospy.logwarn(
                "Frontiers and sizes must be non-empty and of the same length."
            )
            return None
        
        current_pose: Pose = self.get_current_pose()
        current_position = np.array([current_pose.position.x, current_pose.position.y])
        
        # use goal pose if passsed, or current_pose
        target_pose = target_pose or current_pose
        
        ALIGNMENT_WEIGHT = 7 # Tunable
        MIN_DISTANCE = 1.0 # Tunable
        MAX_DISTANCE = 30.0 # Tunable
        MIN_SIZE = max(40.0, sum(sizes) / len(sizes))
        
        scored_frontiers: List[Tuple[Point, Union[np.floating, float]]] = []
        target_position = np.array([target_pose.position.x, target_pose.position.y])
        
        for frontier, size in zip(frontiers, sizes):
            # filter by size and distance
            frontier_position = np.array([frontier.x, frontier.y])
            distance_from_current: np.floating = np.linalg.norm(frontier_position - current_position)
            if size <= MIN_SIZE or distance_from_current < MIN_DISTANCE or distance_from_current > MAX_DISTANCE:
                continue
            
            distance_to_target: np.floating = np.linalg.norm(frontier_position - target_position)
            
            # Base score is inverse of distance to target (closer to target = higher score)
            base_score = 1.0 / max(distance_to_target, 0.1) * 2

            # Add size contribution to the score
            size_contribution = size / 90.0
            base_score += size_contribution
            
            direction_to_frontier = frontier_position - current_position
            direction_to_target = current_position - target_position
            
            if np.linalg.norm(direction_to_frontier) > 0 and np.linalg.norm(direction_to_target) > 0:
                # Normalize vectors
                direction_to_frontier = direction_to_frontier / np.linalg.norm(direction_to_frontier)
                direction_to_target = direction_to_target / np.linalg.norm(direction_to_target)
                
                # Calculate how aligned the frontier is with the target direction
                target_alignment = np.dot(direction_to_frontier, direction_to_target)
                target_alignment_factor = (target_alignment + 1) / 2
                
                # Boost score based on alignment with target direction
                score = base_score * (1 + ALIGNMENT_WEIGHT * target_alignment_factor)
            else:
                score = base_score
                
            # Add debug information
            if GraceNavigation.verbose:
                rospy.loginfo(f"Frontier ({frontier.x:.2f}, {frontier.y:.2f}): " 
                            f"Distance to target: {distance_to_target:.2f}, "
                            f"Base score: {base_score:.4f}, "
                            f"Final score: {score:.4f}")
            
            scored_frontiers.append((frontier, score))
        
        # Sort by score (highest first)
        scored_frontiers.sort(key=lambda x: x[1], reverse=True)
        
        if scored_frontiers and GraceNavigation.verbose:
            best_frontier_position = scored_frontiers[0][0]
            distance_from_current_pose = np.linalg.norm(
                np.array([best_frontier_position.x, best_frontier_position.y])
                - np.array([current_position[0], current_position[1]])
            )
            distance_from_target = np.linalg.norm(
                np.array([best_frontier_position.x, best_frontier_position.y])
                - target_position
            )
            rospy.loginfo(
                f"Best Frontier Position: ({best_frontier_position.x:.2f}, {best_frontier_position.y:.2f}), "
                f"Distance from Current Pose: {distance_from_current_pose:.2f} meters, "
                f"Distance from Target: {distance_from_target:.2f} meters, "
                f"Size: {sizes[frontiers.index(best_frontier_position)]:.2f}, "
                f"Score: {scored_frontiers[0][1]:.4f}"
            )
        
        return scored_frontiers if scored_frontiers else None

    def compute_frontier_goal_pose(self, heuristic_pose: Pose) -> Union[Pose, None]:
        keypoints: List[Point]
        sizes: List[float]
        keypoints, sizes = self.frontier_search.compute_centroids()

        if not keypoints or len(keypoints) == 0:
            return None

        scored_frontiers = self.score_frontiers(keypoints, sizes, heuristic_pose)
        if scored_frontiers is None:
            return None
        self.marker_utils.publish_scored_markers(
            scored_points=scored_frontiers,
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
        self, point: Point, is_goal: bool = False
    ) -> Union[Pose, None]:
        """
        Calculate an offset point using BFS with a customizable heuristic function.

        Args:
            point (Point): A goal in map coordinates

        Returns:
            A valid Pose or None if no valid offset could be found
        """
        MIN_OFFSET = 10 if is_goal else 3
        MAX_OFFSET = 25  if is_goal else 100 # Tunable
        map_image = np.array(self.frontier_search.global_costmap.data).reshape(
            (
                self.frontier_search.global_costmap.info.height,
                self.frontier_search.global_costmap.info.width,
            )
        )
        start = self.frontier_search.convert_map_coords_to_img_coords(
            (point.x, point.y)
        )

        current_pose = self.get_current_pose()
        end = self.frontier_search.convert_map_coords_to_img_coords(
            (current_pose.position.x, current_pose.position.y)
        )

        def bresenham_line(x0, y0, x1, y1):
            """Bresenham's line algorithm to generate points between two coordinates."""
            points = []
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx - dy

            while True:
                points.append((x0, y0))
                if x0 == x1 and y0 == y1:
                    break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x0 += sx
                if e2 < dx:
                    err += dx
                    y0 += sy
            return points

        line_points = bresenham_line(start[0], start[1], end[0], end[1])

        for i, (img_x, img_y) in enumerate(line_points):
            if i < MIN_OFFSET:
                continue
            if i > MAX_OFFSET:
                break
            distance_to_robot = np.linalg.norm(
                np.array([point.x, point.y])
                - np.array([current_pose.position.x, current_pose.position.y])
            )
            # when robot is already close to the goal obj
            GOAL_PROXIMITY_THRESHOLD = 1.5  # Tunable
            if distance_to_robot <= GOAL_PROXIMITY_THRESHOLD and is_goal:
                pose = Pose()
                pose.position = current_pose.position
                pose.orientation = self.calculate_direction_between_points(
                    point, current_pose.position
                )
                if GraceNavigation.verbose:
                    rospy.loginfo(
                        f"Offset was close to robot. Distance to robot: {distance_to_robot:.2f} meters"
                    )
                return pose
            # find closest unoccupied cell in direction of robot
            if (
                0 <= img_x < map_image.shape[1]
                and 0 <= img_y < map_image.shape[0]
                and map_image[img_y, img_x] == 0  # Check for unoccupied cell
            ):
                map_x, map_y = self.frontier_search.convert_img_coords_to_map_coords(
                    (img_x, img_y)
                )
                pose = Pose()
                pose.position = Point(map_x, map_y, 0)
                pose.orientation = self.calculate_direction_between_points(
                    point, current_pose.position
                )
                return pose

        if GraceNavigation.verbose:
            rospy.logwarn("No valid unoccupied cell found within MAX_OFFSET.")
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
        """
        Returns:
            (Pose): The current pose of the Turtlebot using transforms
        """
        if self.transform_srv is None:
            self.connect_to_transform_service()
            if self.transform_srv is None:
                rospy.logerr(
                    "Cannot get current pose! Failed to connect to the Transform server!"
                )
                GraceNavigation.verbose_log("Returning odom instead...")
                return self.odom.pose.pose
        try:
            transform: TransformResponse = self.transform_srv("base_link", "map")

            if transform.success:
                current_pose = Pose()

                transform_translation = transform.transform.transform.translation
                transform_orientation = transform.transform.transform.rotation
                current_pose.position = Point(
                    transform_translation.x,
                    transform_translation.y,
                    transform_translation.z,
                )
                current_pose.orientation = Quaternion(
                    transform_orientation.x,
                    transform_orientation.y,
                    transform_orientation.z,
                    transform_orientation.w,
                )

                return current_pose
        except rospy.ServiceException as e:
            self.transform_srv = None  # Reset connection
            rospy.logerr(f"Failed to get current pose! {e}")

        GraceNavigation.verbose_log(
            "Returning odom instead of transform of current pose."
        )
        return self.odom.pose.pose

    # region Run Node
    def run(self) -> None:
        rospy.spin()

    def shutdown(self) -> None:
        GraceNavigation.verbose_log("Shutting down GraceNavigation.")
        self.move_base.cancel_all_goals()


if __name__ == "__main__":
    rospy.init_node("grace_navigation")
    verbose = rospy.get_param("~verbose", False)
    is_sim = rospy.get_param("~sim", False)
    assert type(verbose) is bool
    assert type(is_sim) is bool
    grace_navigation = GraceNavigation(verbose=verbose, is_sim=is_sim)
    rospy.on_shutdown(grace_navigation.shutdown)
    rospy.sleep(5)
    try:
        grace_navigation.run()
    except rospy.ROSInterruptException:
        grace_navigation.shutdown()
        rospy.loginfo("GraceNavigation shut down.")
