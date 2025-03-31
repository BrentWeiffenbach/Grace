import queue
from math import atan2
from typing import Dict, List, Tuple, Union

import actionlib
import numpy as np
import rospy
from frontier_search import FrontierSearch
from geometry_msgs.msg import Point, Pose, Quaternion
from grace_node import GraceNode, RobotGoal, get_constants_from_msg, rotate_360
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseResult,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
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
        # rospy.loginfo("updating costmap")
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
        scale: Tuple[float, float, float] = (0.15, 0.15, 0.15)
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
            rospy.logwarn("Failed to receive message for has_object. Defaulting to False.")
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
                # rospy.loginfo("Publishing object goal")
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

    # region Exploration
    def explore(self) -> None:
        # BUG (?): self.goal can possibly be undefined. It is difficult to reproduce how it happened.

        EXPLORE_SECONDS = 12 * 60 # 12 minutes
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
                # Commented from cherry picked commit because it makes the robot not go to reasonable goals sometimes
                # if self.frontier_search.is_point_in_occupancy_grid(obj_point, True):
                #     # most likely a haclucination if not inside the inflation layer
                #     # is_point_in_occupancy_grid returns True if it is reachable by turtlebot
                #     continue
                self.publish_markers([obj_point], "Object_Goal", color=(0, 1, 0))
                self.goal_pose: Union[Pose, None] = self.calculate_offset(obj_point)

            if self.goal_pose is not None:
                self.publish_markers([self.goal_pose.position], "Object_Goal_Offset", color=(0, 0, 1))
                # Found object
                # Goal is in occupancy grid
                GraceNavigation.verbose_log(
                    f"Goal for {target_obj_name} is accessible, navigating to it"
                )
                navigating_to_object = True
                
                if self.done_flag:
                    self.done_flag = False
                    self.goal_pose = None
                    return True

                self.goto(
                    self.goal_pose,
                    yield_when_done=True,
                )
                
                self.goal_pose = None
                
                continue
            else:
                GraceNavigation.verbose_log(
                    f"Goal for {target_obj_name} is not accessible yet"
                )
                navigating_to_object = False


            if not navigating_to_object:
                frontier_pose = self.compute_frontier_goal_pose(
                    heuristic_pose=self.get_current_pose()
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
                    self.goto(
                        self.create_navigable_goal(frontier_pose),
                        yield_when_done=False,
                    )
                    rospy.sleep(
                        4
                    )  # Increase to increase time between updating which frontier should be navigated to

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
            np.array([current_pose.position.x, current_pose.position.y]) -
            np.array([obj.position.x, obj.position.y])
        )
        # distance from last to current goal
        last_goal_distance = np.linalg.norm(
            np.array([self.last_goal.position.x, self.last_goal.position.y]) -
            np.array([obj.position.x, obj.position.y])
        )
        # if either are too close, don't goto
        # BUG: If two goal semantic objects for pick and palce are right next to each other this doesnt work
        THRESHOLD = 0.5  # TODO: Tune this
        if pose_distance <= THRESHOLD:
            # rospy.logwarn("Robot is already within the threshold distance to the goal.")
            return
        if last_goal_distance <= THRESHOLD:
            # rospy.logwarn("Robot tried to goto the same goal.")
            return
        
        rospy.loginfo("Publishing goal to MoveBase")
        dest = MoveBaseGoal()
        dest.target_pose.header.frame_id = "map"
        dest.target_pose.header.stamp = rospy.Time.now()
        dest.target_pose.pose = obj
        self.move_base.send_goal(
            goal=dest,
            done_cb=self.done_cb if yield_when_done else self.no_yield_done_cb,
        )
        if yield_when_done:
            rospy.sleep(2)
        # rememebr last goto goal
        self.last_goal = obj

    def done_cb(self, status: int, _: MoveBaseResult) -> None:
        """The callback for when the robot has finished with it's goal

        Args:
            status (int): The status of the robot at the end. Use the `goal_statuses` dict to get a user-friendly string.
            _ (MoveBaseResult): The result of the move base. Ignored
        """
        self.done_flag = False
        if goal_statuses[status] not in ["PREEMPTED"]:
            self.publish_status(status)
        if status == actionlib.GoalStatus.ABORTED:
            self.should_update_goal = True
        if goal_statuses[status] in ["SUCCEEDED"]:
            self.done_flag = True
            
    def no_yield_done_cb(self, status: int, _: MoveBaseResult) -> None:
        if goal_statuses[status] in ["SUCCEEDED"]:
            self.should_update_goal = True

    def find_object_in_semantic_map(self, obj: str) -> Union[Point, None]:
        """
        Returns:
            Union (Point, None): The object found's coordinates, or None if it was not found. In map coordinates.
        """
        assert self.semantic_map  # I do not want to deal with this not being initialized # fmt: skip
        if self.semantic_map.objects is None or len(self.semantic_map.objects) == 0:
            rospy.loginfo("self.semantic_map.object has no objects. Returning none")
            return None
        
        cls_objects = [
            map_obj for map_obj in self.semantic_map.objects if map_obj.cls == obj
        ]

        if not cls_objects or len(cls_objects) == 0:
            rospy.logwarn(f"No objects with class: {obj}")
            return None
        
        current_pose = self.get_current_pose()
        current_position = np.array([current_pose.position.x, current_pose.position.y])
        closest_obj = min(
            cls_objects,
            key=lambda obj: np.linalg.norm(
            np.array([obj.x, obj.y]) - current_position
            ),
        )
        return Point(closest_obj.x, closest_obj.y, 0)

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
    ) -> Union[List[Tuple[Point, Union[np.floating, float]]], None]:
        if not frontiers:
            return None

        current_pose: Pose = self.get_current_pose()
        current_position = np.array([current_pose.position.x, current_pose.position.y])

        MIN_DISTANCE = 1.0 
        MAX_DISTANCE = 10.0

        scored_frontiers: List[Tuple[Point, Union[np.floating, float]]] = []

        for frontier in frontiers:
            frontier_position = np.array([frontier.x, frontier.y])
            distance: np.floating = np.linalg.norm(frontier_position - current_position)

            if distance < MIN_DISTANCE:
                continue

            if distance > MAX_DISTANCE:
                continue

            # Basic score is inverse distance
            score: Union[np.floating, float] = 1 / max(distance, 0.1)

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
                    # boost score if frontier is in direction of target object
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
            rospy.loginfo(
                f"Best Frontier Position: ({best_frontier_position.x:.2f}, {best_frontier_position.y:.2f}, {best_frontier_position.z:.2f}), "
                f"Distance from Current Pose: {distance_from_current_pose:.2f} meters, "
                f"Score: {scored_frontiers[0][1]:.2f}"
            )
            return scored_frontiers
        return None

    def compute_frontier_goal_pose(self, heuristic_pose: Pose) -> Union[Pose, None]:
        keypoints: List[Point] = self.frontier_search.compute_centroids()

        if not keypoints or len(keypoints) == 0:
            return None

        scored_frontiers = self.score_frontiers(keypoints, heuristic_pose)
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
    def calculate_offset(self, point: Point) -> Union[Pose, None]:
        # point is a goal in map coordinates
        map_image = np.array(self.frontier_search.global_costmap.data).reshape(
            (self.frontier_search.global_costmap.info.height, self.frontier_search.global_costmap.info.width)
        )        
        max_offset_distance = 400.0  # Maximum depth for BFS
        visited = set()
        bfs_queue = queue.Queue()
        img_point = self.frontier_search.convert_map_coords_to_img_coords((point.x, point.y))
        bfs_queue.put(img_point)
        
        depth: float = 0.0

        best_poses = []
                
        while not bfs_queue.empty():
            img_x, img_y = bfs_queue.get()
            map_x, map_y = self.frontier_search.convert_img_coords_to_map_coords((img_x, img_y))
            if (map_x, map_y) in visited:
                continue

            visited.add((map_x, map_y))
            depth += 1.0
            # uncomment to debug bfs
            # self.publish_markers([Point(map_x, map_y, 0)], "BFS", color=(1, 1, 0))
            # rospy.sleep(0.1)

            # Check if the point is not in the occupancy grid
            if self.frontier_search.is_point_in_occupancy_grid(Point(map_x, map_y, 0), True):
                new_pose = Pose()
                new_pose.position = Point(map_x, map_y, 0)
                new_pose.orientation = self.calculate_direction_between_points(point, new_pose.position)
                best_poses.append(new_pose)
                if (len(best_poses) >= 1 and depth >= 0.75 * max_offset_distance) or len(best_poses) >= 30: # Tune this
                    current_pose = self.get_current_pose()
                    current_position = np.array([current_pose.position.x, current_pose.position.y])
                    best_pose = min(
                        best_poses,
                        key=lambda pose: np.linalg.norm(
                            np.array([pose.position.x, pose.position.y]) - current_position
                        ),
                    )
                    return best_pose
            
            if depth >= max_offset_distance:
                rospy.logwarn("Could not find a valid offset point using BFS. Depth Limited")
                return None
            
            # Add neighbors to the queue
            for dx, dy in [(-2, 0), (2, 0), (0, -2), (0, 2)]:
                nx, ny = img_x + dx, img_y + dy
                if 0 <= nx < map_image.shape[1] and 0 <= ny < map_image.shape[0] and (nx, ny) not in visited:
                    bfs_queue.put((nx, ny))

        rospy.logwarn("Could not find a valid offset point using BFS.")
        return None


    def calculate_direction_towards_object(self, point: Point) -> Quaternion:
        return self.calculate_direction_between_points(point, self.get_current_pose().position)
    
    def calculate_direction_between_points(self, point: Point, point2: Point) -> Quaternion:
        # Calculate angle to point
        dx = point.x - point2.x
        dy = point.y - point2.y
        target_angle = atan2(dy, dx)

        # Convert to quaternion
        qz = np.sin(target_angle / 2)
        qw = np.cos(target_angle / 2)

        return Quaternion(x=0, y=0, z=qz, w=qw)

    def create_navigable_goal(self, goal: Pose, safety_margin: int = 8) -> Pose:
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
