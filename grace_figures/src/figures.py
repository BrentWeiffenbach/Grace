import os
import sys
import threading
from collections import defaultdict

import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid

from grace_navigation.msg import Object2D, Object2DArray, RobotState
from grace_grasping.srv import GetObjectPose
from typing import Tuple

# TODO:
# HALF DONE: Save occupancy grid map with overlayed semantic objects (only the most recent of each type of class)
# DONE: Save only occupancy grid image
# Time to explore, time to first object, time to second object
#  ??? Maybe the line cast for calc offset ???


class GraceFigures:
    def __init__(self, verbose: bool = False) -> None:
        self.verbose = verbose
        self.state = RobotState(RobotState.WAITING)
        self.occupancy_grid = OccupancyGrid()
        self.semantic_map = Object2DArray()
        self.semantic_map.objects = []
        self.save_lock = threading.Lock()
        self.is_saved = False
        """Flag to prevent double saving"""
        self.unique_classes = set()
        self.class_poses = defaultdict(lambda: [])
        self.color_map = {}
        self.colored_image: cv2.typing.MatLike = np.array([])
        rospy.Subscriber("/grace/state", RobotState, self.state_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.occupancy_grid_callback)
        rospy.Subscriber("/semantic_map", Object2DArray, self.semantic_map_callback)
        rospy.sleep(5)
        rospy.wait_for_service("get_object_pose",timeout=5)
        self.get_object_pose_srv = rospy.ServiceProxy("get_object_pose", GetObjectPose)

    def state_callback(self, msg: RobotState) -> None:
        previous_state = self.state
        self.state = msg

        if previous_state == RobotState.PLACING and self.state == RobotState.WAITING:
            # Start saving everything
            self.save()

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        self.occupancy_grid = msg

    def semantic_map_callback(self, msg: Object2DArray) -> None:
        self.semantic_map = msg
    def imwrite(self, filename, image, directory=["..", "figures"]):
        # type: (str, cv2.typing.MatLike, list[str] | str) -> bool
        """
        Args:
            filename (str): The name of the file to output. Should include a filetype (e.g. `image.png`).
            image (cv2.typing.MatLike): The image to print.
            directory (list[str] | str, optional): The output directory of the image. Defaults to ../figures

        Returns:
            bool: The default return value of cv2's imwrite.
        """
        # Allow both strings and a list of strings
        cleaned_directory_path = directory
        if not isinstance(directory, list):
            cleaned_directory_path = [directory]

        # Get the ROS package path instead of using __file__
        script_dir = os.path.dirname(os.path.abspath(os.path.realpath(sys.argv[0])))
        folder_path = os.path.join(script_dir, *cleaned_directory_path)
        os.makedirs(folder_path, exist_ok=True)
        full_path = os.path.join(folder_path, filename)
        full_path = os.path.join(folder_path, filename)
        if os.path.exists(full_path):
            os.remove(full_path)
        return cv2.imwrite(full_path, image)

    def get_writable_occupancy_grid(self) -> cv2.typing.MatLike:
        grid_data = np.array(self.occupancy_grid.data, dtype=np.int8).reshape(
            (self.occupancy_grid.info.height, self.occupancy_grid.info.width)
        )

        map_image = grid_data.copy()

        map_image[grid_data == -1] = 127  # Handle unknown spaces being -1
        return map_image

    def save_only_occupancy_grid(self) -> None:
        if self.verbose:
            rospy.loginfo("Saving Occupancy Grid")

        map_image = self.get_writable_occupancy_grid()
        self.imwrite("occupancy_grid.png", map_image)

        if self.verbose:
            rospy.loginfo("Saved Occupancy Grid")

    def save_semantic_map(self) -> None:
        # TODO: Only grab the latest ones
        if self.verbose:
            rospy.loginfo("Saving semantic map")    
            
        # filtered_semantic_objects: List[Object2D] = []
        
        # # 4 is too small
        # # 5 too small
        # distances = []
        # threshold_distance = 3
        # for semantic_object in self.semantic_map.objects:
        #     is_close_to_existing_of_same_cls = False
        #     for obj in filtered_semantic_objects:
        #         if obj.cls == semantic_object.cls:
        #             distance = ((obj.x - semantic_object.x) ** 2 + (obj.y - semantic_object.y) ** 2) ** 0.5
        #             if distance < threshold_distance:
        #                 distances.append(distance)
        #                 rospy.loginfo(f"Distance: {distance}, threshold_distance: {threshold_distance}")
        #                 is_close_to_existing_of_same_cls = True
        #             break
        #     if not is_close_to_existing_of_same_cls:
        #         filtered_semantic_objects.append(semantic_object)
                
        # rospy.loginfo(f"Mean distance: {np.mean(distances)}" )
        for cls in self.unique_classes:
            cls: str
            for semantic_object in self.class_poses[cls]:
                semantic_object: Tuple[float, float]
                color = self.color_map[cls]
                # Skip drawing circles on gray/unknown areas
                cv2.circle(self.colored_image, (int(semantic_object[0]), int(semantic_object[1])), 2, color, -1)

        legend_height = len(self.unique_classes) * 25 + 10
        legend_width = 200
        legend_img = np.ones((legend_height, legend_width, 3), dtype=np.uint8) * 255

        y_pos = 20
        for cls, color in self.color_map.items():
            cv2.circle(legend_img, (20, y_pos), 5, color, -1)
            cv2.putText(
                legend_img,
                cls,
                (40, y_pos + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )
            y_pos += 25
        main_height, main_width = self.colored_image.shape[:2]
        combined_height = max(main_height, legend_height)
        combined_width = main_width + legend_width
        combined_img = (
            np.ones((combined_height, combined_width, 3), dtype=np.uint8) * 255
        )

        combined_img[0:main_height, 0:main_width] = self.colored_image
        combined_img[0:legend_height, main_width:combined_width] = legend_img
        # Print poses for debugging
        rospy.loginfo("All object poses in semantic map:")
        for cls, poses in self.class_poses.items():
            pose_str = ', '.join([f"({x:.2f}, {y:.2f})" for x, y in poses])
            rospy.loginfo(f"  {cls}: {pose_str}")
        self.imwrite("semantic_objects.png", combined_img)

        if self.verbose:
            rospy.loginfo("Saved semantic map")

    def save(self) -> None:
        self.is_saved = True
        with self.save_lock:
            self.save_only_occupancy_grid()
            self.save_semantic_map()

    def run(self) -> None:
        while not rospy.is_shutdown():
            if self.semantic_map.objects is None:
                rospy.loginfo("No semantic objects to store for figure")
                continue
            self.unique_classes = set(obj.cls for obj in self.semantic_map.objects)
            self.color_map = {}

            colors = [
                (0, 0, 255),
                (0, 255, 0),
                (255, 0, 0),
                (255, 0, 255),
                (0, 255, 255),
                (255, 255, 0),
                (128, 0, 0),
                (0, 128, 0),
                (0, 0, 128),
                (128, 128, 0),
            ]

            for i, cls in enumerate(self.unique_classes):
                self.color_map[cls] = colors[i % len(colors)]

            # Initialize empty list of poses for each unique class
            # if len(self.class_poses.keys()) == 0:
            #     self.class_poses = {cls: [] for cls in self.unique_classes}

            for cls in self.unique_classes:
                result = self.get_object_pose_srv(cls, False)
                if result.success:
                    pose_x, pose_y = result.pose.pose.position.x, result.pose.pose.position.y
                    
                    is_close_to_existing = False
                    threshold_distance = 0.5
                    
                    for existing_pose in self.class_poses[cls]:
                        distance = ((existing_pose[0] - pose_x) ** 2 + (existing_pose[1] - pose_y) ** 2) ** 0.5
                        if distance < threshold_distance:
                            is_close_to_existing = True
                            if self.verbose:
                                rospy.loginfo(f"Discarding close pose for {cls}, distance: {distance}")
                            break
                    
                    if not is_close_to_existing:

                        # Process Occupancy Grid
                        map_image = self.get_writable_occupancy_grid()
                        map_uint8 = np.zeros_like(map_image, dtype=np.uint8)

                        map_uint8[map_image == 0] = 255
                        map_uint8[map_image == 100] = 0
                        map_uint8[map_image > 0] = 255 - (map_image[map_image > 0] * 255 // 100)

                        map_uint8[map_image == -1] = 127

                        self.colored_image = cv2.cvtColor(map_uint8, cv2.COLOR_GRAY2BGR)
                        resolution = self.occupancy_grid.info.resolution

                        origin_x = self.occupancy_grid.info.origin.position.x
                        origin_y = self.occupancy_grid.info.origin.position.y
                        
                        grid_x = int((pose_x - origin_x) / resolution)
                        grid_y = int((pose_y - origin_y) / resolution)

                        if map_image[grid_y, grid_x] == 127:  # Check if this point is unknown/gray
                            continue
                        self.class_poses[cls].append((grid_x, grid_y))
                        if self.verbose:
                            rospy.loginfo(f"Added new pose for {cls} at ({grid_x}, {grid_y})")

    def shutdown(self) -> None:
        if not self.is_saved:  # Only save when it hasn't been saved manually
            self.save()


if __name__ == "__main__":
    rospy.init_node("grace_figures")
    verbose = rospy.get_param("~verbose", False)
    pick_location = rospy.get_param("~pick_location", "dining table")
    place_location = rospy.get_param("~place_location", "suitcase")
    pick_object = rospy.get_param("~pick_object", "cup")
    assert type(verbose) is bool
    assert type(pick_location) is str
    assert type(place_location) is str
    assert type(pick_object) is str

    grace_figures = GraceFigures(verbose=verbose)
    rospy.on_shutdown(grace_figures.shutdown)

    try:
        grace_figures.run()
    except rospy.ROSInterruptException:
        grace_figures.shutdown()
        rospy.loginfo("GraceFigures terminated GRACEfully.")  # type: ignore
