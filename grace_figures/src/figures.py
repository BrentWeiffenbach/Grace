import rospy
import cv2
from grace_navigation.msg import RobotState, Object2DArray, Object2D
import threading
from nav_msgs.msg import OccupancyGrid
import numpy as np
import os

# TODO:
# Save occupancy grid map with overlayed semantic objects (only the most recent of each type of class)
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
        rospy.Subscriber("/grace/state", RobotState, self.state_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.occupancy_grid_callback)
        rospy.Subscriber("/semantic_map", Object2DArray, self.semantic_map_callback)

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

    def imwrite(self, filename: str, image: cv2.typing.MatLike) -> bool:
        """
        Args:
            filename (str): The name of the file to output. Should include a filetype (e.g. `image.png`).
            image (cv2.typing.MatLike): The image to print.
        """
        folder_path = os.path.join(os.path.dirname(__file__), "..", "figures")
        os.makedirs(folder_path, exist_ok=True)
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

        # Process Occupancy Grid
        map_image = self.get_writable_occupancy_grid()
        map_uint8 = np.zeros_like(map_image, dtype=np.uint8)
    
        map_uint8[map_image == 0] = 255
        map_uint8[map_image == 100] = 0
        map_uint8[map_image > 0] = 255 - (map_image[map_image > 0] * 255 // 100)
        
        map_uint8[map_image == -1] = 127

        colored_image = cv2.cvtColor(map_uint8, cv2.COLOR_GRAY2BGR)
        resolution = self.occupancy_grid.info.resolution

        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y

        assert self.semantic_map and self.semantic_map.objects
        for semantic_object in self.semantic_map.objects:
            semantic_object: Object2D
            # Convert from map coords to image coords
            grid_x = int(semantic_object.x - origin_x / resolution)
            grid_y = int(semantic_object.y - origin_y / resolution)

            cv2.putText(
                colored_image,
                semantic_object.cls,
                (grid_x + 5, grid_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
            
        self.imwrite("semantic_objects.png", colored_image)


        if self.verbose:
            rospy.loginfo("Saved semantic map")

    def save(self) -> None:
        self.is_saved = True
        with self.save_lock:
            self.save_only_occupancy_grid()
            self.save_semantic_map()

    def run(self) -> None:
        rospy.spin()

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
