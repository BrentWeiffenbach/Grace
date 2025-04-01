from typing import List, Tuple

import cv2
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid


class FrontierSearch:
    def __init__(self) -> None:
        self._map: OccupancyGrid
        self.map_img: cv2.typing.MatLike
        """A variable to store the computed grayscale map"""

        self._global_costmap: OccupancyGrid
        self.global_costmap_img: cv2.typing.MatLike

    @property
    def map(self) -> OccupancyGrid:
        return self._map

    @map.setter
    def map(self, new_map: OccupancyGrid) -> None:
        self._map = new_map
        self.map_img = self.convert_to_img(self._map)

    @property
    def global_costmap(self) -> OccupancyGrid:
        return self._global_costmap

    @global_costmap.setter
    def global_costmap(self, new_global_costmap: OccupancyGrid) -> None:
        self._global_costmap = new_global_costmap
        if self._global_costmap is not None:
            self.global_costmap_img = self.convert_to_img(self._global_costmap)

    def global_map_cb(self, msg: OccupancyGridUpdate) -> None:
        # Dynamically resize the costmap if it changed
        current_width = self._global_costmap.info.width
        current_height = self._global_costmap.info.height
        if msg.width > current_width or msg.height > current_height:
            new_width = max(msg.width, current_width)
            new_height = max(msg.height, current_height)
            new_data = [0] * (new_width * new_height)

            # Copy data
            for y in range(current_height):
                for x in range(current_width):
                    old_index = y * current_width + x
                    new_index = y * new_width + x
                    new_data[new_index] = self._global_costmap.data[old_index]

            # Update costmap
            self._global_costmap.info.width = new_width
            self._global_costmap.info.height = new_height
            self._global_costmap.data = new_data

        data_list = list(self._global_costmap.data)

        # Add data from msg
        for i in range(msg.height):
            for j in range(msg.width):
                global_index = (msg.y + i) * self._global_costmap.info.width + (
                    msg.x + j
                )
                if global_index < len(data_list):
                    data_list[global_index] = msg.data[i * msg.width + j]

        self._global_costmap.data = data_list
        self.global_costmap = self._global_costmap

    def convert_to_img(self, occupancy_grid: OccupancyGrid) -> cv2.typing.MatLike:
        """ Normalizes the occupancy grid provided and turns it into a cv2.typing.MatLike image.
        
        * (Unknown) -1 -> 0
        * (Known) 1-127 -> 128
        * (Occupied) 128->255 -> 255
        """
        grid_data = np.array(occupancy_grid.data).reshape(
            (occupancy_grid.info.height, occupancy_grid.info.width)
        )
        grayscale_image = np.where(
            grid_data == -1,
            0,
            np.where(grid_data == 0, 128, np.where(grid_data == 100, 255, 0)),
        )
        grayscale_image = np.uint8(grayscale_image)
        return grayscale_image  # type: ignore
    
    def compute_centroids(self) -> Tuple[List[Point], List[float]]:
        """Flag for viewing image pipeline in action. VERY computationally expensive."""
        # Load the image
        use_me = self.map_img
        image = use_me.copy()

        # Threshold the map
        _, binary_gray = cv2.threshold(image, 100, 255, cv2.THRESH_BINARY)

        # Do edge detection to get frontier edges
        edges = cv2.Canny(binary_gray, 50, 150)

        # Make an obstacle mask
        obstacle_mask = cv2.inRange(image, np.array([255], dtype=image.dtype), np.array([255], dtype=image.dtype))
        
        # Dilate obstacle mask
        obstacle_dilated = cv2.dilate(obstacle_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6)))
        
        # Merge the dilated mask back into the original image so it is easier to see non obstacle frontiers
        dilated_image = cv2.bitwise_or(image, obstacle_dilated)
        
        # Get all the contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter only edges that are the border between gray and black pixels (frontiers that are not on obstacles)
        valid_cnts = []
        for cnt in contours:
            for point in cnt:
                x, y = point[0]
                if dilated_image[y, x] == 128:  # Gray pixel
                    # Check neighbors to see if any are black
                    neighbors = [
                        (x - 1, y),
                        (x + 1, y),
                        (x, y - 1),
                        (x, y + 1),
                    ]
                    if any(
                        0 <= nx < dilated_image.shape[1] and 0 <= ny < dilated_image.shape[0] and dilated_image[ny, nx] < 50
                        for nx, ny in neighbors
                    ):
                        valid_cnts.append(cnt)
                        break
        
        frontier_mask = np.zeros_like(dilated_image)
        cv2.drawContours(frontier_mask, valid_cnts, -1, (255,), 1)
        
        # Invert the obstacle mask
        inverted_obstacle_dilated = cv2.bitwise_not(obstacle_dilated)
        
        # Apply the inverted mask to the frontier borders
        obstacle_frontier_mask = cv2.bitwise_and(frontier_mask, inverted_obstacle_dilated)

        # Threshold the image to black and white
        _, inflation_layer = cv2.threshold(self.global_costmap_img, 127, 255, cv2.THRESH_BINARY)

        # Apply connected components analysis
        _, labels, stats, _ = cv2.connectedComponentsWithStats(inflation_layer, connectivity=8)
       
        # Find the largest white blob (excluding background)
        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])  # Skip the first background component
        
        # clean representation of inflation layer for largest area
        clean_inflation_mask = (labels == largest_label).astype(np.uint8) * 255
        
        # Resize inflation layer to match the dimensions of valid frontiers
        clean_inflation_mask = cv2.resize(clean_inflation_mask, (obstacle_frontier_mask.shape[1], obstacle_frontier_mask.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Remove impractical frontiers by removing parts of frontiers in the inflation layer
        valid_frontier_mask = cv2.bitwise_and(obstacle_frontier_mask, clean_inflation_mask)
        
        # Dialate frontiers to make blob detection better
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        valid_frontier_mask = cv2.dilate(valid_frontier_mask, kernel, None)

        _, labels, stats, centroid_cv = cv2.connectedComponentsWithStats(valid_frontier_mask, connectivity=4)

        # Sort components by area in descending order, excluding the background (label 0)
        sorted_indices = np.argsort(stats[1:, cv2.CC_STAT_AREA])[::-1] + 1  # Add 1 to skip background
        top_indices = sorted_indices[:10]  # Get the top 10 components

        # store sizes of frontiers for scoring
        centroids = []
        sizes = []
        for idx in top_indices:
            if idx < len(centroid_cv):
                centroid_x, centroid_y = centroid_cv[idx]
                map_coords = self.convert_img_coords_to_map_coords((centroid_x, centroid_y))
                centroids.append(Point(*map_coords, 0.0))

                sizes.append(stats[idx, cv2.CC_STAT_AREA])  # Use the area of the component as its size
                # print(f"Debug: Adding frontier of size: {stats[idx, cv2.CC_STAT_AREA]}")

            
        # Save the resulting images
        save_imgs: bool = False
        if save_imgs:
            # output = cv2.drawKeypoints(valid_frontier_mask, kps, None, color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT) # type: ignore
            cv2.imwrite("1 map.png", self.map_img)
            cv2.imwrite("2 explored.png", binary_gray)
            cv2.imwrite("3 edges.png", edges)
            cv2.imwrite("4 obstacle_dialated.png", obstacle_dilated)
            cv2.imwrite("5 frontier_mask.png", frontier_mask)
            cv2.imwrite("6 obstacle_frontier_mask.png", obstacle_frontier_mask)
            cv2.imwrite("7 valid_frontier_mask.png", valid_frontier_mask)
            # cv2.imwrite("8 centroids.png", output)
            
            cv2.imwrite("9 inflation_layer.png", inflation_layer)
            cv2.imwrite("10 clean_inflation_mask.png", clean_inflation_mask)
            cv2.imwrite("11 global_costmap.png", self.global_costmap_img)
        return centroids, sizes

    def is_pose_in_occupancy_grid(self, pose: Pose, use_inflation_layer: bool = False) -> bool:
        """
        Args:
            pose (Pose): The pose to check in occupancy grid. In map coordinates. 
        """
        map_to_use = self.global_costmap_img if use_inflation_layer else self.map_img
        
        img_coords = self.convert_map_coords_to_img_coords(
            (pose.position.x, pose.position.y)
        )
        x, y = img_coords
        height, width = map_to_use.shape[:2]

        if x < 0 or x >= width or y < 0 or y >= height:
            # Most likely the wrong coordinate frame
            return False
        return map_to_use[y, x] == 128

    def is_point_in_occupancy_grid(self, point: Point, use_inflation_layer: bool = False) -> bool:
        """
        Args:
            point (Point): The point to check if in occupancy grid. In map coordinates.
            use_inflation_layer (bool, optional): Whether to use the inflation layer for the check. Defaults to False.
        """
        pose = Pose()
        pose.position = point
        pose.orientation = Quaternion(0, 0, 0, 1)
        return self.is_pose_in_occupancy_grid(pose, use_inflation_layer)

    def convert_img_coords_to_map_coords(
        self, img_coords: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Converts the given coordinates from image coordinates to coordinates in the actual map.

        Args:
            img_coords (Tuple[float, float]): The image coordinates to convert.

        Returns:
            tuple (x: float, y: float): The converted coordinates in the map frame.
        """
        assert self.map
        return self.map.info.origin.position.x + float(
            img_coords[0]
        ) * self.map.info.resolution, self.map.info.origin.position.y + float(
            img_coords[1]
        ) * self.map.info.resolution

    def convert_map_coords_to_img_coords(
        self, map_coords: Tuple[float, float]
    ) -> Tuple[int, int]:
        assert self.map
        x = int(
            (map_coords[0] - self.map.info.origin.position.x) / self.map.info.resolution
        )

        y = int(
            (map_coords[1] - self.map.info.origin.position.y) / self.map.info.resolution
        )
        return x, y
