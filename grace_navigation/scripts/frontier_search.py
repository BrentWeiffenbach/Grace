from typing import List, Sequence, Tuple

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
        grayscale_map = {-1: 0, 0: 128, 100: 255}
        grid_data = np.array(occupancy_grid.data).reshape(
            (occupancy_grid.info.height, occupancy_grid.info.width)
        )
        # grayscale_image = np.vectorize(grayscale_map.get)(grid_data)
        grayscale_image = np.where(
            grid_data == -1,
            0,
            np.where(grid_data == 0, 128, np.where(grid_data == 100, 255, 0)),
        )
        grayscale_image = np.uint8(grayscale_image)
        return grayscale_image  # type: ignore

    def compute_centroids(self) -> List[Point]:
        # sift: cv2.SIFT = cv2.SIFT.create()
        orb = cv2.ORB.create()

        use_me = self.global_costmap_img if hasattr(self, 'global_costmap_img') else self.map_img
        image = use_me.copy()
        # image = np.uint8(image)  # type: ignore
        # image = cv2.Canny(image, 1.2, 0.5, None)
        six_by_six = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))
        three_by_three = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        cv2.imwrite("a.png", image)
        image = cv2.erode(image, six_by_six, iterations=2)
        cv2.imwrite("b.png", image)
        image = cv2.dilate(image, three_by_three, iterations=1)
        cv2.imwrite("c.png", image)
        kp: Sequence[cv2.KeyPoint]
        kp, _ = orb.detectAndCompute(image, None, None, False)  # type: ignore

        im3 = cv2.drawKeypoints(
            image,
            kp,
            None,  # type: ignore
            (0, 255, 0),
            cv2.DRAW_MATCHES_FLAGS_DEFAULT,
        )  # type: ignore

        valid_kps = [k for k in kp if 128 >= image[int(k.pt[1]), int(k.pt[0])] > 0]
        keypoints: List[Point] = [
            Point(*self.convert_img_coords_to_map_coords((k.pt[0], k.pt[1])), 0.0)
            for k in valid_kps
        ]

        # Group keypoints by proximity
        clusters = []
        for k in keypoints:
            added = False
            for cluster in clusters:
                if any(np.linalg.norm((k.x - ck.x, k.y - ck.y)) < 10 for ck in cluster):
                    cluster.append(k)
                    added = True
                    break
            if not added:
                clusters.append([k])

        # Filter clusters with 3 or more keypoints
        useful_kps = [k for cluster in clusters if len(cluster) >= 10 for k in cluster]

        cv2.imwrite("5.png", im3)
        return useful_kps

    def is_pose_in_occupancy_grid(self, pose: Pose) -> bool:
        img_coords = self.convert_map_coords_to_img_coords(
            (pose.position.x, pose.position.y)
        )
        x, y = img_coords
        height, width = self.map_img.shape[:2]

        if x < 0 or x >= width or y < 0 or y >= height:
            return False

        radius = 5  # px
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                # Skip points outside the circle
                if dx * dx + dy * dy > radius * radius:
                    continue

                check_x, check_y = x + dx, y + dy
                # Skip points outside the map
                if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                    continue

                if self.map_img[check_y, check_x] == 255:
                    return False

        return self.map_img[y, x] == 128

    def is_point_in_occupancy_grid(self, point: Point) -> bool:
        pose = Pose()
        pose.position = point
        pose.orientation = Quaternion(0, 0, 0, 1)
        return self.is_pose_in_occupancy_grid(pose)

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
