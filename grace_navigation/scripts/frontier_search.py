#!/home/brent/mqp_ws/src/Grace/grace_navigation/yolovenv/bin/python


from typing import List, Sequence, Tuple

import cv2
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid


class FrontierSearch:
    def __init__(self) -> None:
        self._map: OccupancyGrid
        self.map_img: cv2.typing.MatLike
        """A variable to store the computed grayscale map"""

    @property
    def map(self) -> OccupancyGrid:
        return self._map

    @map.setter
    def map(self, new_map: OccupancyGrid) -> None:
        self._map = new_map
        self.map_img = self.convert_to_img()

    def convert_to_img(self) -> cv2.typing.MatLike:
        grayscale_map = {-1: 0, 0: 128, 100: 255}
        grid_data = np.array(self.map.data).reshape(
            (self.map.info.height, self.map.info.width)
        )
        grayscale_image = np.vectorize(grayscale_map.get)(grid_data)
        grayscale_image = np.uint8(grayscale_image)
        return grayscale_image  # type: ignore

    def compute_centroids(self) -> List[Point]:
        sift: cv2.SIFT = cv2.SIFT.create()

        image = self.map_img
        # image = np.uint8(image)  # type: ignore
        kp: Sequence[cv2.KeyPoint]
        kp, _ = sift.detectAndCompute(image, None, None, False)  # type: ignore

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
                if any(
                    np.linalg.norm((k.x - ck.x, k.y - ck.y)) < 10
                    for ck in cluster
                ):
                    cluster.append(k)
                    added = True
                    break
            if not added:
                clusters.append([k])

        # Filter clusters with 3 or more keypoints
        useful_kps = [k for cluster in clusters if len(cluster) >= 3 for k in cluster]

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
