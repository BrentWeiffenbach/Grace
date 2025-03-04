#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python


from typing import Sequence, Tuple
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


class FrontierSearch:
    def __init__(self) -> None:
        self.map: OccupancyGrid
        self.map_img: cv2.typing.MatLike
        """A variable to store the computed grayscale map"""
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.est_goal_sub = rospy.Subscriber(
            "/semantic_map/est_goal_pose", Pose, self.est_goal_cb
        )
        self.markpub = rospy.Publisher(
            "/frontier/centroids", MarkerArray, queue_size=10
        )

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map = msg
        self.map_img = self.convert_to_img()

        # TODO: Move this out of map callback to its own on demand callback
        self.compute_centroids(self.map_img)

    def est_goal_cb(self, msg: Pose) -> None: ...  # TODO: Use for selecting frontiers

    def convert_to_img(self) -> cv2.typing.MatLike:
        grayscale_map = {-1: 0, 0: 128, 100: 255}
        grid_data = np.array(self.map.data).reshape(
            (self.map.info.height, self.map.info.width)
        )
        grayscale_image = np.vectorize(grayscale_map.get)(grid_data)
        grayscale_image = np.uint8(grayscale_image)
        return grayscale_image  # type: ignore

    def compute_centroids(self, image: cv2.typing.MatLike):
        sift: cv2.SIFT = cv2.SIFT.create()

        image = np.uint8(image)  # type: ignore
        kp: Sequence[cv2.KeyPoint]
        kp, _ = sift.detectAndCompute(image, None, None, False)  # type: ignore

        valid_kps = [k for k in kp if 128 >= image[int(k.pt[1]), int(k.pt[0])] > 0]

        # TODO: Change this from a MarkerArray to a more suitable topic type
        marker_array = MarkerArray()
        marker_array.markers = []
        marker_id = 0

        for k in valid_kps:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "sift_keypoints"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = Point(
                *self.convert_img_coords_to_map_coords((k.pt[0], k.pt[1])), 0.0
            )

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.orientation = Quaternion(0, 0, 0, 1)

            marker.lifetime = rospy.Duration(0)

            marker_array.markers.append(marker)
            marker_id += 1
        self.markpub.publish(marker_array)
        # return cv2.drawKeypoints(image, valid_kps, None, (255, 0, 0), flags=0)  # type: ignore

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

    def run(self) -> None:
        rospy.spin()

    def shutdown(self) -> None:
        rospy.loginfo("Shutting down FrontierSearch...")


if __name__ == "__main__":
    rospy.init_node("frontier_search")
    frontier_search = FrontierSearch()
    rospy.on_shutdown(frontier_search.shutdown)
    try:
        frontier_search.run()
    except rospy.ROSInterruptException:
        frontier_search.shutdown()
        rospy.loginfo("FrontierSearch shut down.")
