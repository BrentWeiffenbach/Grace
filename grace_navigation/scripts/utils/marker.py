from typing import List, Tuple, Union, Callable, Optional
import numpy as np
import rospy
from geometry_msgs.msg import Point, Quaternion
from grace_node import GraceNode
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class MarkerPublisher:
    def __init__(
        self,
        marker_publisher: rospy.Publisher,
        frame_id: str = "map",
        verbose: bool = False,
    ):
        self.marker_publisher = marker_publisher
        self.frame_id = frame_id
        self.verbose = verbose

    def publish_point_markers(
        self,
        points: List[Point],
        namespace: str = "frontiers",
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
        scale: Tuple[float, float, float] = (0.15, 0.15, 0.15),
        marker_type: int = Marker.SPHERE,
        lifetime: rospy.Duration = rospy.Duration(0),
    ):
        marker_array = MarkerArray()
        marker_array.markers = []

        for marker_id, point in enumerate(points):
            marker = self._create_base_marker(
                namespace, marker_id, point, marker_type, scale, color, lifetime
            )
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def publish_text_markers(
        self,
        points: List[Point],
        texts: List[str],
        namespace: str = "labels",
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
        scale: Tuple[float, float, float] = (0.15, 0.15, 0.15),
        lifetime: rospy.Duration = rospy.Duration(0),
    ):
        if len(points) != len(texts):
            if self.verbose:
                rospy.logwarn("Number of points and texts don't match")
            return

        marker_array = MarkerArray()
        marker_array.markers = []

        for marker_id, (point, text) in enumerate(zip(points, texts)):
            marker = self._create_base_marker(
                namespace,
                marker_id,
                point,
                Marker.TEXT_VIEW_FACING,
                scale,
                color,
                lifetime,
            )
            marker.text = text
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def publish_scored_markers(
        self,
        scored_points: List[Tuple[Point, Union[np.floating, float]]],
        namespace: str = "scored_points",
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
        scale: Tuple[float, float, float] = (0.15, 0.15, 0.15),
        text_format: Callable[[float], str] = lambda score: f"Score: {round(score, 2)}",
        lifetime: rospy.Duration = rospy.Duration(0),
    ):
        points = [p for p, _ in scored_points]
        texts = [text_format(float(score)) for _, score in scored_points]

        self.publish_text_markers(
            points=points,
            texts=texts,
            namespace=namespace,
            color=color,
            scale=scale,
            lifetime=lifetime,
        )

    def publish_object_goal_markers(
        self,
        points: List[Point],
        goal_name: str,
        has_object: Optional[bool] = None,
        has_object_topic: str = GraceNode.HAS_OBJECT_TOPIC,
        pick_location_name: str = "",
        place_location_name: str = "",
        namespace: str = "Object_Goal",
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
        scale: Tuple[float, float, float] = (0.15, 0.15, 0.15),
    ):
        if has_object is None:
            try:
                has_object_msg = rospy.wait_for_message(
                    topic=has_object_topic,
                    topic_type=Bool,
                    timeout=rospy.Duration(10),
                )
                assert has_object
                has_object = has_object_msg.data
            except (rospy.ROSException, rospy.ROSInterruptException):
                if self.verbose:
                    rospy.logwarn(
                        "Failed to receive message for has_object. Defaulting to False."
                    )
                has_object = False

        if pick_location_name and place_location_name:
            target_obj_name = place_location_name if has_object else pick_location_name
        else:
            target_obj_name = goal_name

        texts = [target_obj_name] * len(points)

        self.publish_text_markers(
            points=points, texts=texts, namespace=namespace, color=color, scale=scale
        )

    def _create_base_marker(
        self,
        namespace: str,
        marker_id: int,
        position: Point,
        marker_type: int,
        scale: Tuple[float, float, float],
        color: Tuple[float, float, float],
        lifetime: rospy.Duration,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.lifetime = lifetime

        return marker
