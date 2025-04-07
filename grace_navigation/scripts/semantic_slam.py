#!/usr/bin/env python2.7
# TODO: Remove all of the extra calculations and optimzie
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int16
from tf import transformations
from tf2_ros import (
    ConnectivityException,  # type: ignore
    ExtrapolationException,  # type: ignore
    TransformException,  # type: ignore
)

from grace_navigation.msg import Object2D, Object2DArray, RangeBearingArray


class MapObject:
    def __init__(self, object_id, pos, obj_class):
        # type: (int, np.ndarray, str) -> None
        self.id = object_id  # type: int
        """The MapObject's id.
        """
        self.pos = pos
        """A 2D Position with [x, y]
        """
        self.pos_var = np.diag([0.01, 0.01])  # Small fixed variance
        self.obj_class = obj_class  # type: str

    def update(self, pos, obj_class):
        # type: (np.ndarray, str) -> None
        self.pos = pos
        self.obj_class = obj_class

    def __repr__(self):
        return "MapObject(id={}, pos={}, pos_var={}, obj_class='{}')".format(
            self.id, self.pos, self.pos_var, self.obj_class
        )


class SemanticSLAM:
    def __init__(self):
        self.target_frame = "camera_rgb_optical_frame"
        # self.target_frame = "camera_link"
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.ang = None
        self.pos = None

        self.classes = ["person","bicycle","car","motorcycle","airplane","bus","train",
                         "truck","boat","traffic light","fire hydrant","stop sign",
                         "parking meter","bench","bird","cat","dog","horse","sheep",
                         "cow","elephant","bear","zebra","giraffe","backpack","umbrella",
                         "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
                         "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
                         "bottle","wine glass","cup","fork","knife","spoon","bowl","banana",
                         "apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
                         "donut","cake","chair","couch","potted plant","bed","dining table",
                         "toilet","tvmonitor","laptop","mouse","remote","keyboard","cell phone",
                         "microwave","oven","toaster","sink","refrigerator","book","clock",
                         "vase","scissors","teddy bear","hair drier","toothbrush"]  # fmt: skip

        self.numberOfClasses = len(self.classes)

        self.range_sub = rospy.Subscriber(
            "/range_bearing", RangeBearingArray, self.range_callback
        )
        self.remove_sub = rospy.Subscriber(
            "/semantic_map/remove", Int16, self.remove_callback
        )
        self.map_pub = rospy.Publisher(
            "/semantic_map", Object2DArray, queue_size=10, latch=True
        )
        self.point_pub = rospy.Publisher(
            "/semantic_map/point", PointStamped, queue_size=10
        )
        self.t = 0  # type: int
        """Timestamp, in seconds"""

        self.objects = {}  # type: dict[int, MapObject]
        self.max_obj_id = 0  # type: int

    def remove_callback(self, msg):
        semantic_map_msg = Object2DArray()
        # Check if the object ID in the message exists in the current map
        # Filter out the object with the given ID and re-add the rest
        self.objects = {
            obj.id: obj for obj in self.objects.values() if obj.id != msg.data
        }

        # Update the semantic map message with the remaining objects
        semantic_map_msg.objects = self.create_objects()
        self.map_pub.publish(semantic_map_msg)

    def range_callback(self, msg):
        from_frame_rel = self.target_frame
        to_frame_rel = "map"
        # region TF
        try:
            trans = self.tfBuffer.lookup_transform(
                to_frame_rel, from_frame_rel, msg.header.stamp
            )

            self.pos = np.asarray(
                [trans.transform.translation.x, trans.transform.translation.y]
            )

            self.ang = transformations.euler_from_quaternion(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )[-1]
            self.ang += np.pi / 2

        except (TransformException,ConnectivityException, ExtrapolationException):  # fmt: skip
            rospy.loginfo(
                "Could not transform %s to %s: ", to_frame_rel, from_frame_rel
            )
            return
        # endregion
        for range_bearing in msg.range_bearings:
            obj_range = range_bearing.range
            bearing = range_bearing.bearing
            obj_id = range_bearing.id  # type: int
            obj_class = range_bearing.obj_class  # type: str
            if obj_class == "person":
                continue

            # Robot's current position
            ux = self.pos[0]  # type: np.float64
            uy = self.pos[1]  # type: np.float64

            # Object's position in world frame
            mx = ux + np.cos(self.ang + bearing) * obj_range  # type: np.float64
            my = uy + np.sin(self.ang + bearing) * obj_range  # type: np.float64

            dist, matched_obj = self.get_closest_object(obj_class, mx, my)

            if dist >= 0.02:
                object_pos = np.asarray([mx, my])

                self.objects[self.max_obj_id] = MapObject(obj_id, object_pos, obj_class)

                self.max_obj_id += 1

            elif dist != 0.0 and matched_obj is not None:
                obj_pos = matched_obj.pos * 0.7 + np.asarray([mx, my]) * 0.3
                matched_obj.update(
                    obj_pos,
                    obj_class,
                )
            else:
                rospy.logerr("Garbage Range Bearings")

        semantic_map_msg = Object2DArray()
        semantic_map_msg.header = msg.header
        objects = self.create_objects()
        semantic_map_msg.objects = objects
        self.map_pub.publish(semantic_map_msg)

    def get_closest_object(self, obj_class, mx, my):
        """Gets the closest object with the class `obj_class` based on the world coordinates (`mx`, `my`)

        Args:
            obj_class (str): The object class.
            mx (numpy.float64): The world coordinates x.
            my (numpy.float64): The world coordinates y.

        Returns:
            tuple(float, MapObject | None): dist, matched_obj
        """
        dist = np.inf
        matched_obj = None  # Clostest object
        # Search for the closest object of the same class
        for obj in self.objects.values():
            if obj_class == obj.obj_class:
                dist_temp = np.sqrt((mx - obj.pos[0]) ** 2 + (my - obj.pos[1]) ** 2)
                if dist_temp < dist:
                    dist = dist_temp
                    matched_obj = obj
        return dist, matched_obj

    def create_objects(self):
        """Creates a list of Object2D's based on `self.objects`.

        Returns:
            list[Object2D]: A list of the objects in msg form.
        """
        objects = []  # type: list[Object2D]
        used_ids = set()
        for obj_id in reversed(list(self.objects.keys())):
            obj = self.objects[obj_id]

            if obj.id in used_ids:
                continue

            used_ids.add(obj.id)

            obj_msg = Object2D()
            obj_msg.x = obj.pos[0]
            obj_msg.y = obj.pos[1]

            obj_msg.covariance = obj.pos_var.flatten()
            obj_msg.id = obj.id
            obj_msg.cls = obj.obj_class

            objects.append(obj_msg)
        return objects


if __name__ == "__main__":
    rospy.init_node("semantic_slam")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = SemanticSLAM()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
