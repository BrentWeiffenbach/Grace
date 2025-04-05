#!/usr/bin/env python2.7
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion
from nav_msgs.msg import Odometry
from numpy.linalg import inv, norm
from scipy.stats import dirichlet
from tf import transformations
from tf2_ros import (
    ConnectivityException,  # type: ignore
    ExtrapolationException,  # type: ignore
    TransformException,  # type: ignore
)
from visualization_msgs.msg import Marker, MarkerArray

from grace_navigation.msg import Object2D, Object2DArray, RangeBearingArray, RobotState


class MapObject:
    def __init__(self, object_id, pos, pos_var, class_probs, obj_class):
        # type: (int, np.ndarray, np.ndarray, np.ndarray, str) -> None
        self.id = object_id  # type: int
        """The MapObject's id.
        """
        self.pos = pos
        """A 2D Position with [x, y]
        """
        self.pos_var = pos_var
        self.pos_var_inv = None
        self._update_inverse()
        self.class_probs = class_probs
        self.obj_class = obj_class  # type: str

    def _update_inverse(self):
        self.pos_var_inv = inv(self.pos_var)

    def update(self, pos, pos_var, class_probs, obj_class):
        # type: (np.ndarray, np.ndarray, np.ndarray, str) -> None
        self.pos = pos
        self.pos_var = pos_var
        self._update_inverse()
        self.class_probs = class_probs
        self.obj_class = obj_class

    def __repr__(self):
        return "MapObject(id={}, pos={}, pos_var={}, class_probs={}, obj_class='{}')".format(
            self.id, self.pos, self.pos_var, self.class_probs, self.obj_class
        )


class SemanticSLAM:
    def __init__(self):
        self.target_frame = "camera_rgb_optical_frame"
        # self.target_frame = "camera_link"
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.ang_var = None
        """Robot angle variance"""
        self.pos_var = None
        """Robot pose variance"""

        self.RANGE_VAR = 5e-03
        self.BEARING_VAR = 0.01
        self.sigma_delta = np.diag([self.RANGE_VAR, self.BEARING_VAR])
        self.sigma_delta_inv = inv(self.sigma_delta)

        self.ang = None
        self.pos = None
        self.sigma_p = None
        self.sigma_p_inv = None

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

        self.ALPHA_CONSTANT = 1.05

        self.all_alphas = []
        self.numberOfClasses = len(self.classes)
        for i in range(self.numberOfClasses):
            alpha = np.ones(self.numberOfClasses)
            alpha[i] = self.ALPHA_CONSTANT
            self.all_alphas.append(alpha)

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.range_sub = rospy.Subscriber(
            "/range_bearing", RangeBearingArray, self.range_callback
        )
        self.map_pub = rospy.Publisher(
            "/semantic_map", Object2DArray, queue_size=10, latch=True
        )
        self.point_pub = rospy.Publisher(
            "/semantic_map/point", PointStamped, queue_size=10
        )
        self.marker_pub = rospy.Publisher(
            "/semantic_map/marker", MarkerArray, queue_size=30
        )
        self.t = 0  # type: int
        """Timestamp, in seconds"""

        self.objects = {}  # type: dict[int, MapObject]
        self.max_obj_id = 0  # type: int
        self.marker_array = MarkerArray()
        self.marker_array.markers = []
        

    def odom_callback(self, msg):
        # only takes the covariance, the pose is taken from tf transformation
        self.pos_var = msg.pose.covariance[0]
        self.ang_var = msg.pose.covariance[-1]
        self.sigma_p = np.diag([self.pos_var, self.pos_var, self.ang_var])
        self.sigma_p_inv = inv(self.sigma_p)
        self.t = msg.header.stamp.to_sec()

    def range_callback(self, msg):
        if self.pos_var is None:
            rospy.loginfo("robot pose covariance is not set")
            return

        # Note that this should not be possible if self.pos_var is not None
        # However, this will sooth Pylance
        if self.ang_var is None:
            rospy.logerr(
                "Angle covariance is somehow None while robot pose covariance is not None."
            )
            return

        from_frame_rel = self.target_frame
        to_frame_rel = "map"

        # region TF
        try:
            trans = self.tfBuffer.lookup_transform(
                to_frame_rel, from_frame_rel, rospy.Time(0)
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

            probability = range_bearing.probability
            probability = np.asarray(probability) / np.sum(probability)  # Normalize probability # fmt: skip

            # Robot's current position
            ux = self.pos[0]  # type: np.float64
            uy = self.pos[1]  # type: np.float64

            # Object's position in world frame
            mx = ux + np.cos(self.ang + bearing) * obj_range  # type: np.float64
            my = uy + np.sin(self.ang + bearing) * obj_range  # type: np.float64

            dist, matched_obj = self.get_closest_object(obj_class, mx, my)

            if dist >= 0.02:
                x_var = (
                    self.pos_var
                    + np.power(np.sin(self.ang + bearing) * obj_range, 2)
                    * (self.ang_var + self.BEARING_VAR)
                    + np.power(np.cos(self.ang + bearing), 2) * self.RANGE_VAR
                )

                y_var = (
                    self.pos_var
                    + np.power(np.cos(self.ang + bearing) * obj_range, 2)
                    * (self.ang_var + self.BEARING_VAR)
                    + np.power(np.sin(self.ang + bearing), 2) * self.RANGE_VAR
                )

                object_pos = np.asarray([mx, my])
                object_pos_var = np.diag([x_var, y_var])

                class_probs = np.zeros(self.numberOfClasses)
                for i in range(self.numberOfClasses):
                    class_probs[i] = dirichlet.pdf(probability, self.all_alphas[i])

                class_probs = class_probs / np.sum(class_probs)  # Normalize class_probs

                self.objects[self.max_obj_id] = MapObject(
                    obj_id, object_pos, object_pos_var, class_probs, obj_class
                )

                if not np.isfinite(object_pos_var).all():
                    print("encounter inf/NAN in initialization")
                    print("current pos var is ", self.pos_var)
                    print("initialized pos variance is ", object_pos_var)
                    print("obj id", self.max_obj_id)
                    print("current bearing is", bearing)
                    print("obj range is ", obj_range)

                self.max_obj_id += 1

            elif dist != 0.0:
                # If no closest object is found, keeping looking at the other objects
                if matched_obj is None:
                    # Should it be continuing, or should it create a new object?
                    continue
                obj_pos = matched_obj.pos
                class_probs = matched_obj.class_probs
                obj_x = obj_pos[0]
                obj_y = obj_pos[1]

                # region Kalman Filter
                sigma_m = matched_obj.pos_var  # Position Variance
                sigma_m_inv = matched_obj.pos_var_inv

                x = self.pos[0]
                y = self.pos[1]

                # Distance between robot and object
                d = norm(np.asarray([x, y]) - obj_pos)

                # Reduce duplicate computations
                x_diff = obj_x - x
                y_diff = obj_y - y
                inv_d = 1.0 / d
                inv_d_squared = inv_d * inv_d

                # Jacobian matrixes
                K1 = np.asarray(
                    [
                        [x_diff * inv_d, y_diff * inv_d],
                        [(-y_diff) * inv_d_squared, x_diff * inv_d_squared],
                    ]
                )

                K2 = np.asarray(
                    [
                        [(-x_diff) * inv_d, (-y_diff) * inv_d, 0],
                        [y_diff * inv_d_squared, (-x_diff) * inv_d_squared, -1],
                    ]
                )
                z = np.asarray([obj_range, bearing])

                # Difference between actual measurements and predicited measurements
                dz = z - np.asarray([d, np.arctan2(y_diff, x_diff) - self.ang])
                dz[1] = np.arctan2(
                    np.sin(dz[1]), np.cos(dz[1])
                )  # Normalize to be within -pi to pi

                psi_inv = (
                    np.matmul(np.matmul(K2.T, self.sigma_delta_inv), K2)
                    + self.sigma_p_inv
                )
                psi = inv(psi_inv)

                # Used to update the pos variance
                M1 = self.sigma_delta_inv - np.matmul(
                    np.matmul(
                        np.matmul(np.matmul(self.sigma_delta_inv, K2), psi), K2.T
                    ),
                    self.sigma_delta_inv,
                )

                updated_pos_var = inv(np.matmul(np.matmul(K1.T, M1), K1) + sigma_m_inv)

                if not np.isfinite(updated_pos_var).all():
                    print("encounter inf/NAN in update")
                    print("updated pos variance is ", updated_pos_var)
                    print("current pos var is ", self.pos_var)
                    print("old obj pos var is ", sigma_m)
                    print("old obj pos var invert is ", sigma_m_inv)
                    print("K1 is", K1)
                    print("M1 is", M1)

                # Calculate Kalman gain
                K = np.matmul(np.matmul(updated_pos_var, K1.T), M1)
                updated_pos = obj_pos + np.matmul(K, dz)
                # endregion

                for i in range(self.numberOfClasses):
                    alpha = np.ones(self.numberOfClasses)
                    alpha[i] = self.ALPHA_CONSTANT
                    class_probs[i] = dirichlet.pdf(probability, alpha) * class_probs[i]

                class_probs = np.asarray(class_probs)
                class_probs = np.asarray(class_probs) / np.sum(class_probs)
                class_probs = (class_probs + 0.004) / (
                    class_probs + 0.004 * len(self.classes)
                )
                # BUG: self.classes[np.argmax(class_probs)] is not actually returning the correct value,
                # and defaulting to 0 (person)
                # if self.classes[np.argmax(class_probs)] == "person":
                #     continue
                # print(class_probs)
                # matched_obj.update(
                #     updated_pos,
                #     updated_pos_var,
                #     class_probs,
                #     self.classes[np.argmax(class_probs)],
                # )
                matched_obj.update(
                    updated_pos,
                    updated_pos_var,
                    class_probs,
                    obj_class,
                )
            else:
                rospy.logerr("Garbage Range Bearings")

        semantic_map_msg = Object2DArray()
        semantic_map_msg.header = msg.header
        objects = self.create_objects()
        semantic_map_msg.objects = objects
        # self.add_markers(to_frame_rel)
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
    
    def publish_objects(self, to_frame_rel):
        for obj_id in self.objects:
            obj = self.objects[obj_id]
            _point = PointStamped()
            _point.header.frame_id = to_frame_rel
            _point.header.stamp = rospy.Time.now()
            _point.point.x = obj.pos[0]
            _point.point.y = obj.pos[1]
            # self.point_pub.publish(_point)
            yield obj
            
    def add_markers(self, to_frame_rel):
        if self.marker_array.markers is None:
            self.marker_array.markers = []

        for obj in self.publish_objects(to_frame_rel):
            # Check if marker with the same id already exists
            # if any(_marker.id == obj.id and _marker.ns == obj.obj_class for _marker in self.marker_array.markers):
            #     continue

            # Check if the object already exists
            existing_marker = None  # type: Marker | None
            for _marker in self.marker_array.markers:
                if _marker.id == obj.id and _marker.ns == obj.obj_class:
                    existing_marker = _marker
                    break

            if existing_marker:
                self.edit_marker(existing_marker, obj)
            else:
                marker = self.create_new_marker(to_frame_rel, obj)  # type: Marker
                self.marker_array.markers.append(marker)

        self.marker_pub.publish(self.marker_array)

    def edit_marker(self, marker, obj):
        # type: (Marker, MapObject) -> Marker
        """Edit an existing marker with obj's details

        Args:
            marker (Marker): The marker to edit
            obj (MapObject): The details to transform Marker into

        Returns:
            Marker: The changed Marker object
        """
        marker.header.stamp = rospy.Time.now()
        marker.pose.position = Point(obj.pos[0], obj.pos[1], 0)
        marker.text = "{} [{}]".format(obj.obj_class, obj.id)

        return marker

    def create_new_marker(self, to_frame_rel, obj):
        # type: (str, MapObject) -> Marker
        """Creates a marker in `to_frame_rel` frame with `obj`'s attributes.

        Args:
            to_frame_rel (str): The frame to put the marker in
            obj (MapObject): The MapObject containing the marker's information.

        Returns:
            Marker: A marker with the same position as `obj` and the text as `obj.obj_class [obj.id]`
        """
        marker = Marker()
        marker.frame_locked = False
        marker.header.frame_id = to_frame_rel
        marker.header.stamp = rospy.Time.now()
        marker.ns = obj.obj_class
        marker.id = obj.id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(obj.pos[0], obj.pos[1], 0)
        marker.text = "{} [{}]".format(obj.obj_class, obj.id)

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1

        marker.pose.orientation = Quaternion(0, 0, 0, 1)

        return marker
    
    def remove_marker(self, marker_id):
        assert self.marker_array.markers is not None
        _markers = self.marker_array.markers # type: list[Marker]
        for marker in _markers:
            if marker.id == marker_id:
                self.marker_array.markers.remove(marker)

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
            obj_msg.probability = obj.class_probs.tolist()
            obj_msg.cls = obj.obj_class

            objects.append(obj_msg)
        return objects


if __name__ == "__main__":
    rospy.init_node("semantic_slam")
    rospy.loginfo("Press Ctrl + C to terminate")
    # while True:
    #     state_msg = rospy.wait_for_message(topic="/grace/state", topic_type=RobotState)
    #     if state_msg.state == RobotState.EXPLORING:  # type: ignore
    #         break
    whatever = SemanticSLAM()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
