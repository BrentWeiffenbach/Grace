#!/usr/bin/env python2.7
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from tf import TransformBroadcaster, transformations
from tf2_ros import (
    ConnectivityException,  # type: ignore
    ExtrapolationException,  # type: ignore
    TransformException,  # type: ignore
)

from grace_grasping.srv import (
    GetObjectPose,
    GetObjectPoseRequest,  # noqa: F401
    GetObjectPoseResponse,
)
from grace_navigation.msg import RangeBearing, RangeBearingArray


class GetObjectPoseServer:
    def __init__(self):
        rospy.init_node("get_object_pose_server")
        self.br = TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.ref_link_frame = "map"
        self.camera_height = 0.3048  # measured
        self.RANGE_BEARING_CACHE_SIZE = 5
        """The number of range bearings to store in cache"""
        self.range_bearings_buffer = []
        """Cached range bearings. The most recent one is located at index 0. Cleared after every service call.
        """

        self.range_bearing_sub = rospy.Subscriber(
            "/range_bearing", RangeBearingArray, self.range_bearing_callback
        )
        self.service = rospy.Service(
            "get_object_pose", GetObjectPose, self.service_callback
        )

    def range_bearing_callback(self, msg):
        """Callback to store the latest RangeBearingArray message."""
        if len(self.range_bearings_buffer) == self.RANGE_BEARING_CACHE_SIZE:
            self.range_bearings_buffer.pop()  # Remove oldest element
        self.range_bearings_buffer.insert(0, msg)  # Insert newest range bearing
        
    def flush_rb_buffer(self):
        self.range_bearings_buffer = []

    def service_callback(self, msg):
        # type: (GetObjectPoseRequest) -> GetObjectPoseResponse
        success, obj_range_bearing = self.check_for_object(msg.target_obj_name)
        if msg.only_detect:
            return GetObjectPoseResponse(PoseStamped(), success)

        if not success:
            return GetObjectPoseResponse(PoseStamped(), success)
        return self.get_object_pose(obj_range_bearing)

    def check_for_object(self, obj):
        # type: (str) -> (bool, RangeBearing) # type: ignore
        rospy.loginfo("Received request for object: {}".format(obj))
        self.flush_rb_buffer()
        MAX_TIMEOUT = 5 # 5 seconds
        start_time = rospy.Time.now()

        while (len(self.range_bearings_buffer) < self.RANGE_BEARING_CACHE_SIZE):
            # Wait until range_bearings_buffer is RANGE_BEARING_CACHE_SIZE items long
            if (rospy.Time.now() - start_time).to_sec() > MAX_TIMEOUT:
                rospy.logwarn("Timeout waiting for range bearings buffer to fill")
                break
            rospy.sleep(0.1)
        
        for range_bearing_array in self.range_bearings_buffer:
            for detection in range_bearing_array.range_bearings:
                    if detection.obj_class == obj:
                        rospy.loginfo("Object {} found.".format(obj))
                        self.flush_rb_buffer()
                        return True, detection

        rospy.logwarn("Object {} not found.".format(obj))
        self.flush_rb_buffer()
        return False, RangeBearing()

    def get_object_pose(self, range_bearing):
        # type: (RangeBearing) -> GetObjectPoseResponse
        res = GetObjectPoseResponse()
        res.pose = PoseStamped()

        target_frame_rel = self.ref_link_frame
        source_frame_rel = "camera_rgb_optical_frame"
        # use tf to find position
        try:
            trans = self.tfBuffer.lookup_transform(
                target_frame_rel, source_frame_rel, rospy.Time(0)
            )

            pos = np.asarray(
                [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                ]
            )

            ang = transformations.euler_from_quaternion(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )[-1]
            ang += np.pi / 2
            # Extract ID

            # Robot's current position
            ux = pos[0]  # type: np.float64
            uy = pos[1]  # type: np.float64

            # Object's position in world frame
            object_x = ux + np.cos(ang + range_bearing.bearing) * range_bearing.range  # type: np.float64
            object_y = uy + np.sin(ang + range_bearing.bearing) * range_bearing.range  # type: np.float64
            object_z = pos[2] + range_bearing.range * np.sin(
                range_bearing.elevation
            )  # Calculate object's z coordinate

            res.pose.header.frame_id = self.ref_link_frame
            res.pose.header.stamp = rospy.Time.now()
            res.pose.pose.position = Point(x=object_x, y=object_y, z=object_z)
            res.pose.pose.orientation.x = 0
            res.pose.pose.orientation.y = 0
            res.pose.pose.orientation.z = 0
            res.pose.pose.orientation.w = 1

            # DEBUG: Create a new TF frame for the object
            # object_id = range_bearing.id
            # object_frame = "object_{}".format(object_id)

            # self.br.sendTransform(
            #     (object_x, object_y, object_z),
            #     (0, 0, 0, 1),  # No rotation (quaternion)
            #     rospy.Time.now(),
            #     object_frame,
            #     self.ref_link_frame,
            # )
            res.success = True
        except (TransformException, ConnectivityException, ExtrapolationException):
            res.success = False

        return res


if __name__ == "__main__":
    try:
        GetObjectPoseServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
