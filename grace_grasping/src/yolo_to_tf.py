#!/usr/bin/env python2.7
import rospy
from tf import TransformBroadcaster, transformations
import tf2_ros
from tf2_ros import (
    ConnectivityException,  # type: ignore
    ExtrapolationException,  # type: ignore
    TransformException,  # type: ignore
)
from geometry_msgs.msg import PoseStamped, Point
from grace_grasping.srv import GetObjectPoseResponse, GetObjectPoseRequest  # noqa: F401
import numpy as np


class GetObjectPoseServer:
    def __init__(self):
        rospy.init_node("range_to_tf_broadcaster")
        self.br = TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.ref_link_frame = "base_link"
        self.camera_height = 0.3048  # measured
        self.service = rospy.Service(
            "get_object_pose", GetObjectPoseServer, self.callback
        )

    def callback(self, msg):
        # type: (GetObjectPoseRequest) -> GetObjectPoseResponse
        res = GetObjectPoseResponse()
        res.pose = PoseStamped()
        
        target_frame_rel = "base_link"
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
            object_x = (
                ux + np.cos(ang + msg.range_bearing.bearing) * msg.range_bearing.range
            )  # type: np.float64
            object_y = (
                uy + np.sin(ang + msg.range_bearing.bearing) * msg.range_bearing.range
            )  # type: np.float64
            object_z = pos[2] + msg.range_bearing.range * np.sin(
                msg.range_bearing.elevation
            )  # Calculate object's z coordinate

            res.pose.header.frame_id = self.ref_link_frame
            res.pose.header.stamp = rospy.Time.now()
            res.pose.pose.position = Point(x=object_x, y=object_y, z=object_z)
            res.pose.pose.orientation.x = 0
            res.pose.pose.orientation.y = 0
            res.pose.pose.orientation.z = 0
            res.pose.pose.orientation.w = 1
            # DEBUG: Create a new TF frame for the object
            # object_id = msg.range_bearing.id
            # object_frame = "object_{}".format(object_id)

            # self.br.sendTransform(
            #     (object_x, object_y, object_z),
            #     (0, 0, 0, 1),  # No rotation (quaternion)
            #     rospy.Time.now(),
            #     object_frame,
            #     self.ref_link_frame,
            # )
        except (TransformException, ConnectivityException, ExtrapolationException):
            pass

        return res


if __name__ == "__main__":
    try:
        GetObjectPoseServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
