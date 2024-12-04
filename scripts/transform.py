#!/usr/bin/env python2.7
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped


class TransformNode:
    def __init__(self):
        rospy.init_node(name="transform_node")
        rospy.loginfo_once(msg="TransformNode node initialized")

        # Publishers
        self.transformed_point_pub = rospy.Publisher(
            name="/transform_node/base_link_frame_point",
            data_class=PointStamped,
            queue_size=5,
        )

        # Subscribers
        # scripts/yolo_detect.py
        self.camera_point_sub = rospy.Subscriber(
            name="/yolo_detect/detections/objects",
            data_class=PointStamped,
            callback=self.transform_point,
        )

    def transform_point(self, point):
        """Callback function which detects objects"""
        if point is None:
            return

        # https://stackoverflow.com/questions/56054356/in-ros-how-to-transform-pose-from-kinect-frame-to-pr2s-base-link-frame
        TF_BUFFER_LENGTH = rospy.Duration(secs=1200)
        """The buffer duration (in seconds) for the tf listener
        """

        tf_buffer = tf2_ros.Buffer(TF_BUFFER_LENGTH)  # tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        transform = tf_buffer.lookup_transform(
            target_frame="base_link",
            source_frame=point.header.frame_id,  # source frame
            time=rospy.Time(0),  # get the tf at first available time
            timeout=rospy.Duration(1),
        )

        point_transformed = tf2_geometry_msgs.do_transform_point(point, transform)
        self.transformed_point_pub.publish(point_transformed)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        transform_node = TransformNode()
        transform_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Transform node terminated.")
