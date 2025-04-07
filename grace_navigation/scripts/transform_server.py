#!/usr/bin/env python2.7
import rospy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException  # type: ignore
from grace_navigation.srv import Transform, TransformRequest, TransformResponse  # noqa: F401


class TransformServer:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.service = rospy.Service("transform", Transform, self.transform)

    def transform(self, req):
        # type: (TransformRequest) -> TransformResponse
        res = TransformResponse()
        try:
            trans = self.tf_buffer.lookup_transform(
                req.target_frame,
                req.source_frame,
                rospy.Time(0),
                rospy.Duration(0.01),  # 10 ms # type: ignore
            )
            res.transform = trans
            res.success = True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)
            res.success = False

        return res

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("transform_server")
    transform_server = TransformServer()
    transform_server.run()
