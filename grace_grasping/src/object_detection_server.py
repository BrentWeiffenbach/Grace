#!/usr/bin/env python2.7
import rospy
from grace_grasping.srv import ObjectDetection, ObjectDetectionResponse
from grace_navigation.msg import RangeBearing, RangeBearingArray


class ObjectDetectionServer:
    def __init__(self):
        rospy.init_node("object_detection_server")
        self.range_bearing_sub = rospy.Subscriber(
            "/range_bearing", RangeBearingArray, self.range_bearing_callback
        )
        self.service = rospy.Service(
            "object_detection", ObjectDetection, self.handle_request
        )
        self.range_bearings = None
        rospy.loginfo("RangeBearingServer initialized and waiting for requests.")

    def range_bearing_callback(self, msg):
        """Callback to store the latest RangeBearingArray message."""
        self.range_bearings = msg.range_bearings

    def handle_request(self, req):
        """Handle incoming service requests."""
        target_obj_name = req.target_obj_name
        rospy.loginfo("Received request for object: {}".format(target_obj_name))

        if self.range_bearings is not None:
            for detection in self.range_bearings:
                if detection.obj_class == target_obj_name:
                    rospy.loginfo("Object {} found.".format(target_obj_name))
                    return ObjectDetectionResponse(
                        range_bearing=detection, success=True
                    )

        rospy.logwarn("Object {} not found.".format(target_obj_name))
        return ObjectDetectionResponse(range_bearing=RangeBearing(), success=False)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    server = ObjectDetectionServer()
    server.run()
