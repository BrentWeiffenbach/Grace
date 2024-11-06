#!/home/brent/grace_ws/src/grace/yolovenv/bin/python
import rospy
import ros_numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO
from ultralytics.engine.results import Results

class YoloDetect:
    def __init__(self):
        rospy.init_node('yolo_detect')
        
        # Publisher
        # publish detections on an image
        self.detection_image_pub = rospy.Publisher(
            "/yolo_detect/detections/image", Image, queue_size=5 
        )

        self.detection_distance_pub = rospy.Publisher(
            "/yolo_detect/detections/distance", String, queue_size=5
        )

        # Subscribers
        # subscribe to raw image data
        rospy.Subscriber("/camera/rgb/image_raw", Image, callback=self.detect_objects)
        
        # subscribe to depth image data
        rospy.Subscriber("/camera/depth/image_raw", Image, callback=self.calculate_depth)

        # load yolo model
        self.model: YOLO = YOLO("yolo11s.pt")

        self.detections = []

    def detect_objects(self, msg: Image):
        """Callback function to detect objects on RGB image"""
        array = ros_numpy.numpify(msg)
        if self.detection_image_pub.get_num_connections():
            result = self.model(array) # get the results
            det_annotated = result[0].plot(show=False) # plot the annotations
            self.detection_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8")) # publish the annotated image

    def calculate_depth(self, msg: Image):
        """Callback function to find depth of detections"""
        return None

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    yolo_detect = YoloDetect()
    yolo_detect.run()