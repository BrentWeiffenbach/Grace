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

        # Subscribers
        # subscribe to depth image data
        rospy.Subscriber("/camera/depth/image_raw", Image, callback=self.depth_callback)

        # load yolo model
        self.model: YOLO = YOLO("yolo11s.pt")

        # Timer to call the detection function at a fixed interval (4 times a second)
        self.timer = rospy.Timer(rospy.Duration(0.25), self.detect_objects)

        # Store the latest depth image
        self.latest_depth_image = None

    def depth_callback(self, msg: Image):
        """Callback function to store the latest depth image"""
        self.latest_depth_image = msg

    def detect_objects(self, event):
        """Callback function to detect objects on RGB image"""
        if self.latest_depth_image is not None:
            image = rospy.wait_for_message("/camera/rgb/image_raw", Image)
            array = ros_numpy.numpify(image)
            depth_array = ros_numpy.numpify(self.latest_depth_image)
            if self.detection_image_pub.get_num_connections():
                result = self.model(array, conf=0.5) # get the results
                det_annotated = result[0].plot(show=False) # plot the annotations
                self.detection_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8")) # publish the annotated image

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    yolo_detect = YoloDetect()
    yolo_detect.run()