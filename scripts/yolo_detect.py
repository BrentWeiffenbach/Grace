#!/home/brent/grace_ws/src/grace/yolovenv/bin/python
import os
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO
import requests
from ultralytics.engine.results import Results
import cv2

# Set the environment variable
os.environ['YOLO_VERBOSE'] = 'False'

class YoloDetect:
    def __init__(self):
        rospy.init_node('yolo_detect')
        
        # Publisher
        self.detection_image_pub = rospy.Publisher(
            "/yolo_detect/detections/image", Image, queue_size=5 
        )
        self.semantic_objects_pub = rospy.Publisher(
            "/yolo_detect/detections/objects", PointStamped, queue_size=5
        )

        # Subscribers
        self.rgb_image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_image_callback)

        model_name = 'yolo11n.pt'
        # Download the YOLO model
        if not os.path.isfile(model_name):
            print(f'{model_name} does not exist. Downloading...')
            download_url = 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt'

            response = requests.get(download_url)

            if response.status_code == 200:
                with open(model_name, 'wb') as file:
                    file.write(response.content)
                print(f'Downloaded {model_name}')
            else:
                print(f'Failed to download {model_name}')

        # Load YOLO model
        self.model: YOLO = YOLO(model_name)

        # Store the latest images
        self.latest_rgb_image = None
        self.latest_depth_image = None

        # Timer to call the detection function at a fixed interval (4 times a second)
        self.timer = rospy.Timer(rospy.Duration(0.25), self.detect_objects)

    def depth_image_callback(self, img):
        self.latest_depth_image = ros_numpy.numpify(img)

    def rgb_image_callback(self, img):
        self.latest_rgb_image = ros_numpy.numpify(img)

    def detect_objects(self, event):
        """Callback function to detect objects on RGB image"""
        if self.latest_rgb_image is not None and self.latest_depth_image is not None:
            image_array = self.latest_rgb_image
            depth_array = self.latest_depth_image

            if self.detection_image_pub.get_num_connections():
                result = self.model(image_array, conf=0.5)  # get the results
                det_annotated = result[0].plot(show=False)  # plot the annotations

                # Find the depths of each detection and display them
                for detection in result[0].boxes:
                    x1, y1, x2, y2 = map(int, detection.xyxy[0])
                    det_annotated = cv2.circle(det_annotated, (x1, y1), 5, (0, 255, 0), -1)
                    det_annotated = cv2.circle(det_annotated, (x2, y2), 5, (0, 255, 0), -1)
                    depth = np.nanmean(depth_array[y1:y2, x1:x2])
                    if not np.isnan(depth):
                        label = f"{int(detection.cls.item())} {depth:.2f}m"
                        det_annotated = cv2.putText(det_annotated, label, (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        # TODO FIGURE OUT THE MATH TO PUBLISH IN WORLD FRAME PROPERLY
                        # point = PointStamped()
                        # point.header.stamp = rospy.Time.now()
                        # point.header.frame_id = "camera_link"
                        # cx = (x1 + x2) / 2
                        # cy = (y1 + y2) / 2
                        # fx = 525.0  # Focal length in pixels
                        # fy = 525.0  # Focal length in pixels
                        # cx_offset = 319.5  # Principal point x-coordinate
                        # cy_offset = 239.5  # Principal point y-coordinate
                        # # Set the z height to be the same as the camera link
                        # point.point.z = 0.0
                        # # Calculate the x and y coordinates based on depth and bounding box info
                        # point.point.x = (cx - cx_offset) * depth / fx
                        # point.point.y = (cy - cy_offset) * depth / fy
                        # self.semantic_objects_pub.publish(point)

                self.detection_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))  # publish the annotated image

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    yolo_detect = YoloDetect()
    yolo_detect.run()