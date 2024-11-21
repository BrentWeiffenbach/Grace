#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python
import os
from typing import Final, List, Union
import rospy
import ros_numpy
import numpy as np 
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO
import requests
from ultralytics.engine.results import Boxes, Results
import cv2
import math

# Set the environment variable
os.environ['YOLO_VERBOSE'] = 'False'
DETECTION_INTERVAL: Final[Union[int, float]] = 0.25
"""The interval (in seconds) between detections. 0.25 is 4 times per second. 1 is 1 time per second.
"""

class YoloDetect:
    def __init__(self) -> None:
        rospy.init_node(name='yolo_detect')
        
        # Publisher
        self.detection_image_pub: rospy.Publisher = rospy.Publisher(
            name="/yolo_detect/detections/image", data_class=Image, queue_size=5 
        )
        self.semantic_objects_pub: rospy.Publisher = rospy.Publisher(
            name="/yolo_detect/detections/objects", data_class=PointStamped, queue_size=5
        )

        # Subscribers
        self.rgb_image_sub = rospy.Subscriber(name="/camera/rgb/image_raw", data_class=Image, callback=self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber(name="/camera/depth/image_raw", data_class=Image, callback=self.depth_image_callback)
        self.transformed_point_sub = rospy.Subscriber(name="transform_node/base_link_frame_point", data_class=PointStamped, callback=self.transformed_point_callback)

        MODEL_NAME: Final = 'yolo11s.pt'
        # Download the YOLO model to the grace folder
        MODEL_PATH: Final[str] = os.path.join(os.path.dirname(__file__), MODEL_NAME)
        if not os.path.isfile(path=MODEL_PATH):
            print(f'{MODEL_PATH} does not exist. Downloading...')
            download_url = 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s.pt'

            response: requests.Response = requests.get(url=download_url)

            if response.status_code == 200:
                with open(file=MODEL_PATH, mode='wb') as file:
                    file.write(response.content)
                print(f'Downloaded {MODEL_PATH}')
            else:
                print(f'Failed to download {MODEL_PATH}')

        # MODEL_NAME: str = model_path # Why would you need to change the name of this when you can just use the path below instead?

        # Load YOLO model
        self.model: YOLO = YOLO(model=MODEL_PATH)

        # Store the latest images
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.transformed_point = None

        # Timer to call the detection function at a fixed interval (4 times a second)
        self.timer = rospy.Timer(period=rospy.Duration(secs=DETECTION_INTERVAL), callback=self.detect_objects) # type: ignore

    def depth_image_callback(self, img) -> None:
        self.latest_depth_image = ros_numpy.numpify(msg=img)

    def rgb_image_callback(self, img) -> None:
        self.latest_rgb_image = ros_numpy.numpify(msg=img)
    
    def transformed_point_callback(self, point) -> None:
        self.transformed_point = point

    def detect_objects(self, event: rospy.timer.TimerEvent) -> None:
        """Callback function to detect objects on RGB image"""
        # if self.latest_rgb_image is not None and self.latest_depth_image is not None:
        if self.latest_rgb_image is None or self.latest_depth_image is None: # Extracted to reduce the level of indentation
            return
        
        image_array = self.latest_rgb_image
        depth_array = self.latest_depth_image

        if self.detection_image_pub.get_num_connections():
            CONFIDENCE_SCORE: Final[float] = 0.5
            SHOW_DETECTION_BOXES: Final[bool] = False
            result: List[Results] = self.model(image_array, conf=CONFIDENCE_SCORE)  # get the results
            det_annotated: cv2.typing.MatLike = result[0].plot(show=SHOW_DETECTION_BOXES)  # plot the annotations # Is a NDArray[NDArray[NDArray[uint8]]] here

            KINECT_FOCAL_LENGTH_PX: Final[float] = 525.0 # http://wiki.ros.org/kinect_calibration/technical#Focal_lengths

            # Find the depths of each detection and display them
            for detection in result[0].boxes:
                detection: Boxes # Add typing for detection
                x1, y1, x2, y2 = map(int, detection.xyxy[0])
                det_annotated = cv2.circle(det_annotated, (x1, y1), 5, (0, 255, 0), -1)
                det_annotated = cv2.circle(det_annotated, (x2, y2), 5, (0, 255, 0), -1)
                depth = np.nanmean(depth_array[y1:y2, x1:x2])

                if not np.isnan(depth):
                    label: str = f"{int(detection.cls.item())} {depth:.2f}m"
                    det_annotated = cv2.putText(det_annotated, label, (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    # * https://towardsdatascience.com/camera-intrinsic-matrix-with-example-in-python-d79bf2478c12
                    # https://learnopencv.com/monocular-slam-in-python/
                    point: PointStamped = PointStamped()
                    point.header.stamp = rospy.Time.now()
                    point.header.frame_id = "camera_link"
                    # cx: float = (x1 + x2) / 2 # Mean of x
                    # cy: float = (y1 + y2) / 2 # Mean of y
                    # Obtain the parameters of the camera by doing the following:
                    # Run the camera, then run rostopic echo /camera/rgb/camera_info
                    # K is the intrinsics matrix. Per the header file definition of CameriaInfo,

                    ## Intrinsic camera matrix for the raw (distorted) images.
                    #     [fx  0 cx]
                    # K = [ 0 fy cy]
                    #     [ 0  0  1]
                    # Projects 3D points in the camera coordinate frame to 2D pixel
                    # coordinates using the focal lengths (fx, fy) and principal point
                    # (cx, cy).

                    KINECT_HEIGHT = 480
                    KINECT_WIDTH = 640
                    fx = 554.25 # Kinect's focal length
                    fy = 554.25 # Kinect's focal length
                    cx = 320.5 # Principal point x-coordinate
                    cy = 240.5 # Principal point y-coordinate

                    x_coordinate: float = (x1 + x2) / 2
                    y_coordinate: float = (y1 + y2) / 2

                    # Unproject x and y to camera coordinates
                    x_camera: float = depth * (x_coordinate - cx) / fx
                    y_camera: float = depth * (y_coordinate - cy) / fy
                    z_camera: float = depth

                    point.point.x = x_camera
                    point.point.y = y_camera
                    point.point.z = z_camera
                    
                    self.semantic_objects_pub.publish(point)
            self.detection_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))  # publish the annotated image

    def run(self) -> None:
        rospy.spin()

if __name__ == "__main__": 
    yolo_detect = YoloDetect()
    yolo_detect.run()