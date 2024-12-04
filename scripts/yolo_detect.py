#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python
import copy
import os
from typing import Final, List, Tuple, Union
from torch import Tensor
import rospy
import ros_numpy
import numpy as np
from numpy.typing import NDArray
from py3_cv_bridge import imgmsg_to_cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from object_ros_msgs.msg import RangeBearings, RangeBearing
from ultralytics import YOLO
import requests
from ultralytics.engine.results import Boxes, Results
import cv2

# Set the environment variable
os.environ['YOLO_VERBOSE'] = 'False'
DETECTION_INTERVAL: Final[Union[int, float]] = 0.25
"""The interval (in seconds) between detections. 0.25 is 4 times per second. 1 is 1 time per second.
"""

class YoloDetect:
    def __init__(self) -> None:
        rospy.init_node(name='yolo_detect')

        self.isUpdated: bool = True
        """A flag representing if the detection has been updated."""
        self.latch: bool = False
        """A flag that prohibits new callback information from being recieved while True"""
        
        # Publisher
        self.detection_image_pub: rospy.Publisher = rospy.Publisher(
            name="/yolo_detect/detections/image", data_class=Image, queue_size=5 
        )
        self.semantic_objects_pub: rospy.Publisher = rospy.Publisher(
            name="/yolo_detect/detections/objects", data_class=PointStamped, queue_size=5
        )

        self.range_pub = rospy.Publisher(name="/range_bearing", data_class=RangeBearings, queue_size=10)

        # Subscribers
        self.rgb_image_sub = rospy.Subscriber(name="/camera/rgb/image_raw", data_class=Image, callback=self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber(name="/camera/depth/image_raw", data_class=Image, callback=self.depth_image_callback)
        self.transformed_point_sub = rospy.Subscriber(name="/transform_node/base_link_frame_point", data_class=PointStamped, callback=self.transformed_point_callback)


        MODEL_NAME: Final = 'yolo11s.pt'
        # Download the YOLO model to the grace folder
        MODEL_PATH: Final[str] = os.path.join(os.path.dirname(__file__), MODEL_NAME)
        if not os.path.isfile(path=MODEL_PATH):
            print(f'{MODEL_PATH} does not exist. Downloading...')
            download_url = f'https://github.com/ultralytics/assets/releases/download/v8.3.0/{MODEL_NAME}'

            response: requests.Response = requests.get(url=download_url)

            if response.status_code == 200:
                with open(file=MODEL_PATH, mode='wb') as file:
                    file.write(response.content)
                print(f'Downloaded {MODEL_PATH}')
            else:
                print(f'Failed to download {MODEL_PATH}')

        # Load YOLO model
        self.model: YOLO = YOLO(model=MODEL_PATH)

        # Store the latest images
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.transformed_point = None
        self.camera_info: CameraInfo = rospy.wait_for_message(topic="/camera/depth/camera_info", topic_type=CameraInfo) # type: ignore

        # Timer to call the detection function at a fixed interval (4 times a second)
        self.timer = rospy.Timer(period=rospy.Duration(secs=DETECTION_INTERVAL), callback=self.detect_objects) # type: ignore
        # Could do what Zhentian did and use an ApproximateTimeSynchronizer as the callback instead of using the timer to call detect_objects

    def depth_image_callback(self, img: Image) -> None:
        if not self.latch:
            # self.latest_depth_image = ros_numpy.numpify(msg=img)
            self.latest_depth_image = self.convert_Image_to_cv2(img)
            self.isUpdated = True

    def rgb_image_callback(self, img: Image) -> None:
        # self.latest_rgb_image = ros_numpy.numpify(msg=img)
        self.latest_rgb_image = img
        self.isUpdated = True
    
    def transformed_point_callback(self, point: PointStamped) -> None:
        self.transformed_point = point

    def __call__(self) -> None:
        """Make it so that you can do `YoloDetect()` and it starts the class.
        """
        self.run()
    
    def convert_Image_to_cv2(self, img: Image) -> cv2.typing.MatLike:
        return imgmsg_to_cv2(img)
    def _detect_objects(self, event: rospy.timer.TimerEvent) -> None:
        """Callback function to detect objects on RGB image"""
        # if self.latest_rgb_image is not None and self.latest_depth_image is not None:
        if self.latest_rgb_image is None or self.latest_depth_image is None: # Extracted to reduce the level of indentation
            return
        if not self.detection_image_pub.get_num_connections():
            return
        if not self.isUpdated:
            return

        # self.isUpdated is True, so change it to indicate that the change has been recieved
        self.isUpdated = False
        self.latch = True # Prevent callbacks from modifying any data
        image_array = self.latest_rgb_image
        depth_array = self.latest_depth_image
        
        CONFIDENCE_SCORE: Final[float] = 0.5
        SHOW_DETECTION_BOXES: Final[bool] = False
        result: List[Results] = self.model(image_array, conf=CONFIDENCE_SCORE)  # get the results
        det_annotated: cv2.typing.MatLike = result[0].plot(show=SHOW_DETECTION_BOXES)  # plot the annotations # Is a NDArray[NDArray[NDArray[uint8]]] here

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
                top_left_corner: tuple[int, int] = (x1, y1)
                bottom_right_corner: tuple[int, int] = (x2, y2)

                point: PointStamped = self.image_coordinates_to_camera_frame(top_left_corner=top_left_corner, bottom_right_corner=bottom_right_corner, depth=depth)
                point = self.camera_to_world_coordinates(point)
                
                self.semantic_objects_pub.publish(point) # Publish the camera frame point
        self.detection_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))
        self.latch = False

    def detect_objects(self, event: rospy.timer.TimerEvent) -> None:
        """Callback function to detect objects on RGB image"""
        # if self.latest_rgb_image is not None and self.latest_depth_image is not None:
        if self.latest_rgb_image is None or self.latest_depth_image is None: # Extracted to reduce the level of indentation
            return
        if not self.detection_image_pub.get_num_connections():
            return
        if not self.isUpdated:
            return

        # self.isUpdated is True, so change it to indicate that the change has been recieved
        self.isUpdated = False
        self.latch = True # Prevent callbacks from modifying any data
        image_array: np.ndarray = ros_numpy.numpify(self.latest_rgb_image)
        depth_array = self.latest_depth_image
        
        CONFIDENCE_SCORE: Final[float] = 0.5
        SHOW_DETECTION_BOXES: Final[bool] = False
        result: List[Results] = self.model.track(image_array, conf=CONFIDENCE_SCORE)  # get the results
        det_annotated: cv2.typing.MatLike = result[0].plot(show=SHOW_DETECTION_BOXES)  # plot the annotations # Is a NDArray[NDArray[NDArray[uint8]]] here

        range_msg = RangeBearings()
        range_bearings = []
        classes: dict[int, str] = result[0].names
        # Find the depths of each detection and display them
        # print(result[0])
        for detection in result[0].boxes:
            detection: Boxes # Add typing for detection
            x1, y1, x2, y2 = map(int, detection.xyxy[0])
            det_annotated = cv2.circle(det_annotated, (x1, y1), 5, (0, 255, 0), -1)
            det_annotated = cv2.circle(det_annotated, (x2, y2), 5, (0, 255, 0), -1)
            depth_mask = depth_array[y1:y2, x1:x2]
            obj_coords = np.nonzero(depth_mask)

            z: np.floating = np.nanmean(obj_coords)
            obj_coords = np.asarray(obj_coords).T

            obj_coords = obj_coords + np.asarray([y1, x1])

            ux = obj_coords[:, 1]
            uy = obj_coords[:, 0]

            fx: float = self.camera_info.K[0] # Kinect's focal length
            fy: float = self.camera_info.K[4] # Kinect's focal length
            cx: float = self.camera_info.K[2] # Principal point x-coordinate
            cy: float = self.camera_info.K[5] # Principal point y-coordinate

            x = (ux - cx) * z / fx
            y = (uy - cy) * z / fx

            x_mean: np.floating = np.mean(x)
            y_mean: np.floating = np.mean(y)
            z_mean: np.floating = np.mean(z)

            Oc: list[np.floating] = [x_mean, y_mean, z_mean]

            obj_range: float = np.sqrt(Oc[0] * Oc[0] + Oc[2] * Oc[2])

            bearing: Tensor = np.arctan2(-Oc[0], Oc[2])

            # print(f"obj_range: {obj_range}")
            # print(f"bearing: {bearing}")
            # print(f"obj_range is finite: {np.isfinite(obj_range)}")
            # print(f"bearing is finite: {np.isfinite(bearing)}")
            if np.isfinite(obj_range) and np.isfinite(bearing):
                # print(f"Detection: {detection}")
                # print(f"id?: {detection.data[:, -3]}")
                range_bearing = RangeBearing()
                range_bearing.range = obj_range # float
                range_bearing.bearing = float(bearing.item()) # float
                range_bearing.id = int(detection.id.item()) # int
                range_bearing.obj_class = classes[int(detection.cls.item())]
                # range_bearing.probability = detection.Class_distribution
                range_bearings.append(range_bearing)
        if range_bearings:
            range_msg.header = copy.deepcopy(self.latest_rgb_image.header) # This should be self.img.header, but there is no header on the np array??
            range_msg.range_bearings = range_bearings
            self.range_pub.publish(range_msg)

        self.detection_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))
        self.latch = False

    def image_coordinates_to_camera_frame(self, top_left_corner: Tuple[int, int], bottom_right_corner: Tuple[int, int], depth) -> PointStamped:
        point: PointStamped = PointStamped()
        point.header.stamp = rospy.Time.now()
        # point.header.frame_id = "camera_link"
        point.header.frame_id = "camera_rgb_optical_frame"
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
        fx: float = self.camera_info.K[0] # Kinect's focal length
        fy: float = self.camera_info.K[4] # Kinect's focal length
        cx: float = self.camera_info.K[2] # Principal point x-coordinate
        cy: float = self.camera_info.K[5] # Principal point y-coordinate

        x_coordinate: float = (top_left_corner[0] + bottom_right_corner[0]) / 2
        y_coordinate: float = (top_left_corner[1] + bottom_right_corner[1]) / 2

        # Unproject x and y to camera coordinates
        x_camera: float = depth / 1000.0 * (x_coordinate - cx) / fx
        y_camera: float = depth / 1000.0 * (y_coordinate - cy) / fy
        z_camera: float = depth / 1000.0

        point.point.x = x_camera
        point.point.y = y_camera
        point.point.z = z_camera
        return point

    def camera_to_world_coordinates(self, camera_coordinates: PointStamped) -> PointStamped:
        point: PointStamped = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map" # This COULD be base_link if world doesn't work

        rotation_matrix: NDArray[np.float64] = np.reshape(self.camera_info.R, (3, 3))
        world_point = np.dot(rotation_matrix, [camera_coordinates.point.x, camera_coordinates.point.y, camera_coordinates.point.z])
        
        point.point.x = world_point[0]
        point.point.y = world_point[1]
        point.point.z = world_point[2]
        return point
    def run(self) -> None:
        rospy.spin()

if __name__ == "__main__": 
    yolo_detect = YoloDetect()
    yolo_detect.run()