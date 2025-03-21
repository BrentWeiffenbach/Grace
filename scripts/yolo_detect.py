#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python
import copy
import os
from typing import Dict, Final, List, Tuple, Union

import cv2
import numpy as np
import requests
import ros_numpy
import rospy
import ultralytics_patch
from geometry_msgs.msg import PointStamped
from numpy.typing import NDArray
from object_ros_msgs.msg import RangeBearing, RangeBearings
from py3_cv_bridge import imgmsg_to_cv2  # type: ignore
from sensor_msgs.msg import CameraInfo, Image
from torch import Tensor
from ultralytics import YOLO
from ultralytics.engine.results import Boxes, Results

# Set the environment variable
os.environ["YOLO_VERBOSE"] = "False"
DETECTION_INTERVAL: Final[Union[int, float]] = 0.25
"""The interval (in seconds) between detections. 0.25 is 4 times per second. 1 is 1 time per second.
"""


def download_model(model_name: str) -> str:
    """Checks if the model provided is downloaded, and returns its path. Downloads the YOLO model with the model name if it does not exist.

    Args:
        model_name (str): The name of the YOLO model to download. (e.g. `yolo11s.pt`)

    Returns:
        str: The path of the existing or downloaded model.
    """
    MODEL_PATH: Final[str] = os.path.join(os.path.dirname(__file__), model_name)
    if not os.path.isfile(path=MODEL_PATH):
        print(f"{MODEL_PATH} does not exist. Downloading...")
        download_url: str = f"https://github.com/ultralytics/assets/releases/download/v8.3.0/{model_name}"

        response: requests.Response = requests.get(url=download_url)

        if response.status_code == 200:
            with open(file=MODEL_PATH, mode="wb") as file:
                file.write(response.content)
            print(f"Downloaded {MODEL_PATH}")
        else:
            print(f"Failed to download {MODEL_PATH}")
    return MODEL_PATH


class YoloDetect:
    verbose = False

    def __init__(self, verbose: bool = False) -> None:
        """Initialzier for YoloDetect.

        Args:
            verbose (bool, optional): Whether verbose output should be enabled. Defaults to false.
        """
        # ultralytics_patch.patch()
        YoloDetect.verbose: bool = verbose

        self.isUpdated: bool = False
        """A flag representing if the detection has been updated."""
        self.latch: bool = False
        """A flag that prohibits new callback information from being recieved while True"""

        # Publisher
        self.detection_image_pub: rospy.Publisher = rospy.Publisher(
            name="/yolo_detect/detections/image", data_class=Image, queue_size=5
        )
        self.semantic_objects_pub: rospy.Publisher = rospy.Publisher(
            name="/yolo_detect/detections/objects",
            data_class=PointStamped,
            queue_size=5,
        )

        self.range_pub = rospy.Publisher(
            name="/range_bearing", data_class=RangeBearings, queue_size=10
        )

        # Subscribers
        self.rgb_image_sub = rospy.Subscriber(
            name="/camera/rgb/image_raw",
            data_class=Image,
            callback=self.rgb_image_callback,
        )
        self.depth_image_sub = rospy.Subscriber(
            name="/camera/depth/image_raw",
            data_class=Image,
            callback=self.depth_image_callback,
        )

        # Download the YOLO model to the grace folder
        MODEL_NAME: Final = "yolo11s.pt"
        MODEL_PATH: str = download_model(MODEL_NAME)

        # Load YOLO model
        self.model: YOLO = YOLO(model=MODEL_PATH)

        # Store the latest images
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.transformed_point = None
        self.camera_info: CameraInfo = rospy.wait_for_message(
            topic="/camera/depth/camera_info", topic_type=CameraInfo
        )  # type: ignore

        # Timer to call the detection function at a fixed interval (4 times a second)
        self.timer = rospy.Timer(
            period=rospy.Duration(secs=DETECTION_INTERVAL),  # type: ignore
            callback=self.detect_objects,
        )

        if YoloDetect.verbose:
            rospy.loginfo("YoloDetect successfully initialized.")

    def depth_image_callback(self, img: Image) -> None:
        if not self.latch:
            # self.latest_depth_image = ros_numpy.numpify(msg=img)
            self.latest_depth_image = self.convert_Image_to_cv2(img)
            self.isUpdated = True

    def rgb_image_callback(self, img: Image) -> None:
        # self.latest_rgb_image = ros_numpy.numpify(msg=img)
        if not self.latch:
            self.latest_rgb_image = img
            self.isUpdated = True

    def __call__(self) -> None:
        """Make it so that you can do `YoloDetect()` and it starts the class."""
        self.run()

    def convert_Image_to_cv2(self, img: Image) -> cv2.typing.MatLike:
        return imgmsg_to_cv2(img)

    def detect_objects(self, event: rospy.timer.TimerEvent) -> None:
        """Callback function to detect objects on RGB image"""
        # if self.latest_rgb_image is not None and self.latest_depth_image is not None:
        if (
            self.latest_rgb_image is None or self.latest_depth_image is None
        ):  # Extracted to reduce the level of indentation
            return
        if not self.detection_image_pub.get_num_connections():
            return
        if not self.isUpdated:
            return

        # self.isUpdated is True, so change it to indicate that the change has been received
        self.isUpdated = False
        self.latch = True  # Prevent callbacks from modifying any data
        image_array: np.ndarray = ros_numpy.numpify(self.latest_rgb_image)  # type: ignore
        depth_array = self.latest_depth_image

        CONFIDENCE_SCORE: Final[float] = 0.5
        SHOW_DETECTION_BOXES: Final[bool] = False
        TRACKED_CLASSES: Final[list[int]] = [
            32,
            41,
            56,
            57,
            58,
            60,
            61,
            62,
            63,
            64,
            65,
            66,
            67,
            68,
            69,
            70,
            71,
            72,
            73,
            74,
            75,
            76,
            77,
            78,
            79,
        ]
        result: List[Results] = self.model.track(
            source=image_array, conf=CONFIDENCE_SCORE, persist=True
        )  # get the results
        # Use classes=TRACKED_CLASSES to add a filter onto the results
        det_annotated: cv2.typing.MatLike = result[0].plot(
            show=SHOW_DETECTION_BOXES
        )  # plot the annotations

        range_msg = RangeBearings()
        range_bearings: list = []
        classes: dict[int, str] = result[0].names
        # Find the depths of each detection and display them
        # print(result[0].summary())
        # print(result[0].boxes)
        for detection in result[0].boxes:
            detection: Boxes  # Add typing for detection
            x1, y1, x2, y2 = map(int, detection.xyxy[0])
            det_annotated = cv2.circle(det_annotated, (x1, y1), 5, (0, 255, 0), -1)
            det_annotated = cv2.circle(det_annotated, (x2, y2), 5, (0, 255, 0), -1)
            # Prevent taking the mean of an empty slice
            if np.all(np.isnan(depth_array[y1:y2, x1:x2])):
                continue
            obj_range: float
            bearing: Tensor
            obj_range, bearing = self.calculate_bearing_and_obj_range(
                depth_array, x1, y1, x2, y2
            )

            if (
                np.isfinite(obj_range)
                and np.isfinite(bearing)
                and detection.id is not None
            ):
                range_bearing: RangeBearing = self.create_range_bearing(
                    classes, detection, obj_range, bearing
                )
                range_bearings.append(range_bearing)
        if range_bearings:
            range_msg.header = copy.deepcopy(
                self.latest_rgb_image.header
            )  # This should be self.img.header, but there is no header on the np array??
            range_msg.range_bearings = range_bearings
            self.range_pub.publish(range_msg)

        self.detection_image_pub.publish(
            ros_numpy.msgify(Image, det_annotated, encoding="rgb8")
        )
        self.latch = False

    @staticmethod
    def create_range_bearing(
        classes: Dict[int, str], detection: Boxes, obj_range: float, bearing: Tensor
    ) -> RangeBearing:
        """Creates a RangeBearing based on the classes, detections, ranges, and bearings.

        Args:
            classes (Dict[int, str]): A dict of the classes to use.
            detection (Boxes): The detections.
            obj_range (float): The distance from the object.
            bearing (Tensor): The direction that the object is in.

        Returns:
            RangeBearing: Polar Coordinates representation of the range (distance) and bearing (direction) of an object from the robot.
        """
        if YoloDetect.verbose:
            rospy.loginfo(detection)
        # TODO: Currently, range_bearing.id is actually the obj_class. I don't think it is actually getting the tracked id's...
        range_bearing = RangeBearing()
        range_bearing.range = obj_range  # float
        range_bearing.bearing = float(bearing.item())  # float
        range_bearing.id = int(detection.id.item())  # int
        range_bearing.obj_class = classes[int(detection.cls.item())]
        # range_bearing.probability = detection.Class_distribution
        # Create an array of all one's, except for the known probability
        # Hacky way to interface with Zhentian's code without actually getting Class_distributions
        # BUG: If there are enough detections, this causes an IndexError
        # Bandaid fixed this by moduloing it by the length of _alpha
        _alpha: NDArray[np.float64] = np.ones(len(classes.keys()))
        _alpha[range_bearing.id % len(_alpha)] = detection.conf
        range_bearing.probability = _alpha
        return range_bearing

    def calculate_bearing_and_obj_range(
        self, depth_array, x1: int, y1: int, x2: int, y2: int
    ) -> Tuple[float, Tensor]:
        depth_mask = depth_array[y1:y2, x1:x2]

        z: np.floating = np.nanmean(depth_mask)
        obj_coords = np.nonzero(depth_mask)
        obj_coords = np.asarray(obj_coords).T

        obj_coords = obj_coords + np.asarray([y1, x1])

        ux: NDArray[np.int64] = obj_coords[:, 1]
        uy: NDArray[np.int64] = obj_coords[:, 0]

        fx: float = self.camera_info.K[0]  # Kinect's focal length
        fy: float = self.camera_info.K[4]  # Kinect's focal length
        cx: float = self.camera_info.K[2]  # Principal point x-coordinate
        cy: float = self.camera_info.K[5]  # Principal point y-coordinate

        x: NDArray[np.float64] = (ux - cx) * z / fx
        y: NDArray[np.float64] = (uy - cy) * z / fy

        x_mean: np.floating = np.nanmean(x)  # float64
        y_mean: np.floating = np.nanmean(y)  # float64
        z_mean: np.floating = np.nanmean(z)  # float32

        Oc: list[np.floating] = [x_mean, y_mean, z_mean]

        # Hypotenuse of x_mean and z_mean
        obj_range: float = np.sqrt(Oc[0] * Oc[0] + Oc[2] * Oc[2])

        bearing: Tensor = np.arctan2(-Oc[0], Oc[2])
        return obj_range, bearing

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node(name="yolo_detect")
    verbose = rospy.get_param("~verbose", False)
    assert type(verbose) is bool
    yolo_detect = YoloDetect(verbose=verbose)
    yolo_detect.run()
