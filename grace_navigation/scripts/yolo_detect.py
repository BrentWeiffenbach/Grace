import copy
import os
from typing import Dict, Final, List, Tuple, Union

import cv2
import numpy as np
import requests
import ros_numpy
import rospy
from numpy.typing import NDArray
from sensor_msgs.msg import CameraInfo, Image
import torch
from torch import Tensor
from ultralytics import YOLO
from ultralytics.engine.results import Boxes, Results

from grace_navigation.msg import RangeBearing, RangeBearingArray

# Set the environment variable
os.environ["YOLO_VERBOSE"] = "False"
DETECTION_INTERVAL: Final[Union[int, float]] = 0.01
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

    def __init__(self, verbose: bool = False, is_sim: bool = False) -> None:
        """Initialzier for YoloDetect.

        Args:
            verbose (bool, optional): Whether verbose output should be enabled. Defaults to false.
        """
        YoloDetect.verbose: bool = verbose
        self.is_sim = is_sim

        self.isUpdated: bool = False
        """A flag representing if the detection has been updated."""
        self.latch: bool = False
        """A flag that prohibits new callback information from being recieved while True"""

        # Publisher
        if YoloDetect.verbose:
            self.detection_image_pub: rospy.Publisher = rospy.Publisher(
                name="/yolo_detect/detections/image", data_class=Image, queue_size=5
            )

        self.range_pub = rospy.Publisher(
            name="/range_bearing", data_class=RangeBearingArray, queue_size=10
        )

        # Subscribers
        rgb_topic = (
            "/camera/rgb/image_raw" if self.is_sim else "/camera/rgb/image_color"
        )
        self.rgb_image_sub = rospy.Subscriber(
            name=rgb_topic,
            data_class=Image,
            callback=self.rgb_image_callback,
        )
        depth_topic = (
            "/camera/depth/image_raw"
            if self.is_sim
            else "/camera/depth_registered/image_raw"
        )
        self.depth_image_sub = rospy.Subscriber(
            name=depth_topic,
            data_class=Image,
            callback=self.depth_image_callback,
        )

        # Download the YOLO model to the grace folder
        # this must be a segement model so that the masks for range bearings work
        MODEL_NAME: Final = "yolo11m-seg.pt"
        MODEL_PATH: str = download_model(MODEL_NAME)

        # Load YOLO model
        self.model: YOLO = YOLO(model=MODEL_PATH)

        # Store the latest images
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.transformed_point = None
        self.camera_info: CameraInfo = rospy.wait_for_message(
            topic="/camera/rgb/camera_info",
            topic_type=CameraInfo,
            timeout=rospy.Duration(20),
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
            self.latest_depth_image = img
            self.isUpdated = True

    def rgb_image_callback(self, img: Image) -> None:
        if not self.latch:
            self.latest_rgb_image = img
            self.isUpdated = True

    def __call__(self) -> None:
        """Make it so that you can do `YoloDetect()` and it starts the class."""
        self.run()

    def detect_objects(self, event: rospy.timer.TimerEvent) -> None:
        """Callback function to detect objects on RGB image"""
        # if self.latest_rgb_image is not None and self.latest_depth_image is not None:
        if (
            self.latest_rgb_image is None or self.latest_depth_image is None
        ):  # Extracted to reduce the level of indentation
            if self.latest_rgb_image is None:
                rospy.logwarn("YOLO rgb is not loaded")
            else:
                rospy.logwarn("YOLO depth is not loaded")
            return

        if not self.isUpdated:
            return

        # self.isUpdated is True, so change it to indicate that the change has been received
        self.isUpdated = False
        self.latch = True  # Prevent callbacks from modifying any data
        image_array: np.ndarray = ros_numpy.numpify(self.latest_rgb_image)  # type: ignore
        depth_array: np.ndarray = ros_numpy.numpify(self.latest_depth_image)  # type: ignore

        CONFIDENCE_SCORE: Final[float] = 0.30
        result: List[Results] = []

        # Get the results
        # Define the class indices for chair, suitcase, cup, sportsball, dining table, and bottle
        target_classes = [56, 28, 41, 37, 60, 39]  # COCO class indices for the specified objects
        result = self.model.track(
            source=image_array, conf=CONFIDENCE_SCORE, persist=True, classes=target_classes
        )  # get the results
        if YoloDetect.verbose:
            SHOW_DETECTION_BOXES: Final[bool] = False
            det_annotated: cv2.typing.MatLike = result[0].plot(
                show=SHOW_DETECTION_BOXES
            )  # plot the annotations
            self.detection_image_pub.publish(
                ros_numpy.msgify(Image, det_annotated, encoding="rgb8")
            )

        range_msg = RangeBearingArray()
        range_bearings: list = []
        classes: dict[int, str] = result[0].names
        # Find the depths of each detection and display them
        if result[0].masks is None:
            # rospy.logwarn("No masks found in the detection results.")
            self.latch = False
            return
        for detection, mask in zip(result[0].boxes, result[0].masks.data):  # type: ignore
            detection: Boxes  # Add typing for detection
            # mask: np.ndarray  # Segmentation mask for the detection
            mask = mask.cpu().numpy().astype(bool)

            if depth_array.shape != mask.shape:
                rospy.logwarn("Depth array and segmentation mask shapes do not match.")
                continue
            depth_mask = depth_array * mask

            # Write the depth mask to a file for debugging
            # cv2.imwrite(f"/tmp/depth_mask_{detection.id.item()}.png", depth_mask.astype(np.uint8))  # type: ignore

            # Prevent taking the mean of an empty slice
            if depth_mask is None or np.all(np.isnan(depth_mask)):
                rospy.logwarn("Depth mask is empty or invalid.")
                continue
            obj_range: float
            bearing: Tensor
            obj_range, bearing, elevation = self.calculate_range_bearing(depth_mask)

            if (
                np.isfinite(obj_range)
                and np.isfinite(bearing)
                and detection.id is not None
            ):
                range_bearing: RangeBearing = self.create_range_bearing(
                    classes, detection, obj_range, bearing, elevation
                )
                range_bearings.append(range_bearing)
        if range_bearings:
            range_msg.header = copy.deepcopy(
                self.latest_rgb_image.header
            )  # This should be self.img.header, but there is no header on the np array??
            range_msg.range_bearings = range_bearings
            self.range_pub.publish(range_msg)
        self.latch = False

    @staticmethod
    def create_range_bearing(
        classes: Dict[int, str], detection: Boxes, obj_range: float, bearing: Tensor, elevation: Tensor
    ) -> RangeBearing:
        """Creates a RangeBearing based on the classes, detections, ranges, and bearings.

        Args:
            classes (Dict[int, str]): A dict of the classes to use.
            detection (Boxes): The detections.
            obj_range (float): The distance from the object.
            bearing (Tensor): The direction that the object is in.
            elevation (Tensor): The y direction that the object is in.

        Returns:
            RangeBearing: Polar Coordinates representation of the range (distance) and bearing (direction) of an object from the robot.
        """
        # TODO: Currently, range_bearing.id is actually the obj_class. I don't think it is actually getting the tracked id's...
        range_bearing = RangeBearing()
        range_bearing.range = obj_range  # float
        range_bearing.bearing = float(bearing.item())  # float
        range_bearing.elevation = float(elevation.item())
        range_bearing.id = int(detection.id.item())  # type: ignore # int
        range_bearing.obj_class = classes[int(detection.cls.item())]
        return range_bearing

    def calculate_range_bearing(
        self, depth_mask: np.ndarray
    ) -> Tuple[float, Tensor, Tensor]:
        DEPTH_SCALE_FACTOR = (
            1.0 if self.is_sim else 0.001
        )  # IRL Kinect is in mm instead of meters
        depth_mask = depth_mask * DEPTH_SCALE_FACTOR
        masked_depth_values = depth_mask[depth_mask > 0]

        # Threshold out anything above the mean in masked_depth_values
        if masked_depth_values.size > 0:
            mean_depth = np.nanmedian(masked_depth_values)
        else:
            # rospy.logwarn("Masked depth values are empty.")
            return float("nan"), torch.tensor(float("nan")), torch.tensor(float("nan"))
        masked_depth_values = masked_depth_values[masked_depth_values <= mean_depth]

        z: np.floating = np.nanmedian(masked_depth_values)

        if not np.isfinite(z) or z <= 0:
            rospy.logwarn("Invalid median depth value.")
        obj_coords = np.nonzero(depth_mask)
        obj_coords = np.asarray(obj_coords).T

        ux: NDArray[np.int64] = obj_coords[:, 1]
        uy: NDArray[np.int64] = obj_coords[:, 0]

        fx: float = self.camera_info.K[0]  # Kinect's focal length
        fy: float = self.camera_info.K[4]  # Kinect's focal length
        cx: float = self.camera_info.K[2]  # Principal point x-coordinate
        cy: float = self.camera_info.K[5]  # Principal point y-coordinate

        x: NDArray[np.float64] = (ux - cx) * z / fx
        y: NDArray[np.float64] = (uy - cy) * z / fy

        x_median: np.floating = np.nanmedian(x)  # float64
        y_median: np.floating = np.nanmedian(y)  # float64
        z_median: np.floating = np.nanmedian(z)  # float32

        Oc: list[np.floating] = [x_median, y_median, z_median]

        # Hypotenuse of x_mean and z_mean
        obj_range: float = np.sqrt(Oc[0] * Oc[0] + Oc[2] * Oc[2])

        bearing: Tensor = np.arctan2(-Oc[0], Oc[2])
        elevation: Tensor = np.arctan2(-Oc[1], Oc[2])
        return obj_range, bearing, elevation

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node(name="yolo_detect")
    verbose = rospy.get_param("~verbose", False)
    is_sim = rospy.get_param("~sim", False)
    assert type(verbose) is bool
    assert type(is_sim) is bool
    yolo_detect = YoloDetect(verbose=verbose, is_sim=is_sim)
    yolo_detect.run()
