#!/usr/bin/env python2.7
"""
    Provides conversions between OpenCV and ROS image formats in a hard-coded way.  
    CV_Bridge, the module usually responsible for doing this, is not compatible with Python 3,
     - the language this all is written in.  So we create this module, and all is... well, all is not well,
     - but all works.  :-/
     Code from https://robotics.stackexchange.com/questions/95630/cv-bridge-throws-boost-import-error-in-python-3-and-ros-melodic
"""
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image

def _imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr(f"This Coral detect node has been hardcoded to the 'bgr8' encoding. Come change the code if you're actually trying to implement a new camera. You are using {img_msg.encoding}")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def imgmsg_to_cv2(img_msg):
    # 32FC1 means that each pixel value is stored as one channel floating point with single precision
    # Per https://answers.opencv.org/question/135621/encoding-32fc1-explained/
    if img_msg.encoding != "32FC1":
        rospy.logerr(f"This Coral detect node has been hardcoded to the '32FC1' encoding. Come change the code if you're actually trying to implement a new camera. You are using {img_msg.encoding}")
    dtype = np.dtype("float32") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width), #32FC1 uses 1 channel
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg