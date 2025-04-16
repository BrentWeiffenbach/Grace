import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from datetime import datetime
import os
from typing import Optional, Any
import sys
import traceback

class VideoSaver:
    def __init__(self) -> None:
        # Initialize the ROS node
        rospy.init_node('video_saver', anonymous=True)
        
        # Get parameters from ROS parameter server
        self.image_topic: str = rospy.get_param('~image_topic', '/camera/image_raw')
        self.output_dir: str = rospy.get_param('~output_dir', os.path.expanduser('~'))
        self.fps: int = rospy.get_param('~fps', 30)
        self.is_depth: bool = rospy.get_param('~is_depth', False)
        
        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # Variables for video writer
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.frame_count: int = 0
        self.width: Optional[int] = None
        self.height: Optional[int] = None
        
        # Subscribe to the raw image topic
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        rospy.loginfo(f"Video saver initialized. Subscribing to {self.image_topic}")
        rospy.loginfo(f"Output directory: {self.output_dir}")
        rospy.loginfo(f"FPS: {self.fps}")
        rospy.loginfo(f"Is depth: {self.is_depth}")
        
    def process_depth_image(self, msg: Image) -> np.ndarray:
        """Process depth images (32FC1 or 16UC1) into BGR format for video"""
        if msg.encoding == "32FC1":
            # For 32FC1 depth images
            # Extract data from ROS message
            depth_array = np.frombuffer(msg.data, dtype=np.float32).copy()  # Make a copy to avoid read-only issues
            depth_array = depth_array.reshape(msg.height, msg.width)
            
            # Handle NaN and Inf values
            depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=0.0, neginf=0.0)
            
            # Normalize to 0-255 range for visualization
            depth_min = np.min(depth_array)
            depth_max = np.max(depth_array)
            if depth_max > depth_min:
                depth_array_norm = ((depth_array - depth_min) / (depth_max - depth_min) * 255.0).astype(np.uint8)
            else:
                depth_array_norm = np.zeros(depth_array.shape, dtype=np.uint8)
            
            # Convert to colormap for visualization
            return cv2.applyColorMap(depth_array_norm, cv2.COLORMAP_JET)
        
        elif msg.encoding == "16UC1":
            # For 16UC1 depth images
            depth_array = np.frombuffer(msg.data, dtype=np.uint16).copy()  # Make a copy to avoid read-only issues
            depth_array = depth_array.reshape(msg.height, msg.width)
            
            # Normalize to 0-255 range
            depth_array_norm = (depth_array / 256).astype(np.uint8)  # Scale down from 16-bit to 8-bit
            
            # Convert to colormap for visualization
            return cv2.applyColorMap(depth_array_norm, cv2.COLORMAP_JET)
        
        else:
            raise ValueError(f"Unsupported depth image encoding: {msg.encoding}")
    
    def process_color_image(self, msg: Image) -> np.ndarray:
        """Process color or grayscale images into BGR format for video"""
        if msg.encoding == "rgb8":
            # For RGB images
            img_data = np.frombuffer(msg.data, dtype=np.uint8).copy()  # Make a copy to avoid read-only issues
            img_data = img_data.reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)
        
        elif msg.encoding == "bgr8":
            # For BGR images (already in the right format)
            img_data = np.frombuffer(msg.data, dtype=np.uint8).copy()  # Make a copy to avoid read-only issues
            return img_data.reshape(msg.height, msg.width, 3)
        
        elif msg.encoding == "mono8" or msg.encoding == "8UC1":
            # For grayscale images
            img_data = np.frombuffer(msg.data, dtype=np.uint8).copy()  # Make a copy to avoid read-only issues
            gray_image = img_data.reshape(msg.height, msg.width)
            # Convert to 3-channel BGR for video
            return cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
        
        else:
            raise ValueError(f"Unsupported color image encoding: {msg.encoding}")
            
    def image_callback(self, msg: Image) -> None:
        try:
            # Log the image encoding
            rospy.loginfo_once(f"Received first image with encoding: {msg.encoding}")
            
            # Process image based on type (depth or color)
            if self.is_depth or msg.encoding in ["32FC1", "16UC1"]:
                cv_image = self.process_depth_image(msg)
            else:
                cv_image = self.process_color_image(msg)
            
            # Initialize video writer if this is the first frame
            if self.video_writer is None:
                self.height, self.width = cv_image.shape[:2]
                timestamp: str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                topic_name = self.image_topic.replace('/', '_')
                video_path: str = os.path.join(self.output_dir, f"{topic_name}_{timestamp}.mp4")
                
                # Use H.264 codec
                fourcc: int = cv2.VideoWriter.fourcc(*'mp4v')  # or 'avc1' for MP4 format
                self.video_writer = cv2.VideoWriter(
                    video_path, 
                    fourcc, 
                    self.fps, 
                    (self.width, self.height)
                )
                
                rospy.loginfo(f"Started recording to {video_path}")
            
            # Write the frame to video file
            self.video_writer.write(cv_image)
            self.frame_count += 1
            
            # Log progress periodically
            if self.frame_count % 300 == 0:
                rospy.loginfo(f"Recorded {self.frame_count} frames from {self.image_topic}")
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
            rospy.logerr(f"Error details: {traceback.format_exc()}")
            
    def stop_recording(self) -> None:
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo(f"Stopped recording. Saved {self.frame_count} frames from {self.image_topic}.")
            self.video_writer = None
            self.frame_count = 0
            
    def run(self) -> None:
        # Keep the node running until shutdown
        rospy.spin()
        
        # Ensure we close the video writer when the node is shutdown
        self.stop_recording()
            
if __name__ == '__main__':
    try:
        video_saver = VideoSaver()
        video_saver.run()
    except rospy.ROSInterruptException:
        pass