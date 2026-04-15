#! /usr/bin/env python3

#-----------------------------------------------------------------------------------
# MIT License

# Copyright (c) 2025 ItsKalvik

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#-----------------------------------------------------------------------------------

"""
This script is a ROS 2 node that processes raw sonar data from an 'echo.py' node.
It subscribes to the sonar's image topic, converts the raw data into a polar image,
and can save the output as a video file or publish it as a new image topic.
The node can also process data from a ROS 2 bag file.
"""

import rclpy
from rclpy import qos
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos_overriding_options import QoSOverridingOptions

import os
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class EchoImager(Node):
    """
    The main class for the sonar imager node.
    """
    def __init__(self, node_name='echo_imager'):
        """
        Initializes the node, parameters, subscriber, and other necessary objects.
        """
        super().__init__(node_name)

        # --- Parameters ---
        # Declare and get parameters for the node.
        params = {
            'data_topic': '/sonar/echo/compressed',
            'image_topic': '/sonar/echo/image',
            'contrast': 10.0,
            'bag_file': '',
            'video_file': '',
        }

        for param, value in params.items():
            self.declare_parameter(param, value)
            setattr(self, param, self.get_parameter(param).value)
            self.get_logger().info(f'{param}: {value}')

        self.br = CvBridge()
        self.video_fps = 15  # Default FPS for live stream recording
        
        qos_override_opts = QoSOverridingOptions(
            policy_kinds=(
                qos.QoSPolicyKind.HISTORY,
                qos.QoSPolicyKind.DEPTH,
                qos.QoSPolicyKind.RELIABILITY,
            )
        )
        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

        # Handle parameter updates
        self.add_on_set_parameters_callback(self.set_param_callback)

        # Determine if topic is compressed
        self.compressed = 'compressed' in self.data_topic
        data_type = CompressedImage if self.compressed else Image

        # Determine input source
        if len(self.bag_file) == 0:
            self.from_bag = False
            self.subscription = self.create_subscription(
                data_type, self.data_topic, self.data_callback, 
                SENSOR_QOS, qos_overriding_options=qos_override_opts)
            self.get_logger().info("Reading data from ros2 node")
        else:
            self.from_bag = True
            self.get_logger().info("Reading data from bag file")

        # Determine output destination
        if len(self.video_file) == 0 and not self.from_bag:
            self.to_video = False
            self.publisher = self.create_publisher(
                Image, self.image_topic, SENSOR_QOS, 
                qos_overriding_options=qos_override_opts) 
            self.get_logger().info("Publishing data to ros2 topic")
        else:
            if len(self.video_file) == 0:
                self.video_file = 'echo_sonar.mp4'
            self.video_writer = None
            self.to_video = True
            self.get_logger().info(f"Saving data to video file: {self.video_file}")

        if self.from_bag:
            self.process_bag()

    def warp_sonar(self, scan_image, contrast, fov):
        """
        The new version of sonar warping logic.
        """
        # Contrast adjustment
        scan_image = cv2.convertScaleAbs(scan_image, alpha=contrast, beta=0)
        h, w = scan_image.shape
        
        # Calculate padding to represent a full 360 degrees
        px_per_deg = w / fov
        total_360_px = int(px_per_deg * 360)
        padding = (total_360_px - w) // 2
        scan_image = cv2.copyMakeBorder(scan_image, 0, 0, padding, padding, 
                                        cv2.BORDER_CONSTANT, value=0)

        # Transpose for warpPolar
        scan_image = scan_image.T 
        
        # Polar Warp
        scan_image = cv2.warpPolar(scan_image,
                                   dsize=(h*2, h*2),
                                   center=(h, h),
                                   maxRadius=h,
                                   flags=cv2.WARP_INVERSE_MAP | cv2.WARP_FILL_OUTLIERS)
        
        # Rotate so the sonar head is at the top center
        scan_image = cv2.rotate(scan_image, cv2.ROTATE_90_CLOCKWISE)

        # Calculate final crop to remove empty pixels
        d = int(np.ceil(2 * h * np.sin(np.deg2rad(fov / 2))))
        s = (h * 2 - d) // 2
        scan_image = scan_image[0:h, s:s+d]
        
        # Flip to match standard sonar display
        scan_image = cv2.flip(scan_image, 1)
        
        return scan_image

    def data_callback(self, msg):
        """
        Callback function to process incoming sonar image data.
        This function is called for each message received on the data_topic.
        """ 
        # Convert the ROS 2 Image message to an OpenCV image (numpy array).
        if self.compressed:
            scan_image = self.br.compressed_imgmsg_to_cv2(msg)
        else:
            scan_image = self.br.imgmsg_to_cv2(msg)

        # Logic for range and fov
        max_range = scan_image[0, 0]
        fov = 120 if max_range <= 30 else 90

        # Apply the updated warping logic
        processed_image = self.warp_sonar(scan_image, self.contrast, fov)

        # Post-processing: Colormap and Text
        processed_image = cv2.applyColorMap(processed_image, cv2.COLORMAP_VIRIDIS)
        cv2.putText(processed_image, f'Range: {max_range} m', (1, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # Publishing/Saving
        if self.to_video:
            if self.video_writer is None:
                height, width = processed_image.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(self.video_file, fourcc,
                                                    self.video_fps, (width, height))
                if not self.video_writer.isOpened():
                    self.get_logger().error("Failed to open video writer.")
                    return
            self.video_writer.write(processed_image)
        else:
            self.publisher.publish(self.br.cv2_to_imgmsg(processed_image, encoding="bgr8"))

    def process_bag(self):
        """
        Processes a ROS 2 bag file to extract and convert sonar images
        """
        # Check if the bag file exists.
        if not os.path.exists(self.bag_file):
            self.get_logger().error(f"Bag file not found: {self.bag_file}!")
            return
        self.get_logger().info(f"Processing bag file: {self.bag_file}")

        # --- Setup Bag Reader ---
        storage_options = StorageOptions(uri=self.bag_file,
                                         storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr',
                                              output_serialization_format='cdr')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topic_types}
        msg_type_str = type_map.get(self.data_topic, None)
        
        if not msg_type_str:
            self.get_logger().error(f"Topic '{self.data_topic}' not found in bag!")
            return
        
        msg_type = get_message(msg_type_str)

        # First Pass: Estimate FPS
        times = []
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == self.data_topic:
                times.append(t)
        
        if len(times) > 1:
            time_del = np.diff(times)
            self.video_fps = int(np.round(1.0 / (np.median(time_del) * 1e-9)))
        
        self.get_logger().info(f'Estimated FPS: {self.video_fps} | Total Frames: {len(times)}')

        # Second Pass: Process
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        frame_num = 0
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == self.data_topic:
                msg = deserialize_message(data, msg_type)
                self.data_callback(msg)
                frame_num += 1
                if frame_num % 50 == 0:
                    self.get_logger().info(f'Processed: {frame_num}/{len(times)}')

        if self.video_writer:
            self.video_writer.release()
        self.get_logger().info(f'Finished writing to {self.video_file}')

    def set_param_callback(self, params):
        """
        This function is the callback for when parameters are changed.
        It updates the node's attributes and sends the new settings to the sonar.
        """
        result = SetParametersResult(successful=True)
        for param in params:
            if "qos" in param.name:
                continue
            if hasattr(self, param.name):
                setattr(self, param.name, param.value)
                self.get_logger().info(f'Updated {param.name}: {param.value}')
        return result


def main(args=None):
    rclpy.init(args=args)
    echo_imager = EchoImager()
    if not echo_imager.from_bag:
        try:
            rclpy.spin(echo_imager)
        except KeyboardInterrupt:
            pass
    echo_imager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()