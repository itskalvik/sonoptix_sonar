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
            'data_topic': ['/sonar/echo/compressed', str],
            'image_topic': ['/sonar/echo/image', str],
            'contrast': [10.0, float],
            'bag_file': ['', str],
            'video_file': ['', str],
        }

        for name, [value, dtype] in params.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)

        # Log current configuration
        for name in params.keys():
            val = getattr(self, name)
            self.get_logger().info(f'{name}: {val}')

        # --- State Variables ---
        self.br = CvBridge()
        
        # Cache for cv2.remap
        self.map_x = None
        self.map_y = None
        self.last_h = -1
        self.last_w = -1
        self.last_fov = -1

        # Video recording state
        self.video_writer = None
        self.video_size = None  # (width, height)
        self.video_fps = 15 
        
        # --- QoS Setup ---
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

        # --- Logic for Input/Output ---
        if len(self.bag_file) == 0:
            self.from_bag = False
            self.subscription = self.create_subscription(
                data_type, self.data_topic, self.data_callback, 
                SENSOR_QOS, qos_overriding_options=qos_override_opts)
            self.get_logger().info("Reading data from live ROS 2 topic")
        else:
            self.from_bag = True
            self.get_logger().info("Reading data from bag file")

        if len(self.video_file) == 0 and not self.from_bag:
            self.to_video = False
            self.publisher = self.create_publisher(
                Image, self.image_topic, SENSOR_QOS, 
                qos_overriding_options=qos_override_opts) 
            self.get_logger().info("Publishing processed images to ROS 2")
        else:
            if len(self.video_file) == 0:
                self.video_file = 'echo_sonar.mp4'
            self.to_video = True
            self.get_logger().info(f"Saving processed video to: {self.video_file}")

        if self.from_bag:
            self.process_bag()

    def _prepare_remap(self, h, w, fov):
        """
        Calculates the transformation maps for cv2.remap.
        Maps the sonar polar scan to a Cartesian fan shape.
        """
        self.get_logger().info(f"Recalculating maps for H:{h} W:{w} FOV:{fov}")
        fov_rad = np.deg2rad(fov)
        
        # Calculate destination dimensions (chord length at max radius)
        dst_w = int(np.ceil(2 * h * np.sin(fov_rad / 2)))
        dst_h = h
        
        # Create destination coordinate grid
        x = np.arange(dst_w, dtype=np.float32)
        y = np.arange(dst_h, dtype=np.float32)
        x_grid, y_grid = np.meshgrid(x, y)

        # Center sonar head at bottom-middle
        dx = x_grid - (dst_w / 2.0)
        dy = h - y_grid 

        radius = np.sqrt(dx**2 + dy**2)
        theta = np.arctan2(dx, dy)

        # Map to source image: X is beam angle, Y is range (radius)
        self.map_x = (theta / fov_rad) * w + (w / 2.0)
        self.map_y = radius
        
        self.last_h, self.last_w, self.last_fov = h, w, fov

        # --- Proportional Text Scaling Logic ---
        # Scales text based on image height (e.g., 0.8 scale for ~400px height)
        self.font_scale = h / 600.0  
        self.thickness = max(1, int(self.font_scale * 2))

        # Anchor position to fixed margin scaled by font size.
        margin_x = int(20 * self.font_scale)
        margin_y = int(40 * self.font_scale)
        self.text_org = (margin_x, h-margin_y)

    def warp_sonar(self, scan_image, contrast, fov):
        """
        Efficiently warps the image using a single-pass remap.
        """
        scan_image = cv2.convertScaleAbs(scan_image, alpha=contrast, beta=0)
        h, w = scan_image.shape

        if (h != self.last_h or w != self.last_w or fov != self.last_fov):
            self._prepare_remap(h, w, fov)

        return cv2.remap(
            scan_image, self.map_x, self.map_y, 
            interpolation=cv2.INTER_LINEAR, 
            borderMode=cv2.BORDER_CONSTANT, 
            borderValue=0
        )

    def data_callback(self, msg):
        """
        Processes incoming frames and handles video resizing.
        """
        if self.compressed:
            scan_image = self.br.compressed_imgmsg_to_cv2(msg)
        else:
            scan_image = self.br.imgmsg_to_cv2(msg)

        # Metadata extraction
        max_range = scan_image[0, 0]
        fov = 120 if max_range <= 30 else 90

        # Warp and Colormap
        processed_image = self.warp_sonar(scan_image, self.contrast, fov)
        processed_image = cv2.applyColorMap(processed_image, cv2.COLORMAP_VIRIDIS)

        # Annotate
        cv2.putText(processed_image, f'Max Range: {max_range} m', self.text_org,
                    cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (255, 255, 255), 
                    self.thickness, cv2.LINE_AA)

        if self.to_video:            
            # Initialize VideoWriter based on the VERY FIRST frame's dimensions
            h, w = processed_image.shape[:2]
            if self.video_writer is None:
                self.video_size = (w, h)
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(self.video_file, fourcc,
                                                    self.video_fps, self.video_size)
                self.get_logger().info(f"Video fixed at resolution: {self.video_size}")

            # If sonar range changes, resize the frame to match the existing video container
            if (w, h) != self.video_size:
                processed_image = cv2.resize(processed_image, self.video_size, 
                                             interpolation=cv2.INTER_AREA)

            self.video_writer.write(processed_image)
        else:
            self.publisher.publish(self.br.cv2_to_imgmsg(processed_image, encoding="bgr8"))

    def process_bag(self):
        """
        Processes a ROS 2 bag file to extract and convert sonar images
        """
        # Check if the bag file exists.
        if not os.path.exists(self.bag_file):
            self.get_logger().error(f"Bag file not found: {self.bag_file}")
            return
        self.get_logger().info(f"Processing bag file: {self.bag_file}")

        storage_options = StorageOptions(uri=self.bag_file, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr',
                                              output_serialization_format='cdr')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topic_types}
        msg_type_str = type_map.get(self.data_topic, None)
        
        if not msg_type_str:
            self.get_logger().error(f"Topic {self.data_topic} not in bag")
            return
        
        msg_type = get_message(msg_type_str)

        # Estimate FPS
        times = []
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == self.data_topic:
                times.append(t)
        
        if len(times) > 1:
            self.video_fps = int(np.round(1.0 / (np.median(np.diff(times)) * 1e-9)))
            self.get_logger().info(f"Estimated Video FPS: {self.video_fps}")

        # Reset reader for processing
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
            if echo_imager.video_writer:
                echo_imager.video_writer.release()
                echo_imager.get_logger().info(f'Finished writing to {echo_imager.video_file}')
    echo_imager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()