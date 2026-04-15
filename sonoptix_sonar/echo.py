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
This script is a ROS 2 node that interfaces with a Sonoptix sonar.
It captures data from the sonar, processes it, and publishes it as a ROS 2 Image message.
The node also provides a service to control the sonar's power state, range, and other parameters.
"""

import rclpy
from rclpy import qos
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos_overriding_options import QoSOverridingOptions

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import requests


class EchoNode(Node):
    """
    The main class for the sonar node.
    """
    def __init__(self, node_name='echo'):
        """
        Initializes the node, parameters, publisher, and services.
        """
        super().__init__(node_name)

        # --- Parameters ---
        # Declare and get parameters for the node.
        params = {
            'range': [50, int],
            'ip': ['192.168.2.42', str],
            'tx_mode': ['auto', str],
            'power_state': [True, bool],
            'topic': ['/sonar/echo/data', str],
            'frame_id': ['echo', str],
        }

        for name, [value, dtype] in params.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)

        # Log current configuration
        for name in params.keys():
            val = getattr(self, name)
            self.get_logger().info(f'{name}: {val}')

        qos_override_opts = QoSOverridingOptions(
            policy_kinds=(
                qos.QoSPolicyKind.HISTORY,
                qos.QoSPolicyKind.DEPTH,
                qos.QoSPolicyKind.RELIABILITY,
            )
        )
        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

        # Handle parameter updates
        self.param_handler_ptr_ = self.add_on_set_parameters_callback(self.set_param_callback)

        self.rtsp_url = f'rtsp://{self.ip}:8554/raw'
        self.api_url = f'http://{self.ip}:8000/api/v2'

        self.get_logger().info('Configuring Sonar...')

        # Initial sonar configuration
        self._update_sonar_settings()

        # Set the data stream type to RTSP
        try:
            requests.put(self.api_url + '/datastream', json={"stream_type": 'rtsp'}, timeout=2.0)
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to set stream type: {e}")

        # Initialize CV bridge, video capture, and ROS 2 publisher
        self.br = CvBridge()
        self.cap = cv2.VideoCapture(self.rtsp_url)
        self.publisher = self.create_publisher(Image, self.topic, SENSOR_QOS,
                                               qos_overriding_options=qos_override_opts) 
        self.get_logger().info('Sonoptix Echo Initialized')

        # Main Loop
        try:
            while rclpy.ok():
                if not self.power_state:
                    rclpy.spin_once(self, timeout_sec=1.0)
                    continue
                
                ret, frame = self.cap.read()
                if not ret:
                    continue

                # Process frame
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Embed range in top-left pixel for the imager node to read
                frame[0, 0] = self.range 
                
                # Convert and publish
                msg = self.br.cv2_to_imgmsg(frame, encoding='mono8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                self.publisher.publish(msg)

                # Process callbacks (like parameter updates)
                rclpy.spin_once(self, timeout_sec=0.001)

        finally:
            self.shutdown_sonar()

    def _update_sonar_settings(self):
        """Helper to sync class attributes with the physical sonar via API."""
        state = 'on' if self.power_state else 'off'
        payload = {
            "power_state": state,
            "range": self.range,
            "tx_mode": self.tx_mode
        }
        try:
            requests.put(self.api_url + '/transceiver', json=payload, timeout=2.0)
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to update sonar settings: {e}")

    def shutdown_sonar(self):
        """Disables the transceiver on exit."""
        try:
            requests.put(self.api_url + '/transceiver', json={"power_state": 'off'}, timeout=2.0)
            self.get_logger().info('Transceiver disabled')
        except:
            pass
        self.destroy_node()
        rclpy.shutdown()

    def set_param_callback(self, params):
        """
        Updates node attributes when ROS parameters change and syncs with sonar.
        """
        result = SetParametersResult(successful=True)
        settings_changed = False

        for param in params:
            if "qos" in param.name:
                continue
            
            # Check if attribute exists and if the value is actually different
            if hasattr(self, param.name):
                old_value = getattr(self, param.name)
                if old_value != param.value:
                    setattr(self, param.name, param.value)
                    self.get_logger().info(f'Updated {param.name}: {param.value}')
                    
                    # If it's a sonar hardware setting, flag for API update
                    if param.name in ['range', 'power_state', 'tx_mode']:
                        settings_changed = True

        if settings_changed:
            self._update_sonar_settings()

        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        echo = EchoNode()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()