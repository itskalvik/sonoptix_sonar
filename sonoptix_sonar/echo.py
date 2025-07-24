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

        for param, [value, dtype] in params.items():
            self.declare_parameter(param, value)
            exec(f"self.{param}:dtype = self.get_parameter(param).value")
        params = self.get_parameters(params.keys())
        for param in params:
            self.get_logger().info(f'{param.name}: {param.value}')

        qos_override_opts = QoSOverridingOptions(
            policy_kinds=(
                qos.QoSPolicyKind.HISTORY,
                qos.QoSPolicyKind.DEPTH,
                qos.QoSPolicyKind.RELIABILITY,
                )
        )
        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

        # Handle parameter updates
        self.param_handler_ptr_ = self.add_on_set_parameters_callback(
            self.set_param_callback)

        self.rtsp_url = f'rtsp://{self.ip}:8554/raw'
        self.api_url = f'http://{self.ip}:8000/api/v2'

        self.get_logger().info(f'Configuring Sonar')

        # Set the sonar range, tx_mode, and enable the transponder
        state = 'on' if self.power_state else 'off'
        requests.put(self.api_url + '/transceiver',
                     json={
                           "power_state": state,
                           "range": self.range,
                           "tx_mode": self.tx_mode
                       })

        # Set the data stream type to RTSP
        requests.put(self.api_url + '/datastream', 
                     json={"stream_type": 'rtsp'})

        # Initialize CV bridge, video capture, and ros2 publisher
        self.br = CvBridge()
        self.get_logger().info(f'Accessing RTSP stream')
        self.cap = cv2.VideoCapture(self.rtsp_url)
        self.publisher = self.create_publisher(Image, self.topic, SENSOR_QOS,
                                               qos_overriding_options=qos_override_opts) 
        self.get_logger().info(f'Sonoptix Echo Initialized')

        # Read and publish sonar data when available
        try:
            while True:
                if not self.power_state:
                    rclpy.spin_once(self, timeout_sec=1.0)
                    continue
                _, frame = self.cap.read()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame[0, 0] = self.range
                frame = self.br.cv2_to_imgmsg(frame, encoding='mono8')
                frame.header.stamp = self.get_clock().now().to_msg()
                frame.header.frame_id = self.frame_id

                self.publisher.publish(frame)

                # Allow for params callback to be processed
                rclpy.spin_once(self, timeout_sec=0.01)
        finally:
            # Stop the transponder before destroying the node
            requests.put(self.api_url + '/transceiver', json={"power_state": 'off',})
            self.get_logger().info(f'Transceiver disabled')
            self.destroy_node()
            rclpy.shutdown()

    def set_param_callback(self, params):
        """
        This function is the callback for when parameters are changed.
        It updates the node's attributes and sends the new settings to the sonar.
        """
        result = SetParametersResult(successful=True)
        for param in params:
            # QoS setting are handled by QoSOverridingOptions
            if "qos" in param.name:
                continue
            exec(f"self.flag = self.{param.name} != param.value")
            if self.flag:
                exec(f"self.{param.name} = param.value")
                self.get_logger().info(f'Updated {param.name}: {param.value}')

            # Configure sonar if necessary
            if param.name in ['range', 'power_state']:
                state = 'on' if self.power_state else 'off'
                requests.put(self.api_url + '/transceiver',
                             json={
                                "power_state": state,
                                "range": self.range,
                                "tx_mode": self.tx_mode
                             })
        return result


def main(args=None):
    """
    The main function to run the node.
    """
    rclpy.init(args=args)
    echo = EchoNode()
    rclpy.spin(echo)
    echo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
