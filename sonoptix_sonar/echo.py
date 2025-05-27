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

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import requests


class EchoNode(Node):

    def __init__(self, node_name='echo'):
        super().__init__(node_name)

        # Declare Parameters
        params = {
            'range': [50, int],
            'ip': ['192.168.2.42', str],
            'enable_transponder': [True, bool],
            'topic': ['/sonar/echo/data', str],
            'frame_id': ['echo', str],
        }

        for param, [value, dtype] in params.items():
            self.declare_parameter(param, value)
            exec(f"self.{param}:dtype = self.get_parameter(param).value")
        params = self.get_parameters(params.keys())
        for param in params:
            self.get_logger().info(f'{param.name}: {param.value}')

        # Handle parameter updates
        self.param_handler_ptr_ = self.add_on_set_parameters_callback(
            self.set_param_callback)

        self.rtsp_url = f'rtsp://{self.ip}:8554/raw'
        self.api_url = f'http://{self.ip}:8000/api/v1'

        self.get_logger().info(f'Configuring Sonar')
        # Set the sonar range and enable the transponder
        requests.patch(self.api_url + '/transponder',
                       json={
                           "enable": self.enable_transponder,
                           "sonar_range": self.range
                       })

        # Set the data stream type to RTSP
        requests.put(self.api_url + '/streamtype', json={"value": 2})

        self.br = CvBridge()
        self.get_logger().info(f'Accessing RTSP stream')
        self.cap = cv2.VideoCapture(self.rtsp_url)
        self.publisher = self.create_publisher(Image, self.topic,
                                               rclpy.qos.qos_profile_sensor_data)
        self.get_logger().info(f'Sonoptix Echo Initialized')

        try:
            while True:
                if not self.enable_transponder:
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
            requests.patch(self.api_url + '/transponder', json={"enable": False})
            self.get_logger().info(f'Transponder disabled')
            self.destroy_node()
            rclpy.shutdown()

    def set_param_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            exec(f"self.flag = self.{param.name} != param.value")
            if self.flag:
                exec(f"self.{param.name} = param.value")
                self.get_logger().info(f'Updated {param.name}: {param.value}')

            if param.name in ['range', 'enable_transponder']:
                requests.patch(self.api_url + '/transponder',
                               json={
                                   "enable": self.enable_transponder,
                                   "sonar_range": self.range
                               })
        return result


def main():
    rclpy.init()
    node = EchoNode()


if __name__ == '__main__':
    main()
