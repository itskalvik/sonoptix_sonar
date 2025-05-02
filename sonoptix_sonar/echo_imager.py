#! /usr/bin/env python3

#-----------------------------------------------------------------------------------
# MIT License

# Copyright (c) 2025 Kalvik Jakkala

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


class EchoImager(Node):
    def __init__(self, node_name='echo_imager'):
        super().__init__(node_name)

        # Declare Parameters
        params = {
          'data_topic': ['sonar/echo/data', str],
          'image_topic': ['sonar/echo/image', str],
          'contrast': [30.0, float],
        }

        for param, [value, dtype] in params.items():
            self.declare_parameter(param, value)
            exec(f"self.{param}:dtype = self.get_parameter(param).value")
            self.get_logger().info(f'{param}: {value}')

        # Handle parameter updates
        self.add_on_set_parameters_callback(self.set_param_callback)

        self.br = CvBridge()
        self.publisher = self.create_publisher(Image,
                                               self.image_topic,
                                               10)
        self.subscrber = self.create_subscription(Image,
                                                  self.data_topic,
                                                  self.data_callback,
                                                  10)
        self.get_logger().info("Node initialized!")

    def data_callback(self, msg):
        scan_image = self.br.imgmsg_to_cv2(msg)
        max_range = scan_image[0, 0]
        fov = 120 if max_range <= 30 else 90        
        scan_image = cv2.convertScaleAbs(scan_image, alpha=self.contrast, beta=0)
        total_pix = int(((256 / 256 * (256/fov) * 360)-256)/2)
        scan_image = cv2.copyMakeBorder(scan_image, 
                                        0, 0, total_pix, total_pix, 
                                        cv2.BORDER_CONSTANT)
        scan_image = cv2.warpPolar(scan_image.T,
                                   dsize=(1500, 1500), 
                                   center=(750, 750), 
                                   maxRadius=750, 
                                   flags=cv2.WARP_INVERSE_MAP|cv2.WARP_FILL_OUTLIERS)
        scan_image = cv2.rotate(scan_image, cv2.ROTATE_90_CLOCKWISE)
        if fov == 90:
            scan_image = scan_image[0:750, 200:1300]
        elif fov == 120:
            scan_image = scan_image[0:750,75:1425]
        scan_image = cv2.applyColorMap(scan_image, cv2.COLORMAP_VIRIDIS)
        scan_image = cv2.putText(scan_image, 
                                 f'Range: {max_range} m',
                                 (1, 25), 
                                 cv2.FONT_HERSHEY_SIMPLEX, 
                                 1, (255, 255, 255), 2, 
                                 cv2.LINE_AA)
        self.publisher.publish(self.br.cv2_to_imgmsg(scan_image))

    def set_param_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            exec(f"self.flag = self.{param.name} != param.value")
            if self.flag:
                exec(f"self.{param.name} = param.value")
                self.get_logger().info(f'Updated {param.name}: {param.value}')
        return result

def main():
    rclpy.init()
    node = EchoImager()
    rclpy.spin(node)

if __name__ == '__main__':
    main()