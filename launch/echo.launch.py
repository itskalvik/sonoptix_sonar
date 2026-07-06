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

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    # Higher is more compressed but requires more compute [1-9]
    compression_level = 1
    sonar_range = 12  # meters
    data_topic = '/sonar/echo/data'
    reliability = 'best_effort' # QoS Config
    disable_compressed = False

    echo_data = Node(package='sonoptix_sonar',
                     executable='echo',
                     parameters=[{
                         'topic': data_topic,
                         'range': sonar_range,
                         'disable_compressed': disable_compressed,
                         'png_compression_level': compression_level,
                         f'qos_overrides.{data_topic}.publisher.reliability': reliability
                     }],
                     output='screen')

    return LaunchDescription([echo_data])
