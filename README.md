# Sonoptix Sonar ROS 2 Package

This ROS 2 package provides a robust driver and processing node for the [**Sonoptix Echo sonar**](https://bluerobotics.com/store/sonars/imaging-sonars/sonoptix-echo/). It allows you to interface with the sonar, capture raw data via RTSP, safely encode telemetry, and convert it into a polar image for visualization.

#### Sonar Data from a Joy Ride in a Pool
![Alt text](.assets/joy_ride.gif)

#### Sonar Data with Different Range Settings
![Alt text](.assets/range_demo.gif)

---

## Table of Contents
- [Overview](#overview)
- [Data Encoding Architecture](#data-encoding-architecture)
- [Installation](#installation)
- [Launch Files](#launch-files)
- [Nodes](#nodes)
  - [echo](#echo)
  - [echo_imager](#echo_imager)
- [License](#license)

---

## Overview
This package contains two main nodes designed to work together safely over lossy networks:

* **`echo`**: A driver node that interfaces directly with the Sonoptix sonar hardware REST API and RTSP stream. It captures raw 8-bit grayscale sonar data, embeds telemetry, and publishes it as a ROS 2 `sensor_msgs/Image`.
* **`echo_imager`**: A processing node that subscribes to the raw or compressed sonar data, extracts the embedded telemetry, converts the data into a polar (fan-shaped) visualization, and can publish the result or save it directly to an `.mp4` video file (ideal for processing `rosbag` files).

---

## Data Encoding Architecture
To robustly transmit the sonar's physical `range` setting alongside the image payload—without relying on secondary synchronized ROS topics—this package uses a **pixel-embedded 8-bit binary encoding**. 

1. **Sender (`echo`)**: Converts the active integer range into an 8-bit binary array. It writes these 8 bits into the absolute bottom-left 8 pixels of the image (`frame[-1, :8]`), scaling the values to `0` (binary 0) and `255` (binary 1) to survive video compression artifacts.
2. **Receiver (`echo_imager`)**: Reads the bottom 8 pixels, thresholds them at `127` to reconstruct the binary bits, and decodes the exact range integer. It then blacks out those 8 pixels so they do not appear as visual artifacts in the final polar warp.

---

## Installation
To install the [sonoptix_sonar](https://github.com/itskalvik/sonoptix_sonar) package, clone this repository into your ROS 2 workspace and build it using `colcon`:

```bash
cd ~/ros2_ws/src
git clone [https://github.com/itskalvik/sonoptix_sonar.git](https://github.com/itskalvik/sonoptix_sonar.git)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

```

---

## Launch Files

### `echo.launch.py`

Launches the `echo` node to publish sonar data and compresses the data stream for efficient transport. The node's publisher uses a `best_effort` reliability QoS setting by default. This can be changed via the launch file or parameters.

**Published Topics**:

* Raw sonar data: `/sonar/echo/data` (`sensor_msgs/Image`)
* Compressed sonar data: `/sonar/echo/compressed` (`sensor_msgs/CompressedImage`)

**Run Example**:

```bash
ros2 launch sonoptix_sonar echo.launch.py

```

**Update Range Dynamically**:

```bash
ros2 param set /echo range 50

```

### `echo_decompress.launch.py`

Decompresses the previously compressed sonar echo data for downstream processing or visualization. This is typically used when playing back compressed sonar data from a bag file.

**Subscribed Topics**: `/sonar/echo/compressed`

**Published Topics**:  `/sonar/echo/data` (`sensor_msgs/Image`)

**Run Example**:

```bash
ros2 launch sonoptix_sonar echo_decompress.launch.py

```

---

## Nodes

### `echo`

**Description**: Configures the Sonoptix Echo via its REST API, manages a safe RTSP connection with dropped-frame protection, encodes the max range into the image payload, and publishes the profile.

**Published Topics**: `/sonar/echo/data` (`sensor_msgs/Image`)

**Parameters**: *(Can be updated dynamically while the node is running)*

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `range` | int | 50 | Sonar range in meters [max 255 via 8-bit] |
| `ip` | str | "192.168.2.42" | IP address of the sonar device |
| `tx_mode` | str | "auto" | Transmit mode [`auto`, `hf`, `lf`, `lflr`] |
| `power_state` | bool | `True` | The power state of the transceiver |
| `topic` | str | `/sonar/echo/data` | Topic to publish sonar frames |
| `frame_id` | str | `echo` | TF frame ID |

**Run Example**:

```bash
ros2 run sonoptix_sonar echo

```

---

### `echo_imager`

**Description**: Extracts the 8-bit encoded max range from the raw stream, dynamically calculates the required field of view (FOV) and mapping coordinates, and converts the raw data into a `cv2.COLORMAP_VIRIDIS` polar image.

**Subscribed Topics**: `/sonar/echo/compressed` (or raw depending on `data_topic`)

**Published Topics**: `/sonar/echo/image` (`sensor_msgs/Image`)

**Parameters**: *(Can be updated dynamically while the node is running)*

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `data_topic` | str | `/sonar/echo/compressed` | Input topic for sonar data |
| `image_topic` | str | `/sonar/echo/image` | Output topic for visualized image |
| `contrast` | float | `10.0` | Contrast multiplier for visualization |
| `bag_file` | str | `""` | Optional path to an input ros2 bag file |
| `video_file` | str | `""` | Optional path to an output mp4 video file |

**Run Example**:

```bash
ros2 run sonoptix_sonar echo_imager --ros-args -p contrast:=10.0 -p data_topic:=/sonar/echo/data

```

*Note: When reading from a `bag_file`, the node will export the output to an `echo_sonar.mp4` video file by default if `video_file` is left empty.*

---

## License

This package is licensed under the MIT License. See the top of each file or [LICENSE](https://www.google.com/search?q=LICENSE) for details.
EOF
