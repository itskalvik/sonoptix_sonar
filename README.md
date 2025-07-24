# Sonoptix Sonar ROS2 Package

This ROS 2 package provides a driver and processing node for a [**Sonoptix Echo**](https://bluerobotics.com/store/sonars/imaging-sonars/sonoptix-echo/). It allows you to interface with the sonar, capture raw data, and convert it into a polar image for visualization.

#### Sonar Data from a Joy Ride in a Pool
![Alt text](.assets/joy_ride.gif)

#### Sonar Data with Different Range Settings
![Alt text](.assets/range_demo.gif)
---

## Table of Contents
- [Overview](#overview)
- [Installation](#installation)
- [Launch Files](#launch-files)
  - [echo.launch.py](#echolaunchpy)
  - [echo_decompress.launch.py](#echo_decompresslaunchpy)
- [Nodes](#nodes)
  - [echo](#echo)
  - [echo_imager](#echo_imager)
- [License](#license)

---

## Overview
This package contains two main nodes:

* **`echo.py`**: A driver node that interfaces directly with the Sonoptix sonar hardware. It captures raw sonar data and publishes it as a ROS 2 sensor_msgs/Image message.

* **`echo_imager.py`**: A processing node that subscribes to the raw sonar data, converts it into a polar (fan-shaped) image, and can either publish this image on a new topic or save it to a video file. This node can also process data from a ROS 2 bag file.

## Installation
To install the [sonoptix_sonar]((https://github.com/itskalvik/sonoptix_sonar) ) package, clone this repository into your ROS 2 workspace and build it using `colcon`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/itskalvik/sonoptix_sonar.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Launch Files

### `echo.launch.py`

Launches the [echo](#echo) node to publish sonar data and compresses the data stream for efficient transport.
The [echo](#echo) node publisher with `best_effort` reliability QoS setting, this can be changed from the launch file or using the `ros2 param set` command.

**Published Topics**:
- Raw sonar data: `/sonar/echo/data` (`sensor_msgs/Image`)
- Compressed sonar data: `/sonar/echo/compressed` (`sensor_msgs/CompressedImage`)

**Run Example**:

```bash
ros2 launch sonoptix_sonar echo.launch.py
```

Use the following command to upate the range setting:

```bash
ros2 param set /echo range:=<[3 - 200] integer value>
```

### `echo_decompress.launch.py`

Decompresses the previously compressed sonar echo data for downstream processing or visualization. This node is usually used when processing sonar data from a bag file.

**Subscribed Topics**: `/sonar/echo/compressed`

**Published Topics**:  `/sonar/echo/data` (`sensor_msgs/Image`)

**Run Example**:

```bash
ros2 launch sonoptix_sonar decompress.launch.py
```

---

## Nodes

### `echo`

**Description**: Publishes sonar profiles from the Sonoptix Echo.

**Published Topics**: `/sonar/echo/data` (`sensor_msgs/Image`)

**Parameters**: The parameters can be updated while running the node.

| Name                 | Type    | Default            | Description                              |
|----------------------|---------|--------------------|------------------------------------------|
| `range`              | int     | 50                 | Sonar range in meters [3-200]            |
| `ip`                 | str     | "192.168.2.42"     | IP address of the sonar device           |
| `tx_mode`            | str     | "auto"             | The transmit mode of the transceiver [`auto`, `hf`, `lf`, `lflr`] |
| `power_state`        | bool    | `True`             | The power state of the transceiver       |
| `topic`              | str     | `/sonar/echo/data` | Topic to publish sonar frames            |
| `frame_id`           | str     | `echo`             |  TF frame ID                             |

**Run Example**:
```bash
ros2 run sonoptix_sonar echo
```

---

### `echo_imager`

**Description**: Converts Echo sonar data into a polar image and publishes it.

**Subscribed Topics**: `/sonar/echo/data`

**Published Topics**: `/sonar/echo/image` (`sensor_msgs/Image`)

**Parameters**: The parameters can be updated while running the node.

| Name           | Type   | Default              | Description                               |
|----------------|--------|----------------------|-------------------------------------------|
| `data_topic`   | str    | `/sonar/echo/data`    | Input topic for sonar data (raw or compressed) |
| `image_topic`  | str    | `/sonar/echo/image`   | Output topic for visualized image         |
| `contrast`     | float  | `10.0`               | Contrast multiplier for visualization     |
| `bag_file`     | str    |                      | Optional path to an input ros2 bag file with sonar data |
| `video_file`   | str    |                      | Optional path to an output mp4 video file |

**Run Example**:
```bash
ros2 run sonoptix_sonar echo_imager
```

Note that when reading from a bag file, the node will export the output to `echo_sonar.mp4` video file by default. 

---

## License

This package is licensed under the MIT License. See the top of each file or [LICENSE](LICENSE) for details.
