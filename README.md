# Sonoptix Sonar ROS2 Package

## Overview

This ROS2 package provides nodes for interfacing with and visualizing data from the [**Sonoptix Echo**](https://bluerobotics.com/store/sonars/imaging-sonars/sonoptix-echo/) sonar. It includes two nodes:

- `echo`: Publishes sonar data from the Echo device.
- `echo_imager`: Converts and visualizes the published sonar data into a color-mapped polar image for better interpretability.

---

## Table of Contents

- [Installation](#installation)
- [Nodes and Usage](#nodes-and-usage)
  - [echo](#echo)
  - [echo_imager](#echo_imager)
- [License](#license)

---

## Installation
Install [Sonoptix Sonar](https://github.com/itskalvik/sonoptix_sonar) ROS2 package:

```bash
cd ~/ros2_ws/src
git clone https://github.com/itskalvik/sonoptix_sonar.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Nodes and Usage

### `echo`

**Description**: Publishes sonar profiles from the Sonoptix Echo.

**Published Topics**: `sonar/echo/data` (`sensor_msgs/Image`)

**Parameters:** The parameters can be updated while running the node.

| Name                 | Type    | Default            | Description                              |
|----------------------|---------|--------------------|------------------------------------------|
| `range`              | int     | 50                 | Sonar range in meters                    |
| `ip`                 | str     | "192.168.2.42"     | IP address of the sonar device          |
| `enable_transponder`| bool    | `True`             | Enable or disable the sonar transponder |
| `topic`              | str     | `sonar/echo/data`  | Topic to publish sonar frames           |
| `frame_id`           | str     | `echo`             |  TF frame ID             |

**Run Example:**
```bash
ros2 run sonoptix_sonar echo
```

---

### `echo_imager`

**Description**: Converts Echo sonar data into a polar image and publishes it.

**Subscribed Topics**: `sonar/echo/data`

**Published Topics**: `sonar/echo/image` (`sensor_msgs/Image`)

**Parameters:** The parameters can be updated while running the node.

| Name           | Type   | Default              | Description                              |
|----------------|--------|----------------------|------------------------------------------|
| `data_topic`   | str    | `sonar/echo/data`    | Input topic for raw sonar data         |
| `image_topic`  | str    | `sonar/echo/image`   | Output topic for visualized image       |
| `contrast`     | float  | `30.0`               | Contrast multiplier for visualization   |

**Run Example:**
```bash
ros2 run sonoptix_sonar echo_imager
```

---

## License

This package is licensed under the MIT License. See the top of each file or [LICENSE](LICENSE) for details.
