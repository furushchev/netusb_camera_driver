# netusb_camera_driver
- - -

The ROS Interface for NETUSB Camera with Driver

## Installation

0. Download package

```bash
$ cd /path/to/catkin_ws
$ wstool set netusb_camera_driver --git https://github.com/furushchev/netusb_camera_driver -t src
$ wstool up netusb_camera_driver
```

1. Install dependencies

```bash
$ rosdep install --from-paths src --ignore-src -r -n -y
```

2. Build this package

```bash
$ catkin build netusb_camera_driver
```

2. Install udev rules to recognize camera

```bash
$ roscd netusb_camera_driver
$ sudo cp udev/99-netusbcam.rules /etc/udev/rules.d
```

## Launching Node

1. Launch roscore

```bash
$ roscore
```

2. Launch Driver

```bash
$ rosrun netusb_camera_driver netusb_camera_node
```

3. Launch viewer

```bash
$ rosrun image_view image_view image:=/image_raw
```
