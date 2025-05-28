# Cepton ROS2
- [Cepton ROS2](#cepton-ros2)
  - [1. Overview](#1-overview)
  - [2. Getting Started](#2-getting-started)
    - [2.0 Building source locally](#20-building-source-locally)
    - [2.1 Installation](#21-installation)
    - [2.2 Launching the Driver](#22-launching-the-driver)
    - [2.3 Displaying point cloud in Rviz2](#23-displaying-point-cloud-in-rviz2)
  - [3. Configuration Parameter Arguments](#3-configuration-parameter-arguments)
    - [3.1 Publisher Parameter Arguments](#31-publisher-parameter-arguments)
    - [3.2 Subscriber Parameter Arguments](#32-subscriber-parameter-arguments)
  - [4. Topics](#4-topics)

## 1. Overview

This ROS2 package provides support for Cepton LiDAR with SDK Version >= 2.0.

The driver includes two things:

`cepton_subscriber`
* A sample of how to receive data for Cepton ROS topics

`cepton_publisher`
* Publishes topics for different lidar messages. See [Topics](#4-topics) for available topics.

## 2. Getting Started
### 2.0 Building Source Locally

This step is only needed if building the SDK binaries locally. If you are running a released driver package, the binaries should already be included and you can skip to section 2.1.

See root `README.md`. In short:
```bash
mkdir build-linux
cd build-linux
cmake .. -DINSTALL_INPLACE=ON -DINPLACE_ROS2=ON
cmake --build . --config Release
cmake --install . --config Release
```

### 2.1 Installation
All steps are run from the `ros2` directory. Substitute `[VER]` for the SDK version you are using (i.e. 22, 23, 24, etc)
``` sh
# Setup the ROS2 environment with the installation version you have (humble example below)
source /opt/ros/humble/setup.bash

# Build the messages, publisher, and subscriber
colcon build --packages-select cepton_messages cepton_subscriber cepton_publisher
```
### 2.2 Launching the Driver

All steps are run from the `ros2` directory
```sh
# Terminal 1
. install/setup.bash
ros2 run cepton_publisher cepton_publisher_node
```
```
# Terminal 2
. install/setup.bash
ros2 run cepton_subscriber cepton_subscriber_node
```
### 2.3 Displaying point cloud in Rviz2
After installing `rviz` and starting it with `ros2 run rviz2 rviz2`, and also starting the publisher node, do the following configuration in the RVIZ UI to show the proper point cloud:

* Under `global options`, change the `Fixed Frame` setting to read `cepton2`
* Select `Add Topic` and add the `/cepton_pcl2` topic
* Deselect `Autocompute Intensity` and change `Max Intensity` to `1`
```
Example of replaying a pcap on RVIZ2: 
ros2 run cepton_publisher cepton_publisher_node --ros-args -p capture_file:="/path/to/pcap" -p capture_loop:=true
```

## 3. Configuration Parameter Arguments

### 3.1 Publisher Parameter Arguments
The publisher node can optionally be with any of the following arguments:
* `capture_file` (string): the absolute path to a pcap file for replay
* `capture_loop` (boolean): whether to replay the pcap in a loop, default=false
* `sensor_port` (int): the port to listen for sensor UDP data, default=8808
* `half_frequency_mode` (boolean): concatenates 2 frames into 1, publishing them effectively at half rate (not guaranteed every time.), default=false
* `cepp_output_type` (string): configure if `cepp_points` format point cloud should be outputted. Can either output topic based on "IP", "SN" (serial number), "NONE", or "BOTH". default="BOTH".
* `pcl2_output_type` (string): configure if `pcl2` format point cloud should be outputted. Can either output topic based on "IP", "SN" (serial number), "NONE", or "BOTH". default="BOTH".
* `include_saturated_points` (boolean): include points with the CEPTON_POINT_SATURATED flag bit, default=false
* `include_second_return_points` (boolean): include points with the CEPTON_POINT_SECOND_RETURN flag bit, default=false
* `include_invalid_points` (boolean): include points with the CEPTON_POINT_NO_RETURN flag bit, default=false
* `include_noise_points` (boolean): include points with the CEPTON_POINT_NOISE flag bit, default=false
* `include_blocked_points` (boolean): include points with the CEPTON_POINT_BLOCKED flag bit, default=false
* `min_altitude` (double): minimum altitude angle (in degrees) that is allowed, default=-90.0°
* `max_altitude` (double): maximum altitude angle (in degrees) that is allowed, default=90.0°
* `min_azimuth` (double): minimum azimuth angle (in degrees) that is allowed, default=-180.0°
* `max_azimuth` (double): maximum azimuth angle (in degrees) that is allowed, default=180.0°
```
Example:
ros2 run cepton_publisher cepton_publisher_node --ros-args -p capture_file:="/path/to/pcap" -p capture_loop:=true -p include_saturated_points:=true -p min_altitude:=-10.0
```

### 3.2 Subscriber Parameter Arguments
**Optional subscriber options**

If running the subscriber node sample, the following options are provided:
* `subscribe_pcl2` (boolean)
* `subscribe_cepton_points` (boolean)
* `subscribe_cepton_info` (boolean)
* `export_to_csv` (boolean): if true, then export `CeptonPointData` messages to a new directory `frames`, located in current working directory.

## 4. Topics
| Topic              | Type                                      | Description                                                             |
| ------------------ | ----------------------------------------- | ----------------------------------------------------------------------- |
| `handle_<ID>`      | `sensor_msgs::msg::PointCloud2`           | Per-sensor frame, where `<ID>` is the sensor's IP address as a `uint32` |
| `serial_<ID>`      | `sensor_msgs::msg::PointCloud2`           | Per-sensor frame, where `<ID>` is the sensor's serial number            |
| `cepp_handle_<ID>` | `cepton_messages::msg::CeptonPointData`   | Per-sensor frame, where `<ID>` is the sensor's IP address as a `uint32` |
| `cepp_serial_<ID>` | `cepton_messages::msg::CeptonPointData`   | Per-sensor frame, where `<ID>` is the sensor's serial number            |
| `cepton_points`    | `cepton_messages::msg::CeptonPointData`   | All sensor frames are published over this topic                         |
| `cepton_pcl2`      | `sensor_msgs::msg::PointCloud2`           | All sensor frames are published over this topic                         |
| `cepton_info`      | `cepton_messages::msg::CeptonSensorInfo`  | Sensor info messages                                                    |
| `cepton_panic`     | `cepton_messages::msg::CeptonSensorPanic` | Sensor panic messages                                                   |

* Note that all `serial` fields only start publishing, after receiving the first info packet for the sensor, since the driver needs to associate IP to serial number in order to publish these topics.
