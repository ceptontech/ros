# Cepton ROS1
- [Cepton ROS1](#cepton-ros1)
  - [1. Overview](#1-overview)
  - [2. Getting Started](#2-getting-started)
    - [2.0 Building source locally](#20-building-source-locally)
    - [2.1 Installation](#21-installation)
    - [2.2 Launching the driver](#22-launching-the-driver)
    - [2.3 Visualizing the point cloud](#23-visualizing-the-point-cloud)
  - [3. Configuration Parameter Arguments](#3-configuration-parameter-arguments)
  - [4. Points Topic](#4-points-topic)
  - [5. Sensor Information Topic](#5-sensor-information-topic)
  - [6. Sensor Panic Topic](#6-sensor-panic-topic)
  - [7. Known Issues](#7-known-issues)

## 1. Overview

This ROS package provides support for Cepton LiDAR with SDK Version >= 2.0.

The ROS package for Cepton LiDAR with SDK Version <2.0 is located at:
https://github.com/ceptontech/cepton_sdk_redist/tree/master/ros

## 2. Getting Started
### 2.0 Building source locally

This step is only needed if building the SDK binaries locally. If you are running a released driver package, the binaries should already be included and you can skip to section 2.1.

See root `README.md`. In short:
```bash
mkdir build-linux
cd build-linux
cmake .. -DINSTALL_INPLACE=ON -DINPLACE_ROS1=ON
cmake --build . --config Release
cmake --install . --config Release
```

### 2.1 Installation
- Install ROS and create catkin workspace. Then, change to the catkin_ws directory.
- Clone/copy the package into catkin_ws/src/.

  - Path to this README.md file would be catkin_ws/src/ros/README.md

- Then, run the following:
```sh
catkin_make
source ./devel/setup.bash
```
### 2.2 Launching the driver

To use the driver standalone, you must first launch the manager nodelet
```sh
roslaunch cepton2_ros manager.launch
```

Then you can launch the Publisher nodelet with the default yaml config file path (default is default_params.yaml)
```sh
roslaunch cepton2_ros publisher.launch
```

This loads the parameters from cepton2_ros/config/default_params.yaml into the rosparam server and starts the publisher nodelet to begin publishing point data and sensor information from a live sensor.

You may also define your own yaml config file and pass in as an argument:
```sh
roslaunch cepton2_ros publisher.launch config_path:=<path_to_file>
```

The YAML config is described in Section 4.0.

### 2.3 Visualizing the point cloud

The driver publishes sensor_msgs/PointCloud2.h messages which can be visualized in RViz. Currently the frame_id is set to "cepton2" and not configurable, but this may be changed upon request.

Once the publisher is launched, you can open up RViz, add a new PointCloud2 display and configure the topic to match the publisher data. You will need to also configure the "Fixed Frame" under "Global Options" to be `cepton2`.

To see what topics are being published use:
```sh
rostopic list
```

**NOTE**: the channel name of the PointCloud2 display should be changed from "intensity" to "reflectivity" in order to visualize the reflectivity with each point.

## 3. Configuration Parameter Arguments
Located in ros/config/default_params.yaml :
* `capture_file` (string): the absolute path to a pcap file for replay
* `capture_loop` (boolean): whether to replay the pcap in a loop, default=false
* `half_frequency_mode` (boolean): concatenates 2 frames into 1, publishing them effectively at half rate (not guaranteed every time.), default=false
* `include_saturated_points` (boolean): include points with the CEPTON_POINT_SATURATED flag bit, default=false
* `include_second_return_points` (boolean): include points with the CEPTON_POINT_SECOND_RETURN flag bit, default=false
* `include_invalid_points` (boolean): include points with the CEPTON_POINT_NO_RETURN flag bit, default=false
* `include_noise_points` (boolean): include points with the CEPTON_POINT_NOISE flag bit, default=false
* `include_blocked_points` (boolean): include points with the CEPTON_POINT_BLOCKED flag bit, default=false
* `output_by_handle` (boolean): output topic with handle info as part of the topic name, default=true
* `output_by_sn` (boolean): output topic with serial number  as part of the topic name, default=true
* `min_altitude` (double): minimum altitude angle (in degrees) that is allowed, default=-90.0°
* `max_altitude` (double): maximum altitude angle (in degrees) that is allowed, default=90.0°
* `min_azimuth` (double): minimum azimuth angle (in degrees) that is allowed, default=-180.0°
* `max_azimuth` (double): maximum azimuth angle (in degrees) that is allowed, default=180.0°

## 4. Points Topic
The nodelet will publish messages containing arrays of points with the `CustomCeptonPoint` structure defined in `include/cepton2_ros/point.hpp`.

- x, y, z - cartestian coordinates of the point in meters.

- reflectivity - reflectivity measurement from 0-255%

- relative_timestamp - time in ns from the previous laser firing

- flags - bitwise flags, see include/cepton2_sdk.h for more information

- channel_id - the laser from which the point was detected

- valid - true if the point is valid, false if not

The header timestamp of each message sent with this topic will contain the start_timestamp of the frame. To calculate the precise timestamp of each point you would need to use this value.

The prefix of the point topic is configurable through the yaml file. For each sensor connected there will be a separate topic with a different seriai number suffix. The naming convention is as follows:

```sh
<topic_prefix>_<serial_num>
```

Example: If topic prefix is cepton2/points and two sensors (10123 & 10126) are connected, then the nodelet will publish two topics:
```sh
cepton2/points_10123
cepton2/points_10126
```

## 5. Sensor Information Topic
The nodelet publishes a sensor information message that is universal for all sensors connected. This topic name can be configurable via the publisher configuration parameters. See `SensorInformation.msg` for the message fields.

## 6. Sensor Panic Topic
The nodelet also publishes a PANIC packet for when the Lidar detects a fatal fault. The `cepton_panic` topic facilitates panic messages from all lidars. See `SensorPanic.msg` for the message fields.

## 7. Known Issues
- Publishing sensor transformations is not supported at this time.
- RViz launch file not working with config.





BUILDING

catkin_make -DSDK_DOWNLOAD_LOCATION=/home/cepton/ros