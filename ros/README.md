# Setting up and building
```bash
# Clone the repo
git clone git@github.com:ceptontech/ros.git

# Set up a catkin workspace and symlink the repo
mkdir -p catkin_ws/src
pushd catkin_ws/src
ln -s $(pwd)/../../ros/ros
popd

# Build
pushd catkin_ws
catkin_make
```

# Configuration 
# Cepton ROS Publisher Nodelet Configuration Guide

This guide covers all configuration options available for the Cepton ROS Publisher Nodelet based on the `publisher_nodelet.cpp` implementation.

## Basic Configuration

### Data Source Options

#### `capture_path` (string, default: "")
- **Description**: Path to a PCAP capture file for replay mode
- **Usage**: When specified, the nodelet will replay data from the file instead of listening to live sensors
- **Example**: `capture_path: "/path/to/capture.pcap"`

#### `capture_loop` (bool, default: true)
- **Description**: Whether to loop the capture replay continuously
- **Usage**: Only applies when `capture_path` is specified
- **Note**: Sets the `CEPTON_REPLAY_FLAG_PLAY_LOOPED` flag internally

### Spatial Filtering

#### `min_altitude` (double, default: varies)
- **Description**: Minimum altitude angle in degrees for point filtering
- **Range**: Clamped to [-89.9°, 89.9°] internally
- **Usage**: Points below this altitude will be filtered out

#### `max_altitude` (double, default: varies)
- **Description**: Maximum altitude angle in degrees for point filtering
- **Range**: Clamped to [-89.9°, 89.9°] internally
- **Usage**: Points above this altitude will be filtered out

#### `min_azimuth` (double, default: varies)
- **Description**: Minimum azimuth angle in degrees for point filtering
- **Range**: Clamped to [-89.9°, 89.9°] internally
- **Usage**: Points to the left of this azimuth will be filtered out

#### `max_azimuth` (double, default: varies)
- **Description**: Maximum azimuth angle in degrees for point filtering
- **Range**: Clamped to [-89.9°, 89.9°] internally
- **Usage**: Points to the right of this azimuth will be filtered out

#### `min_distance` (double, default: varies)
- **Description**: Minimum distance threshold in meters
- **Usage**: Points closer than this distance will be filtered out

#### `max_distance` (double, default: varies)
- **Description**: Maximum distance threshold in meters
- **Usage**: Points farther than this distance will be filtered out

## Output Configuration

### Publisher Options

#### `output_by_handle` (bool, default: varies)
- **Description**: Enable publishing point clouds on handle-specific topics
- **Topics Created**: `cepton3/points_handle_<HANDLE>`
- **Usage**: Useful when working with multiple sensors and need separate topics per sensor handle

#### `output_by_sn` (bool, default: varies)
- **Description**: Enable publishing point clouds on serial number-specific topics
- **Topics Created**: `cepton3/points_sn_<SERIAL_NUMBER>`
- **Usage**: Useful when working with multiple sensors and need separate topics per serial number

### Frame Processing

#### `aggregation_mode` (int, default: varies)
- **Description**: Sets the frame aggregation mode for the Cepton SDK
- **Usage**: Controls how points are aggregated into frames before publishing

## Network Configuration

#### `expected_sensor_ips` (array of strings, default: empty)
- **Description**: List of expected sensor IP addresses for monitoring
- **Usage**: Helps with sensor status monitoring and timeout detection
- **Example**: 
  ```yaml
  expected_sensor_ips:
    - "192.168.1.10"
    - "192.168.1.11"
  ```

## Point Quality Filtering

### Point Inclusion Flags

#### `include_saturated_points` (bool, default: true)
- **Description**: Include points marked as saturated
- **SDK Flag**: `CEPTON_POINT_SATURATED`

#### `include_second_return_points` (bool, default: true)
- **Description**: Include second return points
- **SDK Flag**: `CEPTON_POINT_SECOND_RETURN`

#### `include_invalid_points` (bool, default: false)
- **Description**: Include invalid points (no return detected)
- **SDK Flag**: `CEPTON_POINT_NO_RETURN`

#### `include_noise_points` (bool, default: false)
- **Description**: Include points classified as noise
- **SDK Flag**: `CEPTON_POINT_NOISE`

#### `include_blocked_points` (bool, default: false)
- **Description**: Include points that are blocked
- **SDK Flag**: `CEPTON_POINT_BLOCKED`

**Note**: The following flags are always included internally:
- `CEPTON_POINT_BLOOMING`
- `CEPTON_POINT_FRAME_PARITY` 
- `CEPTON_POINT_FRAME_BOUNDARY`

## Published Topics

### Standard Topics

- **`cepton3/points`**: Main point cloud topic (all sensors combined)
- **`cepton3/sensor_information`**: Sensor information and status
- **`cepton3/cepton_sensor_status`**: Sensor status monitoring

### Conditional Topics (when enabled)

- **`cepton3/points_handle_<HANDLE>`**: Per-handle point clouds (when `output_by_handle` is true)
- **`cepton3/points_sn_<SERIAL_NUMBER>`**: Per-serial-number point clouds (when `output_by_sn` is true)
- **`cepton3/info_handle_<SERIAL_NUMBER>`**: Per-sensor information topics

## Message Types

### Point Cloud Data (`cepton_ros::Cloud`)
Contains the following fields per point:
- `x, y, z`: 3D coordinates (converted to ROS coordinate system)
- `reflectivity`: Reflectivity value (scaled by 0.01)
- `relative_timestamp`: Relative timestamp within the frame
- `channel_id`: Sensor channel identifier
- `flags`: Point quality flags
- `azimuth`: Calculated azimuth angle in radians
- `elevation`: Calculated elevation angle in radians
- `valid`: Boolean indicating if point has valid return

### Sensor Information (`cepton_ros::SensorInformation`)
- `handle`: Sensor handle
- `serial_number`: Sensor serial number
- `model_name`: Sensor model name
- `model`: Model identifier
- `part_number`: Part number
- `firmware_version`: Firmware version
- `power_up_timestamp`: Power-up timestamp
- Various timing and status fields

## Example Configuration

```yaml
# Data source
capture_path: ""                    # Empty for live data
capture_loop: true

# Spatial filtering
min_altitude: -25.0
max_altitude: 15.0
min_azimuth: -60.0
max_azimuth: 60.0
min_distance: 0.3
max_distance: 200.0

# Output options
output_by_handle: true
output_by_sn: false
aggregation_mode: 0

# Network
expected_sensor_ips:
  - "192.168.1.201"

# Point filtering
include_saturated_points: true
include_second_return_points: true
include_invalid_points: false
include_noise_points: false
include_blocked_points: false
```

## Coordinate System Transformation

The nodelet automatically converts from Cepton's coordinate system to ROS standard:
- **Cepton**: X (forward), Y (left), Z (up)
- **ROS**: X (right), Y (forward), Z (up)
- **Transformation**: `(x_ros, y_ros, z_ros) = (y_cepton, -x_cepton, z_cepton)`

## Monitoring and Status

The nodelet includes built-in sensor monitoring with a 3-second timeout. If a sensor stops sending data for more than 3 seconds, a timeout status message is published on the `cepton3/cepton_sensor_status` topic.