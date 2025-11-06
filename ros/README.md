# Cepton ROS Publisher Nodelet Configuration Guide

This guide covers all configuration options available for the Cepton ROS Publisher Nodelet based on the implementation in `publisher_nodelet.cpp`.

## Basic Configuration Parameters

### Data Source Options

#### `capture_path` (string, default: "")
- **Description**: Path to a PCAP capture file for replay mode
- **Usage**: When specified, the nodelet will replay data from the file instead of listening to live sensors
- **Example**: `capture_path: "/path/to/capture.pcap"`

#### `capture_loop` (bool, default: true)
- **Description**: Whether to loop the capture replay continuously
- **Usage**: Only applies when `capture_path` is specified
- **Note**: Sets the `CEPTON_REPLAY_FLAG_PLAY_LOOPED` flag internally

#### `sensor_ports` (vector<int>, default: empty)
- **Description**: List of UDP ports for receiving live sensor data
- **Usage**: Only applies when not using capture file replay. Empty list uses default port 8808. Can specify multiple ports to listen on simultaneously.
- **Note**: Standard Cepton sensor communication port is 8808
- **Example**:
```yaml
sensor_ports: [8808, 8809, 8810]
```

### Output Configuration

#### `output_by_handle` (bool, default: varies)
- **Description**: Enable publishing point clouds on handle-specific topics
- **Topics Created**: `cepton3/points_handle_<HANDLE>`
- **Usage**: Useful when working with multiple sensors and need separate topics per sensor handle

#### `output_by_sn` (bool, default: varies)
- **Description**: Enable publishing point clouds on serial number-specific topics
- **Topics Created**: `cepton3/points_sn_<SERIAL_NUMBER>`
- **Usage**: Useful when working with multiple sensors and need separate topics per serial number

### Network Configuration

#### `expected_sensor_ips` (vector<string>, default: empty)
- **Description**: List of expected sensor IP addresses for monitoring
- **Usage**: Helps with sensor status monitoring and timeout detection
- **Example**:
```yaml
expected_sensor_ips:
  - "192.168.1.10"
  - "192.168.1.11"
```

## Point Filtering Options

 
### Spatial Filtering

#### `min_altitude` (double, default: varies)
- **Description**: Minimum elevation angle in degrees for point filtering
- **Range**: Clamped to [-89.9°, 89.9°] internally
- **Usage**: Points below this elevation will be filtered out

#### `max_altitude` (double, default: varies)
- **Description**: Maximum elevation angle in degrees for point filtering
- **Range**: Clamped to [-89.9°, 89.9°] internally
- **Usage**: Points above this elevation will be filtered out

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

## Published Topics

### Standard Topics
- **`cepton3/points`**: Main point cloud topic (cepton_ros::Cloud)
- **`cepton3/sensor_information`**: Unified sensor information topic
- **`cepton3/cepton_sensor_status`**: Sensor status monitoring topic

### Conditional Topics (when enabled)
- **`cepton3/points_handle_<HANDLE>`**: Per-handle point clouds (when `output_by_handle` is true)
- **`cepton3/points_sn_<SERIAL_NUMBER>`**: Per-serial-number point clouds (when `output_by_sn` is true)
- **`cepton3/info_handle_<SERIAL_NUMBER>`**: Per-sensor information topics

## Message Types

### Point Cloud Data (cepton_ros::Cloud)
Contains the following fields per point:
- **x, y, z**: 3D coordinates (converted to ROS coordinate system)
- **intensity**: Reflectivity value (scaled by 0.01)

#### Optional Fields (compile-time flags)

**WITH_TS_CH_F Fields**:
- **relative_timestamp**: Relative timestamp within the frame
- **channel_id**: Sensor channel identifier  
- **flags**: Point quality flags
- **valid**: Boolean indicating if point has valid return

**WITH_POLAR Fields**:
- **azimuth**: Calculated azimuth angle in radians
- **elevation**: Calculated elevation angle in radians

### Sensor Information (cepton_ros::SensorInformation)
- **handle**: Sensor handle
- **serial_number**: Sensor serial number
- **model_name**: Sensor model name
- **model**: Model identifier
- **part_number**: Part number
- **firmware_version**: Firmware version
- **power_up_timestamp**: Power-up timestamp
- **time_sync_offset**: Time synchronization offset
- **time_sync_drift**: Time synchronization drift
- **return_count**: Number of returns
- **channel_count**: Number of channels
- **status_flags**: Status flags
- **fault_summary**: Fault summary
- **fault_entries**: Array of fault entries

### Sensor Status (cepton_ros::CeptonSensorStatus)
- **handle**: Sensor handle
- **serial_number**: Sensor serial number  
- **status**: Status code (e.g., `SENSOR_TIMED_OUT`)

## Coordinate System Transformation

The nodelet automatically converts from Cepton's coordinate system to ROS standard:

## Build Configuration

### CMake Flags
- **`-DWITH_TS_CH_F=ON`**: Enable timestamp, channel, and flag fields
- **`-DWITH_POLAR=ON`**: Enable polar coordinate fields (azimuth, elevation)

### Build Commands
```bash


# Clone the repo
git clone git@github.com:ceptontech/ros.git

# Set up a catkin workspace and symlink the repo
mkdir -p catkin_ws/src
pushd catkin_ws/src
ln -s $(pwd)/../../ros/ros
popd

# Build (standard build)
pushd catkin_ws
catkin_make

# Build with fewer fields
catkin_make -DWITH_POLAR=OFF -DWITH_TS_CH_F=OFF
```

### Run Commands
```bash
# In terminal 1
source devel/setup.bash
roslaunch cepton_ros manager.launch

# In terminal 2
source devel/setup.bash
roslaunch cepton_ros publisher.launch
```


## Monitoring and Status

### Sensor Timeout Detection
- **Timeout Duration**: 3 seconds (`SENSOR_POINTS_TIMEOUT`)
- **Monitoring Thread**: Runs every 1 second to check sensor status
- **Timeout Action**: Publishes `CeptonSensorStatus` message with `SENSOR_TIMED_OUT` status

### Replay Mode Features
- **Watchdog Timer**: Monitors replay completion (100ms interval)
- **Auto-shutdown**: Automatically shuts down when replay is finished
- **Loop Support**: Continuous replay when `capture_loop` is enabled
