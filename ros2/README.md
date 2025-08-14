# Cepton ROS Publisher Configuration Guide

This guide covers all configuration options available for the Cepton ROS Publisher based on the implementation in `cepton_publisher.cpp`.

## Basic Configuration Parameters

### Data Source Options

#### `capture_file` (string, default: "")
- **Description**: Path to a PCAP capture file for replay mode
- **Usage**: When specified, the publisher will replay data from the file instead of listening to live sensors
- **Example**: `capture_file: "/path/to/capture.pcap"`

#### `capture_loop` (bool, default: false)
- **Description**: Whether to loop the capture replay continuously
- **Usage**: Only applies when `capture_file` is specified
- **Note**: Sets the `CEPTON_REPLAY_FLAG_PLAY_LOOPED` flag internally

#### `sensor_port` (int, default: 8808)
- **Description**: UDP port for receiving live sensor data
- **Usage**: Only applies when not using capture file replay
- **Note**: Standard Cepton sensor communication port

### Point Cloud Output Configuration

#### `pcl2_output_type` (string, default: "BOTH")
- **Description**: Controls how point cloud topics are published
- **Options**:
  - `"NONE"`: No point cloud publishing
  - `"IP"`: Publish per-handle topics only
  - `"SN"`: Publish per-serial-number topics only  
  - `"BOTH"`: Publish both per-handle and per-serial-number topics
- **Topics Created**:
  - Main topic: `cepton_pcl2`
  - Per-handle: `handle_<HANDLE>`
  - Per-serial: `serial_<SERIAL_NUMBER>`

## Point Filtering Options

### Spatial Filtering

#### `min_altitude` (double, default: -90.0)
- **Description**: Minimum elevation angle in degrees
- **Range**: Internally clamped to [-89.9°, 89.9°]
- **Usage**: Points below this elevation will be filtered out

#### `max_altitude` (double, default: 90.0)
- **Description**: Maximum elevation angle in degrees
- **Range**: Internally clamped to [-89.9°, 89.9°]
- **Usage**: Points above this elevation will be filtered out

#### `min_azimuth` (double, default: -90.0)
- **Description**: Minimum azimuth angle in degrees
- **Range**: Internally clamped to [-89.9°, 89.9°]
- **Usage**: Points to the left of this azimuth will be filtered out

#### `max_azimuth` (double, default: 90.0)
- **Description**: Maximum azimuth angle in degrees
- **Range**: Internally clamped to [-89.9°, 89.9°]
- **Usage**: Points to the right of this azimuth will be filtered out

#### `min_distance` (double, default: 0.0)
- **Description**: Minimum distance threshold in meters
- **Usage**: Points closer than this distance will be filtered out

#### `max_distance` (double, default: numeric_limits<float>::max())
- **Description**: Maximum distance threshold in meters
- **Usage**: Points farther than this distance will be filtered out
- **Note**: Points beyond 500m are automatically filtered as invalid

### Network Configuration

#### `expected_sensor_ips` (vector<string>, default: empty)
- **Description**: List of expected sensor IP addresses for monitoring
- **Usage**: Enables proactive timeout detection for expected sensors
- **Example**:
```yaml
expected_sensor_ips:
  - "192.168.1.10"
  - "192.168.1.11"
```

## Published Topics

### Standard Topics
- **`cepton_pcl2`**: Main point cloud topic (sensor_msgs::PointCloud2)
- **`cepton_info`**: Unified sensor information topic
- **`cepton_sensor_status`**: Sensor status monitoring topic

### Conditional Topics
- **`handle_<HANDLE>`**: Per-handle point clouds (when `pcl2_output_type` includes "IP")
- **`serial_<SERIAL_NUMBER>`**: Per-serial-number point clouds (when `pcl2_output_type` includes "SN")
- **`info_handle_<HANDLE>`**: Per-sensor information topics

## Point Cloud Fields

### Standard Fields (always included)
- **x, y, z** (float32): 3D coordinates in ROS coordinate system
- **intensity** (float32): Reflectivity value (scaled by 0.01)

### Optional Fields (compile-time flags)

#### WITH_TS_CH_F Fields
- **timestamp_s** (int32): Timestamp seconds portion
- **timestamp_us** (int32): Timestamp microseconds portion  
- **flags** (uint16): Point quality flags
- **channel_id** (uint16): Sensor channel identifier

#### WITH_POLAR Fields
- **azimuth** (float32): Azimuth angle in radians
- **elevation** (float32): Elevation angle in radians
- **range** (float32): Distance measurement (0 for no-return points)

## Coordinate System Transformation

The publisher automatically converts from Cepton's coordinate system to ROS standard:

## Message Types

### CeptonSensorInfo
Contains comprehensive sensor information:
- `serial_number`, `handle`, `model_name`, `model`
- `part_number`, `firmware_version`
- `power_up_timestamp`, `time_sync_offset`, `time_sync_drift`
- `return_count`, `channel_count`, `status_flags`
- `temperature`, `fault_summary`, `fault_entries`

### CeptonSensorStatus
Monitoring message with:
- `serial_number`, `handle`
- `status` (e.g., `SENSOR_TIMED_OUT`)

## Build Configuration

### CMake Flags
- **`-DWITH_POLAR=OFF`**: Disable polar coordinate fields
- **`-DWITH_TS_CH_F=OFF`**: Disable timestamp, channel, and flag fields

### Build Commands
```bash
# Standard build with all fields
colcon build --packages-select cepton_messages cepton_subscriber cepton_publisher

# Build with filtered fields
colcon build --packages-select cepton_messages cepton_subscriber cepton_publisher --cmake-args -DWITH_POLAR=OFF -DWITH_TS_CH_F=OFF
```

## Monitoring and Status

### Sensor Timeout Detection
- **Timeout Duration**: 3 seconds (configurable via `SENSOR_POINTS_TIMEOUT`)
- **Monitoring Thread**: Runs every 1 second to check sensor status
- **Timeout Action**: Publishes `CeptonSensorStatus` message with `SENSOR_TIMED_OUT` status

### Thread Safety
- Uses mutex locks for sensor status updates
- Asynchronous publishing to prevent blocking data processing
- Future-based publishing coordination
 
## Performance Considerations

### Memory Management
- Static buffer allocation per sensor handle
- Point cloud resizing based on filtered point count
- Asynchronous publishing to prevent data loss

## Troubleshooting

### Common Issues
- **No point clouds**: Check `pcl2_output_type` setting
- **Sensor timeout**: Verify network connectivity and `expected_sensor_ips`
- **Missing fields**: Check compile-time flags (`WITH_POLAR`, `WITH_TS_CH_F`)
- **Coordinate issues**: Verify ROS coordinate system transformation

### Debug Information
- Enable debug logging to see parameter values
- Check sensor information topics for connectivity status
- Monitor sensor status topic for timeout notifications