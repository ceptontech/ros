# Cepton ROS Publisher Nodelet - Usage Guide

## Overview
The Cepton ROS Publisher Nodelet is a ROS component that interfaces with Cepton LiDAR sensors to publish point cloud data and sensor information. It supports both live networking and PCAP file replay modes.

## Node Information
- **Namespace**: `cepton_ros`
- **Class**: `PublisherNodelet`
- **Base Class**: `nodelet::Nodelet`

## Parameters

### File Replay Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `capture_path` | string | `""` | Path to PCAP file for replay. If empty, uses live networking mode |
| `capture_loop` | bool | `true` | Whether to loop the PCAP replay continuously |

### Sensor Field of View Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_azimuth` | double | N/A | Minimum azimuth angle in degrees (clamped to -89.9°) |
| `max_azimuth` | double | N/A | Maximum azimuth angle in degrees (clamped to 89.9°) |
| `min_altitude` | double | N/A | Minimum altitude angle in degrees (clamped to -89.9°) |
| `max_altitude` | double | N/A | Maximum altitude angle in degrees (clamped to 89.9°) |

### Distance Filtering Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_distance` | double | N/A | Minimum point distance threshold |
| `max_distance` | double | N/A | Maximum point distance threshold |

### Operating Mode Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `half_frequency_mode` | bool | N/A | Enable half frequency operation mode |
| `aggregation_mode` | int | N/A | Frame aggregation mode for point processing |

### Output Configuration Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `output_by_handle` | bool | N/A | Publish separate topics for each sensor handle |
| `output_by_sn` | bool | N/A | Publish separate topics for each sensor serial number |
| `expected_sensor_ips` | string[] | N/A | List of expected sensor IP addresses for monitoring |

### Point Filtering Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `include_saturated_points` | bool | `true` | Include points marked as saturated |
| `include_second_return_points` | bool | `true` | Include second return points |
| `include_invalid_points` | bool | `false` | Include points with no return (invalid) |
| `include_noise_points` | bool | `false` | Include points marked as noise |
| `include_blocked_points` | bool | `false` | Include points marked as blocked |

## Published Topics

### Main Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `cepton3/points` | `cepton_ros::Cloud` | Main point cloud output (all sensors combined) |
| `cepton3/sensor_information` | `cepton_ros::SensorInformation` | Sensor metadata and status information |
| `cepton3/cepton_sensor_status` | `cepton_ros::CeptonSensorStatus` | Sensor timeout and status monitoring |

### Per-Sensor Topics (Optional)
When `output_by_handle` is enabled:
- `cepton3/cepp_handle_<handle_id>` - Point clouds by sensor handle
- `cepton3/info_handle_<serial_number>` - Sensor info by handle

When `output_by_sn` is enabled:
- `cepton3/cepp_serial_<serial_number>` - Point clouds by serial number

## Point Cloud Message Fields
Each point in the published cloud contains:
- `x, y, z`: 3D coordinates (converted to ROS coordinate system)
- `reflectivity`: Surface reflectivity (scaled by 0.01)
- `relative_timestamp`: Timestamp relative to frame start
- `channel_id`: LiDAR channel identifier
- `flags`: Point status flags
- `azimuth`: Azimuth angle in radians
- `elevation`: Elevation angle in radians  
- `valid`: Boolean indicating if point has valid return

## Usage Examples

### Basic Live Operation
```xml
<node pkg="nodelet" type="nodelet" name="cepton_publisher" 
      args="standalone cepton_ros/PublisherNodelet">
  <param name="max_distance" value="200.0"/>
  <param name="min_distance" value="1.0"/>
  <param name="output_by_handle" value="true"/>
</node>
```

### PCAP File Replay
```xml
<node pkg="nodelet" type="nodelet" name="cepton_publisher" 
      args="standalone cepton_ros/PublisherNodelet">
  <param name="capture_path" value="/path/to/data.pcap"/>
  <param name="capture_loop" value="false"/>
  <param name="max_distance" value="100.0"/>
</node>
```

### Filtered Point Cloud
```xml
<node pkg="nodelet" type="nodelet" name="cepton_publisher" 
      args="standalone cepton_ros/PublisherNodelet">
  <param name="min_azimuth" value="-45.0"/>
  <param name="max_azimuth" value="45.0"/>
  <param name="min_altitude" value="-10.0"/>
  <param name="max_altitude" value="30.0"/>
  <param name="include_noise_points" value="false"/>
  <param name="include_invalid_points" value="false"/>
</node>
```

### Multiple Sensor Setup
```xml
<node pkg="nodelet" type="nodelet" name="cepton_publisher" 
      args="standalone cepton_ros/PublisherNodelet">
  <param name="output_by_sn" value="true"/>
  <param name="output_by_handle" value="true"/>
  <rosparam name="expected_sensor_ips">
    - "192.168.1.100"
    - "192.168.1.101"  
    - "192.168.1.102"
  </rosparam>
</node>
```

## Coordinate System
The nodelet converts from Cepton's coordinate system to ROS standard:
- **Cepton**: X=forward, Y=left, Z=up
- **ROS**: X=left, Y=forward, Z=up

Transformation applied: `(x_ros, y_ros, z_ros) = (y_cepton, -x_cepton, z_cepton)`

## Monitoring and Status
- **Sensor Timeout**: 3 seconds - sensors not sending data trigger timeout status
- **Replay Completion**: Automatically shuts down when PCAP replay finishes (unless looping)
- **Status Thread**: Continuously monitors sensor connectivity and publishes status updates

## Notes
- Azimuth and altitude angles are automatically clamped to ±89.9° to prevent tangent function overflow
- The nodelet uses asynchronous publishing to avoid blocking the main processing thread
- Point filtering is applied based on distance, field of view, and point flags
- Frame ID for all point clouds is set to `"cepton3"`