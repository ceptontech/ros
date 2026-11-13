# CeptonSdk2 ROS2 Driver Release Notes

## Version 2.3.0

## Version 2.2.0 2026-10-29
- Changed default point filter parameters.
  - `include_invalid_points` and `include_noise_points` now default to `true`.
  - `include_ambient_points` and `include_second_return_points` now default to `false`.
  - `expected_sensor_ips` is commented out in `parameters.yaml` by default.
- Added `include_retro_weak_points` parameter (default `true`).
- Removed the 500 m distance filter.
- Updated CeptonSDK to 3.0.24.3.
  - `channel_count` in `CeptonSensorInfo.msg` changed from `uint8` to `uint16`.
- The publisher/subscriber version string is now read from `package.xml`.
- Added build options for PointCloud2 fields.
  - `WITH_POLAR` and `WITH_TS_CH_F` are now applied to the ROS2 publisher.
  - `WITH_TS_CH_F` is split into `WITH_TS`, `WITH_CH`, and `WITH_F`. `WITH_TS_CH_F` is kept as a legacy switch and takes precedence when specified.
  - Added `RELATIVE` option (default `OFF`) that adds a `relative_time_us` field: time relative to the PointCloud2 header timestamp.
  - Added `WITH_PTP` option (default `ON`) that applies the sensor time sync offset to timestamps.
- Added `fastdds.xml` profile for unstable frame rates.

## Version 2.1.3 2023-11-3
- Now allow enabling/disabling of the point cloud topics.
  - Specifically, one can now choose whether to output a combination of PCL2/CEPP/both messages, as well as whether to output via IP/Serial Number/Both for each of the message types.
- Added fault summary field to the info messages.
- New topic for PanicMessage.
- Fixed minor performance issue that copied PCL buffer twice.

## Version 2.1.2.4 2023-7-19
- Fixed bug that would drift the timestamp from PTP timestamp when PTP was linked to the sensors
- Revised Frame Behavior to catch up in timestamp after a new frame is started, instead of compensating in the middle of the frame.

## Version 2.1.2.3 2023-7-11
- Removed Old Logic on time-debt for timestamping.

## Version 2.1.2.2 2023-5-12
- Add support for topics by sensor handle.
- Fixed Default Configs to a more suitable Default.

## Version 2.1.2.1 2023-5-9
- Excluded BLOOM points (deprecated flag).
- Add Reflectivity Look Up Table for ROS2 point reflectivities.

## Version 2.1.2.0 2023-4-25
- Revised READMEs to be updated to the new configs
- Addition of Azimuth/Altitude cuts on FOV via configuration.

## Version 2.0.22.0 2023-4-11
- Added configs to exclude/include points via each valid flag.
- Added feature to output the frames in 2s (effectively halving the output frequency but doubling the output each time).