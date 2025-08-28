# CeptonSdk2 ROS2 Driver Release Notes

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