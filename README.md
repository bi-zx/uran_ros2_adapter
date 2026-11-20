# URAN ROS 2 adapter

This repository contains the ROS 2 boundary for URAN: the lifecycle runtime
host, ROS 2 platform services, pluginlib/native loader, typed contract bridge,
legacy topic/service conversions, and a generic bringup package. Business
workers remain in independently packaged repositories and are loaded through
the installed framework and contract packages.

Build it from an installed framework and interface prefix:

```bash
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/opt/uran/framework:/opt/uran/interfaces:$AMENT_PREFIX_PATH
colcon build --base-paths src --merge-install
colcon test --base-paths src --merge-install
```

The adapter links `uran_core_framework`, `uran_platform_native`,
`uran_plugin_api`, and `uran_interfaces` as installed packages. The installed
library uses `$ORIGIN` and the runtime executable uses `$ORIGIN/..`; no build
machine prefix is written into the installed RPATH. ROS 1 and PX4 are outside
this repository and `uran_profiles` is the deferred deployment repository.

`config/test_runtime.yaml` and `examples/uran_ros2_test_plugin` provide a
hardware-free pluginlib/contract smoke test. The generic bringup assembly is
intentionally empty; a deployment profile supplies manifests and component
instances.

The latched `/uran/runtime/state_snapshot` String topic carries the component
aware Runtime snapshot. Its `components` array contains every loaded
`component_id`, groups multiple `instance_id` values under the same component,
and retains components that have no state fields. The legacy
`/uran/core/state/broadcast` message remains flat for old consumers.


The dispatch topic also accepts `uran.sensors.observation.v1` using
`sensors/observation.schema.json`. The document names the sensor source. The ROS
envelope selects the destination worker and supplies a unique correlation ID.
The destination must register the observation message type.

`ros2 run uran_ros2_adapter map_observations` publishes mapped poses, base-frame
scans and depth rays from standard ROS topics. Its ROS parameters select frames,
topics and the destination. An external SLAM system supplies a map ID and TF.
For a SLAM system that publishes a stamped body pose, set `pose_topic` to pair it with local odometry. When the upstream estimator already integrates odometry, set `pose_includes_odometry:=true` to forward each estimate at its acquisition stamp. Surface projection aligns the two pose histories at their latest shared time.
Products that manage SLAM sessions or use vendor measurements can supply a
`slam_adapter` or `sensor_adapter` Python factory as `module:callable`.

A rectified depth camera can supply `depth_image_topic` and
`depth_camera_info_topic`. The bridge projects `16UC1` millimetre or `32FC1`
metre depth images into the calibrated optical frame. The image and camera
info must use the same optical frame.

Use `odom_body_alias` when the odometry producer gives the physical body a
different child frame name. Use a calibrated TF for offset odometry sensors.
For a level floor coinciding with odometry Z=0, setting `ground_frame` to the
odometry parent projects the robot position onto that plane. Use a measured
local ground frame on terrain with changing elevation.

For a planar laser map, set `use_odometry_ground_plane:=true` in `map_observations` when the odometry source supplies body height above the support plane and gravity-aligned roll and pitch. Set `pose_topic` to the measured planar body pose. In `fused_scan`, select that same pose with `planar_pose_topic` and set `ground_frame` to the support odometry parent. At each exposure both nodes take horizontal position and heading from laser SLAM, and height and tilt from the support odometry.

Set `depth_rate_hz` to the rate the onboard terrain consumer can process.
The bridge uses the latest synchronized depth for each publication. Zero uses
the input rate. The CyberDog2 board was tested at 2 Hz for depth while its
laser and pose streams ran independently.

### Bounded mapping

On devices where both mapping nodes use the same rectified depth camera, `ros2 run uran_ros2_adapter survey_sensors` runs `fused_scan` and `map_observations` together. The nodes share one subscription per resolved depth image and camera-info topic. Configure each node by name with standard ROS parameters and remaps. Set the task observation node’s `scan_topic` to the low-latency physical scan for motion control. The standalone executables use the same sensor-processing code.

Run `ros2 run uran_ros2_adapter fused_scan` before starting SLAM. The node accepts an odometry topic, calibrated rectified depth or an XYZ cloud, and optionally a ToF adapter. It publishes `scan_fused` in the original laser frame. Finite laser hits normally take priority. Depth and ToF can fill a missing beam with two nearby measured returns in a 5 cm slice centered on the scan plane. The body pose at each acquisition compensates sensor motion. The window defaults to 60 ms and the ground clearance for supplemental points to 8 cm. Set the floor frame to the local ground reference. Topics and mount frames are device parameters.

The horizontal observation radius is 5 m about the body at acquisition. A distant hit supplies no map update. The per-beam `scan_fused/sources` mono8 image has the scan timestamp and encodes unknown (0), lidar (1), depth (2) and ToF (3). Standard output uses NaN for unknown. Set `unknown_range:=zero` for a scanner consumer that expects zero below its minimum range. Feed this buffered stream to `survey_map` for observed coverage. A laser-driven pose estimator and low-latency motion controller can subscribe directly to the physical scan.

The survey grid marks isolated planar returns as unknown until repeated endpoints support them. Multi-frame depth and ToF surface evidence is overlaid after this filter, including narrow obstacles and drops. Local obstacle avoidance reads the live scan.

A device with false close returns can opt into `depth_check_range_m` (metres from the scanner, default 0). Within the calibrated camera overlap, the mapping scan holds unverified near hits as unknown. Two distinct depth exposures must both see beyond the projected lidar point by `depth_disagreement_m` before a measured scan-plane substitute is used. At least 69 of 81 pixels in each patch must provide in-range depth. A foreground pixel or a matching scan-plane ToF point preserves the nearer return. The current exposure must meet the scan pairing tolerance. `scan_fused/lidar_check` is a per-beam mono8 diagnostic. Values are 0 for usual processing, 1 for pending or unknown, and 2 for repeated disagreement. A beam with repeated disagreement can still have source 0 if no replacement was measured. The camera overlap and all projection distances use calibrated extrinsics and acquisition-time odometry.

For a scanner with unreliable near-field distances, set `require_near_lidar_support:=true` and a positive `depth_check_range_m`. This applies the check to every close lidar beam, including camera blind spots. A depth measurement at the lidar distance or a matching scan-plane ToF return can retain the hit. An unconfirmed beam has check value 3 and stays unknown unless depth or ToF measures a supported return at the same or a shorter range. A farther replacement requires two depth exposures with clear evidence beyond the reported hit. Select this setting per device.

`blind_surface_range_m` (default 0) permits short lidar surfaces outside the calibrated image when near-hit support is required. In this range the node compares adjacent scans in odometry. It retains measured points on line segments at least 8 cm long when the ranges repeat and fit a line better than a scanner-centred ring. Unobserved gaps stay unknown. Check value 4 identifies lidar accepted this way. In the near-hit support mode, the acquisition time of each laser ray determines whether a depth exposure can verify or replace that ray. Select the radius using close targets and empty-scene recordings for the device.

For mappers that integrate IMU samples when each laser scan arrives, `scan_imu_sync` buffers `scan` until the `imu` stream passes its acquisition time. It publishes scans in order on `scan_synchronized` with their original headers and ranges. Use that topic for the mapper. A delayed IMU can delay the map pose. The obstacle monitor can subscribe directly to `scan`.

Use `ros2 run uran_ros2_adapter survey_map` to publish the occupancy grid used by RViz and map export. Set `map_id`, `map_frame`, `grid_topic` (the SLAM raster), `scan_topic` (the fused scan), and `coverage_file` (a writable SQLite path per map). By default the node uses the timestamped map-to-body TF. Set `pose_topic` to use a stamped body pose in the same map frame. The output `survey_map` retains the SLAM occupancy values for cells supported by measured rays from at least one in-range viewpoint. Unobserved cells carry -1. Run `map_observations` with the same map ID and frame to add calibrated depth and ToF surfaces. Both nodes use the relative `surface_topic` named `survey_surfaces` by default. Set the same absolute topic on both nodes if they run in different namespaces. Each metric sensor exposure contributes endpoints at its acquisition pose. The map accepts observations up to `surface_max_age_s` (default 2 s) when pose history is available. Control observations retain the 0.3 s freshness limit.

The occupancy grid extends to measured floor outside the planar raster. Two distinct acquisitions must support a depth cell. Nearby floor samples establish the local ground tilt. `ground_band_m` (default 0.045 m) is the height tolerance around this plane. A measured drop or a raised return within `body_height_m` (default 0.45 m) can mark an obstacle. Existing occupied laser cells keep priority, and gaps and cells beyond the 5 m observation radius stay unknown. Traversability also requires the task's ground continuity and body clearance checks.

Reopening the same map and coverage file retains the survey as the robot moves. Save the published layer with the standard ROS map saver. For patrol use the saved survey and load the vendor pose graph for localization.
