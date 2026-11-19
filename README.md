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

Set `depth_rate_hz` to the rate the onboard terrain consumer can process.
The bridge uses the latest synchronized depth for each publication. Zero uses
the input rate. The CyberDog2 board was tested at 2 Hz for depth while its
laser and pose streams ran independently.
