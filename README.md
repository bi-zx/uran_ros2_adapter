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
