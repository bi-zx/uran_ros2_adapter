from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start the generic lifecycle Runtime host with an empty assembly."""
    share = Path(get_package_share_directory("uran_bringup"))

    return LaunchDescription([
        Node(
            package="uran_ros2_adapter",
            executable="uran_runtime_node",
            name="uran_runtime",
            output="screen",
            parameters=[{
                "plugin_config": str(share / "config" / "plugins.yaml"),
                "snapshot_period_ms": 1000,
                "heartbeat_period_ms": 5000,
            }],
        ),
    ])
