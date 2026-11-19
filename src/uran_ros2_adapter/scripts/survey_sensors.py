#!/usr/bin/env python3
"""Run scan fusion and surface observations with one subscription per depth stream."""
import rclpy
from rclpy.executors import SingleThreadedExecutor
from uran_ros2_adapter.fused_scan import FusedScan
from uran_ros2_adapter.map_observations import MapObservations


def main():
    rclpy.init()
    readers = {}
    executor = SingleThreadedExecutor()
    fusion = FusedScan(readers)
    surfaces = MapObservations(readers)
    executor.add_node(fusion)
    executor.add_node(surfaces)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        fusion.destroy_node()
        surfaces.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
