import os
from pathlib import Path
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def test_scans_wait_for_their_imu_and_keep_acquisition_order():
    rclpy.init()
    namespace = "/scan_imu_test_" + str(os.getpid())
    node = rclpy.create_node("observer", namespace=namespace)
    scans = node.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
    imu = node.create_publisher(Imu, "imu", qos_profile_sensor_data)
    received = []
    output = node.create_subscription(LaserScan, "scan_synchronized", received.append, qos_profile_sensor_data)
    program = Path(get_package_prefix("uran_ros2_adapter")) / "lib/uran_ros2_adapter/scan_imu_sync"
    process = subprocess.Popen([str(program), "--ros-args", "-r", "__ns:=" + namespace])

    def wait_for(predicate, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not predicate():
            rclpy.spin_once(node, timeout_sec=.02)
        return predicate()

    def measure(publisher, cls, sec):
        msg = cls()
        msg.header.stamp.sec = sec
        if cls is LaserScan:
            msg.header.frame_id = "laser"
            msg.ranges = [sec / 10.]
        publisher.publish(msg)

    try:
        assert wait_for(lambda: scans.get_subscription_count() and imu.get_subscription_count() and
                        node.count_publishers(namespace + "/scan_synchronized"), 8)
        measure(imu, Imu, 10)
        for sec in (9, 11, 12):
            measure(scans, LaserScan, sec)
        assert not wait_for(lambda: bool(received), .2)
        measure(imu, Imu, 12)
        assert wait_for(lambda: len(received) == 1, 2)
        measure(scans, LaserScan, 11)
        measure(imu, Imu, 13)
        assert wait_for(lambda: len(received) == 2, 2)
        assert [msg.header.stamp.sec for msg in received] == [11, 12]
        assert all(msg.header.frame_id == "laser" for msg in received)
        assert [round(msg.ranges[0], 2) for msg in received] == [1.1, 1.2]
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_subscription(output)
        node.destroy_node()
        rclpy.shutdown()
