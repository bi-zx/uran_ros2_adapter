#!/usr/bin/env python3
"""Deliver current laser measurements to navigation independently of image callbacks."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from uran_ros2_msgs.msg import ContractEnvelope

from uran_ros2_adapter.observation_packets import publish_scan


class LaserObservations(Node):
    def __init__(self):
        super().__init__("laser_observations")
        defaults = {"device_id":"","component_id":"","instance_id":"","source_component":"sensors","body_frame":"base_link","scan_topic":"scan"}
        self.target = {key:self.declare_parameter(key,default).value for key,default in defaults.items()}
        if not self.target["device_id"] or not self.target["component_id"]:
            raise ValueError("provide device_id and component_id")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.out = self.create_publisher(ContractEnvelope,"/uran/runtime/dispatch",1)
        qos = QoSProfile(depth=1,reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan,self.target["scan_topic"],self.on_scan,qos)

    def on_scan(self,msg):
        publish_scan(self,msg,self.target,self.tf,self.out)


if __name__ == "__main__":
    rclpy.init()
    node = LaserObservations()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
