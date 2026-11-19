#!/usr/bin/env python3
"""Publish the SLAM occupancy grid restricted to the history of measured scan rays."""
from collections import deque
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from uran_ros2_adapter.map_geometry import matrix, stamped_pose, within_survey
from uran_ros2_adapter.survey_coverage import SurveyCoverage


class SurveyMap(Node):
    def __init__(self):
        super().__init__("survey_map")
        defaults = {"scan_topic":"scan_fused","grid_topic":"map","output_topic":"survey_map",
                    "body_frame":"base_link","map_frame":"map","pose_topic":"","map_id":"","coverage_file":""}
        self.cfg = {k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        if not self.cfg["map_id"]:
            raise ValueError("provide the SLAM map_id")
        if self.resolve_topic_name(self.cfg["grid_topic"]) == self.resolve_topic_name(self.cfg["output_topic"]):
            raise ValueError("provide distinct input and output grid topics")
        self.coverage = SurveyCoverage(self.cfg["coverage_file"],self.cfg["map_id"],self.cfg["map_frame"])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.poses = Buffer() if self.cfg["pose_topic"] else self.tf
        if self.cfg["pose_topic"]:
            self.create_subscription(PoseStamped,self.cfg["pose_topic"],self.pose,qos_profile_sensor_data)
        self.scans = deque(maxlen=25)
        self.grid,self.last_warning = None,-10.0
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid,self.cfg["output_topic"],qos)
        self.create_subscription(OccupancyGrid,self.cfg["grid_topic"],self.on_grid,qos)
        self.create_subscription(LaserScan,self.cfg["scan_topic"],self.scans.append,qos_profile_sensor_data)
        self.create_timer(0.1,self.observe)
        self.create_timer(1.0,self.draw)

    def pose(self,msg):
        if msg.header.frame_id == self.cfg["map_frame"]:
            self.poses.set_transform(stamped_pose(msg.header,self.cfg["body_frame"],msg.pose),"slam")

    def warn(self,text):
        now = time.monotonic()
        if now-self.last_warning > 5:
            self.get_logger().warning(str(text))
            self.last_warning = now

    def on_grid(self,msg):
        if msg.header.frame_id == self.cfg["map_frame"]:
            self.grid = msg
        else:
            self.warn("Grid frame differs from map_frame: "+msg.header.frame_id)

    def observe(self):
        while self.scans:
            scan = self.scans[0]
            stamp = Time.from_msg(scan.header.stamp)
            try:
                pose = matrix(self.poses.lookup_transform(self.cfg["map_frame"],self.cfg["body_frame"],stamp).transform)
                mount = matrix(self.tf.lookup_transform(self.cfg["body_frame"],scan.header.frame_id,stamp).transform)
            except TransformException as exc:
                if self.get_clock().now().nanoseconds-stamp.nanoseconds < 1000000000:
                    return
                self.warn(exc)
                self.scans.popleft()
                continue
            self.scans.popleft()
            sensor = pose @ mount
            ranges = np.asarray(scan.ranges)
            i = np.flatnonzero(np.isfinite(ranges) & (ranges > 0) & (ranges >= scan.range_min) & (ranges <= scan.range_max))
            angles = scan.angle_min+i*scan.angle_increment
            xy = np.column_stack((ranges[i]*np.cos(angles),ranges[i]*np.sin(angles)))
            ends = xy @ sensor[:2,:2].T + sensor[:2,3]
            ends = ends[within_survey(ends,pose[:2,3])]
            self.coverage.observe(sensor[:2,3],ends,pose[:2,3])

    def draw(self):
        self.coverage.save()
        if self.grid is None:
            return
        grid,self.grid = self.grid,None
        data = np.array(grid.data,dtype=np.int8)
        known = np.flatnonzero(data >= 0)
        if len(known):
            rows,cols = np.divmod(known,grid.info.width)
            local = (np.column_stack((cols,rows))+.5)*grid.info.resolution
            origin = grid.info.origin
            t = TransformStamped().transform
            t.rotation = origin.orientation
            rotation = matrix(t)[:2,:2]
            centers = local @ rotation.T + [origin.position.x,origin.position.y]
            data[known[~self.coverage.permits(centers)]] = -1
        grid.data = data.tolist()
        self.pub.publish(grid)


if __name__ == "__main__":
    rclpy.init()
    node = SurveyMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.coverage.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
