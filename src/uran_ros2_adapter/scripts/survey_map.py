#!/usr/bin/env python3
"""Combine planar SLAM occupancy with measured depth and ToF surfaces."""
from collections import deque
from copy import deepcopy
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from uran_ros2_msgs.msg import ContractEnvelope
from tf2_ros import Buffer, TransformException, TransformListener

from uran_ros2_adapter.map_geometry import matrix, stamped_pose
from uran_ros2_adapter.survey_coverage import SurveyCoverage


class SurveyMap(Node):
    def __init__(self):
        super().__init__("survey_map")
        defaults = {"scan_topic":"scan_fused","grid_topic":"map","output_topic":"survey_map",
                    "body_frame":"base_link","map_frame":"map","pose_topic":"","map_id":"","coverage_file":"",
                    "surface_topic":"survey_surfaces","ground_band_m":0.045,"body_height_m":0.45,"surface_confirmation_s":1.0}
        self.cfg = {k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        if not self.cfg["map_id"]:
            raise ValueError("provide the SLAM map_id")
        if self.resolve_topic_name(self.cfg["grid_topic"]) == self.resolve_topic_name(self.cfg["output_topic"]):
            raise ValueError("provide distinct input and output grid topics")
        self.coverage = SurveyCoverage(self.cfg["coverage_file"],self.cfg["map_id"],self.cfg["map_frame"],
                                       surface_confirmation_s=self.cfg["surface_confirmation_s"])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.poses = Buffer() if self.cfg["pose_topic"] else self.tf
        if self.cfg["pose_topic"]:
            self.create_subscription(PoseStamped,self.cfg["pose_topic"],self.pose,qos_profile_sensor_data)
        if self.cfg["surface_topic"]:
            if not 0<self.cfg["ground_band_m"]<self.cfg["body_height_m"]:
                raise ValueError("ground_band_m must be positive and smaller than body_height_m")
            self.create_subscription(ContractEnvelope,self.cfg["surface_topic"],self.on_surface,qos_profile_sensor_data)
        self.scans = deque(maxlen=25)
        self.grid,self.last_warning = None,-10.0
        self.redraw,self.map_stamp = False,0
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
        # The SLAM mapper publishes an empty grid until its first submap is ready.
        if not msg.info.width or not msg.info.height or msg.info.resolution <= 0:
            return
        if msg.header.frame_id == self.cfg["map_frame"]:
            self.grid = msg
            self.redraw = True
            self.map_stamp = max(self.map_stamp,Time.from_msg(msg.header.stamp).nanoseconds)
        else:
            self.warn("Grid frame differs from map_frame: "+msg.header.frame_id)

    def on_surface(self,msg):
        if msg.message_type!="uran.sensors.observation.v1":
            return
        doc = json.loads(msg.payload_json)
        if doc["observation_type"]!="depth_rays" or doc["frame_id"]!=self.cfg["map_frame"]:
            return
        value = doc["value"]
        if value["map_id"]==self.cfg["map_id"]:
            self.map_stamp = max(self.map_stamp,doc["timestamp_ns"])
            self.redraw = True
            self.coverage.observe_surfaces(value["beams"],value["ground_xyz"],self.cfg["ground_band_m"],
                                           self.cfg["body_height_m"],doc["timestamp_ns"])

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
            self.coverage.observe(sensor[:2,3],ends,pose[:2,3],stamp.nanoseconds)
            self.map_stamp = max(self.map_stamp,stamp.nanoseconds)
            self.redraw = True

    def draw(self):
        self.coverage.save()
        if self.grid is None or not self.redraw:
            return
        self.redraw = False
        grid = self.grid
        origin = grid.info.origin
        t = TransformStamped().transform
        t.rotation = origin.orientation
        rotation = matrix(t)[:2,:2]
        grid_xy = np.array([origin.position.x,origin.position.y])
        data = np.array(grid.data,dtype=np.int8)
        known = np.flatnonzero(data >= 0)
        if len(known):
            rows,cols = np.divmod(known,grid.info.width)
            local = (np.column_stack((cols,rows))+.5)*grid.info.resolution
            centers = local @ rotation.T + grid_xy
            data[known[~self.coverage.permits(centers)]] = -1
        raster = data.reshape(grid.info.height,grid.info.width)
        raster,grid_xy = self.coverage.overlay(raster,grid_xy,rotation,grid.info.resolution)
        out = OccupancyGrid()
        out.header,out.info = deepcopy(grid.header),deepcopy(grid.info)
        out.header.stamp = Time(nanoseconds=self.map_stamp).to_msg()
        out.info.height,out.info.width = raster.shape
        out.info.origin.position.x,out.info.origin.position.y = map(float,grid_xy)
        out.data = raster.ravel().tolist()
        self.pub.publish(out)


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
