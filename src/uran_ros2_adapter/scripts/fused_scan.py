#!/usr/bin/env python3
"""Feed measured, bounded returns to a planar SLAM system before a map exists."""
from collections import deque
from importlib import import_module
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from uran_ros2_adapter.map_geometry import body_in_odom, cloud_points, fuse_ranges, matrix, odometry_transform, rectified_depth_points, within_survey


class FusedScan(Node):
    def __init__(self):
        super().__init__("fused_scan")
        defaults = {"scan_topic":"scan", "output_topic":"scan_fused", "body_frame":"base_link", "odom_topic":"odom",
                    "odom_body_alias":"", "ground_frame":"base_footprint", "depth_image_topic":"",
                    "depth_camera_info_topic":"", "depth_topic":"", "sensor_adapter":"", "pair_tolerance_s":0.060,
                    "unknown_range":"nan", "max_wait_s":0.45, "min_ground_height_m":0.08, "depth_pixel_stride":2}
        self.cfg = {k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        if self.cfg["unknown_range"] not in ("nan","zero"):
            raise ValueError("unknown_range must be nan or zero")
        if bool(self.cfg["depth_image_topic"]) != bool(self.cfg["depth_camera_info_topic"]):
            raise ValueError("rectified depth requires both image and calibration topics")
        if self.resolve_topic_name(self.cfg["scan_topic"]) == self.resolve_topic_name(self.cfg["output_topic"]):
            raise ValueError("provide distinct input and output scan topics")
        self.tf,self.odom = Buffer(),Buffer()
        self.listener = TransformListener(self.tf,self)
        self.frames,self.info = None,None
        self.odom_stamp = 0
        self.pending,self.layers = deque(maxlen=16),{}
        self.last_warning = {}
        self.pub = self.create_publisher(LaserScan,self.cfg["output_topic"],5)
        self.sources = self.create_publisher(Image,self.cfg["output_topic"]+"/sources",5)
        self.create_subscription(Odometry,self.cfg["odom_topic"],self.on_odom,qos_profile_sensor_data)
        self.create_subscription(LaserScan,self.cfg["scan_topic"],self.on_scan,qos_profile_sensor_data)
        if self.cfg["depth_topic"]:
            self.create_subscription(PointCloud2,self.cfg["depth_topic"],lambda m:self.queue("cloud",m,m.header.stamp),qos_profile_sensor_data)
        if self.cfg["depth_image_topic"]:
            self.create_subscription(CameraInfo,self.cfg["depth_camera_info_topic"],self.on_info,qos_profile_sensor_data)
            self.create_subscription(Image,self.cfg["depth_image_topic"],lambda m:self.queue("image",m,m.header.stamp),qos_profile_sensor_data)
        if self.cfg["sensor_adapter"]:
            module,factory = self.cfg["sensor_adapter"].split(":")
            getattr(import_module(module),factory)(self)
        self.create_timer(0.02,self.flush)

    def warn(self,key,text):
        now = time.monotonic()
        if now-self.last_warning.get(key,-10) > 5:
            self.get_logger().warning(str(text))
            self.last_warning[key] = now

    def on_info(self,msg):
        self.info = msg

    def on_odom(self,msg):
        frames = (msg.header.frame_id,msg.child_frame_id)
        if frames != self.frames:
            self.odom.clear()
            self.frames = frames
        self.odom.set_transform(odometry_transform(msg),"odometry")
        self.odom_stamp = Time.from_msg(msg.header.stamp).nanoseconds

    def on_scan(self,msg):
        if len(msg.ranges) and msg.angle_increment:
            self.pending.append((time.monotonic(),msg))

    def queue(self,name,msg,stamp):
        self.layers.setdefault(name,deque(maxlen=12)).append((Time.from_msg(stamp),msg))

    def pose(self,stamp):
        return body_in_odom(self.odom,self.tf,self.frames,self.cfg["body_frame"],self.cfg["odom_body_alias"],stamp)

    def mount(self,frame,stamp):
        return matrix(self.tf.lookup_transform(self.cfg["body_frame"],frame,stamp).transform)

    def samples(self,name,msg,stamp,laser_in_odom):
        if name == "image":
            if self.info is None:
                return
            points = rectified_depth_points(msg,self.info,self.cfg["depth_pixel_stride"])
            frame = msg.header.frame_id
        elif name == "cloud":
            points,frame = cloud_points(msg,self.cfg["depth_pixel_stride"]),msg.header.frame_id
        else:
            frame,_,points = msg
        pose = self.pose(stamp)
        sensor = pose @ self.mount(frame,stamp)
        world = points @ sensor[:3,:3].T + sensor[:3,3]
        ground = self.cfg["ground_frame"]
        floor_z = 0 if ground == self.frames[0] else (pose @ self.mount(ground,stamp))[2,3]
        use = within_survey(world,pose[:2,3]) & (world[:,2] >= floor_z + self.cfg["min_ground_height_m"])
        laser = (world[use]-laser_in_odom[:3,3]) @ laser_in_odom[:3,:3]
        return (2 if name in ("image","cloud") else 3),laser

    def flush(self):
        if not self.frames or not self.pending:
            return
        received,scan = self.pending[0]
        tolerance = self.cfg["pair_tolerance_s"]
        stamp = Time.from_msg(scan.header.stamp)
        age = time.monotonic()-received
        latest = stamp.nanoseconds+tolerance*1e9
        depth_names = [key for key,topic in (("image","depth_image_topic"),("cloud","depth_topic")) if self.cfg[topic]]
        awaiting_depth = any(not self.layers.get(key) or self.layers[key][-1][0].nanoseconds < latest for key in depth_names)
        if age < tolerance or (age < self.cfg["max_wait_s"] and (self.odom_stamp < latest or awaiting_depth)):
            return
        self.pending.popleft()
        try:
            pose = self.pose(stamp)
            laser = pose @ self.mount(scan.header.frame_id,stamp)
        except TransformException as exc:
            self.warn("scan",exc)
            return
        supplements = []
        duration_ns = abs(scan.time_increment)*(len(scan.ranges)-1)*1e9
        # Bound skew to the first AND last ray of this scan.
        middle = stamp.nanoseconds + duration_ns/2
        for name,frames in self.layers.items():
            t,msg = min(frames,key=lambda row:abs(row[0].nanoseconds-middle))
            if abs(t.nanoseconds-middle) + duration_ns/2 > tolerance*1e9:
                continue
            try:
                layer = self.samples(name,msg,t,laser)
                if layer is not None:
                    supplements.append(layer)
            except (TransformException,ValueError) as exc:
                self.warn(name,exc)
        hits,source = fuse_ranges(scan,laser,pose[:2,3],supplements)
        if self.cfg["unknown_range"] == "zero":
            hits[source==0] = 0.0
        scan.ranges = hits.astype(np.float32).tolist()
        if len(scan.intensities) == len(hits):
            scan.intensities = np.where(source==1,scan.intensities,0).astype(np.float32).tolist()
        self.pub.publish(scan)
        mask = Image()
        mask.header,mask.encoding,mask.height,mask.width = scan.header,"mono8",1,len(source)
        mask.step,mask.data = len(source),source.tobytes()
        self.sources.publish(mask)


if __name__ == "__main__":
    rclpy.init()
    node = FusedScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
