#!/usr/bin/env python3
"""Feed measured, bounded returns to a planar SLAM system before a map exists."""
from collections import deque
from importlib import import_module
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from uran_ros2_adapter.depth_images import subscribe_depth
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from uran_ros2_adapter.map_geometry import (NearSurfaceHistory, body_in_odom, grounded_pose, stamped_pose, cloud_points, fuse_ranges, matching_depth_views, matrix,
                                           near_lidar_actions, odometry_transform, rectified_depth_model,
                                           rectified_depth_points, rectified_intrinsics, synchronized_scan_points, take_ready_scan, within_survey)


class FusedScan(Node):
    def __init__(self, depth_readers=None):
        super().__init__("fused_scan")
        defaults = {"scan_topic":"scan", "output_topic":"scan_fused", "body_frame":"base_link", "odom_topic":"odom",
                    "odom_body_alias":"", "planar_pose_topic":"", "ground_frame":"base_footprint", "depth_image_topic":"",
                    "depth_camera_info_topic":"", "depth_topic":"", "sensor_adapter":"", "pair_tolerance_s":0.060,
                    "depth_check_range_m":0.0, "depth_disagreement_m":0.25, "require_near_lidar_support":False,
                    "blind_surface_range_m":0.0, "unknown_range":"nan", "max_wait_s":0.45, "min_ground_height_m":0.08, "depth_pixel_stride":2}
        self.cfg = {k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        if self.cfg["unknown_range"] not in ("nan","zero"):
            raise ValueError("unknown_range must be nan or zero")
        if bool(self.cfg["depth_image_topic"]) != bool(self.cfg["depth_camera_info_topic"]):
            raise ValueError("rectified depth requires both image and calibration topics")
        if self.resolve_topic_name(self.cfg["scan_topic"]) == self.resolve_topic_name(self.cfg["output_topic"]):
            raise ValueError("provide distinct input and output scan topics")
        if self.cfg["depth_check_range_m"] < 0 or self.cfg["depth_disagreement_m"] <= 0:
            raise ValueError("provide a nonnegative cross-check range and a positive depth difference")
        if self.cfg["require_near_lidar_support"] and not self.cfg["depth_check_range_m"]:
            raise ValueError("require_near_lidar_support needs a positive depth_check_range_m")
        if self.cfg["depth_check_range_m"] and not self.cfg["depth_image_topic"]:
            raise ValueError("the lidar cross-check requires a calibrated depth image")
        surface_range = self.cfg["blind_surface_range_m"]
        if surface_range < 0 or surface_range > self.cfg["depth_check_range_m"] or (surface_range and not self.cfg["require_near_lidar_support"]):
            raise ValueError("blind_surface_range_m must be within the enabled near lidar support range")
        self.surfaces = NearSurfaceHistory(surface_range) if surface_range else None
        self.tf,self.odom = Buffer(),Buffer()
        self.listener = TransformListener(self.tf,self)
        self.frames,self.info = None,None
        self.odom_stamp = 0
        self.planar = Buffer() if self.cfg["planar_pose_topic"] else None
        self.map_frame,self.planar_stamp = "",0
        if self.planar is not None:
            self.create_subscription(PoseStamped,self.cfg["planar_pose_topic"],self.on_planar_pose,qos_profile_sensor_data)
        self.pending,self.layers = deque(maxlen=16),{}
        self.last_warning = {}
        self.pub = self.create_publisher(LaserScan,self.cfg["output_topic"],5)
        self.sources = self.create_publisher(Image,self.cfg["output_topic"]+"/sources",5)
        self.check = None
        if self.cfg["depth_check_range_m"]:
            self.check = self.create_publisher(Image,self.cfg["output_topic"]+"/lidar_check",5)
        self.create_subscription(Odometry,self.cfg["odom_topic"],self.on_odom,qos_profile_sensor_data)
        self.create_subscription(LaserScan,self.cfg["scan_topic"],self.on_scan,qos_profile_sensor_data)
        if self.cfg["depth_topic"]:
            self.create_subscription(PointCloud2,self.cfg["depth_topic"],lambda m:self.queue("cloud",m,m.header.stamp),qos_profile_sensor_data)
        if self.cfg["depth_image_topic"]:
            subscribe_depth(self,depth_readers,self.on_info,lambda m:self.queue("image",m,m.header.stamp))
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
            if self.surfaces is not None:
                self.surfaces.previous = None
            self.frames = frames
        self.odom.set_transform(odometry_transform(msg),"odometry")
        self.odom_stamp = Time.from_msg(msg.header.stamp).nanoseconds

    def on_scan(self,msg):
        if len(msg.ranges) and msg.angle_increment:
            self.pending.append((time.monotonic(),Time.from_msg(msg.header.stamp).nanoseconds,msg))

    def queue(self,name,msg,stamp):
        self.layers.setdefault(name,deque(maxlen=12)).append((Time.from_msg(stamp),msg))

    def on_planar_pose(self,msg):
        if self.map_frame != msg.header.frame_id:
            self.planar.clear()
            self.map_frame = msg.header.frame_id
        self.planar.set_transform(stamped_pose(msg.header,self.cfg["body_frame"],msg.pose),"slam")
        self.planar_stamp = Time.from_msg(msg.header.stamp).nanoseconds

    def pose(self,stamp):
        body = body_in_odom(self.odom,self.tf,self.frames,self.cfg["body_frame"],self.cfg["odom_body_alias"],stamp)
        if self.planar is None:
            return body
        planar = self.planar.lookup_transform(self.map_frame,self.cfg["body_frame"],stamp)
        return grounded_pose(matrix(planar.transform),body)

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

    def lidar_check(self,scan,laser,pose,stamp,middle,tolerance_ns,supplements,ray_step_ns):
        frames = [(t.nanoseconds,(t,msg)) for t,msg in self.layers.get("image",())]
        views = []
        info = self.info
        mount = self.mount(info.header.frame_id,stamp)
        footprint = ((info.height,info.width),rectified_intrinsics(info),pose @ mount)
        half_duration = abs(ray_step_ns)*(len(scan.ranges)-1)/2
        for ns,(t,msg) in matching_depth_views(frames,middle,tolerance_ns+half_duration):
            try:
                pose = self.pose(t)
                camera = pose @ self.mount(msg.header.frame_id,t)
                views.append((ns,(rectified_depth_model(msg,self.info),camera,pose[:2,3])))
            except (TransformException,ValueError) as exc:
                self.warn("lidar_check",exc)
        tof_layers = [points for kind,points in supplements if kind==3]
        supported = self.surfaces.update(scan,laser,stamp.nanoseconds) if self.surfaces is not None else None
        return near_lidar_actions(scan,laser,middle,views,tof_layers,self.cfg["depth_check_range_m"],
                                  self.cfg["depth_disagreement_m"],tolerance_ns,footprint,
                                  require_support=self.cfg["require_near_lidar_support"],blind_surface=supported,ray_step_ns=ray_step_ns)

    def flush(self):
        if not self.frames or not self.pending:
            return
        tolerance = self.cfg["pair_tolerance_s"]
        pose_ns = min(self.odom_stamp,self.planar_stamp) if self.planar is not None else self.odom_stamp
        depth_ns = pose_ns
        for key,topic in (("image","depth_image_topic"),("cloud","depth_topic")):
            if self.cfg[topic]:
                frames = self.layers.get(key)
                depth_ns = min(depth_ns,frames[-1][0].nanoseconds if frames else 0)
        scan = take_ready_scan(self.pending,time.monotonic(),tolerance,self.cfg["max_wait_s"],pose_ns,depth_ns)
        if scan is None:
            return
        stamp = Time.from_msg(scan.header.stamp)
        try:
            pose = self.pose(stamp)
            laser = pose @ self.mount(scan.header.frame_id,stamp)
        except TransformException as exc:
            self.warn("scan",exc)
            return
        supplements = []
        duration_ns = abs(scan.time_increment)*(len(scan.ranges)-1)*1e9
        ray_step_ns = scan.time_increment*1e9
        middle = stamp.nanoseconds + scan.time_increment*(len(scan.ranges)-1)*.5e9
        for name,frames in self.layers.items():
            t,msg = min(frames,key=lambda row:abs(row[0].nanoseconds-middle))
            time_limit = tolerance*1e9 + duration_ns/2
            if abs(t.nanoseconds-middle) > time_limit:
                continue
            try:
                layer = self.samples(name,msg,t,laser)
                if layer is not None:
                    if ray_step_ns:
                        layer = (layer[0],synchronized_scan_points(scan,layer[1],t.nanoseconds-stamp.nanoseconds,tolerance*1e9))
                    supplements.append(layer)
            except (TransformException,ValueError) as exc:
                self.warn(name,exc)
        action = None
        if self.check is not None:
            if self.info is None:
                return
            try:
                depth_tolerance = tolerance*1e9 if ray_step_ns else tolerance*1e9-duration_ns/2
                action = self.lidar_check(scan,laser,pose,stamp,middle,depth_tolerance,supplements,ray_step_ns)
            except (TransformException,ValueError) as exc:
                self.warn("lidar_check",exc)
                return
        hits,source = fuse_ranges(scan,laser,pose[:2,3],supplements,action)
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
        if self.check is not None:
            mask.data = action.tobytes()
            self.check.publish(mask)


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
