#!/usr/bin/env python3
"""Translate standard ROS measurements into URAN map observations."""
import json
from importlib import import_module
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from uran_ros2_adapter.map_geometry import rigid, matrix, stamped_pose, odometry_transform, body_in_odom, within_survey, cloud_points, rectified_depth_points
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, NavSatFix
from tf2_ros import Buffer, TransformException, TransformListener
from uran_ros2_msgs.msg import ContractEnvelope, OperationEvent


class MapObservations(Node):
    def __init__(self):
        super().__init__("map_observations")
        defaults = {"map_id":"","map_frame":"map","body_frame":"base_link","ground_frame":"base_footprint",
                    "device_id":"","component_id":"","instance_id":"","source_component":"sensors",
                    "reference_mode":"local","odom_topic":"odom","odom_body_alias":"","slam_timeout_s":2.0, "pose_topic":"",
                    "scan_topic":"scan","depth_topic":"","depth_origin_frame":"","gps_topic":"",
                    "depth_image_topic":"","depth_camera_info_topic":"","depth_rate_hz":0.0,
                    "surface_topic":"survey_surfaces","surface_max_age_s":2.0,"sensor_adapter":"","slam_adapter":""}
        self.cfg = {k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        if not self.cfg["device_id"] or not self.cfg["component_id"]:
            raise ValueError("provide the target device_id and component_id")
        if not (self.cfg["slam_adapter"] or self.cfg["map_id"]):
            raise ValueError("provide a SLAM session adapter or an externally managed map_id")
        if self.cfg["reference_mode"] not in ("local", "geographic"):
            raise ValueError("reference_mode must be local or geographic")
        if bool(self.cfg["depth_image_topic"]) != bool(self.cfg["depth_camera_info_topic"]):
            raise ValueError("rectified depth requires both image and camera info topics")
        if not any(self.cfg[k] for k in ("sensor_adapter","depth_topic","depth_image_topic")):
            raise ValueError("provide a depth cloud, rectified depth image or a sensor adapter")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.map_poses = self.tf
        if self.cfg["pose_topic"]:
            self.map_poses = Buffer()
            self.create_subscription(PoseStamped,self.cfg["pose_topic"],self.on_map_pose,qos_profile_sensor_data)
        self.odom_history = Buffer()
        self.odom_frames = None
        self.odom_stamp = self.anchor_stamp = self.revision = 0
        self.anchor = None
        self.out = self.create_publisher(ContractEnvelope,"/uran/runtime/dispatch",30)
        self.surface_out = self.create_publisher(ContractEnvelope,self.cfg["surface_topic"],qos_profile_sensor_data) if self.cfg["surface_topic"] else None
        self.pending = {}
        self.layers = {}
        self.last_warning = {}
        self.last_pose = self.last_depth = 0
        self.next_depth = 0.0
        self.slam = None
        if self.cfg["slam_adapter"]:
            module,factory = self.cfg["slam_adapter"].split(":")
            self.slam = getattr(import_module(module),factory)(self,self.report_slam)
            self.create_subscription(OperationEvent,"/uran/runtime/events",self.on_event,30)
        if self.cfg["odom_topic"]:
            self.create_subscription(Odometry,self.cfg["odom_topic"],self.on_odom,qos_profile_sensor_data)
        elif self.cfg["reference_mode"] == "local" and not self.slam:
            raise ValueError("a local map requires an odometry topic")
        self.create_subscription(LaserScan,self.cfg["scan_topic"],lambda m:self.queue("scan",m,m.header.stamp),qos_profile_sensor_data)
        if self.cfg["depth_topic"]:
            self.create_subscription(PointCloud2,self.cfg["depth_topic"],lambda m:self.queue("depth",m,m.header.stamp),qos_profile_sensor_data)
        if self.cfg["depth_image_topic"]:
            self.camera_info = None
            self.create_subscription(CameraInfo,self.cfg["depth_camera_info_topic"],self.on_camera_info,qos_profile_sensor_data)
            self.create_subscription(Image,self.cfg["depth_image_topic"],lambda m:self.queue("depth_image",m,m.header.stamp),qos_profile_sensor_data)
        if self.cfg["sensor_adapter"]:
            module,factory = self.cfg["sensor_adapter"].split(":")
            getattr(import_module(module),factory)(self)
        if self.cfg["gps_topic"]:
            self.create_subscription(NavSatFix,self.cfg["gps_topic"],self.on_gps,qos_profile_sensor_data)
        self.create_timer(0.05,self.flush)

    def on_event(self,msg):
        if (msg.device_id != self.cfg["device_id"] or msg.component_id != self.cfg["component_id"]
                or msg.instance_id and msg.instance_id != self.cfg["instance_id"] or not msg.result_json):
            return
        data = json.loads(msg.result_json)
        request = data.get("slam") if isinstance(data,dict) else None
        if request and request.get("operation_id") == msg.operation_id:
            self.slam.request(msg.operation_id,data["map_id"],request["mode"],request["command"])
            if self.slam.operation == msg.operation_id:
                self.cfg["reference_mode"] = data["reference_mode"]

    def report_slam(self,operation,map_id,state,reason):
        if state == "ready":
            self.cfg["map_id"] = map_id
        self.send("slam_session_state",self.get_clock().now().nanoseconds,self.cfg["map_frame"],
                  {"map_id":map_id,"operation_id":operation,"state":state,"reason":reason})

    def queue(self,name,msg,stamp):
        if self.slam and self.slam.stage != "ready":
            return
        self.pending.setdefault(name,deque(maxlen=8)).append((Time.from_msg(stamp),msg))

    def on_camera_info(self,msg):
        self.camera_info = msg

    def on_gps(self,msg):
        if msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.geographic_fix(Time.from_msg(msg.header.stamp).nanoseconds,msg.latitude,msg.longitude,{"horizontal_fix_valid":True})

    def geographic_fix(self,stamp,lat,lon,quality):
        if self.cfg["reference_mode"] == "geographic" and stamp and (not self.slam or self.slam.stage == "ready"):
            self.send("gps_fix",stamp,"wgs84",dict(latitude_deg=lat,longitude_deg=lon,**quality))

    def on_map_pose(self,msg):
        if msg.header.frame_id == self.cfg["map_frame"] and (not self.slam or self.slam.stage == "ready"):
            self.map_poses.set_transform(stamped_pose(msg.header,self.cfg["body_frame"],msg.pose),"slam")

    def on_odom(self,msg):
        if self.slam and self.slam.stage != "ready":
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        frames = (msg.header.frame_id,msg.child_frame_id)
        if self.odom_frames != frames:
            self.odom_history.clear()
            self.anchor,self.anchor_stamp,self.odom_stamp = None,0,0
        if stamp <= self.odom_stamp:
            return
        t = odometry_transform(msg)
        self.odom_history.set_transform(t,"odometry")
        self.odom_frames,self.odom_stamp = frames,stamp

    def body_in_odom(self,stamp):
        return body_in_odom(self.odom_history,self.tf,self.odom_frames,self.cfg["body_frame"],self.cfg["odom_body_alias"],stamp)

    def odometry_corrected_pose(self):
        if not self.odom_stamp:
            return None
        frame = self.cfg["map_frame"]
        odom = self.odom_frames[0]
        try:
            correction = None if self.cfg["pose_topic"] else self.tf.lookup_transform(frame,odom,Time())
        except TransformException:
            correction = None
        if correction is not None and Time.from_msg(correction.header.stamp).nanoseconds:
            stamp = Time.from_msg(correction.header.stamp)
            candidate = matrix(correction.transform)
        else:
            correction = self.map_poses.lookup_transform(frame,self.cfg["body_frame"],Time())
            stamp = Time.from_msg(correction.header.stamp)
            candidate = None
        if stamp.nanoseconds > self.anchor_stamp:
            if candidate is None:
                try:
                    candidate = matrix(correction.transform) @ np.linalg.inv(self.body_in_odom(stamp))
                except TransformException:
                    # The next odometry sample may still bracket this laser time.
                    pass
            if candidate is not None:
                if self.anchor is not None:
                    old = self.anchor @ self.body_in_odom(Time(nanoseconds=self.odom_stamp))
                    new = candidate @ self.body_in_odom(Time(nanoseconds=self.odom_stamp))
                    yaw_delta = math.atan2(new[1,0],new[0,0])-math.atan2(old[1,0],old[0,0])
                    if np.linalg.norm(new[:2,3]-old[:2,3]) > 0.25 or abs(math.atan2(math.sin(yaw_delta),math.cos(yaw_delta))) > math.radians(10):
                        self.revision += 1
                self.anchor,self.anchor_stamp = candidate,stamp.nanoseconds
        now = self.get_clock().now().nanoseconds
        if self.anchor is None or abs(now-self.anchor_stamp) > self.cfg["slam_timeout_s"]*1e9 or abs(now-self.odom_stamp)>0.5e9:
            return None
        pose = self.anchor @ self.body_in_odom(Time(nanoseconds=self.odom_stamp))
        return self.odom_stamp,pose[:3,:3],pose[:3,3]

    def in_map(self,source,stamp):
        if not self.cfg["odom_topic"]:
            if self.cfg["pose_topic"]:
                pose = matrix(self.map_poses.lookup_transform(self.cfg["map_frame"],self.cfg["body_frame"],stamp).transform)
                pose = pose @ matrix(self.tf.lookup_transform(self.cfg["body_frame"],source,stamp).transform)
                return pose[:3,:3],pose[:3,3]
            return rigid(self.tf.lookup_transform(self.cfg["map_frame"],source,stamp).transform)
        assert self.anchor is not None
        mount = self.tf.lookup_transform(self.cfg["body_frame"],source,stamp)
        pose = self.anchor @ self.body_in_odom(stamp) @ matrix(mount.transform)
        return pose[:3,:3],pose[:3,3]

    def send(self,kind,stamp,frame,value,publisher=None):
        target = self.cfg
        packet = ContractEnvelope()
        packet.message_type = "uran.sensors.observation.v1"
        packet.schema_name = "sensors/observation.schema.json"
        packet.device_id,packet.component_id,packet.instance_id = target["device_id"],target["component_id"],target["instance_id"]
        packet.correlation_id = "{}-{}-{}-{}-{}".format(packet.device_id, target["source_component"], packet.instance_id or packet.component_id, kind, stamp)
        packet.payload_json = json.dumps({"schema_version":"1.0","source":{"device_id":packet.device_id,"component_id":target["source_component"]},
            "frame_id":frame,"timestamp_ns":stamp,"observation_type":kind,"value":value},allow_nan=False,separators=(",",":"))
        (self.out if publisher is None else publisher).publish(packet)

    def warn(self,key,message):
        now = time.monotonic()
        if now-self.last_warning.get(key,-100) > 5:
            self.get_logger().warning(message)
            self.last_warning[key] = now

    def scan(self,msg,stamp):
        rot,offset = rigid(self.tf.lookup_transform(self.cfg["body_frame"],msg.header.frame_id,stamp).transform)
        ranges = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
        count = round(2*math.pi/abs(msg.angle_increment))
        increment = 2*math.pi/count
        result = [None]*count
        unit = np.column_stack((np.cos(angles),np.sin(angles),np.zeros(len(angles)))) @ rot.T
        for vector,r in zip(unit,ranges):
            if np.isposinf(r):
                # Only the direction is observable for an unbounded ray.
                x,y = vector[:2]
                distance = "inf"
            elif math.isfinite(r) and msg.range_min <= r <= msg.range_max and r > 0:
                x,y = (r*vector + offset)[:2]
                distance = math.hypot(x,y)
            else:
                continue
            i = round((math.atan2(y,x)+math.pi)/increment) % count
            if not isinstance(result[i],float) or (distance != "inf" and distance < result[i]):
                result[i] = distance
        self.send("laser_scan",stamp.nanoseconds,self.cfg["body_frame"],{"angle_min":-math.pi,"angle_max":math.pi-increment,
            "angle_increment":increment,"range_min":msg.range_min,"range_max":msg.range_max+float(np.linalg.norm(offset)),"ranges":result})

    def depth(self,name,msg,stamp,task_fresh,publish_surface):
        if name in ("depth","depth_image"):
            if name == "depth":
                local_points = cloud_points(msg)
            else:
                if self.camera_info is None:
                    return
                try:
                    local_points = rectified_depth_points(msg,self.camera_info)
                except ValueError as exc:
                    self.warn(name,str(exc))
                    return
            rot,offset = self.in_map(msg.header.frame_id,stamp)
            origin_frame = self.cfg["depth_origin_frame"] or msg.header.frame_id
            _,origin = self.in_map(origin_frame,stamp)
            points = local_points @ rot.T + offset
        else:
            source_frame,origin,points = msg
            rot,offset = self.in_map(source_frame,stamp)
            origin = origin @ rot.T + offset
            points = points @ rot.T + offset
        _,body = self.in_map(self.cfg["body_frame"],stamp)
        points = points[within_survey(points,body[:2])]
        if len(points):
            if self.odom_frames and self.cfg["ground_frame"] == self.odom_frames[0]:
                # With a level floor at odometry Z=0, project this body position onto that plane.
                ground_odom = self.body_in_odom(stamp)[:3,3].copy()
                ground_odom[2] = 0.0
                ground = self.anchor[:3,:3] @ ground_odom + self.anchor[:3,3]
            else:
                _,ground = self.in_map(self.cfg["ground_frame"],stamp)
            ground = ground.tolist()
            group = {"origin":origin.tolist(),"body_xy":body[:2].tolist(),"points":points.tolist()}
            if publish_surface:
                self.send("depth_rays",stamp.nanoseconds,self.cfg["map_frame"],
                          {"map_id":self.cfg["map_id"],"ground_xyz":ground,"beams":[group]},self.surface_out)
            if task_fresh:
                self.layers[name] = (stamp.nanoseconds,ground,group)

    def flush(self):
        if self.slam:
            self.slam.tick()
            if self.slam.stage != "ready":
                self.pending.clear()
                self.layers.clear()
                self.last_pose = self.last_depth = self.odom_stamp = self.anchor_stamp = 0
                self.anchor = self.odom_frames = None
                if self.cfg["pose_topic"]:
                    self.map_poses.clear()
                return
        frame = self.cfg["map_frame"]
        try:
            if self.cfg["odom_topic"]:
                result = self.odometry_corrected_pose()
                if result is None:
                    return
                stamp,rot,xy = result
            else:
                tf = self.map_poses.lookup_transform(frame,self.cfg["body_frame"],Time())
                stamp = Time.from_msg(tf.header.stamp).nanoseconds
                rot,xy = rigid(tf.transform)
            if stamp > self.last_pose:
                self.send("map_pose",stamp,frame,{"map_id":self.cfg["map_id"],"x":xy[0],"y":xy[1],
                         "yaw_deg":math.degrees(math.atan2(rot[1,0],rot[0,0])),"localization_revision":self.revision,
                         "odometry_fused":bool(self.cfg["odom_topic"])})
                self.last_pose = stamp
        except TransformException as exc:
            self.warn("pose",str(exc))
            if self.cfg["odom_topic"]:
                return
        now = self.get_clock().now().nanoseconds
        collect_depth = time.monotonic() >= self.next_depth
        publish_surface = self.surface_out is not None and self.surface_out.get_subscription_count()>0
        depth_processed = False
        for name,pending in self.pending.items():
            if name != "scan" and not collect_depth:
                continue
            for stamp,msg in reversed(pending):
                age = (now-stamp.nanoseconds)*1e-9
                task_fresh = abs(age)<=0.3
                map_fresh = name!="scan" and publish_surface and 0<=age<=self.cfg["surface_max_age_s"]
                if not (task_fresh or map_fresh):
                    self.warn(name+"-stamp","{} acquisition stamp is unavailable or stale".format(name))
                    break
                if name != "scan" and self.cfg["odom_topic"] and stamp.nanoseconds > self.odom_stamp:
                    continue
                try:
                    if name == "scan":
                        self.scan(msg,stamp)
                    else:
                        self.depth(name,msg,stamp,task_fresh,publish_surface)
                        depth_processed = True
                except TransformException as exc:
                    self.warn(name,str(exc))
                    continue
                while pending and pending[0][0].nanoseconds <= stamp.nanoseconds:
                    pending.popleft()
                break
        if self.layers:
            latest,ground,_ = max(self.layers.values(),key=lambda layer:layer[0])
            beams = [group for stamp,_,group in self.layers.values() if latest-stamp <= 0.12e9]
            if latest > self.last_depth:
                self.send("depth_rays",latest,frame,{"map_id":self.cfg["map_id"],"ground_xyz":ground,"beams":beams})
                self.last_depth = latest
            self.layers.clear()
        if depth_processed and self.cfg["depth_rate_hz"]>0:
            self.next_depth = time.monotonic()+1.0/self.cfg["depth_rate_hz"]


if __name__ == "__main__":
    rclpy.init()
    node = MapObservations()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.slam:
            node.slam.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
