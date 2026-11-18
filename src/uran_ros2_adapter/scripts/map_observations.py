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
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, NavSatFix
from tf2_ros import Buffer, TransformException, TransformListener
from uran_ros2_msgs.msg import ContractEnvelope, OperationEvent


def rigid(transform):
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return rotation, np.array([t.x,t.y,t.z])


def cloud_points(msg):
    fields = {f.name:f for f in msg.fields}
    coordinates = []
    stride = max(1, math.ceil(math.sqrt(msg.width*msg.height/2400))) if msg.height > 1 else 1
    for name in ("x","y","z"):
        f = fields[name]
        dtype = (">" if msg.is_bigendian else "<") + {7:"f4",8:"f8"}[f.datatype]
        values = np.ndarray((msg.height,msg.width),dtype,msg.data,offset=f.offset,strides=(msg.row_step,msg.point_step))
        coordinates.append(values[::stride,::stride].ravel())
    points = np.column_stack(coordinates)
    return points[np.isfinite(points).all(axis=1)]


class MapObservations(Node):
    def __init__(self):
        super().__init__("map_observations")
        defaults = {"map_id":"","map_frame":"map","body_frame":"base_link","ground_frame":"base_footprint",
                    "device_id":"","component_id":"","instance_id":"","source_component":"sensors",
                    "reference_mode":"local","odom_topic":"odom","slam_timeout_s":2.0,
                    "scan_topic":"scan","depth_topic":"","depth_origin_frame":"","gps_topic":"",
                    "sensor_adapter":"","slam_adapter":""}
        self.cfg = {k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        if not self.cfg["device_id"] or not self.cfg["component_id"]:
            raise ValueError("provide the target device_id and component_id")
        if not (self.cfg["slam_adapter"] or self.cfg["map_id"]):
            raise ValueError("provide a SLAM session adapter or an externally managed map_id")
        if self.cfg["reference_mode"] not in ("local", "geographic"):
            raise ValueError("reference_mode must be local or geographic")
        if not (self.cfg["sensor_adapter"] or self.cfg["depth_topic"]):
            raise ValueError("provide a depth_topic or a sensor adapter")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.odom_history = Buffer()
        self.odom_frames = None
        self.odom_stamp = self.anchor_stamp = self.revision = 0
        self.anchor = None
        self.out = self.create_publisher(ContractEnvelope,"/uran/runtime/dispatch",30)
        self.pending = {}
        self.layers = {}
        self.last_warning = {}
        self.last_pose = self.last_depth = 0
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

    def on_gps(self,msg):
        if msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.geographic_fix(Time.from_msg(msg.header.stamp).nanoseconds,msg.latitude,msg.longitude,{"horizontal_fix_valid":True})

    def geographic_fix(self,stamp,lat,lon,quality):
        if self.cfg["reference_mode"] == "geographic" and stamp and (not self.slam or self.slam.stage == "ready"):
            self.send("gps_fix",stamp,"wgs84",dict(latitude_deg=lat,longitude_deg=lon,**quality))

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
        t = TransformStamped()
        t.header,t.child_frame_id = msg.header,msg.child_frame_id
        p = msg.pose.pose
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z = p.position.x,p.position.y,p.position.z
        t.transform.rotation = p.orientation
        self.odom_history.set_transform(t,"odometry")
        self.odom_frames,self.odom_stamp = frames,stamp

    @staticmethod
    def matrix(transform):
        result = np.eye(4)
        result[:3,:3],result[:3,3] = rigid(transform)
        return result

    def body_in_odom(self,stamp):
        odom,child = self.odom_frames
        raw = self.matrix(self.odom_history.lookup_transform(odom,child,stamp).transform)
        mount = self.tf.lookup_transform(child,self.cfg["body_frame"],stamp)
        return raw @ self.matrix(mount.transform)

    def odometry_corrected_pose(self):
        if not self.odom_stamp:
            return None
        frame = self.cfg["map_frame"]
        odom = self.odom_frames[0]
        try:
            correction = self.tf.lookup_transform(frame,odom,Time())
        except TransformException:
            correction = None
        if correction is not None and Time.from_msg(correction.header.stamp).nanoseconds:
            stamp = Time.from_msg(correction.header.stamp)
            candidate = self.matrix(correction.transform)
        else:
            correction = self.tf.lookup_transform(frame,self.cfg["body_frame"],Time())
            stamp = Time.from_msg(correction.header.stamp)
            candidate = None
        if stamp.nanoseconds > self.anchor_stamp:
            if candidate is None:
                try:
                    candidate = self.matrix(correction.transform) @ np.linalg.inv(self.body_in_odom(stamp))
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
            return rigid(self.tf.lookup_transform(self.cfg["map_frame"],source,stamp).transform)
        assert self.anchor is not None
        mount = self.tf.lookup_transform(self.cfg["body_frame"],source,stamp)
        pose = self.anchor @ self.body_in_odom(stamp) @ self.matrix(mount.transform)
        return pose[:3,:3],pose[:3,3]

    def send(self,kind,stamp,frame,value):
        target = self.cfg
        packet = ContractEnvelope()
        packet.message_type = "uran.sensors.observation.v1"
        packet.schema_name = "sensors/observation.schema.json"
        packet.device_id,packet.component_id,packet.instance_id = target["device_id"],target["component_id"],target["instance_id"]
        packet.correlation_id = "{}-{}-{}-{}-{}".format(packet.device_id, target["source_component"], packet.instance_id or packet.component_id, kind, stamp)
        packet.payload_json = json.dumps({"schema_version":"1.0","source":{"device_id":packet.device_id,"component_id":target["source_component"]},
            "frame_id":frame,"timestamp_ns":stamp,"observation_type":kind,"value":value},allow_nan=False,separators=(",",":"))
        self.out.publish(packet)

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
            elif msg.range_min <= r <= msg.range_max and r > 0:
                x,y = (r*vector + offset)[:2]
                distance = math.hypot(x,y)
            else:
                continue
            i = round((math.atan2(y,x)+math.pi)/increment) % count
            if not isinstance(result[i],float) or (distance != "inf" and distance < result[i]):
                result[i] = distance
        self.send("laser_scan",stamp.nanoseconds,self.cfg["body_frame"],{"angle_min":-math.pi,"angle_max":math.pi-increment,
            "angle_increment":increment,"range_min":msg.range_min,"range_max":msg.range_max+float(np.linalg.norm(offset)),"ranges":result})

    def depth(self,name,msg,stamp):
        if name == "depth":
            rot,offset = self.in_map(msg.header.frame_id,stamp)
            origin_frame = self.cfg["depth_origin_frame"] or msg.header.frame_id
            _,origin = self.in_map(origin_frame,stamp)
            points = cloud_points(msg) @ rot.T + offset
        else:
            source_frame,origin,points = msg
            rot,offset = self.in_map(source_frame,stamp)
            origin = origin @ rot.T + offset
            points = points @ rot.T + offset
        if len(points):
            _,ground = self.in_map(self.cfg["ground_frame"],stamp)
            self.layers[name] = (stamp.nanoseconds,ground.tolist(),{"origin":origin.tolist(),"points":points.tolist()})

    def flush(self):
        if self.slam:
            self.slam.tick()
            if self.slam.stage != "ready":
                self.pending.clear()
                self.layers.clear()
                self.last_pose = self.last_depth = self.odom_stamp = self.anchor_stamp = 0
                self.anchor = self.odom_frames = None
                return
        frame = self.cfg["map_frame"]
        try:
            if self.cfg["odom_topic"]:
                result = self.odometry_corrected_pose()
                if result is None:
                    return
                stamp,rot,xy = result
            else:
                tf = self.tf.lookup_transform(frame,self.cfg["body_frame"],Time())
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
        for name,pending in self.pending.items():
            for stamp,msg in reversed(pending):
                if abs(now-stamp.nanoseconds) > 0.3e9:
                    self.warn(name+"-stamp","{} acquisition stamp is unavailable or stale".format(name))
                    break
                try:
                    if name == "scan":
                        self.scan(msg,stamp)
                    else:
                        self.depth(name,msg,stamp)
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
