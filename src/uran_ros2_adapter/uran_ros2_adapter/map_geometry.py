"""Geometry shared by the ROS mapping nodes."""
import math

import numpy as np
from geometry_msgs.msg import TransformStamped


def rigid(transform):
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return rotation, np.array([t.x,t.y,t.z])


def cloud_points(msg, stride=None):
    fields = {f.name:f for f in msg.fields}
    coordinates = []
    if msg.height == 1:
        stride = 1
    elif stride is None:
        stride = max(1, math.ceil(math.sqrt(msg.width*msg.height/2400)))
    for name in ("x","y","z"):
        f = fields[name]
        dtype = (">" if msg.is_bigendian else "<") + {7:"f4",8:"f8"}[f.datatype]
        values = np.ndarray((msg.height,msg.width),dtype,msg.data,offset=f.offset,strides=(msg.row_step,msg.point_step))
        coordinates.append(values[::stride,::stride].ravel())
    points = np.column_stack(coordinates)
    return points[np.isfinite(points).all(axis=1)]


def rectified_depth_points(image, info, stride=None):
    """Return XYZ in the optical frame for standard metric ROS depth images."""
    if image.header.frame_id != info.header.frame_id:
        raise ValueError("depth image and calibration frames differ")
    bin_x,bin_y = max(1,info.binning_x),max(1,info.binning_y)
    fx,fy = info.p[0]/bin_x,info.p[5]/bin_y
    if fx <= 0 or fy <= 0:
        raise ValueError("rectified depth camera has no projection calibration")
    cx,cy = (info.p[2]-info.roi.x_offset)/bin_x,(info.p[6]-info.roi.y_offset)/bin_y
    if image.encoding == "16UC1":
        fmt,scale = "u2",0.001
    elif image.encoding == "32FC1":
        fmt,scale = "f4",1.0
    else:
        raise ValueError("unsupported depth encoding " + image.encoding)
    dtype = np.dtype((">" if image.is_bigendian else "<") + fmt)
    raw = np.ndarray((image.height,image.width),dtype,image.data,strides=(image.step,dtype.itemsize))
    if stride is None:
        stride = max(1, math.ceil(math.sqrt(image.width*image.height/2400)))
    sampled = raw[::stride,::stride]
    v,u = np.nonzero(np.isfinite(sampled) & (sampled > 0))
    z = sampled[v,u].astype(float)*scale
    return np.column_stack(((u*stride-cx)*z/fx,(v*stride-cy)*z/fy,z))


def matrix(transform):
    result = np.eye(4)
    result[:3,:3],result[:3,3] = rigid(transform)
    return result


def stamped_pose(header,child,pose):
    t = TransformStamped()
    t.header,t.child_frame_id = header,child
    p = pose.position
    t.transform.translation.x,t.transform.translation.y,t.transform.translation.z = p.x,p.y,p.z
    t.transform.rotation = pose.orientation
    return t


def odometry_transform(msg):
    return stamped_pose(msg.header,msg.child_frame_id,msg.pose.pose)


MAPPING_RADIUS_M = 5.0


def within_survey(points, body_xy):
    radius = np.hypot(points[:,0]-body_xy[0], points[:,1]-body_xy[1])
    return np.less_equal(radius,MAPPING_RADIUS_M,where=np.isfinite(radius),out=np.zeros(len(radius),dtype=bool))


def body_in_odom(history, tf, frames, body_frame, body_alias, stamp):
    odom,child = frames
    pose = matrix(history.lookup_transform(odom,child,stamp).transform)
    if child not in (body_frame,body_alias):
        pose = pose @ matrix(tf.lookup_transform(child,body_frame,stamp).transform)
    return pose


def fuse_ranges(scan, laser_in_odom, body_xy, supplements):
    """Merge measured hits in the laser plane. Supplements are already ground/radius filtered."""
    ranges = np.asarray(scan.ranges,dtype=float)
    angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
    unit = np.column_stack((np.cos(angles),np.sin(angles),np.zeros(len(angles))))
    world_unit = unit @ laser_in_odom[:3,:3].T
    valid = np.isfinite(ranges) & (ranges > 0) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    indices = np.flatnonzero(valid)
    endpoints = laser_in_odom[:3,3] + ranges[indices,None]*world_unit[indices]
    valid[indices] = within_survey(endpoints,body_xy)
    hits = np.full(len(ranges),np.nan)
    source = np.zeros(len(ranges),dtype=np.uint8)
    hits[valid],source[valid] = ranges[valid],1
    middle = (angles[0]+angles[-1])/2
    for kind,points in supplements:
        distance = np.hypot(points[:,0],points[:,1])
        angle = np.arctan2(points[:,1],points[:,0])
        angle += 2*math.pi*np.rint((middle-angle)/(2*math.pi))
        bins = np.rint((angle-scan.angle_min)/scan.angle_increment).astype(int)
        take = ((abs(points[:,2]) <= 0.025) & (distance > 0) & (distance >= scan.range_min) &
                (distance <= scan.range_max) & (bins >= 0) & (bins < len(ranges)))
        bins,distance = bins[take],distance[take]
        # Two nearby hits in the same angular bin reject isolated depth pixels.
        order = np.lexsort((distance,bins))
        bins,distance = bins[order],distance[order]
        paired = (bins[1:]==bins[:-1]) & (np.diff(distance) <= 0.04 + 0.015*distance[:-1])
        supported = np.full(len(ranges),np.inf)
        np.minimum.at(supported,bins[:-1][paired],distance[:-1][paired])
        take = ~valid & np.isfinite(supported) & (supported < np.where(np.isfinite(hits),hits,np.inf))
        hits[take],source[take] = supported[take],kind
    # Check the actual emitted beam direction, including the laser mount and body tilt.
    hits = hits.astype(np.float32).astype(float)
    world = laser_in_odom[:3,3] + hits[:,None]*world_unit
    outside = ~within_survey(world,body_xy)
    hits[outside],source[outside] = np.nan,0
    return hits,source
