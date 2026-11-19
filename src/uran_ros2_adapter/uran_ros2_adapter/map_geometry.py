"""Geometry shared by the ROS mapping nodes."""
import math

import numpy as np
from geometry_msgs.msg import TransformStamped


def take_ready_scan(pending, now, tolerance_s, max_wait_s, odom_ns, depth_ns):
    """Take the newest ready scan, leaving newer exposures queued for their sensor data."""
    selected = None
    while pending:
        received, stamp, scan = pending[0]
        age = now-received
        enough_history = min(odom_ns,depth_ns) >= stamp + tolerance_s*1e9
        if age < tolerance_s or (age < max_wait_s and not enough_history):
            break
        pending.popleft()
        if stamp <= odom_ns:
            selected = scan
    return selected


def rigid(transform):
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return rotation, np.array([t.x,t.y,t.z])


def scan_in_body(msg, rot, offset):
    """Project observed endpoints into a polar scan at the body origin."""
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
    return {"angle_min":-math.pi,"angle_max":math.pi-increment,"angle_increment":increment,
            "range_min":msg.range_min,"range_max":msg.range_max+float(np.linalg.norm(offset)),"ranges":result}


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


def rectified_intrinsics(info):
    bin_x,bin_y = max(1,info.binning_x),max(1,info.binning_y)
    fx,fy = info.p[0]/bin_x,info.p[5]/bin_y
    if fx <= 0 or fy <= 0:
        raise ValueError("rectified depth camera has no projection calibration")
    cx,cy = (info.p[2]-info.roi.x_offset)/bin_x,(info.p[6]-info.roi.y_offset)/bin_y
    return fx,fy,cx,cy


def rectified_depth_model(image, info):
    """Read a rectified metric depth image and its matching projection intrinsics."""
    if image.header.frame_id != info.header.frame_id:
        raise ValueError("depth image and calibration frames differ")
    fx,fy,cx,cy = rectified_intrinsics(info)
    if image.encoding == "16UC1":
        fmt,scale = "u2",0.001
    elif image.encoding == "32FC1":
        fmt,scale = "f4",1.0
    else:
        raise ValueError("unsupported depth encoding " + image.encoding)
    dtype = np.dtype((">" if image.is_bigendian else "<") + fmt)
    raw = np.ndarray((image.height,image.width),dtype,image.data,strides=(image.step,dtype.itemsize))
    return raw,scale,fx,fy,cx,cy


def rectified_depth_points(image, info, stride=None):
    """Return XYZ in the optical frame for standard metric ROS depth images."""
    raw,scale,fx,fy,cx,cy = rectified_depth_model(image,info)
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


def scan_bins(scan, points):
    middle = scan.angle_min+(len(scan.ranges)-1)*scan.angle_increment/2
    angles = np.arctan2(points[:,1],points[:,0])
    angles += 2*np.pi*np.rint((middle-angles)/(2*np.pi))
    return np.rint((angles-scan.angle_min)/scan.angle_increment).astype(int)


def synchronized_scan_points(scan, points, exposure_offset_ns, tolerance_ns):
    """Keep measured scan-plane points synchronized to the laser ray they can fill."""
    planar = points[abs(points[:,2]) <= .025]
    bins = scan_bins(scan,planar)
    timely = abs(bins*(scan.time_increment*1e9)-exposure_offset_ns) <= tolerance_ns
    return planar[timely & (bins>=0) & (bins<len(scan.ranges))]


def matching_depth_views(views, stamp_ns, tolerance_ns):
    """Closest exposure, then an earlier independent exposure in the last 350 ms."""
    recent = [(t,v) for t,v in views if stamp_ns-350000000 <= t <= stamp_ns+tolerance_ns]
    if not recent:
        return []
    first = min(recent,key=lambda row:abs(stamp_ns-row[0]))
    earlier = [row for row in recent if row[0] <= first[0]-20000000]
    return [first,max(earlier,key=lambda row:row[0])] if earlier else [first]


def patch_projection(points, footprint, border=4):
    (h,w),(fx,fy,cx,cy),camera = footprint
    optical = (points-camera[:3,3]) @ camera[:3,:3]
    idx = np.flatnonzero(optical[:,2] > 0)
    x,y,z = optical[idx].T
    u,v = np.rint(fx*x/z+cx).astype(int),np.rint(fy*y/z+cy).astype(int)
    inside = (u>=border)&(u<w-border)&(v>=border)&(v<h-border)
    return idx[inside],u[inside],v[inside],z[inside]


def depth_patch_evidence(points, view, margin, require_support=False):
    """0 is unresolved, -1 is compatible foreground, +1 is a dense farther surface."""
    (pixels,scale,fx,fy,cx,cy),camera,body_xy = view
    result = np.zeros(len(points),dtype=np.int8)
    idx,u,v,z = patch_projection(points,(pixels.shape,(fx,fy,cx,cy),camera))
    if not len(idx):
        return result
    offsets = np.arange(-4,5)
    cols,rows = u[:,None,None]+offsets[None,None,:],v[:,None,None]+offsets[None,:,None]
    measured = pixels[rows,cols].astype(float)*scale
    valid = np.isfinite(measured) & (measured>0)
    metric = np.where(valid,measured,0)
    # A depth point beyond the body's survey radius cannot establish a contradiction.
    px,py = (cols-cx)*metric/fx,(rows-cy)*metric/fy
    dx = camera[0,0]*px+camera[0,1]*py+camera[0,2]*metric+camera[0,3]-body_xy[0]
    dy = camera[1,0]*px+camera[1,1]*py+camera[1,2]*metric+camera[1,3]-body_xy[1]
    valid &= dx*dx+dy*dy <= MAPPING_RADIUS_M**2
    nearest = np.min(np.where(valid,metric,np.inf),axis=(1,2))
    farther = nearest > z+margin
    foreground = ~farther
    if require_support:
        foreground &= nearest >= z-margin
    result[idx[foreground]] = -1
    confirmed = farther & valid[:,4,4] & (np.sum(valid,axis=(1,2)) >= 69)
    result[idx[confirmed]] = 1
    return result


class NearSurfaceHistory:
    """Confirm observed points on short laser line segments in two odometry-aligned scans."""

    def __init__(self, max_range):
        self.max_range = max_range
        self.previous = None

    def update(self, scan, laser_in_odom, stamp_ns):
        ranges = np.asarray(scan.ranges,dtype=float)
        supported = np.zeros(len(ranges),dtype=bool)
        previous = self.previous
        if previous is not None and stamp_ns <= previous[0]:
            return supported
        valid = np.isfinite(ranges) & (ranges>0) & (ranges>=scan.range_min) & (ranges<=min(self.max_range,scan.range_max))
        beams = np.flatnonzero(valid)
        angles = scan.angle_min + beams*scan.angle_increment
        points = np.column_stack((ranges[beams]*np.cos(angles),ranges[beams]*np.sin(angles)))
        self.previous = stamp_ns, points @ laser_in_odom[:3,:2].T + laser_in_odom[:3,3]
        if previous is None or stamp_ns-previous[0] > 350000000 or len(beams)<6:
            return supported

        old = (previous[1]-laser_in_odom[:3,3]) @ laser_in_odom[:3,:3]
        old = old[abs(old[:,2]) <= .025]
        bins = scan_bins(scan,old)
        inside = (bins>=0)&(bins<len(ranges))
        # Allow one bin of angular quantization when comparing the two scans.
        prior_ranges = np.full(len(ranges)+2,np.inf)
        np.minimum.at(prior_ranges,bins[inside]+1,np.hypot(old[inside,0],old[inside,1]))
        repeat_error = abs(prior_ranges[beams[:,None]+np.arange(3)]-ranges[beams,None])
        repeated = np.min(repeat_error,axis=1) <= .020

        gaps = (np.diff(beams)>2) | (np.linalg.norm(np.diff(points,axis=0),axis=1)>.030)
        runs = np.split(np.arange(len(beams)),np.flatnonzero(gaps)+1)
        while runs:
            run = runs.pop()
            if len(run)<6:
                continue
            segment = points[run]
            chord = segment[-1]-segment[0]
            length = np.linalg.norm(chord)
            if length<.08:
                continue
            normal = np.array([-chord[1],chord[0]])/length
            error = abs((segment-segment[0]) @ normal)
            inliers = error <= .014
            if np.mean(inliers)<.90:
                split = int(np.argmax(error))
                runs.extend((run[:split+1],run[split:]))
                continue
            confirmed = run[inliers & repeated[run]]
            if len(confirmed)<max(6,.75*len(run)) or np.linalg.norm(points[confirmed[-1]]-points[confirmed[0]])<.08:
                continue
            # A body-fixed ring also repeats. Require a better fit to a line than to a constant radius.
            centered = points[confirmed]-np.mean(points[confirmed],axis=0)
            line_variance = np.linalg.eigvalsh(centered.T @ centered)[0]/len(confirmed)
            if math.sqrt(max(0.0,line_variance))+.001 < np.std(ranges[beams[confirmed]]):
                supported[beams[confirmed]] = True
        return supported


def near_lidar_actions(scan, laser_in_odom, stamp_ns, views, tof_layers, check_range, margin, tolerance_ns, footprint=None, require_support=False, blind_surface=None, ray_step_ns=0):
    """0 keeps lidar, 1 withholds, 2 permits substitution, 3 permits nearer hits, 4 keeps a blind surface."""
    ranges = np.asarray(scan.ranges,dtype=float)
    action = np.zeros(len(ranges),dtype=np.uint8)
    idx = np.flatnonzero(np.isfinite(ranges) & (ranges>0) & (ranges>=scan.range_min) &
                         (ranges<=scan.range_max) & (ranges<=check_range))
    if not len(idx):
        return action
    a = scan.angle_min+idx*scan.angle_increment
    measured = np.column_stack((ranges[idx]*np.cos(a),ranges[idx]*np.sin(a),np.zeros(len(idx))))
    world = measured @ laser_in_odom[:3,:3].T+laser_in_odom[:3,3]
    if require_support:
        action[idx] = 3
        if blind_surface is not None and footprint is not None:
            visible,_,_,_ = patch_projection(world,footprint,border=0)
            blind = blind_surface[idx].copy()
            blind[visible] = False
            action[idx[blind]] = 4
    elif footprint is not None:
        inside,_,_,_ = patch_projection(world,footprint)
        action[idx[inside]] = 1
    half_duration = abs(ray_step_ns)*(len(ranges)-1)/2
    views = matching_depth_views(views,stamp_ns,tolerance_ns+half_duration)
    if views and abs(stamp_ns-views[0][0]) <= tolerance_ns+half_duration:
        evidence = depth_patch_evidence(world,views[0][1],margin,require_support)
        offsets = (idx-(len(ranges)-1)/2)*ray_step_ns
        evidence[abs((stamp_ns-views[0][0])+offsets)>tolerance_ns] = 0
        action[idx[evidence<0]] = 0
        disputed = np.flatnonzero(evidence>0)
        action[idx[disputed]] = 3 if require_support else 1
        if len(views)==2 and len(disputed):
            repeated = depth_patch_evidence(world[disputed],views[1][1],margin) > 0
            action[idx[disputed[repeated]]] = 2
    for tof in tof_layers:
        positions = np.flatnonzero(action[idx])
        planar = tof[abs(tof[:,2]) <= .035]
        if len(planar) and len(positions):
            distance2 = np.sum((measured[positions,None,:]-planar[None,:,:])**2,axis=2)
            action[idx[positions[np.min(distance2,axis=1) <= .075**2]]] = 0
    return action


def fuse_ranges(scan, laser_in_odom, body_xy, supplements, lidar_action=None):
    """Merge measured hits in the laser plane. Supplements are already ground/radius filtered."""
    ranges = np.asarray(scan.ranges,dtype=float)
    angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
    unit = np.column_stack((np.cos(angles),np.sin(angles),np.zeros(len(angles))))
    world_unit = unit @ laser_in_odom[:3,:3].T
    valid = np.isfinite(ranges) & (ranges > 0) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    indices = np.flatnonzero(valid)
    endpoints = laser_in_odom[:3,3] + ranges[indices,None]*world_unit[indices]
    valid[indices] = within_survey(endpoints,body_xy)
    supplement_limit = np.full(len(ranges),np.inf)
    if lidar_action is not None:
        valid &= (lidar_action == 0) | (lidar_action == 4)
        supplement_limit[lidar_action==1] = 0.0
        pending = lidar_action==3
        supplement_limit[pending] = ranges[pending]
    hits = np.full(len(ranges),np.nan)
    source = np.zeros(len(ranges),dtype=np.uint8)
    hits[valid],source[valid] = ranges[valid],1
    for kind,points in supplements:
        distance = np.hypot(points[:,0],points[:,1])
        bins = scan_bins(scan,points)
        take = ((abs(points[:,2]) <= 0.025) & (distance > 0) & (distance >= scan.range_min) &
                (distance <= scan.range_max) & (bins >= 0) & (bins < len(ranges)))
        bins,distance = bins[take],distance[take]
        # Two nearby hits in the same angular bin reject isolated depth pixels.
        order = np.lexsort((distance,bins))
        bins,distance = bins[order],distance[order]
        paired = (bins[1:]==bins[:-1]) & (np.diff(distance) <= 0.04 + 0.015*distance[:-1])
        supported = np.full(len(ranges),np.inf)
        np.minimum.at(supported,bins[:-1][paired],distance[:-1][paired])
        take = ~valid & (supported <= supplement_limit) & np.isfinite(supported) & (supported < np.where(np.isfinite(hits),hits,np.inf))
        hits[take],source[take] = supported[take],kind
    # Check the actual emitted beam direction, including the laser mount and body tilt.
    hits = hits.astype(np.float32).astype(float)
    world = laser_in_odom[:3,3] + hits[:,None]*world_unit
    outside = ~within_survey(world,body_xy)
    hits[outside],source[outside] = np.nan,0
    return hits,source
