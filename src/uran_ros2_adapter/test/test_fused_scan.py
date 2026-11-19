import math
from types import SimpleNamespace

import numpy as np
from uran_ros2_adapter.map_geometry import fuse_ranges, within_survey


def scan(ranges):
    return SimpleNamespace(angle_min=-0.3, angle_increment=0.1, range_min=0.05, range_max=30.0, ranges=ranges)


def returns(i, distances, height=0.0):
    angle = -0.3 + i*0.1
    return [(r*math.cos(angle),r*math.sin(angle),height) for r in distances]


def test_fuse_only_supported_hits_preserving_laser():
    raw = scan([1.2,0,math.nan,math.inf,0,0,0])
    depth = returns(0,[0.4,0.41]) + returns(1,[2,2.01]) + returns(2,[1.5]) + returns(3,[4.0,3.0])
    depth += returns(4,[2,2.01],height=-0.1)
    tof = returns(5,[0.3,0.31])
    ranges,sources = fuse_ranges(raw,np.eye(4),(0,0),[(2,np.asarray(depth)),(3,np.asarray(tof))])
    assert list(sources) == [1,2,0,0,0,3,0]
    assert np.allclose(ranges[[0,1,5]],[1.2,2.0,0.3])
    assert np.isnan(ranges[[2,3,4,6]]).all()


def test_boundary_is_horizontal_body_centered_and_inclusive():
    laser = np.eye(4)
    laser[:3,3] = [10.25,-2,0.20]
    body = (10,-2)
    raw = scan([0,0,0,4.75,0,0,0])
    supplemental = np.array(returns(4,[4.81,4.82]))
    ranges,sources = fuse_ranges(raw,laser,body,[(2,supplemental)])
    assert ranges[3] == 4.75 and np.count_nonzero(sources)==1
    raw.ranges[3] = 4.751
    ranges,sources = fuse_ranges(raw,laser,body,[])
    assert np.isnan(ranges).all()
    near = np.array(returns(3,[2.0,2.01]))
    ranges,sources = fuse_ranges(raw,laser,body,[(2,near)])
    assert sources[3] == 2 and ranges[3] == 2.0
    assert within_survey(np.array([[13,2,15],[13,2.001,0]]),body).tolist() == [True,False]


def test_rotated_offset_and_laser_returns_beyond_radius_do_not_clear_space():
    transform = np.eye(4)
    transform[:2,:2] = [[0,-1],[1,0]]
    transform[:2,3] = [12,2]
    raw = scan([math.inf,8,0,5.19,5.4,0,0])
    ranges,sources = fuse_ranges(raw,transform,(12,2.2),[])
    assert sources.tolist() == [0,0,0,1,0,0,0]
    assert np.isclose(ranges[3],5.19)


def test_vendor_raster_spill_is_unknown_and_observed_history_reopens(tmp_path):
    from uran_ros2_adapter.survey_coverage import SurveyCoverage
    path = str(tmp_path/"coverage.sqlite")
    points = np.array([[4.999,0],[5.001,0],[3,1],[2,0]])
    coverage = SurveyCoverage(path,"route","map")
    coverage.observe(np.array([.2,0]),np.array([[5.0,0]]),(0,0))
    assert coverage.permits(points).tolist() == [True,False,False,True]
    coverage.save()
    reopened = SurveyCoverage(path,"route","map")
    reopened.observe(np.array([6.2,0]),np.array([[5.0,0]]),(6,0))
    assert reopened.permits(points).tolist() == [True,True,False,True]



def test_reobserving_from_opposite_edge_keeps_both_views(tmp_path):
    from uran_ros2_adapter.survey_coverage import SurveyCoverage
    path = str(tmp_path/"two_views.sqlite")
    coverage = SurveyCoverage(path,"route","map")
    for bx,end in ((-4.997,.002),(5.02,.021)):
        coverage.observe(np.array([bx,.025]),np.array([[end,.025]]),(bx,.025))
    coverage.save()
    reopened = SurveyCoverage(path,"route","map")
    assert reopened.permits(np.array([[.001,.025],[.023,.025]])).all()


def test_close_lidar_conflict_uses_measured_plane_hit_only_after_two_depth_views():
    from uran_ros2_adapter.map_geometry import near_lidar_actions
    # Optical X points right, optical Y down and Z forward in the laser frame.
    optical = np.eye(4)
    optical[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    raw = scan([0,0,0,.48,0,0,0])
    depth = np.full((65,65),1000,dtype=np.uint16)
    view = ((depth,.001,80,80,32,32),optical,(0,0))
    current = 1000000000
    views = [(current,view),(current-70000000,view)]
    action = near_lidar_actions(raw,np.eye(4),current,views,[],1.0,.25,60000000)
    ranges,source = fuse_ranges(raw,np.eye(4),(0,0),[(2,np.array(returns(3,[1,1.01])))],action)
    assert action[3] == 2 and source[3] == 2 and np.isclose(ranges[3],1.0)
    held = near_lidar_actions(raw,np.eye(4),current,views[:1],[],1.0,.25,60000000)
    ranges,source = fuse_ranges(raw,np.eye(4),(0,0),[(2,np.array(returns(3,[1,1.01])))],held)
    assert held[3] == 1 and source[3] == 0 and np.isnan(ranges[3])
    assert near_lidar_actions(raw,np.eye(4),current,[(current,view),(current,view)],[],1.0,.25,60000000)[3] == 1
    tof = np.array([[.48,.01,-.01]])
    assert near_lidar_actions(raw,np.eye(4),current,views,[tof],1.0,.25,60000000)[3] == 0
    assert near_lidar_actions(raw,np.eye(4),current+1000000000,views,[],1.0,.25,60000000)[3] == 0
    # A thin nearer object in the projected patch vetoes background depth.
    depth[32,33] = 475
    assert near_lidar_actions(raw,np.eye(4),current,views,[],1.0,.25,60000000)[3] == 0
    depth[:]=0
    assert near_lidar_actions(raw,np.eye(4),current,views,[],1.0,.25,60000000)[3] == 0
    depth[:]=6500
    assert near_lidar_actions(raw,np.eye(4),current,views,[],1.0,.25,60000000)[3] == 0


def test_missing_synchronized_depth_cannot_reinsert_a_near_camera_sector_hit():
    from uran_ros2_adapter.map_geometry import near_lidar_actions
    optical = np.eye(4)
    optical[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    footprint = ((65,65),(80,80,32,32),optical)
    raw = scan([.48,.48,0,.48,0,.48,.48])
    raw.angle_min,raw.angle_increment = -.9,.3
    action = near_lidar_actions(raw,np.eye(4),1000000000,[],[],1.,.1,60000000,footprint)
    assert action.tolist() == [0,0,0,1,0,0,0]
    tof = [np.array([[.48,0,.01]])]
    assert near_lidar_actions(raw,np.eye(4),1000000000,[],tof,1.,.1,60000000,footprint)[3] == 0


def test_unreliable_near_lidar_requires_independent_support_across_camera_boundary():
    from uran_ros2_adapter.map_geometry import near_lidar_actions
    optical = np.eye(4)
    optical[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    depth = np.full((65,65),480,dtype=np.uint16)
    view = ((depth,.001,80,80,32,32),optical,(0,0))
    footprint = (depth.shape,(80,80,32,32),optical)
    raw = scan([.5,.5,0,.48,0,.5,1.25])
    raw.angle_min,raw.angle_increment = -.9,.3
    now = 1000000000
    tof = np.array([[.5*math.cos(.6),.5*math.sin(.6),.01]])
    action = near_lidar_actions(raw,np.eye(4),now,[(now,view)],[tof],1.,.1,60000000,footprint,require_support=True)
    assert action.tolist() == [3,3,0,0,0,0,0]
    distant = np.array([[r*math.cos(-.9),r*math.sin(-.9),0] for r in (.9,.91)])
    closer = np.array([[r*math.cos(-.6),r*math.sin(-.6),0] for r in (.28,.29)])
    hits,source = fuse_ranges(raw,np.eye(4),(0,0),[(2,distant),(3,closer)],action)
    assert source.tolist() == [0,3,0,1,0,1,1]
    assert np.isnan(hits[0]) and np.isclose(hits[1],.28)
    depth[:] = 260
    action = near_lidar_actions(raw,np.eye(4),now,[(now,view)],[],1.,.1,60000000,footprint,require_support=True)
    assert action[3] == 3
    depth[:] = 0
    assert near_lidar_actions(raw,np.eye(4),now,[(now,view)],[],1.,.1,60000000,footprint,require_support=True)[3] == 3
    depth[:] = 1000
    views = [(now,view),(now-70000000,view)]
    assert near_lidar_actions(raw,np.eye(4),now,views[:1],[],1.,.1,60000000,footprint,require_support=True)[3] == 3
    assert near_lidar_actions(raw,np.eye(4),now,views,[],1.,.1,60000000,footprint,require_support=True)[3] == 2


def test_short_blind_surface_needs_a_repeated_measured_line():
    from uran_ros2_adapter.map_geometry import NearSurfaceHistory, near_lidar_actions
    history = NearSurfaceHistory(.35)
    raw = scan(np.zeros(241))
    raw.angle_min,raw.angle_increment = -1.2,.01
    angles = raw.angle_min + np.arange(241)*raw.angle_increment
    left,right = angles>.65,angles<-.65
    raw.ranges[left] = .18/np.sin(angles[left])
    raw.ranges[right] = -.20/np.sin(angles[right])
    raw.ranges[110:113] = .15  # isolated cluster
    raw.ranges[194:196] = 0   # a gap in the left panel
    first = np.eye(4)
    assert not history.update(raw,first,1000000000).any()
    second = np.eye(4)
    second[1,3] = .012
    raw.ranges[left] = .168/np.sin(angles[left])
    raw.ranges[right] = -.212/np.sin(angles[right])
    raw.ranges[194:196] = 0
    confirmed = history.update(raw,second,1100000000)
    assert confirmed[left].sum()>30 and confirmed[right].sum()>30
    assert not confirmed[194:196].any() and not confirmed[110:113].any()
    camera = second.copy()
    camera[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    footprint = ((101,101),(100,100,50,50),camera)
    action = near_lidar_actions(raw,second,1100000000,[],[],1.,.05,60000000,footprint,True,confirmed)
    hits,src = fuse_ranges(raw,second,second[:2,3],[],action)
    assert np.all(action[confirmed]==4) and np.all(src[confirmed]==1)
    assert np.allclose(hits[confirmed],raw.ranges[confirmed])
    assert np.isnan(hits[194:196]).all()
    assert not history.update(raw,second,1800000000).any()
    shifted = second.copy()
    shifted[1,3] += .12
    assert not history.update(raw,shifted,1900000000).any()


def test_visible_but_missing_or_contradictory_depth_overrules_lidar_shape():
    from uran_ros2_adapter.map_geometry import near_lidar_actions
    camera = np.eye(4)
    camera[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    raw = scan([.27]*7)
    raw.angle_min,raw.angle_increment = -.6,.2
    footprint = ((81,81),(80,80,40,40),camera)
    support = np.ones(7,dtype=bool)
    current = 1000000000
    action = near_lidar_actions(raw,np.eye(4),current,[],[],1.,.05,60000000,footprint,True,support)
    assert action.tolist()==[4,3,3,3,3,3,4]
    depth = np.full((81,81),650,dtype=np.uint16)
    view = ((depth,.001,80,80,40,40),camera,(0,0))
    action = near_lidar_actions(raw,np.eye(4),current,[(current,view),(current-70000000,view)],[],1.,.05,60000000,footprint,True,support)
    assert action[3]==2


def test_repeating_constant_radius_ring_does_not_confirm_a_wall():
    from uran_ros2_adapter.map_geometry import NearSurfaceHistory
    history = NearSurfaceHistory(.35)
    raw = scan(np.full(240,.22))
    raw.angle_min,raw.angle_increment = -1.2,.01
    history.update(raw,np.eye(4),1000000000)
    assert not history.update(raw,np.eye(4),1100000000).any()


def test_depth_uses_acquisition_time_of_each_ray_and_replacement():
    from uran_ros2_adapter.map_geometry import near_lidar_actions, synchronized_scan_points
    camera = np.eye(4)
    camera[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    raw = scan([.27]*7)
    raw.time_increment = .020
    depth = np.full((85,85),800,dtype=np.uint16)
    view = ((depth,.001,70,70,42,42),camera,(0,0))
    footprint = (depth.shape,(70,70,42,42),camera)
    start = 1000000000
    mid = start+60000000
    exposure = start+70000000
    frames = [(exposure,view),(exposure-80000000,view)]
    action = near_lidar_actions(raw,np.eye(4),mid,frames,[],1.,.05,25000000,footprint,True,ray_step_ns=20000000)
    assert action.tolist() == [3,3,3,2,2,3,3]
    pts = np.vstack([returns(i,[.8,.80]) for i in range(7)])
    paired = synchronized_scan_points(raw,pts,exposure-start,25000000)
    hits,src = fuse_ranges(raw,np.eye(4),(0,0),[(2,paired)],action)
    assert src.tolist() == [0,0,0,2,2,0,0] and np.allclose(hits[3:5],.8)
