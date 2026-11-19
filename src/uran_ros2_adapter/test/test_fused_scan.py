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
    assert within_survey(np.array([[13,2,15],[13,2.001,0]]),body).tolist() == [True,False]


def test_rotated_offset_and_laser_returns_beyond_radius_do_not_clear_space():
    transform = np.eye(4)
    transform[:2,:2] = [[0,-1],[1,0]]
    transform[:2,3] = [12,2]
    raw = scan([math.inf,8,0,5.19,5.4,0,0])
    ranges,sources = fuse_ranges(raw,transform,(12,2.2),[])
    assert sources.tolist() == [0,0,0,1,0,0,0]
    assert math.isclose(ranges[3],5.19)


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
