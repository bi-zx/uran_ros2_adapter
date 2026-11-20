from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from uran_ros2_adapter.map_observations import MapObservations


@pytest.fixture
def node():
    rclpy.init(args=["--ros-args","-p","device_id:=test","-p","component_id:=base","-p","map_id:=room",
                     "-p","depth_topic:=cloud","-p","pose_topic:=slam","-p","pose_includes_odometry:=true"])
    obs = MapObservations()
    obs.out = Mock()
    obs.out.get_subscription_count.return_value = 1
    obs.send = Mock()
    yield obs
    obs.destroy_node()
    rclpy.shutdown()


def laser(node, stamp, x):
    msg = PoseStamped()
    msg.header.stamp = Time(nanoseconds=stamp).to_msg()
    msg.header.frame_id = "map"
    msg.pose.position.x, msg.pose.orientation.w = x,1.
    node.on_map_pose(msg)


def odom(node, stamp, x):
    msg = Odometry()
    msg.header.stamp = Time(nanoseconds=stamp).to_msg()
    msg.header.frame_id, msg.child_frame_id = "odom","base_link"
    msg.pose.pose.position.x, msg.pose.pose.orientation.w = x,1.
    node.on_odom(msg)


def test_upstream_fused_pose_reaches_task_on_laser_arrival(node):
    now = node.get_clock().now().nanoseconds
    odom(node,now-900000000,1.)
    laser(node,now,4.)
    kind,stamp,frame,value = node.send.call_args[0]
    assert (kind,stamp,frame,value["x"],value["odometry_fused"]) == ("map_pose",now,"map",4.,True)
    laser(node,now-100000000,3.5)
    node.flush()
    assert node.send.call_count == 1


def test_alignment_uses_latest_time_shared_by_laser_and_odometry(node):
    now = node.get_clock().now().nanoseconds
    laser(node,now-400000000,3.)
    odom(node,now-400000000,1.)
    odom(node,now-200000000,1.2)
    laser(node,now,3.4)
    node.flush()
    assert node.anchor_stamp == now-200000000
    assert np.isclose(node.anchor[0,3],2.)


def test_expired_upstream_pose_does_not_refresh_the_task(node):
    now = node.get_clock().now().nanoseconds
    laser(node,now-3000000000,3.)
    odom(node,now,1.)
    node.flush()
    node.send.assert_not_called()


def test_floor_and_step_heights_follow_support_attitude_and_laser_xy():
    from uran_ros2_adapter.map_geometry import grounded_pose
    yaw,tilt = np.radians([75.,12.])
    c,s = np.cos(tilt),np.sin(tilt)
    support = np.eye(4)
    support[:3,:3] = [[c,0,s],[0,1,0],[-s,0,c]]
    support[:3,3] = [18.,-9.,.24]
    m = np.eye(4)
    m[:3,:3] = [[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]]
    m[:3,3] = [2.,3.,0.]
    body = grounded_pose(m,support)
    assert np.allclose(body[:3,3],[2.,3.,.24])
    assert np.isclose(np.arctan2(body[1,0],body[0,0]),yaw)
    # A floor ray, an 11 cm step and a 9 cm drop retain their physical heights.
    ground_points = np.array([[1.,0.,0.],[1.,0.,.11],[1.,0.,-.09]])
    local = (ground_points-[0.,0.,.24]) @ support[:3,:3]
    assert np.allclose((local @ body[:3,:3].T+body[:3,3])[:,2],ground_points[:,2])
