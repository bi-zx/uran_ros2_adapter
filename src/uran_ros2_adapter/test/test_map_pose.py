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
