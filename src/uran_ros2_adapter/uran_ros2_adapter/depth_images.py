"""Share a rectified depth stream among mapping nodes in the same executor."""
from functools import partial
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def deliver(callbacks, msg):
    for callback in callbacks:
        callback(msg)


def subscribe_depth(node, readers, on_info, on_image):
    if readers is None:
        readers = {}
    qos = QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)
    for kind,field,callback in ((CameraInfo,"depth_camera_info_topic",on_info),(Image,"depth_image_topic",on_image)):
        topic = node.cfg[field]
        key = (kind,node.resolve_topic_name(topic))
        if key not in readers:
            readers[key] = []
            node.create_subscription(kind,topic,partial(deliver,readers[key]),qos)
        readers[key].append(callback)
