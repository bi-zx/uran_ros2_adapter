"""ROS measurement envelopes shared by the survey and the live laser publisher."""
import json

from rclpy.time import Time
from tf2_ros import TransformException
from uran_ros2_msgs.msg import ContractEnvelope

from .map_geometry import rigid, scan_in_body


def observation_packet(target, kind, stamp, frame, value):
    packet = ContractEnvelope()
    packet.message_type = "uran.sensors.observation.v1"
    packet.schema_name = "sensors/observation.schema.json"
    packet.device_id,packet.component_id,packet.instance_id = target["device_id"],target["component_id"],target["instance_id"]
    packet.correlation_id = "{}-{}-{}-{}-{}".format(packet.device_id, target["source_component"], packet.instance_id or packet.component_id, kind, stamp)
    packet.payload_json = json.dumps({"schema_version":"1.0","source":{"device_id":packet.device_id,"component_id":target["source_component"]},
        "frame_id":frame,"timestamp_ns":stamp,"observation_type":kind,"value":value},allow_nan=False,separators=(",",":"))
    return packet


def publish_scan(node, msg, target, transforms, publisher):
    if not publisher.get_subscription_count():
        return
    stamp = Time.from_msg(msg.header.stamp)
    if abs(node.get_clock().now().nanoseconds-stamp.nanoseconds) > 0.30e9:
        return
    frame = target["body_frame"]
    try:
        rot,offset = rigid(transforms.lookup_transform(frame,msg.header.frame_id,stamp).transform)
    except TransformException as exc:
        node.get_logger().warning(str(exc),throttle_duration_sec=5.0)
        return
    publisher.publish(observation_packet(target,"laser_scan",stamp.nanoseconds,frame,scan_in_body(msg,rot,offset)))
