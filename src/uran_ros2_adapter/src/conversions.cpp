#include "uran_ros2_adapter/conversions.hpp"

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace uran_ros2_adapter
{
namespace
{
using Json = nlohmann::json;

std::uint64_t checked_add(std::uint64_t left, std::uint64_t right)
{
  if (right > std::numeric_limits<std::uint64_t>::max() - left) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return left + right;
}

std::string operation_id(const std::string & category, std::uint64_t timestamp, std::uint64_t sequence)
{
  return "legacy-" + category + "-" + std::to_string(timestamp) + "-" + std::to_string(sequence);
}

Json parse_object(const std::string & text, const char * field)
{
  if (text.empty()) {return Json::object();}
  auto value = Json::parse(text, nullptr, false);
  if (value.is_discarded() || !value.is_object()) {
    throw std::invalid_argument(std::string(field) + " must contain a JSON object");
  }
  return value;
}

Json target(const std::string & device_id, const std::string & component_id,
  const std::string & capability_id)
{
  if (device_id.empty() || component_id.empty()) {
    throw std::invalid_argument("legacy conversion requires device_id and component_id");
  }
  return {{"device_id", device_id}, {"component_id", component_id},
    {"capability_id", capability_id}};
}
}

std::string schema_for_message_type(const std::string & message_type)
{
  if (message_type == kMotionMessageType) {return "motion/command.schema.json";}
  if (message_type == kTaskMessageType) {return "tasks/command.schema.json";}
  if (message_type == kMediaMessageType) {return "media/session-command.schema.json";}
  if (message_type == kCancelMessageType) {return "operations/cancel.schema.json";}
  if (message_type == "uran.operations.state.v1") {return "operations/state.schema.json";}
  return {};
}

Json legacy_motion_to_contract(const uran_msgs::msg::UnifiedMoveCmd & message,
  const std::string & default_device_id, const std::string & component_id,
  std::uint64_t sequence, std::uint64_t validity_ns, std::uint64_t received_at_ns)
{
  const auto device = message.device_id.empty() ? default_device_id : message.device_id;
  const auto issued_at = message.timestamp_ns == 0 ? received_at_ns : message.timestamp_ns;
  if (issued_at == 0) {throw std::invalid_argument("legacy motion timestamp and receive time are both zero");}
  Json payload;
  std::string capability;
  if (!message.action.empty()) {
    static const std::set<std::string> allowed{
      "stop", "emergency_stop", "stand", "sit", "takeoff", "land", "return_home"};
    if (allowed.count(message.action) == 0) {
      throw std::invalid_argument("legacy motion action is not in the common contract: " + message.action);
    }
    payload = {{"kind", "safety_action"}, {"action", message.action}};
    capability = "motion.base.safety";
  } else {
    payload = {{"kind", "base_velocity"}, {"frame_id", "base_link"},
      {"linear_mps", {{"x", message.linear_vel_x}, {"y", message.linear_vel_y},
        {"z", message.linear_vel_z}}},
      {"angular_rps", {{"x", 0.0}, {"y", 0.0}, {"z", message.angular_vel_z}}},
      {"posture_rad", {{"roll", message.target_roll}, {"pitch", message.target_pitch}}}};
    if (std::isfinite(message.target_yaw)) {payload["posture_rad"]["yaw"] = message.target_yaw;}
    capability = "motion.base.velocity";
  }
  const auto extra = parse_object(message.extra_json, "extra_json");
  Json metadata = Json::object();
  if (extra.contains("motion_id")) {metadata["motion_id"] = extra.at("motion_id");}
  if (extra.contains("source_pkg")) {metadata["source"] = extra.at("source_pkg");}
  auto legacy_extra = extra;
  legacy_extra.erase("motion_id");
  legacy_extra.erase("source_pkg");
  if (!legacy_extra.empty()) {metadata["legacy_extra"] = legacy_extra;}
  Json result{{"schema_version", "1.0"},
    {"operation_id", operation_id("motion", issued_at, sequence)},
    {"target", target(device, component_id, capability)},
    {"controller", message.controller.empty() ? "legacy" : message.controller},
    {"sequence", sequence}, {"issued_at_ns", issued_at},
    {"valid_until_ns", checked_add(issued_at, validity_ns)}, {"payload", payload}};
  if (!metadata.empty()) {result["metadata"] = metadata;}
  return result;
}

Json legacy_task_to_contract(const uran_msgs::msg::TaskCtrlCmd & message,
  const std::string & device_id, const std::string & component_id, std::uint64_t sequence)
{
  return {{"schema_version", "1.0"},
    {"operation_id", operation_id("task", message.timestamp_ns, sequence)},
    {"task_id", message.task_id}, {"target", target(device_id, component_id, "tasks.command")},
    {"action", message.action}, {"task_type", message.task_type},
    {"parameters", parse_object(message.task_params_json, "task_params_json")},
    {"timestamp_ns", message.timestamp_ns}};
}

Json legacy_media_to_contract(const uran_msgs::msg::MediaCtrlCmd & message,
  const std::string & device_id, const std::string & component_id, std::uint64_t sequence)
{
  const auto session = message.channel_id.empty() ? "legacy-default" : message.channel_id;
  Json result{{"schema_version", "1.0"},
    {"operation_id", operation_id("media", message.timestamp_ns, sequence)},
    {"target", target(device_id, component_id, "media.session")}, {"session_id", session},
    {"action", message.action}, {"protocol", message.protocol},
    {"channel_id", message.channel_id}, {"timestamp_ns", message.timestamp_ns}};
  const auto signal = parse_object(message.signal_json, "signal_json");
  if (!signal.empty()) {result["signal"] = signal;}
  return result;
}

uran_plugin_api::DownlinkEnvelope to_api_envelope(
  const uran_ros2_msgs::msg::ContractEnvelope & message)
{
  return {message.message_type, message.payload_json, message.correlation_id,
    {"external.ros2", "external", "ros2"},
    {message.device_id, message.component_id, message.instance_id, message.capability_id}, 0};
}

uran_ros2_msgs::msg::OperationEvent to_ros_event(const uran_interfaces::OperationEvent & event)
{
  uran_ros2_msgs::msg::OperationEvent message;
  message.operation_id = event.operation_id;
  message.device_id = event.target.device_id;
  message.component_id = event.target.component_id;
  message.instance_id = event.target.instance_id;
  message.capability_id = event.target.capability_id;
  message.sequence = event.sequence;
  message.status = uran_interfaces::to_string(event.phase);
  message.timestamp_ns = event.timestamp_ns;
  message.progress = event.progress;
  message.has_progress = event.has_progress;
  message.cancellable = event.cancellable;
  message.code = event.code;
  message.message = event.message;
  message.result_json = event.result.is_null() ? "" : event.result.dump();
  return message;
}

uran_interfaces::OperationEvent from_ros_event(const uran_ros2_msgs::msg::OperationEvent & message)
{
  uran_interfaces::OperationEvent event;
  event.operation_id = message.operation_id;
  event.target = {message.device_id, message.component_id, message.instance_id, message.capability_id};
  event.sequence = message.sequence;
  event.phase = uran_interfaces::operation_phase_from_string(message.status);
  event.timestamp_ns = message.timestamp_ns;
  event.progress = message.progress;
  event.has_progress = message.has_progress;
  event.cancellable = message.cancellable;
  event.code = message.code;
  event.message = message.message;
  if (!message.result_json.empty()) {event.result = Json::parse(message.result_json);}
  return event;
}
}
