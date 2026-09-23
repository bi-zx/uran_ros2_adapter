#pragma once

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>
#include <uran_msgs/msg/media_ctrl_cmd.hpp>
#include <uran_msgs/msg/task_ctrl_cmd.hpp>
#include <uran_msgs/msg/unified_move_cmd.hpp>
#include <uran_ros2_msgs/msg/contract_envelope.hpp>
#include <uran_ros2_msgs/msg/operation_event.hpp>

#include "uran_interfaces/contracts.hpp"
#include "uran_plugin_api/protocol_plugin.hpp"

namespace uran_ros2_adapter
{
inline constexpr const char * kMotionMessageType = "uran.motion.command.v1";
inline constexpr const char * kTaskMessageType = "uran.tasks.command.v1";
inline constexpr const char * kMediaMessageType = "uran.media.session-command.v1";
inline constexpr const char * kCancelMessageType = "uran.operations.cancel.v1";

std::string schema_for_message_type(const std::string & message_type);

nlohmann::json legacy_motion_to_contract(const uran_msgs::msg::UnifiedMoveCmd & message,
  const std::string & default_device_id, const std::string & component_id,
  std::uint64_t sequence, std::uint64_t validity_ns, std::uint64_t received_at_ns);
nlohmann::json legacy_task_to_contract(const uran_msgs::msg::TaskCtrlCmd & message,
  const std::string & device_id, const std::string & component_id,
  std::uint64_t sequence);
nlohmann::json legacy_media_to_contract(const uran_msgs::msg::MediaCtrlCmd & message,
  const std::string & device_id, const std::string & component_id,
  std::uint64_t sequence);

uran_plugin_api::DownlinkEnvelope to_api_envelope(
  const uran_ros2_msgs::msg::ContractEnvelope & message);
uran_ros2_msgs::msg::OperationEvent to_ros_event(const uran_interfaces::OperationEvent & event);
uran_interfaces::OperationEvent from_ros_event(const uran_ros2_msgs::msg::OperationEvent & event);
}
