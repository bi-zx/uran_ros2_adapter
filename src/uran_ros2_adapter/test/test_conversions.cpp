#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "uran_interfaces/contracts.hpp"
#include "uran_ros2_adapter/conversions.hpp"

TEST(Conversions, legacy_motion_targets_configured_component)
{
  uran_msgs::msg::UnifiedMoveCmd message;
  message.device_id = "robot_001";
  message.controller = "field";
  message.timestamp_ns = 1000;
  message.linear_vel_x = 0.25;
  message.angular_vel_z = -0.5;
  message.extra_json = "{}";
  const auto document = uran_ros2_adapter::legacy_motion_to_contract(
    message, "fallback", "base", 7, 500, 1000);
  uran_interfaces::ContractValidator validator(uran_interfaces::default_schema_directory());
  validator.validate_or_throw("motion/command.schema.json", document);
  EXPECT_EQ(document.at("target").at("component_id"), "base");
  EXPECT_EQ(document.at("sequence"), 7);
  EXPECT_EQ(document.at("valid_until_ns"), 1500);
}

TEST(Conversions, legacy_motion_rejects_unknown_action)
{
  uran_msgs::msg::UnifiedMoveCmd message;
  message.device_id = "robot_001";
  message.controller = "cloud";
  message.timestamp_ns = 1000;
  message.action = "vendor_dance";
  EXPECT_THROW(
    uran_ros2_adapter::legacy_motion_to_contract(message, "fallback", "base", 1, 500, 1000),
    std::invalid_argument);
}

TEST(Conversions, legacy_task_rejects_malformed_json)
{
  uran_msgs::msg::TaskCtrlCmd message;
  message.task_id = "task-1";
  message.task_type = "waypoint";
  message.action = "start";
  message.timestamp_ns = 1000;
  message.task_params_json = "not-json";
  EXPECT_THROW(
    uran_ros2_adapter::legacy_task_to_contract(message, "robot_001", "tasks", 1),
    std::invalid_argument);
}

TEST(Conversions, operation_event_round_trip)
{
  uran_interfaces::OperationEvent expected;
  expected.operation_id = "op-1";
  expected.target = {"robot_001", "arm_left", "arm_plugin", {}};
  expected.sequence = std::numeric_limits<std::uint64_t>::max();
  expected.phase = uran_interfaces::OperationPhase::cancelled;
  expected.timestamp_ns = 123;
  expected.code = "requested";
  const auto actual = uran_ros2_adapter::from_ros_event(uran_ros2_adapter::to_ros_event(expected));
  EXPECT_EQ(actual.operation_id, expected.operation_id);
  EXPECT_EQ(actual.target, expected.target);
  EXPECT_EQ(actual.sequence, expected.sequence);
  EXPECT_EQ(actual.phase, expected.phase);
}

TEST(Conversions, legacy_zero_timestamp_uses_receive_time_and_maps_metadata)
{
  uran_msgs::msg::UnifiedMoveCmd message;
  message.device_id = "robot_001";
  message.controller = "auto";
  message.extra_json = R"({"motion_id":123,"source_pkg":"autotask","other":true})";
  const auto document = uran_ros2_adapter::legacy_motion_to_contract(
    message, "fallback", "base", 8, 500, 9000);
  EXPECT_EQ(document.at("issued_at_ns"), 9000);
  EXPECT_EQ(document.at("target").at("capability_id"), "motion.base.velocity");
  EXPECT_EQ(document.at("metadata").at("motion_id"), 123);
  EXPECT_EQ(document.at("metadata").at("source"), "autotask");
  EXPECT_TRUE(document.at("metadata").at("legacy_extra").at("other"));
}
