#include "uran_ros2_test_plugin/test_plugins.hpp"

#include <algorithm>
#include <chrono>

#include <nlohmann/json.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "uran_plugin_api/plugin_loader.hpp"

namespace uran_ros2_test_plugin
{
uran_plugin_api::PluginDescriptor LoopbackProtocol::descriptor() const
{
  return {"protocol", "1.0.0", uran_plugin_api::kPluginApiVersion,
    "uran_ros2_test_plugin/LoopbackProtocol"};
}

uran_plugin_api::Status LoopbackProtocol::configure(
  const uran_plugin_api::PluginContext & context, const YAML::Node &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  platform_ = context.platform(); configured_ = true;
  return uran_plugin_api::Status::success();
}

uran_plugin_api::Status LoopbackProtocol::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {return uran_plugin_api::Status::failure("not configured");}
  active_ = true;
  return uran_plugin_api::Status::success();
}

uran_plugin_api::Status LoopbackProtocol::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_); active_ = false;
  return uran_plugin_api::Status::success();
}

uran_plugin_api::Status LoopbackProtocol::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {return uran_plugin_api::Status::failure("still active");}
  configured_ = false; callback_ = nullptr; platform_.reset();
  return uran_plugin_api::Status::success();
}

uran_plugin_api::PluginHealth LoopbackProtocol::health() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {active_ ? uran_plugin_api::PluginHealth::Level::healthy :
    uran_plugin_api::PluginHealth::Level::stopped, active_ ? "active" : "stopped",
    platform_ ? platform_->system_now() : std::chrono::system_clock::time_point{}, 0};
}

uran_plugin_api::PluginCapabilities LoopbackProtocol::capabilities() const
{
  return {{"protocol.test.loopback"}, {}, {}};
}

uran_plugin_api::Status LoopbackProtocol::connect()
{
  return activate();
}

uran_plugin_api::Status LoopbackProtocol::disconnect()
{
  return deactivate();
}

uran_plugin_api::Status LoopbackProtocol::publish(const uran_plugin_api::UplinkEnvelope & message)
{
  uran_plugin_api::DownlinkCallback callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {return uran_plugin_api::Status::failure("not active");}
    callback = callback_;
  }
  if (callback) {callback({message.message_type, message.payload, message.correlation_id, {}, {}, 0});}
  return uran_plugin_api::Status::success();
}

void LoopbackProtocol::set_downlink_callback(uran_plugin_api::DownlinkCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_); callback_ = std::move(callback);
}

uran_plugin_api::PluginDescriptor ContractSink::descriptor() const
{
  return {"message", "1.0.0", uran_plugin_api::kPluginApiVersion,
    "uran_ros2_test_plugin/ContractSink"};
}

uran_plugin_api::Status ContractSink::configure(
  const uran_plugin_api::PluginContext & context, const YAML::Node &)
{
  platform_ = context.platform(); messages_ = context.messages(); configured_ = true;
  return uran_plugin_api::Status::success();
}

uran_plugin_api::Status ContractSink::activate()
{
  if (!configured_) {return uran_plugin_api::Status::failure("not configured");}
  active_ = true; return uran_plugin_api::Status::success();
}

uran_plugin_api::Status ContractSink::deactivate()
{
  active_ = false; return uran_plugin_api::Status::success();
}

uran_plugin_api::Status ContractSink::cleanup()
{
  if (active_) {return uran_plugin_api::Status::failure("still active");}
  configured_ = false; messages_.reset(); platform_.reset();
  return uran_plugin_api::Status::success();
}

uran_plugin_api::PluginHealth ContractSink::health() const
{
  return {active_ ? uran_plugin_api::PluginHealth::Level::healthy :
    uran_plugin_api::PluginHealth::Level::stopped, active_ ? "active" : "stopped",
    platform_ ? platform_->system_now() : std::chrono::system_clock::time_point{}, 0};
}

uran_plugin_api::PluginCapabilities ContractSink::capabilities() const
{
  return {{"motion.base.velocity", "motion.base.posture", "motion.base.safety",
    "tasks.waypoint", "tasks.outdoor", "tasks.lifecycle",
    "media.session", "media.webrtc", "media.rtsp"}, {}, {}};
}

std::vector<std::string> ContractSink::message_types() const
{
  return {"uran.motion.command.v1", "uran.tasks.command.v1",
    "uran.media.session-command.v1", "uran.operations.cancel.v1"};
}

uran_plugin_api::Status ContractSink::handle(const uran_plugin_api::DownlinkEnvelope & message)
{
  if (!active_ || !messages_ || !platform_) {
    return uran_plugin_api::Status::failure("not active");
  }
  const auto request = nlohmann::json::parse(message.payload);
  const auto operation_id = request.at("operation_id").get<std::string>();
  const auto timestamp_ns = static_cast<std::uint64_t>(std::max<std::int64_t>(0,
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      platform_->system_now().time_since_epoch()).count()));
  nlohmann::json event{{"schema_version", "1.0"}, {"operation_id", operation_id},
    {"target", request.at("target")}, {"sequence", 1}, {"status", "accepted"},
    {"timestamp_ns", timestamp_ns}, {"payload", nlohmann::json::object()}};
  auto result = messages_->publish(
    {"uran.operations.state.v1", event.dump(), operation_id, true});
  if (!result.ok) {return result;}
  event["sequence"] = 2;
  event["status"] = "succeeded";
  event["timestamp_ns"] = timestamp_ns + 1;
  event["payload"] = {{"test_sink", true}};
  result = messages_->publish(
    {"uran.operations.state.v1", event.dump(), operation_id, true});
  return result.ok ? uran_plugin_api::Status::success("test sink completed") : result;
}
}

PLUGINLIB_EXPORT_CLASS(uran_ros2_test_plugin::LoopbackProtocol, uran_plugin_api::Plugin)
PLUGINLIB_EXPORT_CLASS(uran_ros2_test_plugin::ContractSink, uran_plugin_api::Plugin)
