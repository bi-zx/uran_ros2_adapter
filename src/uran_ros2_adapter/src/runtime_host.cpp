#include "uran_ros2_adapter/runtime_host.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <utility>

#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <uran_msgs/msg/heartbeat_status.hpp>
#include <uran_msgs/msg/media_ctrl_cmd.hpp>
#include <uran_msgs/msg/state_field.hpp>
#include <uran_msgs/msg/state_snapshot.hpp>
#include <uran_msgs/msg/task_ctrl_cmd.hpp>
#include <uran_msgs/msg/unified_move_cmd.hpp>
#include <uran_msgs/msg/uplink_payload.hpp>
#include <uran_ros2_msgs/msg/contract_envelope.hpp>
#include <uran_ros2_msgs/msg/operation_event.hpp>
#include <uran_ros2_msgs/srv/runtime_admin.hpp>
#include <uran_srvs/srv/configure_state_report.hpp>
#include <uran_srvs/srv/connect_protocol.hpp>
#include <uran_srvs/srv/get_network_status.hpp>
#include <uran_srvs/srv/get_state_field.hpp>
#include <uran_srvs/srv/get_task_status.hpp>
#include <uran_srvs/srv/list_sensors.hpp>
#include <uran_srvs/srv/reload_config.hpp>
#include <uran_srvs/srv/set_state_field.hpp>
#include <uran_srvs/srv/switch_move_plugin.hpp>
#include <uran_srvs/srv/trigger_state_report.hpp>
#include <yaml-cpp/yaml.h>

#include "uran_core_framework/runtime.hpp"
#include "uran_platform_native/file_state_storage.hpp"
#include "uran_platform_native/native_plugin_loader.hpp"
#include "uran_ros2_adapter/conversions.hpp"
#include "uran_ros2_adapter/pluginlib_loader.hpp"
#include "uran_ros2_adapter/ros2_platform.hpp"

namespace uran_ros2_adapter
{
namespace
{
using Json = nlohmann::json;
using Text = std_msgs::msg::String;
using LifecycleTextPublisher = rclcpp_lifecycle::LifecyclePublisher<Text>;
using EventPublisher = rclcpp_lifecycle::LifecyclePublisher<uran_ros2_msgs::msg::OperationEvent>;

std::uint64_t clock_ns(const Ros2RuntimeHost & node)
{
  return static_cast<std::uint64_t>(std::max<std::int64_t>(0, node.now().nanoseconds()));
}

template<typename Publisher, typename Message>
void publish_if_active(const std::shared_ptr<Publisher> & publisher, Message message)
{
  if (publisher && publisher->is_activated()) {publisher->publish(std::move(message));}
}

uran_ros2_msgs::msg::ContractEnvelope contract_envelope(
  const std::string & schema, const std::string & type, const Json & document)
{
  uran_ros2_msgs::msg::ContractEnvelope message;
  message.schema_name = schema;
  message.message_type = type;
  message.payload_json = document.dump();
  message.correlation_id = document.value("operation_id", "");
  if (document.contains("target")) {
    const auto & target = document.at("target");
    message.device_id = target.value("device_id", "");
    message.component_id = target.value("component_id", "");
    message.instance_id = target.value("instance_id", "");
    message.capability_id = target.value("capability_id", "");
  }
  return message;
}
}

struct Ros2RuntimeHost::Impl
{
  explicit Impl(Ros2RuntimeHost & owner) : node(owner) {}
  Ros2RuntimeHost & node;
  std::shared_ptr<Ros2Platform> platform;
  std::unique_ptr<uran_core_framework::UranRuntime> runtime;
  std::unique_ptr<uran_interfaces::ContractValidator> validator;
  YAML::Node assembly;
  std::string assembly_file;
  std::atomic<std::uint64_t> sequence{1};
  std::mutex pending_mutex;
  std::map<std::string, uran_interfaces::ComponentTarget> pending_operations;
  std::string default_device;
  std::string motion_component;
  std::string task_component;
  std::string media_component;
  std::uint64_t legacy_validity_ns{500000000};
  std::uint32_t report_interval_ms{1000};
  std::string report_protocol;
  bool report_on_change{false};

  LifecycleTextPublisher::SharedPtr snapshot;
  LifecycleTextPublisher::SharedPtr heartbeat;
  LifecycleTextPublisher::SharedPtr downlink;
  LifecycleTextPublisher::SharedPtr management_reply;
  EventPublisher::SharedPtr events;
  rclcpp_lifecycle::LifecyclePublisher<uran_msgs::msg::StateSnapshot>::SharedPtr legacy_snapshot;
  rclcpp_lifecycle::LifecyclePublisher<uran_msgs::msg::HeartbeatStatus>::SharedPtr legacy_heartbeat;

  rclcpp::Subscription<Text>::SharedPtr uplink;
  rclcpp::Subscription<Text>::SharedPtr state_write;
  rclcpp::Subscription<Text>::SharedPtr management;
  rclcpp::Subscription<uran_ros2_msgs::msg::ContractEnvelope>::SharedPtr dispatch;
  rclcpp::Subscription<uran_ros2_msgs::msg::OperationEvent>::SharedPtr event_input;
  rclcpp::Subscription<uran_msgs::msg::UnifiedMoveCmd>::SharedPtr legacy_motion;
  rclcpp::Subscription<uran_msgs::msg::TaskCtrlCmd>::SharedPtr legacy_task;
  rclcpp::Subscription<uran_msgs::msg::MediaCtrlCmd>::SharedPtr legacy_media;
  rclcpp::Subscription<uran_msgs::msg::StateField>::SharedPtr legacy_state;
  rclcpp::Subscription<uran_msgs::msg::UplinkPayload>::SharedPtr legacy_uplink;
  rclcpp::TimerBase::SharedPtr snapshot_timer;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger;
  rclcpp::Service<uran_ros2_msgs::srv::RuntimeAdmin>::SharedPtr admin;
  rclcpp::Service<uran_srvs::srv::GetStateField>::SharedPtr get_state;
  rclcpp::Service<uran_srvs::srv::SetStateField>::SharedPtr set_state;
  rclcpp::Service<uran_srvs::srv::ConnectProtocol>::SharedPtr connect_protocol;
  rclcpp::Service<uran_srvs::srv::GetNetworkStatus>::SharedPtr network_status;
  rclcpp::Service<uran_srvs::srv::TriggerStateReport>::SharedPtr legacy_trigger;
  rclcpp::Service<uran_srvs::srv::ConfigureStateReport>::SharedPtr configure_report;
  rclcpp::Service<uran_srvs::srv::SwitchMovePlugin>::SharedPtr switch_move;
  rclcpp::Service<uran_srvs::srv::ReloadConfig>::SharedPtr reload_config;
  rclcpp::Service<uran_srvs::srv::ListSensors>::SharedPtr list_sensors;
  rclcpp::Service<uran_srvs::srv::GetTaskStatus>::SharedPtr task_status;

  template<typename Function>
  void guarded(Function function)
  {
    try {function();}
    catch (const std::exception & exception) {RCLCPP_WARN(node.get_logger(), "%s", exception.what());}
  }

  void publish_text(const LifecycleTextPublisher::SharedPtr & publisher, const std::string & value)
  {
    Text message;
    message.data = value;
    publish_if_active(publisher, message);
  }

  void emit_rejection(const Json & contract, std::string message)
  {
    uran_interfaces::OperationEvent event;
    event.operation_id = contract.value("operation_id", "unknown");
    const auto target = contract.value("target", Json::object());
    event.target.device_id = target.is_object() ? target.value("device_id", "unknown") : "unknown";
    event.target.component_id = target.is_object() ? target.value("component_id", "unknown") : "unknown";
    event.target.instance_id = target.is_object() ? target.value("instance_id", "") : "";
    event.target.capability_id = target.is_object() ? target.value("capability_id", "") : "";
    if (event.target.device_id.empty()) {event.target.device_id = "unknown";}
    if (event.target.component_id.empty()) {event.target.component_id = "unknown";}
    event.sequence = 1;
    event.phase = uran_interfaces::OperationPhase::rejected;
    event.timestamp_ns = clock_ns(node);
    event.code = "bridge_rejected";
    event.message = std::move(message);
    publish_if_active(events, to_ros_event(event));
  }

  void handle_dispatch(const uran_ros2_msgs::msg::ContractEnvelope & message)
  {
    Json document;
    try {
      if (!runtime) {throw std::runtime_error("runtime is not active");}
      const auto expected = schema_for_message_type(message.message_type);
      if (expected.empty()) {throw std::invalid_argument("unknown contract message_type");}
      if (message.schema_name != expected) {throw std::invalid_argument("schema_name does not match message_type");}
      document = Json::parse(message.payload_json);
      validator->validate_or_throw(message.schema_name, document);
      if (document.contains("target")) {
        const auto & target = document.at("target");
        if (target.value("device_id", "") != message.device_id ||
          target.value("component_id", "") != message.component_id ||
          target.value("instance_id", "") != message.instance_id ||
          target.value("capability_id", "") != message.capability_id) {
          throw std::invalid_argument("ROS envelope target differs from contract target");
        }
      }
      if (message.correlation_id.empty() || document.at("operation_id") != message.correlation_id) {
        throw std::invalid_argument("correlation_id must equal contract operation_id");
      }
      {
        std::lock_guard<std::mutex> lock(pending_mutex);
        pending_operations[message.correlation_id] =
          uran_interfaces::ComponentTarget::from_json(document.at("target"));
      }
      const auto result = runtime->dispatch(to_api_envelope(message));
      if (!result.ok) {
        std::lock_guard<std::mutex> lock(pending_mutex);
        pending_operations.erase(message.correlation_id);
        throw std::runtime_error(result.message);
      }
    } catch (const std::exception & exception) {
      if (!document.is_object()) {document = Json::object();}
      if (!document.contains("operation_id")) {
        document["operation_id"] = message.correlation_id.empty() ? "unknown" : message.correlation_id;
      }
      if (!document.contains("target")) {
        document["target"] = {{"device_id", message.device_id.empty() ? "unknown" : message.device_id},
          {"component_id", message.component_id.empty() ? "unknown" : message.component_id}};
      }
      emit_rejection(document, exception.what());
    }
  }

  void handle_runtime_message(const uran_plugin_api::DownlinkEnvelope & message)
  {
    const auto parsed = Json::parse(message.payload, nullptr, false);
    if (message.message_type != "uran.operations.state.v1") {
      publish_text(downlink, Json{{"message_type", message.message_type},
        {"payload", parsed.is_discarded() ? Json(message.payload) : parsed},
        {"correlation_id", message.correlation_id},
        {"source", {{"instance_id", message.source.instance_id},
          {"device_id", message.source.device_id}, {"component_id", message.source.component_id}}}}.dump());
      return;
    }
    guarded([&] {
      if (!parsed.is_object()) {throw std::invalid_argument("operation state payload must be an object");}
      Json state = parsed;
      const auto operation_id = state.value("operation_id", message.correlation_id);
      if (operation_id.empty() || (!message.correlation_id.empty() && operation_id != message.correlation_id)) {
        throw std::invalid_argument("operation event correlation does not match operation_id");
      }
      uran_interfaces::ComponentTarget pending;
      bool found = false;
      {
        std::lock_guard<std::mutex> lock(pending_mutex);
        const auto item = pending_operations.find(operation_id);
        if (item != pending_operations.end()) {pending = item->second; found = true;}
      }
      if (!state.contains("target") || !state.at("target").is_object() ||
        state.at("target").value("device_id", "").empty() ||
        state.at("target").value("component_id", "").empty()) {
        if (!found) {throw std::invalid_argument("operation event has no known target");}
        state["target"] = pending.to_json();
      } else if (found && !(uran_interfaces::ComponentTarget::from_json(state.at("target")) == pending)) {
        throw std::invalid_argument("operation event target differs from dispatched operation");
      }
      state["schema_version"] = state.value("schema_version", "1.0");
      state["operation_id"] = operation_id;
      state["timestamp_ns"] = state.value("timestamp_ns", clock_ns(node));
      validator->validate_or_throw("operations/state.schema.json", state);
      auto event = uran_interfaces::OperationEvent::from_json(state);
      event.result = state.value("payload", state.value("result", Json()));
      if (state.contains("error") && state.at("error").is_object()) {
        event.code = state.at("error").value("code", event.code);
        event.message = state.at("error").value("message", event.message);
      }
      publish_if_active(events, to_ros_event(event));
      if (uran_interfaces::terminal(event.phase)) {
        std::lock_guard<std::mutex> lock(pending_mutex);
        pending_operations.erase(operation_id);
      }
    });
  }

  void publish_snapshot()
  {
    if (!runtime) {return;}
    const auto state = runtime->state_snapshot();
    publish_text(snapshot, runtime->state_snapshot_document());
    uran_msgs::msg::StateSnapshot legacy;
    legacy.msg_version = "1.0";
    legacy.timestamp_ns = clock_ns(node);
    legacy.device_id = default_device;
    legacy.fields_json = state;
    publish_if_active(legacy_snapshot, legacy);
  }

  void publish_heartbeat()
  {
    if (!runtime) {return;}
    const auto diagnostics = runtime->diagnostics();
    publish_text(heartbeat, diagnostics);
    uran_msgs::msg::HeartbeatStatus legacy;
    legacy.timestamp_ns = clock_ns(node);
    const auto parsed = Json::parse(diagnostics, nullptr, false);
    legacy.protocol = parsed.is_object() ? parsed.value("primary_protocol_instance", "") : "";
    legacy.last_sent_ts = legacy.timestamp_ns;
    legacy.success = parsed.is_object() && parsed.value("ready", false);
    publish_if_active(legacy_heartbeat, legacy);
  }

  Json administer(const std::string & operation, const std::string & instance, const Json & plugin)
  {
    if (!runtime) {return {{"ok", false}, {"message", "runtime is not active"}};}
    uran_plugin_api::Status result;
    Json extra;
    if (operation == "load") {result = runtime->load(YAML::Load(plugin.dump()));}
    else if (operation == "activate") {result = runtime->activate(instance);}
    else if (operation == "deactivate") {result = runtime->deactivate(instance);}
    else if (operation == "unload") {result = runtime->unload(instance);}
    else if (operation == "primary") {result = runtime->set_primary_protocol(instance);}
    else if (operation == "status") {
      result = uran_plugin_api::Status::success(); extra = Json::parse(runtime->diagnostics());
    } else {result = uran_plugin_api::Status::failure("unknown management operation");}
    Json reply{{"ok", result.ok}, {"message", result.message}};
    if (!extra.is_null()) {reply["result"] = extra;}
    return reply;
  }

  void reset_timers()
  {
    snapshot_timer = node.create_wall_timer(std::chrono::milliseconds(std::max<std::uint32_t>(100, report_interval_ms)),
      [this] {publish_snapshot();});
    heartbeat_timer = node.create_wall_timer(std::chrono::milliseconds(std::max<int64_t>(100,
      node.get_parameter("heartbeat_period_ms").as_int())), [this] {publish_heartbeat();});
  }

  void create_services()
  {
    trigger = node.create_service<std_srvs::srv::Trigger>("/uran/runtime/state_report",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        publish_snapshot(); response->success = true; response->message = "snapshot published";
      });
    admin = node.create_service<uran_ros2_msgs::srv::RuntimeAdmin>("/uran/runtime/admin",
      [this](std::shared_ptr<uran_ros2_msgs::srv::RuntimeAdmin::Request> request,
      std::shared_ptr<uran_ros2_msgs::srv::RuntimeAdmin::Response> response) {
        try {
          Json plugin = request->plugin_json.empty() ? Json::object() : Json::parse(request->plugin_json);
          const auto reply = administer(request->operation, request->instance_id, plugin);
          response->success = reply.at("ok"); response->message = reply.at("message");
          response->result_json = reply.contains("result") ? reply.at("result").dump() : "";
        } catch (const std::exception & exception) {response->success = false; response->message = exception.what();}
      });
    get_state = node.create_service<uran_srvs::srv::GetStateField>("/uran/core/state/get",
      [this](std::shared_ptr<uran_srvs::srv::GetStateField::Request> request,
      std::shared_ptr<uran_srvs::srv::GetStateField::Response> response) {
        try {
          const auto state = Json::parse(runtime->state_snapshot());
          Json selected = Json::object();
          for (const auto & field : request->field_names) {if (state.contains(field)) {selected[field] = state.at(field);}}
          response->fields_json = selected.dump(); response->success = true;
        } catch (...) {response->success = false; response->fields_json = "{}";}
      });
    set_state = node.create_service<uran_srvs::srv::SetStateField>("/uran/core/state/set",
      [this](std::shared_ptr<uran_srvs::srv::SetStateField::Request> request,
      std::shared_ptr<uran_srvs::srv::SetStateField::Response> response) {
        const auto result = runtime->write_state(request->field_name, request->value_json,
          "external.ros2.legacy", request->persistent);
        response->success = result.ok; response->message = result.message;
      });
    connect_protocol = node.create_service<uran_srvs::srv::ConnectProtocol>("/uran/core/network/connect",
      [this](std::shared_ptr<uran_srvs::srv::ConnectProtocol::Request> request,
      std::shared_ptr<uran_srvs::srv::ConnectProtocol::Response> response) {
        const auto result = request->action == "connect" ? runtime->activate(request->protocol) :
          request->action == "disconnect" ? runtime->deactivate(request->protocol) :
          uran_plugin_api::Status::failure("action must be connect or disconnect");
        response->success = result.ok; response->message = result.message;
      });
    network_status = node.create_service<uran_srvs::srv::GetNetworkStatus>("/uran/core/network/status",
      [this](const std::shared_ptr<uran_srvs::srv::GetNetworkStatus::Request>,
      std::shared_ptr<uran_srvs::srv::GetNetworkStatus::Response> response) {
        response->protocol_table_json = runtime->diagnostics();
      });
    legacy_trigger = node.create_service<uran_srvs::srv::TriggerStateReport>("/uran/core/state_report/trigger",
      [this](const std::shared_ptr<uran_srvs::srv::TriggerStateReport::Request>,
      std::shared_ptr<uran_srvs::srv::TriggerStateReport::Response> response) {
        publish_snapshot(); response->success = true; response->message = "snapshot published";
      });
    configure_report = node.create_service<uran_srvs::srv::ConfigureStateReport>("/uran/core/state_report/configure",
      [this](std::shared_ptr<uran_srvs::srv::ConfigureStateReport::Request> request,
      std::shared_ptr<uran_srvs::srv::ConfigureStateReport::Response> response) {
        if (request->interval_ms) {report_interval_ms = request->interval_ms; reset_timers();}
        if (!request->protocol.empty()) {report_protocol = request->protocol;}
        report_on_change = request->report_on_change;
        response->success = true; response->message = "report configuration updated";
        response->current_interval_ms = report_interval_ms; response->current_protocol = report_protocol;
      });
    switch_move = node.create_service<uran_srvs::srv::SwitchMovePlugin>("/uran/move/switch_plugin",
      [](const std::shared_ptr<uran_srvs::srv::SwitchMovePlugin::Request>,
      std::shared_ptr<uran_srvs::srv::SwitchMovePlugin::Response> response) {
        response->success = false;
        response->message = "global plugin switching is unsupported; address a component instance";
        response->current_plugin = "";
      });
    reload_config = node.create_service<uran_srvs::srv::ReloadConfig>("/uran/core/config/reload",
      [](const std::shared_ptr<uran_srvs::srv::ReloadConfig::Request>,
      std::shared_ptr<uran_srvs::srv::ReloadConfig::Response> response) {
        response->success = false;
        response->message = "reload requires a lifecycle deactivate/configure/activate transition";
      });
    list_sensors = node.create_service<uran_srvs::srv::ListSensors>("/uran/core/sensors/list",
      [](const std::shared_ptr<uran_srvs::srv::ListSensors::Request>,
      std::shared_ptr<uran_srvs::srv::ListSensors::Response> response) {
        response->sensors_json = "[]";
      });
    task_status = node.create_service<uran_srvs::srv::GetTaskStatus>("/uran/autotask/status",
      [this](std::shared_ptr<uran_srvs::srv::GetTaskStatus::Request> request,
      std::shared_ptr<uran_srvs::srv::GetTaskStatus::Response> response) {
        const auto state = Json::parse(runtime->state_snapshot());
        const auto key = request->task_id.empty() ? "tasks" : "tasks." + request->task_id;
        response->status_json = state.contains(key) ? state.at(key).dump() : "{}";
      });
  }

  void create_endpoints()
  {
    snapshot = node.create_publisher<Text>("/uran/runtime/state_snapshot", 10);
    heartbeat = node.create_publisher<Text>("/uran/runtime/heartbeat", 10);
    downlink = node.create_publisher<Text>("/uran/runtime/downlink", 10);
    management_reply = node.create_publisher<Text>("/uran/runtime/management/reply", 10);
    events = node.create_publisher<uran_ros2_msgs::msg::OperationEvent>("/uran/runtime/events", 20);
    legacy_snapshot = node.create_publisher<uran_msgs::msg::StateSnapshot>("/uran/core/state/broadcast", 10);
    legacy_heartbeat = node.create_publisher<uran_msgs::msg::HeartbeatStatus>("/uran/core/heartbeat/status", 10);

    dispatch = node.create_subscription<uran_ros2_msgs::msg::ContractEnvelope>(
      "/uran/runtime/dispatch", 20,
      [this](uran_ros2_msgs::msg::ContractEnvelope::SharedPtr message) {handle_dispatch(*message);});
    event_input = node.create_subscription<uran_ros2_msgs::msg::OperationEvent>(
      "/uran/runtime/events/input", 20,
      [this](uran_ros2_msgs::msg::OperationEvent::SharedPtr message) {
        guarded([&] {
          const auto event = from_ros_event(*message);
          validator->validate_or_throw("operations/state.schema.json", event.to_json());
          publish_if_active(events, *message);
        });
      });
    uplink = node.create_subscription<Text>("/uran/runtime/uplink", 10,
      [this](Text::SharedPtr message) {
      guarded([&] {
        const auto body = Json::parse(message->data);
        const auto payload = body.at("payload");
        const auto result = runtime->publish({body.at("message_type"),
          payload.is_string() ? payload.get<std::string>() : payload.dump(),
          body.value("correlation_id", ""), body.value("urgent", false)},
          body.value("preferred_protocol_instance", ""));
        if (!result.ok) {throw std::runtime_error(result.message);}
      });
    });
    state_write = node.create_subscription<Text>("/uran/runtime/state_write", 10,
      [this](Text::SharedPtr message) {
      guarded([&] {
        const auto body = Json::parse(message->data);
        const auto value = body.at("value");
        const auto result = runtime->write_state(body.at("field"),
          value.is_string() ? value.get<std::string>() : value.dump(), "external.ros2",
          body.value("persistent", false));
        if (!result.ok) {throw std::runtime_error(result.message);}
      });
    });
    management = node.create_subscription<Text>("/uran/runtime/management/request", 10,
      [this](Text::SharedPtr message) {
        Json reply;
        try {
          const auto body = Json::parse(message->data);
          reply["correlation_id"] = body.value("correlation_id", "");
          const auto result = administer(body.at("op"), body.value("instance_id", ""),
            body.value("plugin", Json::object()));
          reply.update(result);
        } catch (const std::exception & exception) {
          reply["ok"] = false; reply["message"] = exception.what();
        }
        publish_text(management_reply, reply.dump());
      });

    legacy_motion = node.create_subscription<uran_msgs::msg::UnifiedMoveCmd>(
      "/uran/core/downlink/move_cmd", 20,
      [this](uran_msgs::msg::UnifiedMoveCmd::SharedPtr message) {
        guarded([&] {
          const auto contract = legacy_motion_to_contract(*message, default_device, motion_component,
            sequence.fetch_add(1), legacy_validity_ns, clock_ns(node));
          handle_dispatch(contract_envelope("motion/command.schema.json", kMotionMessageType, contract));
        });
      });
    legacy_task = node.create_subscription<uran_msgs::msg::TaskCtrlCmd>(
      "/uran/core/downlink/task_ctrl", 20,
      [this](uran_msgs::msg::TaskCtrlCmd::SharedPtr message) {
        guarded([&] {
          const auto contract = legacy_task_to_contract(*message, default_device, task_component,
            sequence.fetch_add(1));
          handle_dispatch(contract_envelope("tasks/command.schema.json", kTaskMessageType, contract));
        });
      });
    legacy_media = node.create_subscription<uran_msgs::msg::MediaCtrlCmd>(
      "/uran/core/downlink/media_ctrl", 20,
      [this](uran_msgs::msg::MediaCtrlCmd::SharedPtr message) {
        guarded([&] {
          const auto contract = legacy_media_to_contract(*message, default_device, media_component,
            sequence.fetch_add(1));
          handle_dispatch(contract_envelope("media/session-command.schema.json", kMediaMessageType, contract));
        });
      });
    legacy_state = node.create_subscription<uran_msgs::msg::StateField>(
      "/uran/core/state/write", 10,
      [this](uran_msgs::msg::StateField::SharedPtr message) {
        guarded([&] {
          const auto result = runtime->write_state(message->field_name, message->value_json,
            message->source_pkg.empty() ? "external.ros2.legacy" : message->source_pkg,
            message->persistent);
          if (!result.ok) {throw std::runtime_error(result.message);}
          if (message->urgent || report_on_change) {publish_snapshot();}
        });
      });
    legacy_uplink = node.create_subscription<uran_msgs::msg::UplinkPayload>(
      "/uran/core/uplink/data", 10,
      [this](uran_msgs::msg::UplinkPayload::SharedPtr message) {
        guarded([&] {
          const auto result = runtime->publish({message->data_type, message->payload_json,
            "legacy-" + std::to_string(message->timestamp_ns), message->urgent}, message->preferred_protocol);
          if (!result.ok) {throw std::runtime_error(result.message);}
        });
      });
    create_services();
    reset_timers();
  }

  void activate_publishers()
  {
    snapshot->on_activate(); heartbeat->on_activate(); downlink->on_activate();
    management_reply->on_activate(); events->on_activate();
    legacy_snapshot->on_activate(); legacy_heartbeat->on_activate();
  }

  void deactivate_publishers()
  {
    legacy_heartbeat->on_deactivate(); legacy_snapshot->on_deactivate();
    events->on_deactivate(); management_reply->on_deactivate(); downlink->on_deactivate();
    heartbeat->on_deactivate(); snapshot->on_deactivate();
  }

  void reset_endpoints()
  {
    snapshot_timer.reset(); heartbeat_timer.reset();
    trigger.reset(); admin.reset(); get_state.reset(); set_state.reset();
    connect_protocol.reset(); network_status.reset(); legacy_trigger.reset();
    configure_report.reset(); switch_move.reset(); reload_config.reset();
    list_sensors.reset(); task_status.reset();
    uplink.reset(); state_write.reset(); management.reset(); dispatch.reset(); event_input.reset();
    legacy_motion.reset(); legacy_task.reset(); legacy_media.reset(); legacy_state.reset();
    legacy_uplink.reset();
    legacy_heartbeat.reset(); legacy_snapshot.reset(); events.reset(); management_reply.reset();
    downlink.reset(); heartbeat.reset(); snapshot.reset();
  }
};

Ros2RuntimeHost::Ros2RuntimeHost(const rclcpp::NodeOptions & options)
: LifecycleNode("uran_runtime", options), impl_(std::make_unique<Impl>(*this))
{
  declare_parameter<std::string>("plugin_config", "");
  declare_parameter<std::vector<std::string>>("plugin_manifests", std::vector<std::string>{});
  declare_parameter<std::string>("state_file", "");
  declare_parameter<std::string>("default_device_id", "robot_001");
  declare_parameter<std::string>("default_motion_component", "base");
  declare_parameter<std::string>("default_task_component", "tasks");
  declare_parameter<std::string>("default_media_component", "media");
  declare_parameter<int>("legacy_motion_validity_ms", 500);
  declare_parameter<int>("snapshot_period_ms", 1000);
  declare_parameter<int>("heartbeat_period_ms", 5000);
  declare_parameter<bool>("autostart", true);
}

Ros2RuntimeHost::~Ros2RuntimeHost()
{
  if (impl_->runtime) {impl_->runtime->set_unhandled_handler(nullptr); impl_->runtime->stop();}
  impl_->runtime.reset();
  if (impl_->platform) {impl_->platform->shutdown();}
}

Ros2RuntimeHost::CallbackReturn Ros2RuntimeHost::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    impl_->default_device = get_parameter("default_device_id").as_string();
    impl_->motion_component = get_parameter("default_motion_component").as_string();
    impl_->task_component = get_parameter("default_task_component").as_string();
    impl_->media_component = get_parameter("default_media_component").as_string();
    impl_->legacy_validity_ns = static_cast<std::uint64_t>(
      std::max<int64_t>(1, get_parameter("legacy_motion_validity_ms").as_int())) * 1000000ULL;
    impl_->report_interval_ms = static_cast<std::uint32_t>(
      std::max<int64_t>(100, get_parameter("snapshot_period_ms").as_int()));
    impl_->validator = std::make_unique<uran_interfaces::ContractValidator>(uran_interfaces::default_schema_directory());
    impl_->platform = std::make_shared<Ros2Platform>(get_logger(), get_clock());
    std::shared_ptr<uran_plugin_api::StateStorage> storage;
    const auto state_file = get_parameter("state_file").as_string();
    if (!state_file.empty()) {storage = std::make_shared<uran_platform_native::FileStateStorage>(state_file);}
    auto loader = std::make_shared<PluginlibLoader>();
    const auto config_file = get_parameter("plugin_config").as_string();
    impl_->assembly = config_file.empty() ? YAML::Load("plugins: []") : YAML::LoadFile(config_file);
    impl_->assembly_file = config_file;
    std::set<std::string> manifests;
    for (const auto & manifest : get_parameter("plugin_manifests").as_string_array()) {
      manifests.insert(manifest);
    }
    if (impl_->assembly["plugin_manifests"]) {
      const auto base = config_file.empty() ? std::filesystem::current_path() :
        std::filesystem::absolute(config_file).parent_path();
      for (const auto & manifest : impl_->assembly["plugin_manifests"]) {
        const auto path = std::filesystem::path(manifest.as<std::string>());
        manifests.insert((path.is_absolute() ? path : base / path).lexically_normal().string());
      }
    }
    for (const auto & manifest : manifests) {
      const auto added = loader->add_native_manifest(manifest);
      if (!added.ok) {throw std::runtime_error(added.message);}
    }
    impl_->runtime = std::make_unique<uran_core_framework::UranRuntime>(
      loader, impl_->platform, storage);
    impl_->create_endpoints();
    impl_->runtime->set_unhandled_handler(
      [this](const auto & message) {impl_->handle_runtime_message(message);});
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "configure failed: %s", exception.what());
    return CallbackReturn::FAILURE;
  }
}

Ros2RuntimeHost::CallbackReturn Ros2RuntimeHost::on_activate(const rclcpp_lifecycle::State &)
{
  impl_->activate_publishers();
  const auto result = impl_->assembly_file.empty() ? impl_->runtime->start(impl_->assembly) :
    impl_->runtime->start_file(impl_->assembly_file);
  if (!result.ok) {
    RCLCPP_ERROR(get_logger(), "Runtime start failed: %s", result.message.c_str());
    impl_->deactivate_publishers();
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

Ros2RuntimeHost::CallbackReturn Ros2RuntimeHost::on_deactivate(const rclcpp_lifecycle::State &)
{
  const auto result = impl_->runtime ? impl_->runtime->stop() : uran_plugin_api::Status::success();
  impl_->deactivate_publishers();
  return result.ok ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

Ros2RuntimeHost::CallbackReturn Ros2RuntimeHost::on_cleanup(const rclcpp_lifecycle::State &)
{
  impl_->runtime.reset();
  if (impl_->platform) {impl_->platform->shutdown();}
  impl_->platform.reset();
  impl_->validator.reset();
  impl_->assembly = YAML::Node();
  impl_->assembly_file.clear();
  {
    std::lock_guard<std::mutex> lock(impl_->pending_mutex);
    impl_->pending_operations.clear();
  }
  impl_->reset_endpoints();
  return CallbackReturn::SUCCESS;
}

Ros2RuntimeHost::CallbackReturn Ros2RuntimeHost::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (impl_->runtime) {impl_->runtime->stop();}
  return CallbackReturn::SUCCESS;
}
}
