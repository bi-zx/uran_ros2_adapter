#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "uran_plugin_api/message_plugin.hpp"
#include "uran_plugin_api/protocol_plugin.hpp"

namespace uran_ros2_test_plugin
{
class LoopbackProtocol final : public uran_plugin_api::ProtocolPlugin
{
public:
  uran_plugin_api::PluginDescriptor descriptor() const override;
  uran_plugin_api::Status configure(const uran_plugin_api::PluginContext &, const YAML::Node &) override;
  uran_plugin_api::Status activate() override;
  uran_plugin_api::Status deactivate() override;
  uran_plugin_api::Status cleanup() override;
  uran_plugin_api::PluginHealth health() const override;
  uran_plugin_api::PluginCapabilities capabilities() const override;
  uran_plugin_api::Status connect() override;
  uran_plugin_api::Status disconnect() override;
  uran_plugin_api::Status publish(const uran_plugin_api::UplinkEnvelope &) override;
  void set_downlink_callback(uran_plugin_api::DownlinkCallback) override;

private:
  mutable std::mutex mutex_;
  std::shared_ptr<uran_plugin_api::PlatformServices> platform_;
  uran_plugin_api::DownlinkCallback callback_;
  bool configured_{false};
  bool active_{false};
};

class ContractSink final : public uran_plugin_api::MessagePlugin
{
public:
  uran_plugin_api::PluginDescriptor descriptor() const override;
  uran_plugin_api::Status configure(const uran_plugin_api::PluginContext &, const YAML::Node &) override;
  uran_plugin_api::Status activate() override;
  uran_plugin_api::Status deactivate() override;
  uran_plugin_api::Status cleanup() override;
  uran_plugin_api::PluginHealth health() const override;
  uran_plugin_api::PluginCapabilities capabilities() const override;
  std::vector<std::string> message_types() const override;
  uran_plugin_api::Status handle(const uran_plugin_api::DownlinkEnvelope &) override;

private:
  std::shared_ptr<uran_plugin_api::PlatformServices> platform_;
  std::shared_ptr<uran_plugin_api::MessageAccess> messages_;
  bool configured_{false};
  bool active_{false};
};
}
