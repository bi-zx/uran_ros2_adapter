#pragma once
#include <memory>
#include <string>
#include <vector>

#include "uran_plugin_api/plugin_loader.hpp"

namespace uran_ros2_adapter
{
class PluginlibLoader final : public uran_plugin_api::PluginLoader
{
public:
  PluginlibLoader();
  ~PluginlibLoader() override;
  uran_plugin_api::Status add_native_manifest(const std::string & path, bool replace = false);
  std::vector<std::string> available_classes() const override;
  std::shared_ptr<uran_plugin_api::Plugin> create(const std::string & name) override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
