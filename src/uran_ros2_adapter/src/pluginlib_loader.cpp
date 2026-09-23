#include "uran_ros2_adapter/pluginlib_loader.hpp"

#include <algorithm>
#include <stdexcept>

#include <pluginlib/class_loader.hpp>

#include "uran_platform_native/native_plugin_loader.hpp"

namespace uran_ros2_adapter
{
struct PluginlibLoader::Impl
{
  std::shared_ptr<pluginlib::ClassLoader<uran_plugin_api::Plugin>> loader{
    std::make_shared<pluginlib::ClassLoader<uran_plugin_api::Plugin>>(
      "uran_ros2_adapter", "uran_plugin_api::Plugin")};
  std::shared_ptr<uran_platform_native::NativePluginLoader> native{
    std::make_shared<uran_platform_native::NativePluginLoader>()};
};

PluginlibLoader::PluginlibLoader() : impl_(std::make_unique<Impl>()) {}
PluginlibLoader::~PluginlibLoader() = default;

uran_plugin_api::Status PluginlibLoader::add_native_manifest(
  const std::string & path, bool replace)
{
  return impl_->native->add_manifest(path, replace);
}

std::vector<std::string> PluginlibLoader::available_classes() const
{
  auto classes = impl_->native->available_classes();
  const auto ros_classes = impl_->loader->getDeclaredClasses();
  classes.insert(classes.end(), ros_classes.begin(), ros_classes.end());
  std::sort(classes.begin(), classes.end());
  classes.erase(std::unique(classes.begin(), classes.end()), classes.end());
  return classes;
}

std::shared_ptr<uran_plugin_api::Plugin> PluginlibLoader::create(const std::string & name)
{
  const auto native_classes = impl_->native->available_classes();
  if (std::find(native_classes.begin(), native_classes.end(), name) != native_classes.end()) {
    return impl_->native->create(name);
  }
  const auto ros_classes = impl_->loader->getDeclaredClasses();
  if (std::find(ros_classes.begin(), ros_classes.end(), name) == ros_classes.end()) {
    throw std::runtime_error("plugin class not found: " + name);
  }
  struct Owner
  {
    std::shared_ptr<pluginlib::ClassLoader<uran_plugin_api::Plugin>> loader;
    std::shared_ptr<uran_plugin_api::Plugin> plugin;
  };
  auto owner = std::make_shared<Owner>();
  owner->loader = impl_->loader;
  owner->plugin = impl_->loader->createSharedInstance(name);
  return std::shared_ptr<uran_plugin_api::Plugin>(owner, owner->plugin.get());
}
}
