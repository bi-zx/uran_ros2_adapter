#include "uran_ros2_adapter/ros2_platform.hpp"

#include <rclcpp/logging.hpp>

namespace uran_ros2_adapter
{
Ros2Platform::Ros2Platform(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: logger_(std::move(logger)), clock_(std::move(clock)) {}

void Ros2Platform::log(uran_plugin_api::LogLevel level, const std::string & source,
  const std::string & message)
{
  const auto text = source + ": " + message;
  switch (level) {
    case uran_plugin_api::LogLevel::debug: RCLCPP_DEBUG(logger_, "%s", text.c_str()); break;
    case uran_plugin_api::LogLevel::info: RCLCPP_INFO(logger_, "%s", text.c_str()); break;
    case uran_plugin_api::LogLevel::warning: RCLCPP_WARN(logger_, "%s", text.c_str()); break;
    case uran_plugin_api::LogLevel::error: RCLCPP_ERROR(logger_, "%s", text.c_str()); break;
  }
}

std::chrono::system_clock::time_point Ros2Platform::system_now() const
{
  if (clock_) {
    return std::chrono::system_clock::time_point(std::chrono::nanoseconds(clock_->now().nanoseconds()));
  }
  return NativePlatform::system_now();
}
}
