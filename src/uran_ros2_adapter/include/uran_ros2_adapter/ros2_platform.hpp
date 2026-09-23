#pragma once

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "uran_platform_native/native_platform.hpp"

namespace uran_ros2_adapter
{
class Ros2Platform final : public uran_platform_native::NativePlatform
{
public:
  Ros2Platform(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);
  ~Ros2Platform() override = default;

  void log(uran_plugin_api::LogLevel level, const std::string & source,
    const std::string & message) override;
  std::chrono::system_clock::time_point system_now() const override;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};
}
