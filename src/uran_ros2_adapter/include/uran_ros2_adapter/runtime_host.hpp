#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace uran_ros2_adapter
{
class Ros2RuntimeHost final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Ros2RuntimeHost(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Ros2RuntimeHost() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
};
}
