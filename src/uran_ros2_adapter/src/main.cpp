#include <memory>
#include <stdexcept>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "uran_ros2_adapter/runtime_host.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<uran_ros2_adapter::Ros2RuntimeHost>();
    if (node->get_parameter("autostart").as_bool()) {
      if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
        node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        throw std::runtime_error("automatic lifecycle transition failed");
      }
    }
    rclcpp::spin(node->get_node_base_interface());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("uran_runtime"), "%s", exception.what());
    code = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return code;
}
