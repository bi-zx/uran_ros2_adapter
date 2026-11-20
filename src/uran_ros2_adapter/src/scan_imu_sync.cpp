#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanImuSync : public rclcpp::Node {
public:
  using Scan = sensor_msgs::msg::LaserScan;
  using Imu = sensor_msgs::msg::Imu;

  ScanImuSync() : Node("scan_imu_sync") {
    const auto capacity = declare_parameter<int>("max_pending_scans", 160);
    if (capacity <= 0) throw std::invalid_argument("max_pending_scans must be positive");
    capacity_ = static_cast<std::size_t>(capacity);
    output_ = create_publisher<Scan>("scan_synchronized", rclcpp::SensorDataQoS());
    imu_ = create_subscription<Imu>("imu", rclcpp::SensorDataQoS().keep_last(400),
      [this](Imu::ConstSharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (!first_imu_) first_imu_ = stamp;
        latest_imu_ = std::max(latest_imu_, stamp);
      });
    scan_ = create_subscription<Scan>("scan", rclcpp::SensorDataQoS().keep_last(capacity_),
      [this](Scan::ConstSharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (stamp <= last_scan_) return;
        last_scan_ = stamp;
        if (pending_.size() == capacity_) {
          pending_.pop_front();
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU is behind the buffered laser scans");
        }
        pending_.push_back(msg);
      });
    // Drain in acquisition order, at most one per tick so a late IMU batch cannot flood the mapper.
    timer_ = create_wall_timer(std::chrono::milliseconds(25), [this]() {
      if (pending_.empty() || !first_imu_) return;
      const auto stamp = rclcpp::Time(pending_.front()->header.stamp).nanoseconds();
      if (stamp >= latest_imu_) return;
      if (stamp >= first_imu_) output_->publish(*pending_.front());
      pending_.pop_front();
    });
  }

private:
  int64_t first_imu_ = 0, latest_imu_ = 0, last_scan_ = 0;
  std::size_t capacity_;
  std::deque<Scan::ConstSharedPtr> pending_;
  rclcpp::Subscription<Imu>::SharedPtr imu_;
  rclcpp::Subscription<Scan>::SharedPtr scan_;
  rclcpp::Publisher<Scan>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanImuSync>());
  rclcpp::shutdown();
}
