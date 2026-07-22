#pragma once
#include <chrono>
#include <deque>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace syn_pf_cpp {

class LatencyProbe : public rclcpp::Node {
public:
  explicit LatencyProbe(const rclcpp::NodeOptions & options);

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void report();

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string topic_;
  std::string label_;
  std::deque<double> lat_us_;
  static constexpr size_t kBuf = 500;
  int n_total_{0};
};

}  // namespace syn_pf_cpp
