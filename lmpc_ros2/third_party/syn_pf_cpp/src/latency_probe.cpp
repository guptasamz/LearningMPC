#include "syn_pf_cpp/latency_probe.hpp"
#include <algorithm>
#include <rclcpp_components/register_node_macro.hpp>

namespace syn_pf_cpp {

LatencyProbe::LatencyProbe(const rclcpp::NodeOptions & options)
: rclcpp::Node("latency_probe", options)
{
  topic_ = declare_parameter<std::string>("pose_topic", "/tracked_pose_cpp");
  label_ = declare_parameter<std::string>("label", "probe");

  sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_, rclcpp::QoS(10),
    std::bind(&LatencyProbe::on_pose, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::seconds(10),
    std::bind(&LatencyProbe::report, this));

  RCLCPP_INFO(get_logger(),
    "LatencyProbe [%s] subscribed to %s", label_.c_str(), topic_.c_str());
}

void LatencyProbe::on_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  const auto now_t = now();
  const auto stamp = rclcpp::Time(msg->header.stamp, now_t.get_clock_type());
  const double dt_us = (now_t - stamp).nanoseconds() / 1000.0;
  if (lat_us_.size() >= kBuf) lat_us_.pop_front();
  lat_us_.push_back(dt_us);
  ++n_total_;
}

static double pctl(std::vector<double> & v, double p) {
  if (v.empty()) return 0.0;
  const size_t idx = std::min(v.size() - 1,
    static_cast<size_t>(p / 100.0 * (v.size() - 1)));
  std::nth_element(v.begin(), v.begin() + idx, v.end());
  return v[idx];
}

void LatencyProbe::report()
{
  if (lat_us_.size() < 10) {
    RCLCPP_INFO(get_logger(),
      "[%s] %d msgs (need >=10 for stats)", label_.c_str(), n_total_);
    return;
  }
  std::vector<double> v(lat_us_.begin(), lat_us_.end());
  const double p50 = pctl(v, 50);
  const double p95 = pctl(v, 95);
  const double p99 = pctl(v, 99);
  RCLCPP_INFO(get_logger(),
    "[%s] e2e latency p50=%.1f p95=%.1f p99=%.1f us (n=%zu, rx=%d)",
    label_.c_str(), p50, p95, p99, v.size(), n_total_);
}

}  // namespace syn_pf_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(syn_pf_cpp::LatencyProbe)
