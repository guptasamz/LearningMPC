#include "lmpc_ros2/pure_pursuit_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;

namespace lmpc_ros2 {

namespace {

double yaw_from_quaternion(double x, double y, double z, double w) {
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double wrap_to_pi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

// <track>_centerline.csv: "# x_m, y_m, w_tr_right_m, w_tr_left_m" header
// comment, then comma-separated rows -- only columns 0,1 (x,y) are used,
// matching src_gym/record_initial_ss.py's load_centerline().
std::vector<std::pair<double, double>> load_centerline(const std::string &path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("pure_pursuit_node: cannot open centerline_csv: " + path);
  }
  std::vector<std::pair<double, double>> pts;
  std::string line;
  while (std::getline(file, line)) {
    size_t first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    std::stringstream ss(line);
    std::string field;
    std::vector<double> cols;
    while (std::getline(ss, field, ',')) {
      cols.push_back(std::stod(field));
    }
    if (cols.size() < 2) {
      continue;
    }
    pts.emplace_back(cols[0], cols[1]);
  }
  if (pts.size() < 2) {
    throw std::runtime_error("pure_pursuit_node: centerline_csv has too few points: " + path);
  }
  return pts;
}

// Brake-to-stop sequence parameters (see the stopping_/confirmed_stop_
// branches in control_tick). Two phases, not one:
//   1. stopping_: P-law braking (kp_speed_ * (0 - v_)), same control law as
//      normal driving, until |v_| < kStopVelThresh.
//   2. confirmed_stop_: publish an EXPLICIT 0.0 accel (not the P-law's
//      output) for kConfirmedStopTicks. This matters because gym_bridge
//      holds and reapplies whatever the LAST received drive command is
//      forever once nothing publishes anymore (see the accel-vs-speed note
//      below) -- phase 1's own final tick still has a small nonzero
//      residual (v_ close to but not exactly 0 means accel is close to but
//      not exactly 0 too), and holding THAT forever means the car drifts
//      indefinitely instead of actually stopping. Phase 2 guarantees the
//      truly-last command is an exact zero, repeated enough times to
//      survive the known ROS2 publish-then-shutdown race (publish() hands
//      off to DDS asynchronously; tearing the node down right after can
//      drop a single message before it's actually sent).
constexpr double kStopVelThresh = 0.05;      // m/s
constexpr int kStopSafetyTicks = 100;        // ~5s fallback if v_ never converges
constexpr int kConfirmedStopTicks = 40;      // ~1s at the default 0.025s control_dt_

}  // namespace

PurePursuitNode::PurePursuitNode() : rclcpp::Node("pure_pursuit_node") {
  pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/ego_racecar/odom");
  drive_topic_ = this->declare_parameter<std::string>("drive_topic", "/drive");
  const std::string centerline_csv =
      this->declare_parameter<std::string>("centerline_csv", "");
  output_csv_ = this->declare_parameter<std::string>("output_csv", "");
  max_speed_ = this->declare_parameter<double>("max_speed", 0.0);
  laps_ = this->declare_parameter<int>("laps", 2);
  wheelbase_ = this->declare_parameter<double>("wheelbase", 0.3302);
  lookahead_ = this->declare_parameter<double>("lookahead", 0.9);
  kp_speed_ = this->declare_parameter<double>("kp_speed", 2.0);
  accel_limit_ = this->declare_parameter<double>("accel_limit", 2.0);
  steer_limit_ = this->declare_parameter<double>("steer_limit", 0.41);
  waypoint_spacing_ = this->declare_parameter<double>("waypoint_spacing", 0.05);
  control_dt_ = this->declare_parameter<double>("control_dt", 0.025);

  if (centerline_csv.empty() || output_csv_.empty()) {
    throw std::runtime_error(
        "pure_pursuit_node: centerline_csv and output_csv params are required");
  }
  if (max_speed_ <= 0.0) {
    // Required, not defaulted: this is the safety-critical knob now that
    // this node can drive the real car, not just gym -- see README.md.
    throw std::runtime_error(
        "pure_pursuit_node: max_speed param is required and must be > 0");
  }

  // -- densify the (sparse) centerline to a fixed arc-length spacing, closed
  // loop -- same algorithm as src_gym/record_initial_ss.py's densify() --
  const auto sparse = load_centerline(centerline_csv);
  std::vector<double> cum_s(sparse.size() + 1, 0.0);
  for (size_t i = 0; i < sparse.size(); ++i) {
    const auto &a = sparse[i];
    const auto &b = sparse[(i + 1) % sparse.size()];
    cum_s[i + 1] = cum_s[i] + std::hypot(b.first - a.first, b.second - a.second);
  }
  track_length_ = cum_s.back();
  for (double u = 0.0; u < track_length_; u += waypoint_spacing_) {
    // find the segment containing arc-length u (linear scan; centerline
    // point counts are small -- a few hundred -- so this is cheap and only
    // runs once at startup)
    size_t seg = 0;
    while (seg + 1 < cum_s.size() && cum_s[seg + 1] < u) ++seg;
    const double seg_len = cum_s[seg + 1] - cum_s[seg];
    const double t = seg_len > 1e-9 ? (u - cum_s[seg]) / seg_len : 0.0;
    const auto &a = sparse[seg % sparse.size()];
    const auto &b = sparse[(seg + 1) % sparse.size()];
    dense_x_.push_back(a.first + t * (b.first - a.first));
    dense_y_.push_back(a.second + t * (b.second - a.second));
  }
  RCLCPP_INFO(this->get_logger(),
              "pure_pursuit_node: %zu centerline points -> %zu densified "
              "(track length %.2f m), max_speed=%.2f m/s, laps=%d",
              sparse.size(), dense_x_.size(), track_length_, max_speed_, laps_);

  out_file_.open(output_csv_);
  if (!out_file_.is_open()) {
    throw std::runtime_error("pure_pursuit_node: cannot open output_csv for writing: " +
                              output_csv_);
  }
  // Fixed 6-decimal-place formatting, matching src_gym/record_initial_ss.py's
  // f"{v:.6f}" -- default stream formatting is 6 *significant* digits and
  // can switch to scientific notation, which std::stof (the CSV reader,
  // LMPCCore's init_SS_from_data) would still parse correctly, but this
  // keeps the written format consistent with the reference and avoids
  // needless precision loss on larger-magnitude values.
  out_file_ << std::fixed << std::setprecision(6);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PurePursuitNode::odom_callback, this, std::placeholders::_1));
  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic_, 10);
  control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(control_dt_),
      std::bind(&PurePursuitNode::control_tick, this));
}

void PurePursuitNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  yaw_ = yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  v_ = msg->twist.twist.linear.x;
  have_state_ = true;
}

void PurePursuitNode::control_tick() {
  if (done_ || !have_state_) {
    return;
  }

  if (confirmed_stop_) {
    // Phase 2: explicit, exact zero -- not the P-law's output -- repeated
    // so the truly-last command gym_bridge holds forever is a real zero.
    // See kConfirmedStopTicks's comment for why this phase exists at all.
    ackermann_msgs::msg::AckermannDriveStamped stop;
    stop.header.stamp = this->now();
    stop.drive.speed = 0.0f;
    stop.drive.steering_angle = 0.0f;
    drive_pub_->publish(stop);
    if (++confirmed_stop_ticks_ >= kConfirmedStopTicks) {
      finish();
    }
    return;
  }

  if (stopping_) {
    // Phase 1: brake to a real stop with the same P-law as normal driving,
    // just targeting 0 instead of max_speed_ -- v_ is live odometry
    // feedback, so this actually converges to rest instead of assuming
    // zero commanded accel gets there on its own (it doesn't -- see the
    // accel-vs-speed note below). kStopSafetyTicks is just a fallback
    // bound in case v_ somehow never converges (e.g. odometry stalls); it
    // shouldn't normally be what ends this phase.
    double accel = kp_speed_ * (0.0 - v_);
    accel = std::clamp(accel, -accel_limit_, accel_limit_);
    ackermann_msgs::msg::AckermannDriveStamped stop;
    stop.header.stamp = this->now();
    stop.drive.speed = static_cast<float>(accel);
    stop.drive.steering_angle = 0.0f;
    drive_pub_->publish(stop);
    ++stop_ticks_;
    if (std::abs(v_) < kStopVelThresh || stop_ticks_ >= kStopSafetyTicks) {
      stopping_ = false;
      confirmed_stop_ = true;
    }
    return;
  }

  // -- nearest-point arc-length lookup (linear scan -- dense_x_/dense_y_
  // are a few thousand points at most, called at control_dt_ rate, trivial
  // cost; same approach as record_initial_ss.py's s_of()) --
  size_t nearest = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < dense_x_.size(); ++i) {
    const double dx = dense_x_[i] - x_;
    const double dy = dense_y_[i] - y_;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      nearest = i;
    }
  }
  const double s_curr = static_cast<double>(nearest) * waypoint_spacing_;

  if (!s_prev_valid_) {
    s_prev_ = s_curr;
    s_prev_valid_ = true;
  }
  if (s_curr - s_prev_ < -track_length_ / 2.0) {
    row_t_ = 0;
    lap_++;
    if (lap_ > laps_ - 1) {
      // Recording is done -- close the file now, but hold off on shutdown
      // (see the stopping_ branch above) until the stop command has had a
      // chance to actually reach gym_bridge.
      out_file_.close();
      stopping_ = true;
      return;
    }
  }
  s_prev_ = s_curr;

  // -- pure pursuit steering --
  const size_t i_look =
      (nearest + static_cast<size_t>(std::llround(lookahead_ / waypoint_spacing_))) %
      dense_x_.size();
  const double gx = dense_x_[i_look];
  const double gy = dense_y_[i_look];
  const double dx = gx - x_;
  const double dy = gy - y_;
  const double alpha = wrap_to_pi(std::atan2(dy, dx) - yaw_);
  const double ld = std::max(std::hypot(dx, dy), 1e-3);
  double steer = std::atan2(2.0 * wheelbase_ * std::sin(alpha), ld);
  steer = std::clamp(steer, -steer_limit_, steer_limit_);

  // -- P speed control, capped at max_speed_ --
  double accel = kp_speed_ * (max_speed_ - v_);
  accel = std::clamp(accel, -accel_limit_, accel_limit_);

  out_file_ << row_t_ << "," << x_ << "," << y_ << "," << yaw_ << "," << v_ << ","
            << accel << "," << steer << "," << s_curr << "\n";
  row_t_++;

  // f1tenth_gym's vehicle model (base_classes.py's RaceCar::update_pose(),
  // patched for this project -- see its "accl = vel" line/comment) treats
  // drive_topic's speed field as a raw commanded ACCELERATION, applied
  // directly each physics tick -- not a target velocity to track down
  // toward. accel above (the P-law's own output) is already exactly that;
  // publish it as-is. Integrating it into a synthetic speed_cmd_ first (the
  // previous approach here) double-applied the integration the sim already
  // does -- max_speed_ ended up bounding an "acceleration" instead of the
  // actual velocity (so the car could and did exceed it), and commanding
  // "0" only zeroed acceleration -- i.e. "hold current velocity" -- not an
  // actual stop.
  ackermann_msgs::msg::AckermannDriveStamped msg;
  msg.header.stamp = this->now();
  msg.drive.speed = static_cast<float>(accel);
  msg.drive.steering_angle = static_cast<float>(steer);
  drive_pub_->publish(msg);
}

void PurePursuitNode::finish() {
  // out_file_ is already closed (control_tick's lap-completion branch), and
  // an explicit zero-accel command has been repeated for kConfirmedStopTicks
  // -- safe to actually shut down.
  done_ = true;
  RCLCPP_INFO(this->get_logger(), "wrote %s (%d laps) -- shutting down",
              output_csv_.c_str(), laps_);
  rclcpp::shutdown();
}

}  // namespace lmpc_ros2
