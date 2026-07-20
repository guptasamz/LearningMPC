#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lmpc_ros2 {

// Drives a centerline via pure pursuit + P-speed control, and records the
// same initial-safe-set CSV format LMPCCore's init_SS_from_data expects
// (t, x, y, yaw, v, accel, steer, s). No LMPCCore/Eigen/OSQP dependency --
// this is intentionally a separate, lightweight node from lmpc_node: run
// this first against a new track to produce its seed safe set, then run
// lmpc_node against the file it writes. See README.md.
//
// Same topic-based shape as lmpc_node (pose_topic/drive_topic, no sim-vs-
// real branching) -- this is meant to run identically against
// f1tenth_gym_ros or the real car.
class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void control_tick();
  void finish();  // shut down, once the stop-command grace period has elapsed

  // -- ROS I/O --
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // -- config (params) --
  std::string pose_topic_;
  std::string drive_topic_;
  std::string output_csv_;
  double max_speed_ = 0.0;
  int laps_ = 2;
  double wheelbase_ = 0.3302;
  double lookahead_ = 0.9;
  double kp_speed_ = 2.0;
  double accel_limit_ = 2.0;
  double steer_limit_ = 0.41;
  double waypoint_spacing_ = 0.05;
  double control_dt_ = 0.05;

  // -- densified closed-loop centerline, fixed spacing = waypoint_spacing_ --
  std::vector<double> dense_x_;
  std::vector<double> dense_y_;
  double track_length_ = 0.0;

  // -- live state, updated by odom_callback, consumed by control_tick --
  bool have_state_ = false;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0, v_ = 0.0;

  // -- recording state --
  std::ofstream out_file_;
  int lap_ = 0;
  int row_t_ = 0;
  double s_prev_ = 0.0;
  bool s_prev_valid_ = false;
  bool done_ = false;

  // -- stop sequence: laps done, braking to rest, then holding an explicit
  // zero before shutdown (see finish()/control_tick()) --
  bool stopping_ = false;
  bool confirmed_stop_ = false;
  int stop_ticks_ = 0;
  int confirmed_stop_ticks_ = 0;
};

}  // namespace lmpc_ros2
