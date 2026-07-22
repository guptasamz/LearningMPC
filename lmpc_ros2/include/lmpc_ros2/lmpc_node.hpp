#pragma once

#include <deque>
#include <map>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

// Forward-declared only -- the full definition (src_gym/cpp/lmpc_core.cpp,
// #included directly by lmpc_node.cpp) is a heavy header pulling in Eigen/
// OSQP/the ros_shim headers, which this public header doesn't need to expose.
class LMPCCore;

namespace lmpc_ros2 {

class LmpcNode : public rclcpp::Node {
public:
  LmpcNode();
  ~LmpcNode();  // defined in lmpc_node.cpp, where LMPCCore is a complete type

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pf_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void control_tick();
  void build_controller(const nav_msgs::msg::OccupancyGrid &map_msg);

  // pose_source_ == "pf" state reconstruction: finite-difference consecutive
  // PF (x, y, yaw) samples to recover omega and beta (slip angle), the way
  // f1tenth_ws's DA_MCTS_sim/node.py::_odom_cb does on this same car's real
  // hardware -- wheel-encoder odometry alone can't see lateral slip at all
  // (vesc_to_odom hardcodes twist.linear.y=0 and derives omega from a
  // no-slip kinematic formula, not a measurement). ref_* is the reference
  // sample the finite-diff was taken against (~Ts_ ago) -- needed by
  // pf_pose_callback to forward-project beta from that sample's time to now
  // via LMPCCore::predict_beta(), the same one-step model-based de-lag
  // correction _odom_cb applies via its own dynamics stepper.
  struct PfReconstruction {
    bool ok = false;
    double beta = 0.0, omega = 0.0;
    double ref_x = 0.0, ref_y = 0.0, ref_yaw = 0.0, ref_v = 0.0;
  };
  PfReconstruction reconstruct_from_pf(double t, double x, double y, double yaw);

  // -- ROS I/O --
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pf_pose_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // -- controller --
  std::unique_ptr<LMPCCore> core_;
  std::map<std::string, double> controller_params_;

  // -- config (params) --
  std::string pose_topic_;
  std::string drive_topic_;
  std::string map_topic_;
  std::string waypoint_csv_;
  std::string init_safe_set_csv_;
  std::string reg_warmstart_csv_;
  std::string halfwidth_csv_;
  double Ts_ = 0.025;

  // pose_source_: "odom" (default, sim) trusts pose_topic_'s Odometry
  // pose+twist directly, unchanged from before. "pf" (real car) takes
  // x/y/yaw from a PoseStamped particle-filter topic (pf_pose_topic_) and
  // reconstructs omega/beta via reconstruct_from_pf(); pose_topic_'s
  // Odometry message is then only used for its |v| (wheel-speed twist
  // magnitude), not its pose.
  std::string pose_source_;
  std::string pf_pose_topic_;

  // pose_source_ == "pf" only: whether to use the finite-diff + model-
  // projection beta estimate (reconstruct_from_pf/predict_beta) at all.
  // That estimate is inherently noisy (differentiating a discretely-sampled
  // pose signal, then projecting through an approximate model) -- default
  // false pins beta to exactly 0 unconditionally instead, regardless of
  // speed or PF history. omega is unaffected either way; set true to
  // opt into the estimate.
  bool slip_angle_estimation_ = false;

  // -- live state, updated by odom_callback/pf_pose_callback, consumed by
  // control_tick --
  bool have_state_ = false;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
  double vx_ = 0.0, vy_ = 0.0, yawdot_ = 0.0;

  // Last (accel, steer) LMPCCore::step() returned, i.e. what control_tick()
  // actually published -- the "action_tm1" reconstruct_from_pf's beta
  // correction forward-projects through, mirroring DA_MCTS_sim's
  // self._a_cmd_prev/_sv_cmd_prev.
  double last_accel_cmd_ = 0.0, last_steer_cmd_ = 0.0;

  // pose_source_ == "pf": |v| from pose_topic_'s Odometry twist (real /odom
  // typically only gives a trustworthy linear.x; hypot() with linear.y
  // degrades gracefully to |linear.x| when linear.y is the usual hardcoded
  // 0). Combined with beta from reconstruct_from_pf() to rebuild vx_/vy_.
  double odom_speed_ = 0.0;

  // pose_source_ == "pf": short history of recent PF samples for the
  // finite-diff, same window f1tenth_ws's DA_MCTS_sim uses.
  struct PfSample { double t, x, y, yaw; };
  std::deque<PfSample> pf_history_;
  static constexpr size_t kPfHistoryMax = 64;

  rclcpp::Time last_overrun_warn_;
};

}  // namespace lmpc_ros2
