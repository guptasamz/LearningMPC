#include "lmpc_ros2/lmpc_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

// LMPCCore itself, compiled directly into this translation unit -- same
// single-TU pattern as src_gym/CMakeLists.txt's pybind11 module target.
// LMPC_BUILD_PYBIND is intentionally undefined for this target (see
// ../CMakeLists.txt and lmpc_core.cpp's #ifdef guard), so this pulls in
// zero pybind11/Python dependency.
#include "lmpc_core.cpp"

using namespace std::chrono_literals;

namespace lmpc_ros2 {

namespace {

double yaw_from_quaternion(double x, double y, double z, double w) {
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// First data row of the initial-safe-set CSV: index,x,y,yaw,... (no header --
// matches src_gym/lmpc_gym.py's first_safe_set_pose()). LMPCCore's own
// constructor needs (x0, y0, yaw0) to seed its arc-length bookkeeping; this
// intentionally is NOT read from live odometry (the safe set's own recorded
// start pose is what the controller's internal Track/iteration state must be
// consistent with).
struct InitialPose {
  double x, y, yaw;
};

InitialPose read_initial_pose(const std::string &init_safe_set_csv) {
  std::ifstream file(init_safe_set_csv);
  if (!file.is_open()) {
    throw std::runtime_error(
        "lmpc_ros2: cannot open initial_safe_set_csv: " + init_safe_set_csv);
  }
  std::string line;
  std::getline(file, line);
  std::stringstream ss(line);
  std::string field;
  std::vector<double> cols;
  while (std::getline(ss, field, ',')) {
    cols.push_back(std::stod(field));
  }
  if (cols.size() < 4) {
    throw std::runtime_error(
        "lmpc_ros2: initial_safe_set_csv row has too few columns: " +
        init_safe_set_csv);
  }
  return InitialPose{cols[1], cols[2], cols[3]};
}

}  // namespace

LmpcNode::LmpcNode() : rclcpp::Node("lmpc_node") {
  // -- topics / files --
  pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/ego_racecar/odom");
  drive_topic_ = this->declare_parameter<std::string>("drive_topic", "/drive");
  map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
  waypoint_csv_ = this->declare_parameter<std::string>("waypoint_csv", "");
  init_safe_set_csv_ = this->declare_parameter<std::string>("init_safe_set_csv", "");
  reg_warmstart_csv_ = this->declare_parameter<std::string>("reg_warmstart_csv", "");
  // pose_source: "odom" (default) is the existing sim-compatible behavior --
  // pose_topic_'s Odometry message supplies pose AND twist directly. "pf" is
  // for the real car: pose_topic_ is then just the wheel-odometry topic
  // (only its |v| is used), and pf_pose_topic_ (a PoseStamped particle-
  // filter output, e.g. syn_pf_cpp's /tracked_pose) supplies x/y/yaw, with
  // omega/beta reconstructed by finite-differencing consecutive PF samples
  // -- see reconstruct_from_pf(). Ported from f1tenth_ws's
  // DA_MCTS_sim/node.py::_odom_cb (validated on this car's real hardware),
  // minus its one-step model-based de-lag refinement on beta.
  pose_source_ = this->declare_parameter<std::string>("pose_source", "odom");
  pf_pose_topic_ = this->declare_parameter<std::string>("pf_pose_topic", "/tracked_pose");
  if (pose_source_ != "odom" && pose_source_ != "pf") {
    throw std::runtime_error(
        "lmpc_ros2: pose_source must be 'odom' or 'pf', got: " + pose_source_);
  }
  // pose_source=pf only: the finite-diff + model-projection beta estimate
  // (reconstruct_from_pf/predict_beta) is inherently noisy. Default false
  // pins beta to exactly 0 unconditionally instead -- omega is unaffected
  // either way; set true to opt into the estimate.
  slip_angle_estimation_ =
      this->declare_parameter<bool>("slip_angle_estimation", false);
  // Optional: a TUM-style <track>_centerline.csv (x, y, w_tr_right,
  // w_tr_left) caps Track::initialize_width()'s ray-marched half-widths at
  // the designed track width wherever the ray would otherwise escape
  // through a doorway/wall gap in the map and report a bogus wide-open
  // corridor (see LMPCCore::apply_csv_halfwidths). Empty, missing, or a
  // 2-column centerline (no width data) all degrade gracefully to pure
  // ray-marching -- safe to always pass.
  halfwidth_csv_ = this->declare_parameter<std::string>("halfwidth_csv", "");
  if (waypoint_csv_.empty() || init_safe_set_csv_.empty()) {
    throw std::runtime_error(
        "lmpc_ros2: waypoint_csv and init_safe_set_csv params are required "
        "(see launch/lmpc.launch.py's track_dir/track_name arguments)");
  }

  // -- controller params, forwarded verbatim into LMPCCore::getParameters --
  // Defaults mirror Lmpc_params.yaml exactly (see that file's comments for
  // the reasoning behind each, especially osqp_max_iter/osqp_time_limit).
  auto declare_num = [this](const std::string &name, double def) {
    controller_params_[name] = this->declare_parameter<double>(name, def);
  };
  declare_num("N", 25);
  declare_num("Ts", 0.025);
  declare_num("K_NEAR", 16);
  declare_num("SPEED_MAX", 10.00);
  declare_num("STEER_MAX", 0.41);
  declare_num("ACCELERATION_MAX", 9.51);
  declare_num("DECELERATION_MAX", 9.51);
  declare_num("VEL_THRESHOLD", 0.8);
  declare_num("MAP_MARGIN", 0.45);
  // Ceiling on the track half-width the controller believes it has wherever
  // Track::initialize_width()'s wall search can't find a wall nearby --
  // default matches track.h's historical HALF_WIDTH_MAX (0.8m, sized for
  // wide purpose-built tracks like barc_oval). Override lower for a track
  // with genuinely narrow corridors (see launch/lmpc.launch.py's
  // track_half_width_max argument, used for gold_conference_room).
  declare_num("track_half_width_max", 0.8);
  declare_num("WAYPOINT_SPACE", 0.2);
  declare_num("r_accel", 1.5);
  declare_num("r_steer", 18.0);
  declare_num("r_d_accel", 0.1);
  declare_num("r_d_steer", 0.1);
  declare_num("q_s", 3000.0);
  declare_num("q_s_terminal", 800.0);
  declare_num("osqp_max_iter", 20000);
  declare_num("osqp_time_limit", 0.03);
  declare_num("wheelbase", 0.3302);
  declare_num("friction_coeff", 1.2);
  declare_num("height_cg", 0.08255);
  declare_num("l_cg2rear", 0.17145);
  declare_num("l_cg2front", 0.15875);
  declare_num("C_S_front", 2.3);
  declare_num("C_S_rear", 2.3);
  declare_num("mass", 3.17);
  declare_num("moment_inertia", 0.0398378);
  // 0 = original known-dynamics model (default); 1 = kinematic nominal +
  // online residual regression (online_training/). See README.md.
  declare_num("dynamics_model", 0);

  Ts_ = controller_params_["Ts"];

  last_overrun_warn_ = this->now();

  rclcpp::QoS map_qos(1);
  map_qos.transient_local();  // map_server-style latched publish
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&LmpcNode::map_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LmpcNode::odom_callback, this, std::placeholders::_1));

  if (pose_source_ == "pf") {
    pf_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pf_pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LmpcNode::pf_pose_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "lmpc_ros2: pose_source=pf -- x/y/yaw/omega/beta reconstructed "
                "from '%s' (finite-diff), |v| from '%s' twist",
                pf_pose_topic_.c_str(), pose_topic_.c_str());
  }

  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic_, 10);

  RCLCPP_INFO(this->get_logger(),
              "lmpc_ros2 waiting for map on '%s' before the controller can "
              "initialize (needs a map_server or equivalent already running)",
              map_topic_.c_str());
}

LmpcNode::~LmpcNode() = default;

void LmpcNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (core_) {
    return;  // map assumed static for this node's lifetime, same as lmpc_gym.py
  }
  build_controller(*msg);
}

void LmpcNode::build_controller(const nav_msgs::msg::OccupancyGrid &map_msg) {
  InitialPose pose;
  try {
    pose = read_initial_pose(init_safe_set_csv_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return;
  }

  try {
    core_ = std::make_unique<LMPCCore>(
        controller_params_, map_msg.data, map_msg.info.width,
        map_msg.info.height, map_msg.info.resolution,
        map_msg.info.origin.position.x, map_msg.info.origin.position.y,
        waypoint_csv_, init_safe_set_csv_, pose.x, pose.y, pose.yaw,
        reg_warmstart_csv_, halfwidth_csv_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "lmpc_ros2: failed to construct LMPCCore: %s",
                 e.what());
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "LMPCCore initialized (track length %.2f m, Ts=%.3fs) -- "
              "starting control timer",
              core_->track_length(), Ts_);

  control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(Ts_),
      std::bind(&LmpcNode::control_tick, this));
}

void LmpcNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (pose_source_ == "pf") {
    // pf mode: this topic supplies ONLY |v| here -- x/y/yaw/omega/beta all
    // come from pf_pose_callback instead. hypot() degrades gracefully to
    // |linear.x| when linear.y is the usual real-odometry hardcoded 0 (see
    // vesc_to_odom's twist.linear.y=0, twist.angular.z=no-slip-kinematic-
    // formula-not-a-measurement -- the reason pose_source=pf exists at all).
    odom_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    return;
  }
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  yaw_ = yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // Body-frame velocities, as-reported. Many real odometry sources only
  // populate twist.linear.x reliably (no direct lateral-velocity sensing) --
  // see README.md's real-car section.
  vx_ = msg->twist.twist.linear.x;
  vy_ = msg->twist.twist.linear.y;
  yawdot_ = msg->twist.twist.angular.z;
  have_state_ = true;
}

void LmpcNode::pf_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  const double t = rclcpp::Time(msg->header.stamp).seconds();
  const double x = msg->pose.position.x;
  const double y = msg->pose.position.y;
  const double yaw = yaw_from_quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w);

  const PfReconstruction r = reconstruct_from_pf(t, x, y, yaw);
  if (!r.ok) {
    return;  // building history, not enough PF samples yet
  }

  // slip_angle_estimation_ == false: skip the estimate entirely, beta is
  // pinned to exactly 0 unconditionally (see the member's doc comment --
  // this signal is inherently noisy; omega is unaffected either way).
  double beta = 0.0;
  if (slip_angle_estimation_) {
    // One-step model-based de-lag: r.beta is the finite-diff estimate taken
    // over [ref, now], i.e. it's really the AVERAGE beta over the PREVIOUS
    // control step, lagging "now" by about half a step. Forward-roll it
    // through the controller's own dynamics (same model solve_MPC
    // linearizes around) from the reference sample to project a
    // model-consistent estimate at the current time -- same correction
    // DA_MCTS_sim's _odom_cb applies via its own dynamics stepper. Falls
    // back to the raw finite-diff value before core_ exists (no
    // map/controller yet) or if the projection throws, matching
    // DA_MCTS_sim's try/except fallback.
    beta = r.beta;
    if (core_) {
      try {
        // predict_beta divides by v in the dynamic-model branch (yaw-rate/
        // slip-angle terms) -- near-zero r.ref_v produces NaN rather than a
        // C++ exception, so check finiteness explicitly, not just catch().
        const double projected = core_->predict_beta(
            r.ref_x, r.ref_y, r.ref_yaw, r.ref_v, r.omega, r.beta,
            last_accel_cmd_, last_steer_cmd_, core_->use_dyn());
        if (std::isfinite(projected)) {
          beta = projected;
        }
      } catch (const std::exception &) {
        beta = r.beta;
      }
    }
  }

  // |v| from the trustworthy wheel-speed twist (odom_speed_, updated by
  // odom_callback); DIRECTION (beta) and omega from the PF finite-diff --
  // wheel encoders alone can't see lateral slip at all, but they're a much
  // lower-noise speed source than differentiating PF positions would be.
  // set_state() below re-derives |v|=hypot(vx,vy) and beta=atan2(vy,vx)
  // from these two components, so reconstruct them from odom_speed_ and
  // beta here rather than passing the finite-diff's own (noisier) magnitude.
  x_ = x;
  y_ = y;
  yaw_ = yaw;
  vx_ = odom_speed_ * std::cos(beta);
  vy_ = odom_speed_ * std::sin(beta);
  yawdot_ = r.omega;
  have_state_ = true;
}

LmpcNode::PfReconstruction LmpcNode::reconstruct_from_pf(double t, double x, double y,
                                                           double yaw) {
  PfReconstruction r;

  // Reset detection: a large jump (PF relocalization / RViz "2D Pose
  // Estimate") would otherwise show up as a nonsense velocity spike --
  // clear history and start over. Same 5m guard as f1tenth_ws's
  // DA_MCTS_sim/node.py::_odom_cb.
  if (!pf_history_.empty()) {
    const auto &last = pf_history_.back();
    if (std::hypot(x - last.x, y - last.y) > 5.0) {
      RCLCPP_WARN(this->get_logger(),
                  "pf pose jumped > 5m -- clearing state-reconstruction history");
      pf_history_.clear();
    }
  }

  pf_history_.push_back({t, x, y, yaw});
  while (pf_history_.size() > kPfHistoryMax) {
    pf_history_.pop_front();
  }
  if (pf_history_.size() < 2) {
    return r;  // r.ok stays false
  }

  // Reference sample ~one control step (Ts_) before now, nearest by time --
  // same lookup DA_MCTS_sim uses over its own pose history.
  const double target_t = t - Ts_;
  const PfSample *ref = &pf_history_.front();
  double best_dt = std::abs(ref->t - target_t);
  for (const auto &s : pf_history_) {
    const double dt = std::abs(s.t - target_t);
    if (dt < best_dt) {
      best_dt = dt;
      ref = &s;
    }
  }
  r.ref_x = ref->x;
  r.ref_y = ref->y;
  r.ref_yaw = ref->yaw;

  const double actual_dt = t - ref->t;
  const bool pos_changed = std::abs(x - ref->x) > 1e-9 || std::abs(y - ref->y) > 1e-9;
  const bool dt_ok = actual_dt >= Ts_ * 0.5 && actual_dt <= Ts_ * 1.5;
  if (!pos_changed || !dt_ok) {
    r.ok = true;  // beta/omega/ref_v stay 0
    return r;
  }

  // Finite-diff over PF positions, rotated into the body frame at the
  // reference sample's heading -- captures true world-frame motion
  // including the slip-induced lateral component, which is what makes beta
  // observable at all (wheel-encoder-only odometry cannot see it).
  const double x_dot = (x - ref->x) / Ts_;
  const double y_dot = (y - ref->y) / Ts_;
  const double vx_b = x_dot * std::cos(ref->yaw) + y_dot * std::sin(ref->yaw);
  const double vy_b = -x_dot * std::sin(ref->yaw) + y_dot * std::cos(ref->yaw);
  r.ref_v = std::hypot(vx_b, vy_b);
  // Zero beta below this speed -- matches DA_MCTS_sim's own threshold,
  // avoids atan2 amplifying position noise into a wild heading angle when
  // the car is nearly stationary.
  r.beta = (r.ref_v > 0.05) ? std::atan2(vy_b, vx_b) : 0.0;
  r.omega = (yaw - ref->yaw) / Ts_;
  r.ok = true;
  return r;
}

void LmpcNode::control_tick() {
  if (!core_ || !have_state_) {
    return;
  }

  core_->set_state(x_, y_, yaw_, vx_, vy_, yawdot_);

  const auto t0 = std::chrono::steady_clock::now();
  LMPCCore::StepResult result = core_->step();
  const double elapsed_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  if (elapsed_s > Ts_) {
    // Throttled: this is visibility, not a bounding mechanism -- see
    // Lmpc_params.yaml's osqp_time_limit comment for what actually bounds
    // worst-case solve time.
    if ((this->now() - last_overrun_warn_).seconds() > 1.0) {
      RCLCPP_WARN(this->get_logger(),
                  "control step took %.1f ms, over the %.1f ms Ts budget",
                  elapsed_s * 1000.0, Ts_ * 1000.0);
      last_overrun_warn_ = this->now();
    }
  }
  if (!result.solved) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "QP solve failed -- reapplying previous control "
                          "(LMPCCore's own fallback, not an error state)");
  }

  // Tracked for pose_source=pf's beta one-step de-lag correction (see
  // reconstruct_from_pf/predict_beta) -- the action actually applied over
  // the most recent control step, regardless of whether this step's solve
  // itself succeeded (result.accel/steer already reflect LMPCCore's own
  // solved-vs-fallback choice).
  last_accel_cmd_ = result.accel;
  last_steer_cmd_ = result.steer;

  // f1tenth_gym's vehicle model (base_classes.py's RaceCar::update_pose(),
  // patched for this project -- see its "accl = vel" line/comment) treats
  // drive_topic's speed field as a raw commanded ACCELERATION, applied
  // directly each physics tick -- not a target velocity to track down
  // toward. LMPCCore::step() already returns exactly that (the QP's own
  // acceleration decision); publish it as-is. Re-integrating it into a
  // synthetic speed_cmd_ first (the previous approach here) double-applied
  // the integration the sim already does, and made "0" mean "hold current
  // velocity" instead of an actual stop -- verify against your actual real-
  // car drive stack's own expectation before deploying (see README.md).
  ackermann_msgs::msg::AckermannDriveStamped msg;
  msg.header.stamp = this->now();
  msg.drive.speed = static_cast<float>(result.accel);
  msg.drive.steering_angle = static_cast<float>(result.steer);
  drive_pub_->publish(msg);
}

}  // namespace lmpc_ros2
