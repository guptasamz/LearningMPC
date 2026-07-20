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
  declare_num("Ts", 0.05);
  declare_num("K_NEAR", 16);
  declare_num("SPEED_MAX", 10.00);
  declare_num("STEER_MAX", 0.41);
  declare_num("ACCELERATION_MAX", 9.51);
  declare_num("DECELERATION_MAX", 9.51);
  declare_num("VEL_THRESHOLD", 0.8);
  declare_num("MAP_MARGIN", 0.45);
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
        reg_warmstart_csv_);
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
