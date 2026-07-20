#pragma once

#include <map>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
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
  void control_tick();
  void build_controller(const nav_msgs::msg::OccupancyGrid &map_msg);

  // -- ROS I/O --
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
  double Ts_ = 0.05;
  double v_min_ = -5.0;
  double v_max_ = 20.0;

  // -- live state, updated by odom_callback, consumed by control_tick --
  bool have_state_ = false;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
  double vx_ = 0.0, vy_ = 0.0, yawdot_ = 0.0;

  // -- open-loop speed integration for AckermannDriveStamped.drive.speed
  // (see README.md: verify against the real drive stack's actual interface) --
  double speed_cmd_ = 0.0;

  rclcpp::Time last_overrun_warn_;
};

}  // namespace lmpc_ros2
