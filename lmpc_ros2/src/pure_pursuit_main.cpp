#include "lmpc_ros2/pure_pursuit_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lmpc_ros2::PurePursuitNode>());
  // rclcpp::shutdown() is already called by PurePursuitNode::finish() once
  // recording completes; calling it again here would be redundant/unsafe,
  // so only call it if spin() returned for some other reason (e.g. Ctrl-C).
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
