#include "lmpc_ros2/lmpc_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lmpc_ros2::LmpcNode>());
  rclcpp::shutdown();
  return 0;
}
