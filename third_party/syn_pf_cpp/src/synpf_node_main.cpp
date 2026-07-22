#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "syn_pf_cpp/synpf_component.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<syn_pf_cpp::SynPFComponent>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
