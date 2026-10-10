// litearm_driver_main.cpp — entry point of the litearm maintenance driver.
//
// Runs the lifecycle node in a multi-threaded executor with one mutually exclusive
// callback group: two SDK calls can never interleave on the wire, and a blocking service
// does not deadlock the executor.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "litearm_driver/litearm_driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<litearm_driver::LitearmDriverNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
