#include <memory>

#include "drone_localization/initial_pose.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_localization::InitialPose>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  node->run(exec);
  exec.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
