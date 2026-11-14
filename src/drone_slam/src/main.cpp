#include <memory>

#include "drone_slam/slam_evaluator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_slam::SlamEvaluator>();
  rclcpp::spin(node);
  node->save(false);
  rclcpp::shutdown();
  return 0;
}
