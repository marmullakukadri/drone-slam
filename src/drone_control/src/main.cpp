#include <memory>

#include "drone_control/drone_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_control::DroneNode>());
  rclcpp::shutdown();
  return 0;
}
