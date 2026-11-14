#include <memory>

#include "drone_bringup/drone_model.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_bringup::DroneModel>());
  rclcpp::shutdown();
  return 0;
}
