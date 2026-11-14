#ifndef DRONE_BRINGUP__DRONE_MODEL_HPP_
#define DRONE_BRINGUP__DRONE_MODEL_HPP_

#include <array>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace drone_bringup
{

class DroneModel : public rclcpp::Node
{
public:
  DroneModel();

private:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  struct Static
  {
    Marker m;
    std::array<double, 6> pose;
  };

  struct Prop
  {
    Marker m;
    std::array<double, 6> link, vis;
    double sign;
  };

  struct Part
  {
    const Marker * tmpl;
    std::string ns;
    int id;
    tf2::Vector3 xyz;
    tf2::Quaternion q;
  };

  void load(const std::string & path);
  std::vector<Part> bodyParts() const;
  std::vector<Part> propParts() const;
  Marker marker(
    const Marker & tmpl, const std::string & ns, int id, const std::string & frame,
    const builtin_interfaces::msg::Time & stamp, const tf2::Vector3 & xyz,
    const tf2::Quaternion & q, const std_msgs::msg::ColorRGBA * color = nullptr) const;
  void publishStatic();
  void publishProps();
  void publishGhost();

  std::string frame_;
  std::vector<Static> static_;
  std::vector<Prop> props_;
  Marker label_;
  double spin_speed_{0.0}, angle_{0.0};
  nav_msgs::msg::Odometry::ConstSharedPtr gt_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_, gt_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
  rclcpp::TimerBase::SharedPtr static_timer_, prop_timer_, ghost_timer_;
};

}

#endif
