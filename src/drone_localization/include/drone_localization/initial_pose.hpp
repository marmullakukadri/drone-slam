#ifndef DRONE_LOCALIZATION__INITIAL_POSE_HPP_
#define DRONE_LOCALIZATION__INITIAL_POSE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace drone_localization
{

class InitialPose : public rclcpp::Node
{
public:
  InitialPose();

  void run(rclcpp::Executor & exec);

private:
  static constexpr double kTolerance = 0.3;
  static constexpr int kTries = 5;

  void spinFor(rclcpp::Executor & exec, std::chrono::milliseconds d);
  std::optional<std::pair<double, double>> slamXY();

  double x_, y_, yaw_;
  std::string frame_, base_;
  bool have_map_{false};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}

#endif
