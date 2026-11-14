#include "drone_localization/initial_pose.hpp"

#include <cmath>

#include "tf2/exceptions.h"

using namespace std::chrono_literals;

namespace drone_localization
{

InitialPose::InitialPose()
: Node("initial_pose")
{
  x_ = declare_parameter("x", 0.0);
  y_ = declare_parameter("y", 0.0);
  yaw_ = declare_parameter("yaw", 0.0);
  frame_ = declare_parameter("map_frame", std::string("map"));
  base_ = declare_parameter("base_frame", std::string("base_link"));
  pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_map", rclcpp::QoS(1).transient_local().reliable(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {have_map_ = true;});
}

void InitialPose::run(rclcpp::Executor & exec)
{
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && (!have_map_ || pub_->get_subscription_count() == 0 || !slamXY())) {
    if (std::chrono::steady_clock::now() - start > 120s) {
      RCLCPP_ERROR(get_logger(), "RTAB-Map not ready after 120 s - give \"2D Pose Estimate\"");
      return;
    }
    spinFor(exec, 500ms);
  }
  spinFor(exec, 2s);

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = frame_;
  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
  msg.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
  for (int attempt = 1; attempt <= kTries && rclcpp::ok(); ++attempt) {
    msg.header.stamp = now();
    pub_->publish(msg);
    spinFor(exec, 3s);
    const auto xy = slamXY();
    const double err = xy ? std::hypot(xy->first - x_, xy->second - y_) : INFINITY;
    if (err < kTolerance) {
      RCLCPP_INFO(
        get_logger(), "Initial pose set: x=%.2f y=%.2f yaw=%.0f deg (SLAM %.2f m from it)",
        x_, y_, yaw_ * 180.0 / M_PI, err);
      return;
    }
    RCLCPP_WARN(get_logger(), "Initial pose attempt %d: SLAM is %.2f m off", attempt, err);
  }
  RCLCPP_ERROR(get_logger(), "Initial pose not accepted - give \"2D Pose Estimate\" in RViz");
}

void InitialPose::spinFor(rclcpp::Executor & exec, std::chrono::milliseconds d)
{
  const auto end = std::chrono::steady_clock::now() + d;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    exec.spin_once(50ms);
  }
}

std::optional<std::pair<double, double>> InitialPose::slamXY()
{
  try {
    const auto t = tf_buffer_->lookupTransform(frame_, base_, tf2::TimePointZero);
    return std::make_pair(t.transform.translation.x, t.transform.translation.y);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

}
