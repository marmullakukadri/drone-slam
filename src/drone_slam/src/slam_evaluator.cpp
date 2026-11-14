#include "drone_slam/slam_evaluator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace drone_slam
{

SlamEvaluator::SlamEvaluator()
: Node("slam_evaluator")
{
  map_frame_ = declare_parameter("map_frame", std::string("map"));
  base_frame_ = declare_parameter("base_frame", std::string("base_link"));
  output_dir_ = declare_parameter("output_dir", std::string("/workspace/results"));
  const double sample_period = declare_parameter("sample_period", 0.2);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  err_pub_ = create_publisher<std_msgs::msg::Float64>("/slam/position_error", 10);
  gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom_gt", 50, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onGt(*msg);});
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1,
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr) {onInitialPose();});
  sample_timer_ = create_timer(
    std::chrono::duration<double>(sample_period), [this]() {sample();});
  report_timer_ = create_timer(std::chrono::seconds(10), [this]() {report();});
  csv_path_ = csvPath("");
}

void SlamEvaluator::save(bool quiet)
{
  if (rows_.empty()) {return;}
  std::filesystem::create_directories(output_dir_);
  std::ofstream f(csv_path_);
  f << "t,gt_x,gt_y,gt_z,est_x,est_y,est_z,error\n";
  f.precision(9);
  for (const auto & r : rows_) {
    for (size_t i = 0; i < r.size(); ++i) {f << r[i] << (i + 1 < r.size() ? "," : "\n");}
  }
  if (quiet) {return;}
  const auto s = stats();
  std::printf(
    "[slam_evaluator] saved %zu samples to %s (RMSE %.3f m, max %.3f m)\n",
    rows_.size(), csv_path_.c_str(), s[0], s[1]);
  std::fflush(stdout);
}

std::string SlamEvaluator::csvPath(const std::string & suffix) const
{
  char buf[64];
  const std::time_t t = std::time(nullptr);
  std::strftime(buf, sizeof(buf), "slam_eval_%Y%m%d_%H%M%S", std::localtime(&t));
  return (std::filesystem::path(output_dir_) / (buf + suffix + ".csv")).string();
}

void SlamEvaluator::onInitialPose()
{
  RCLCPP_INFO(get_logger(), "Initial pose received - restarting SLAM error statistics");
  save(true);
  rows_.clear();
  csv_path_ = csvPath("_loc");
}

void SlamEvaluator::onGt(const nav_msgs::msg::Odometry & msg)
{
  gt_frame_ = msg.header.frame_id;
  const auto & p = msg.pose.pose.position;
  gt_.push_back({rclcpp::Time(msg.header.stamp).seconds(), tf2::Vector3(p.x, p.y, p.z)});
  if (gt_.size() > 5000) {gt_.erase(gt_.begin(), gt_.begin() + 1000);}
}

void SlamEvaluator::sample()
{
  if (gt_.empty()) {return;}
  geometry_msgs::msg::TransformStamped tf;
  try {
    if (!map_T_world_) {
      tf2::Transform t;
      tf2::fromMsg(
        tf_buffer_->lookupTransform(map_frame_, gt_frame_, tf2::TimePointZero).transform, t);
      map_T_world_ = t;
    }
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;
  }
  const double t = rclcpp::Time(tf.header.stamp).seconds();
  if (t <= last_sample_) {return;}
  const auto it = std::lower_bound(
    gt_.begin(), gt_.end(), t, [](const Gt & g, double v) {return g.t < v;});
  if (it == gt_.begin() || it == gt_.end()) {return;}
  const auto & g0 = *(it - 1);
  const auto & g1 = *it;
  const double a = g1.t > g0.t ? (t - g0.t) / (g1.t - g0.t) : 0.0;
  const tf2::Vector3 gt = *map_T_world_ * g0.p.lerp(g1.p, a);
  last_sample_ = t;
  const auto & tr = tf.transform.translation;
  const tf2::Vector3 est(tr.x, tr.y, tr.z);
  const double err = gt.distance(est);
  rows_.push_back({t, gt.x(), gt.y(), gt.z(), est.x(), est.y(), est.z(), err});
  std_msgs::msg::Float64 msg;
  msg.data = err;
  err_pub_->publish(msg);
}

std::array<double, 3> SlamEvaluator::stats() const
{
  double sum = 0.0, mx = 0.0;
  for (const auto & r : rows_) {
    sum += r[7] * r[7];
    mx = std::max(mx, r[7]);
  }
  return {std::sqrt(sum / rows_.size()), mx, rows_.back()[7]};
}

void SlamEvaluator::report()
{
  if (rows_.empty()) {
    RCLCPP_INFO(
      get_logger(), "Waiting for SLAM pose (TF %s->%s)...", map_frame_.c_str(),
      base_frame_.c_str());
    return;
  }
  const auto s = stats();
  RCLCPP_INFO(
    get_logger(), "SLAM ATE RMSE %.3f m | max %.3f m | current %.3f m | %zu samples",
    s[0], s[1], s[2], rows_.size());
  save(true);
}

}
