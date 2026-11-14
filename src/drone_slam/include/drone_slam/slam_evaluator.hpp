#ifndef DRONE_SLAM__SLAM_EVALUATOR_HPP_
#define DRONE_SLAM__SLAM_EVALUATOR_HPP_

#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace drone_slam
{

class SlamEvaluator : public rclcpp::Node
{
public:
  SlamEvaluator();

  void save(bool quiet);

private:
  using Row = std::array<double, 8>;

  struct Gt
  {
    double t;
    tf2::Vector3 p;
  };

  std::string csvPath(const std::string & suffix) const;
  void onInitialPose();
  void onGt(const nav_msgs::msg::Odometry & msg);
  void sample();
  std::array<double, 3> stats() const;
  void report();

  std::string map_frame_, base_frame_, output_dir_, gt_frame_, csv_path_;
  std::deque<Gt> gt_;
  std::vector<Row> rows_;
  std::optional<tf2::Transform> map_T_world_;
  double last_sample_{-1.0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr sample_timer_, report_timer_;
};

}

#endif
