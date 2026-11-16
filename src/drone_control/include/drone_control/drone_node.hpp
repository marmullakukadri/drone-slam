#ifndef DRONE_CONTROL__DRONE_NODE_HPP_
#define DRONE_CONTROL__DRONE_NODE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace drone_control
{

class DroneNode : public rclcpp::Node
{
public:
  enum class State { IDLE, TAKEOFF, MISSION, GOTO, LANDING };

  DroneNode();

private:
  static constexpr int kSectors = 36;

  void onOdom(const nav_msgs::msg::Odometry & msg);
  void onGoal(const geometry_msgs::msg::PoseStamped & goal_in);
  bool planGoal(const geometry_msgs::msg::PoseStamped & goal_in);
  void onCommand(const std::string & cmd);
  void onScan(const sensor_msgs::msg::PointCloud2 & msg);
  void buildGrid();

  void setState(State s);
  const char * stateName() const;
  void publishState();
  void startTakeoff(State after);
  void startLanding();
  void updatePropellers();
  bool flyTo(const tf2::Vector3 & target, bool face_travel, std::optional<double> final_yaw = {});
  void publishZero();
  void publishCmd(geometry_msgs::msg::Twist cmd);
  void controlLoop();
  bool isLocalized(std::string & why);
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  std::vector<tf2::Vector3> waypoints_;
  bool run_mission_, face_travel_direction_, loop_mission_, auto_land_;
  double takeoff_altitude_, goal_altitude_, land_altitude_, max_speed_, max_vz_, max_yaw_rate_;
  double kp_xy_, kp_z_, kp_yaw_, tolerance_, start_delay_, teleop_timeout_;
  bool plan_paths_;
  double inflation_radius_, grid_resolution_, obstacle_min_z_, obstacle_max_z_;
  std::string map_frame_;
  bool require_localization_;
  double max_localization_error_;
  std::optional<double> slam_error_;
  std::optional<rclcpp::Time> slam_error_time_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
  bool cloud_dirty_{false};
  std::vector<tf2::Vector3> goto_path_;
  size_t goto_idx_{0};
  State state_{State::IDLE};
  State after_takeoff_{State::IDLE};
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
  std::optional<rclcpp::Time> takeoff_reached_time_;
  size_t wp_index_{0};
  bool have_odom_{false};
  bool started_{false};
  bool mission_started_{false};
  bool teleop_active_{false};
  std::chrono::steady_clock::time_point last_state_pub_{};
  double prop_spin_speed_{40.0};
  bool safety_enabled_{true};
  double safety_distance_{0.9};
  std::array<float, kSectors> sector_range_{};
  std::optional<rclcpp::Time> last_scan_time_;
  std::optional<rclcpp::Time> landing_check_time_;
  double landing_check_z_{0.0};
  std::string odom_frame_{"world"};
  tf2::Vector3 pos_{0, 0, 0};
  tf2::Quaternion orientation_{0, 0, 0, 1};
  double yaw_{0.0};
  tf2::Vector3 takeoff_target_, land_target_, goal_target_;
  double goal_yaw_{0.0};
  rclcpp::Time start_time_;
  std::optional<rclcpp::Time> last_teleop_time_;
  geometry_msgs::msg::Twist last_teleop_;
  nav_msgs::msg::Path path_;
  tf2::Vector3 last_path_pos_{0, 0, 0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr prop_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr slam_error_sub_;
  rclcpp::TimerBase::SharedPtr grid_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}

#endif
