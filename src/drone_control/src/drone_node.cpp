#include "drone_control/drone_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "drone_control/grid_planner.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace drone_control
{

namespace
{
double wrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

double clamp(double v, double lim) {return std::max(-lim, std::min(lim, v));}
}

DroneNode::DroneNode()
: Node("drone_node")
{
  const auto wp_flat = declare_parameter<std::vector<double>>(
    "waypoints", std::vector<double>{0.0, 0.0, 1.5});
  run_mission_ = declare_parameter("run_mission", true);
  takeoff_altitude_ = declare_parameter("takeoff_altitude", 1.5);
  goal_altitude_ = declare_parameter("goal_altitude", 1.5);
  land_altitude_ = declare_parameter("land_altitude", 0.10);
  max_speed_ = declare_parameter("max_speed", 1.0);
  max_vz_ = declare_parameter("max_vertical_speed", 0.5);
  max_yaw_rate_ = declare_parameter("max_yaw_rate", 0.6);
  kp_xy_ = declare_parameter("kp_xy", 0.8);
  kp_z_ = declare_parameter("kp_z", 1.0);
  kp_yaw_ = declare_parameter("kp_yaw", 1.2);
  tolerance_ = declare_parameter("waypoint_tolerance", 0.3);
  face_travel_direction_ = declare_parameter("face_travel_direction", true);
  start_delay_ = declare_parameter("start_delay", 5.0);
  loop_mission_ = declare_parameter("loop_mission", false);
  auto_land_ = declare_parameter("auto_land", true);
  teleop_timeout_ = declare_parameter("teleop_timeout", 1.0);
  prop_spin_speed_ = declare_parameter("prop_spin_speed", 40.0);
  safety_enabled_ = declare_parameter("safety_enabled", true);
  safety_distance_ = declare_parameter("safety_distance", 0.7);
  plan_paths_ = declare_parameter("plan_paths", true);
  inflation_radius_ = declare_parameter("inflation_radius", 0.5);
  map_frame_ = declare_parameter("map_frame", std::string("map"));
  grid_resolution_ = declare_parameter("grid_resolution", 0.1);
  obstacle_min_z_ = declare_parameter("obstacle_min_z", 0.2);
  obstacle_max_z_ = declare_parameter("obstacle_max_z", 3.0);
  const auto rate_hz = declare_parameter("control_rate", 20.0);

  if (wp_flat.size() % 3 != 0 || wp_flat.empty()) {
    throw std::invalid_argument("'waypoints' must be a non-empty flat list of x,y,z triples");
  }
  for (size_t i = 0; i < wp_flat.size(); i += 3) {
    waypoints_.emplace_back(wp_flat[i], wp_flat[i + 1], wp_flat[i + 2]);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  prop_pub_ = create_publisher<std_msgs::msg::Float64>("/drone/prop_speed", 10);
  state_pub_ = create_publisher<std_msgs::msg::String>(
    "mission/state", rclcpp::QoS(1).transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>("gt_path", 10);
  plan_pub_ = create_publisher<nav_msgs::msg::Path>(
    "planned_path", rclcpp::QoS(1).transient_local());
  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "planning_map", rclcpp::QoS(1).transient_local());
  scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/points", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onScan(*msg);});
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_map", rclcpp::QoS(1).transient_local().reliable(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      cloud_ = msg;
      cloud_dirty_ = true;
    });
  grid_timer_ = create_wall_timer(2s, [this]() {
        if (cloud_dirty_) {buildGrid();}
      });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 20, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onOdom(*msg);});
  teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_teleop", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
      last_teleop_ = *msg;
      last_teleop_time_ = now();
    });
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      onGoal(*msg);
    });
  command_sub_ = create_subscription<std_msgs::msg::String>(
    "drone/command", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
      onCommand(msg->data);
    });

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_hz), [this]() {controlLoop();});
  param_cb_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {return onSetParameters(params);});

  setState(State::IDLE);
  RCLCPP_INFO(
    get_logger(), "Drone controller started (%s). Goals: RViz '2D Goal Pose'. "
    "Commands on /drone/command: takeoff | land | hover | mission | shutdown",
    run_mission_ ? "auto mission" : "manual / goals");
}

void DroneNode::onOdom(const nav_msgs::msg::Odometry & msg)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  pos_ = tf2::Vector3(p.x, p.y, p.z);
  orientation_ = tf2::Quaternion(q.x, q.y, q.z, q.w);
  double roll, pitch;
  tf2::Matrix3x3(orientation_).getRPY(roll, pitch, yaw_);
  odom_frame_ = msg.header.frame_id;
  have_odom_ = true;

  if (path_.poses.empty() || (pos_ - last_path_pos_).length() > 0.1) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg.header;
    ps.pose = msg.pose.pose;
    path_.header = msg.header;
    path_.poses.push_back(ps);
    last_path_pos_ = pos_;
    path_pub_->publish(path_);
  }
}

void DroneNode::onGoal(const geometry_msgs::msg::PoseStamped & goal_in)
{
  if (!have_odom_) {
    RCLCPP_WARN(get_logger(), "Goal ignored: no odometry yet");
    return;
  }
  if (pos_.z() < takeoff_altitude_ - 0.3) {
    RCLCPP_INFO(get_logger(), "Goal received while landed - taking off first");
    pending_goal_ = goal_in;
    takeoff_reached_time_.reset();
    startTakeoff(State::GOTO);
    return;
  }
  if (planGoal(goal_in)) {
    setState(State::GOTO);
  }
}

bool DroneNode::planGoal(const geometry_msgs::msg::PoseStamped & goal_in)
{
  geometry_msgs::msg::PoseStamped goal, goal_map;
  tf2::Transform odom_T_map;
  try {
    auto in = goal_in;
    in.header.stamp = rclcpp::Time(0);
    goal = tf_buffer_->transform(in, odom_frame_, tf2::durationFromSec(0.5));
    goal_map = tf_buffer_->transform(in, map_frame_, tf2::durationFromSec(0.5));
    tf2::fromMsg(
      tf_buffer_->lookupTransform(odom_frame_, map_frame_, tf2::TimePointZero).transform,
      odom_T_map);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(
      get_logger(), "Goal ignored: cannot transform %s -> %s/%s: %s",
      goal_in.header.frame_id.c_str(), odom_frame_.c_str(), map_frame_.c_str(), e.what());
    return false;
  }
  const double z = goal_in.pose.position.z > 0.05 ? goal.pose.position.z : goal_altitude_;
  goal_target_ = tf2::Vector3(goal.pose.position.x, goal.pose.position.y, z);
  const auto & q = goal.pose.orientation;
  double r, p;
  tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(r, p, goal_yaw_);

  goto_path_.clear();
  goto_idx_ = 0;
  nav_msgs::msg::Path plan;
  plan.header.frame_id = map_frame_;
  plan.header.stamp = now();
  if (plan_paths_ && cloud_dirty_) {buildGrid();}
  if (plan_paths_ && map_) {
    const tf2::Vector3 start_map = odom_T_map.inverse() * pos_;
    std::string why;
    auto route = planPath(
      *map_, inflation_radius_, start_map.x(), start_map.y(),
      goal_map.pose.position.x, goal_map.pose.position.y, why);
    if (!route) {
      RCLCPP_ERROR(get_logger(), "Goal rejected: %s", why.c_str());
      return false;
    }
    for (const auto & [mx, my] : *route) {
      tf2::Vector3 wp = odom_T_map * tf2::Vector3(mx, my, 0.0);
      goto_path_.emplace_back(wp.x(), wp.y(), z);
      geometry_msgs::msg::PoseStamped ps;
      ps.header = plan.header;
      ps.pose.position.x = mx;
      ps.pose.position.y = my;
      ps.pose.position.z = goal_map.pose.position.z > 0.05 ? goal_map.pose.position.z : z;
      ps.pose.orientation.w = 1.0;
      plan.poses.push_back(ps);
    }
  } else if (plan_paths_) {
    RCLCPP_WARN(get_logger(), "No SLAM map yet - flying straight to the goal");
  }
  goto_path_.push_back(goal_target_);
  plan.poses.push_back(goal_map);
  plan_pub_->publish(plan);

  RCLCPP_INFO(
    get_logger(), "New goal (%s): x=%.2f y=%.2f yaw=%.0f deg, route with %zu segment(s)",
    goal_in.header.frame_id.c_str(), goal_map.pose.position.x, goal_map.pose.position.y,
    goal_yaw_ * 180.0 / M_PI, goto_path_.size());
  return true;
}

void DroneNode::onCommand(const std::string & cmd)
{
  if (cmd == "shutdown") {
    RCLCPP_INFO(get_logger(), "Shutdown requested on /drone/command");
    publishZero();
    rclcpp::shutdown();
    return;
  }
  if (!have_odom_) {
    RCLCPP_WARN(get_logger(), "Command '%s' ignored: no odometry yet", cmd.c_str());
    return;
  }
  if (cmd == "takeoff") {
    startTakeoff(State::IDLE);
  } else if (cmd == "land") {
    startLanding();
  } else if (cmd == "hover" || cmd == "stop") {
    setState(State::IDLE);
  } else if (cmd == "mission") {
    wp_index_ = 0;
    startTakeoff(State::MISSION);
  } else {
    RCLCPP_WARN(
      get_logger(), "Unknown command '%s' (takeoff | land | hover | mission | shutdown)",
      cmd.c_str());
  }
}

void DroneNode::buildGrid()
{
  cloud_dirty_ = false;
  if (!cloud_) {return;}
  auto grid = buildOccupancyGrid(
    *cloud_, grid_resolution_, obstacle_min_z_, obstacle_max_z_, map_frame_, now());
  if (!grid) {return;}
  map_ = grid;
  grid_pub_->publish(*grid);
}

void DroneNode::setState(State s)
{
  state_ = s;
  publishState();
  RCLCPP_INFO(get_logger(), "State -> %s", stateName());
}

const char * DroneNode::stateName() const
{
  static const char * names[] = {"IDLE", "TAKEOFF", "MISSION", "GOTO", "LANDING"};
  return names[static_cast<int>(state_)];
}

void DroneNode::publishState()
{
  std_msgs::msg::String msg;
  msg.data = stateName();
  state_pub_->publish(msg);
  last_state_pub_ = std::chrono::steady_clock::now();
}

void DroneNode::startTakeoff(State after)
{
  after_takeoff_ = after;
  takeoff_target_ = tf2::Vector3(pos_.x(), pos_.y(), std::max(takeoff_altitude_, pos_.z()));
  setState(State::TAKEOFF);
}

void DroneNode::updatePropellers()
{
  const bool airborne = have_odom_ && pos_.z() > land_altitude_ + 0.05;
  const bool spin = teleop_active_ || state_ != State::IDLE || airborne;
  std_msgs::msg::Float64 msg;
  msg.data = spin ? prop_spin_speed_ : 0.0;
  prop_pub_->publish(msg);
}

void DroneNode::startLanding()
{
  land_target_ = tf2::Vector3(pos_.x(), pos_.y(), land_altitude_ - 0.05);
  landing_check_time_.reset();
  setState(State::LANDING);
}

bool DroneNode::flyTo(
  const tf2::Vector3 & target, bool face_travel, std::optional<double> final_yaw)
{
  tf2::Vector3 err = target - pos_;
  const double horiz = std::hypot(err.x(), err.y());

  double heading_err = 0.0;
  double yaw_rate = 0.0;
  if (face_travel && face_travel_direction_ && horiz > 2.0 * tolerance_) {
    heading_err = wrapAngle(std::atan2(err.y(), err.x()) - yaw_);
    yaw_rate = clamp(kp_yaw_ * heading_err, max_yaw_rate_);
  } else if (final_yaw) {
    yaw_rate = clamp(kp_yaw_ * wrapAngle(*final_yaw - yaw_), max_yaw_rate_);
  }

  double vxy = std::min(kp_xy_ * horiz, max_speed_);
  vxy *= std::clamp(1.0 - std::abs(heading_err) / 0.8, 0.0, 1.0);
  tf2::Vector3 v_world(0, 0, clamp(kp_z_ * err.z(), max_vz_));
  if (horiz > 1e-6) {
    v_world.setX(vxy * err.x() / horiz);
    v_world.setY(vxy * err.y() / horiz);
  }

  const tf2::Vector3 v_body = tf2::quatRotate(orientation_.inverse(), v_world);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = v_body.x();
  cmd.linear.y = v_body.y();
  cmd.linear.z = v_body.z();
  cmd.angular.z = yaw_rate;
  publishCmd(cmd);

  const bool yaw_ok = !final_yaw || std::abs(wrapAngle(*final_yaw - yaw_)) < 0.1;
  return err.length() < tolerance_ && yaw_ok;
}

void DroneNode::publishZero() {cmd_pub_->publish(geometry_msgs::msg::Twist());}

rcl_interfaces::msg::SetParametersResult DroneNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name != "max_speed" && name != "max_vertical_speed" && name != "max_yaw_rate") {
      continue;
    }
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || p.as_double() <= 0.0) {
      result.successful = false;
      result.reason = name + " must be a positive number, e.g. 1.0";
      return result;
    }
  }
  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "max_speed") {
      max_speed_ = p.as_double();
    } else if (name == "max_vertical_speed") {
      max_vz_ = p.as_double();
    } else if (name == "max_yaw_rate") {
      max_yaw_rate_ = p.as_double();
    } else {
      continue;
    }
    RCLCPP_INFO(get_logger(), "%s set to %.2f", name.c_str(), p.as_double());
  }
  return result;
}

void DroneNode::onScan(const sensor_msgs::msg::PointCloud2 & msg)
{
  std::array<float, kSectors> nearest;
  nearest.fill(std::numeric_limits<float>::infinity());
  for (sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    x != x.end(); ++x, ++y, ++z)
  {
    if (!std::isfinite(*x) || *z < -0.15f || *z > 0.4f) {continue;}
    const float r = std::hypot(*x, *y);
    if (r > 3.0f) {continue;}
    double a = std::atan2(*y, *x);
    int s = static_cast<int>((a + M_PI) / (2.0 * M_PI) * kSectors) % kSectors;
    nearest[s] = std::min(nearest[s], r);
  }
  sector_range_ = nearest;
  last_scan_time_ = now();
}

void DroneNode::publishCmd(geometry_msgs::msg::Twist cmd)
{
  if (safety_enabled_ && last_scan_time_ && (now() - *last_scan_time_).seconds() < 0.5) {
    bool blocked = false;
    for (int s = 0; s < kSectors; ++s) {
      if (sector_range_[s] >= safety_distance_) {continue;}
      const double a = -M_PI + (s + 0.5) * 2.0 * M_PI / kSectors;
      const double ux = std::cos(a), uy = std::sin(a);
      const double towards = cmd.linear.x * ux + cmd.linear.y * uy;
      if (towards > 0.0) {
        cmd.linear.x -= towards * ux;
        cmd.linear.y -= towards * uy;
        blocked = true;
      }
    }
    if (blocked) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Obstacle closer than %.1f m - blocking motion towards it", safety_distance_);
    }
  }
  cmd_pub_->publish(cmd);
}

void DroneNode::controlLoop()
{
  if (last_teleop_time_ && (now() - *last_teleop_time_).seconds() < teleop_timeout_) {
    if (!teleop_active_) {
      RCLCPP_WARN(get_logger(), "Manual teleop override active");
      teleop_active_ = true;
    }
    publishCmd(last_teleop_);
    updatePropellers();
    return;
  }
  if (teleop_active_) {
    RCLCPP_INFO(get_logger(), "Teleop released");
    teleop_active_ = false;
  }

  if (!have_odom_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry...");
    return;
  }
  if (!started_) {
    start_time_ = now();
    started_ = true;
    if (run_mission_) {
      RCLCPP_INFO(get_logger(), "Odometry received - mission starts in %.1f s", start_delay_);
    } else {
      RCLCPP_INFO(get_logger(), "Odometry received - waiting for teleop, goals or commands");
    }
  }
  if (run_mission_ && !mission_started_ && state_ == State::IDLE &&
    (now() - start_time_).seconds() >= start_delay_)
  {
    mission_started_ = true;
    wp_index_ = 0;
    startTakeoff(State::MISSION);
  }

  switch (state_) {
    case State::IDLE:
      publishZero();
      break;

    case State::TAKEOFF:
      flyTo(takeoff_target_, false);
      if ((takeoff_target_ - pos_).length() < 0.1) {
        if (after_takeoff_ == State::GOTO && pending_goal_) {
          if (!takeoff_reached_time_) {takeoff_reached_time_ = now();}
          if ((now() - *takeoff_reached_time_).seconds() < 2.0) {break;}
          const auto goal = *pending_goal_;
          pending_goal_.reset();
          setState(planGoal(goal) ? State::GOTO : State::IDLE);
        } else {
          setState(after_takeoff_);
        }
      }
      break;

    case State::GOTO: {
        const bool last = goto_idx_ + 1 >= goto_path_.size();
        const auto & target = goto_path_[std::min(goto_idx_, goto_path_.size() - 1)];
        if (flyTo(target, true, last ? std::optional<double>(goal_yaw_) : std::nullopt)) {
          if (last) {
            RCLCPP_INFO(
              get_logger(), "Goal reached (%.2f, %.2f, %.2f) - hovering",
              goal_target_.x(), goal_target_.y(), goal_target_.z());
            setState(State::IDLE);
          } else {
            ++goto_idx_;
          }
        }
        break;
      }

    case State::MISSION: {
        const auto & wp = waypoints_[wp_index_];
        if (flyTo(wp, true)) {
          RCLCPP_INFO(
            get_logger(), "Reached waypoint %zu/%zu (%.1f, %.1f, %.1f)",
            wp_index_ + 1, waypoints_.size(), wp.x(), wp.y(), wp.z());
          if (++wp_index_ >= waypoints_.size()) {
            if (loop_mission_) {
              wp_index_ = 0;
            } else if (auto_land_) {
              startLanding();
            } else {
              setState(State::IDLE);
            }
          }
        }
        break;
      }

    case State::LANDING: {
        flyTo(land_target_, false);
        const bool on_ground = pos_.z() < land_altitude_ + 0.005;
        if (!landing_check_time_ || (now() - *landing_check_time_).seconds() > 1.0) {
          const bool stalled = landing_check_time_ && pos_.z() > landing_check_z_ - 0.01;
          landing_check_time_ = now();
          landing_check_z_ = pos_.z();
          if (stalled && !on_ground) {
            publishZero();
            setState(State::IDLE);
            RCLCPP_INFO(get_logger(), "Landed on an obstacle at z=%.2f.", pos_.z());
            break;
          }
        }
        if (on_ground) {
          publishZero();
          setState(State::IDLE);
          RCLCPP_INFO(get_logger(), "Landed.");
        }
        break;
      }
  }
  updatePropellers();
  if (std::chrono::steady_clock::now() - last_state_pub_ > 1s) {publishState();}
}

}
