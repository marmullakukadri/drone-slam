// drone_node: flight controller for the simulated SLAM drone.
//
// States: IDLE -> TAKEOFF -> MISSION | GOTO -> LANDING -> IDLE
//
// Inputs
//   odom            nav_msgs/Odometry   drone pose (ground truth by default)
//   goal_pose       geometry_msgs/PoseStamped  fly to a goal (RViz "2D Goal Pose");
//                   any frame known to TF (usually "map"), takes off first if needed.
//                   The route is planned with A* on a 2D grid so the drone flies
//                   around walls; published on planned_path.
//   cloud_map       sensor_msgs/PointCloud2  3D SLAM map (map frame). Points whose
//                   height is inside [obstacle_min_z, obstacle_max_z] become
//                   obstacles of the 2D planning grid (published on planning_map);
//                   lower points (floor) mark free space.
//   drone/command   std_msgs/String     "takeoff" | "land" | "hover" | "mission"
//   cmd_vel_teleop  geometry_msgs/Twist manual override (keyboard); pauses the
//                   current state and is forwarded as-is until teleop_timeout
// Outputs
//   cmd_vel         geometry_msgs/Twist body-frame velocity (Gazebo VelocityControl)
//   mission/state   std_msgs/String
//   gt_path         nav_msgs/Path       flown path in the odom frame
//
// If run_mission is true the waypoint mission starts automatically after
// start_delay seconds; otherwise the drone waits on the ground for commands.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace
{
double wrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

double clamp(double v, double lim) {return std::max(-lim, std::min(lim, v));}
}  // namespace

class DroneNode : public rclcpp::Node
{
public:
  enum class State { IDLE, TAKEOFF, MISSION, GOTO, LANDING };

  DroneNode()
  : Node("drone_node")
  {
    // ---- parameters ----
    const auto wp_flat = declare_parameter<std::vector<double>>(
      "waypoints", std::vector<double>{0.0, 0.0, 1.5});
    run_mission_ = declare_parameter("run_mission", true);
    takeoff_altitude_ = declare_parameter("takeoff_altitude", 1.5);
    goal_altitude_ = declare_parameter("goal_altitude", 1.5);
    // base_link height when standing on the skids (spawn height in the world)
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
    prop_spin_speed_ = declare_parameter("prop_spin_speed", 40.0);  // rad/s, visual only
    // Collision guard: block horizontal motion towards obstacles seen by the
    // lidar closer than safety_distance (teleop and autonomous flight).
    safety_enabled_ = declare_parameter("safety_enabled", true);
    safety_distance_ = declare_parameter("safety_distance", 0.7);
    plan_paths_ = declare_parameter("plan_paths", true);
    inflation_radius_ = declare_parameter("inflation_radius", 0.5);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    grid_resolution_ = declare_parameter("grid_resolution", 0.1);
    obstacle_min_z_ = declare_parameter("obstacle_min_z", 0.2);   // map frame
    obstacle_max_z_ = declare_parameter("obstacle_max_z", 3.0);   // map frame
    const auto rate_hz = declare_parameter("control_rate", 20.0);

    if (wp_flat.size() % 3 != 0 || wp_flat.empty()) {
      throw std::invalid_argument("'waypoints' must be a non-empty flat list of x,y,z triples");
    }
    for (size_t i = 0; i < wp_flat.size(); i += 3) {
      waypoints_.emplace_back(wp_flat[i], wp_flat[i + 1], wp_flat[i + 2]);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- ROS interfaces ----
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

    setState(State::IDLE);
    RCLCPP_INFO(
      get_logger(), "Drone controller started (%s). Goals: RViz '2D Goal Pose'. "
      "Commands on /drone/command: takeoff | land | hover | mission",
      run_mission_ ? "auto mission" : "manual / goals");
  }

private:
  // ------------------------------------------------------------------ inputs
  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    pos_ = tf2::Vector3(p.x, p.y, p.z);
    orientation_ = tf2::Quaternion(q.x, q.y, q.z, q.w);
    double roll, pitch;
    tf2::Matrix3x3(orientation_).getRPY(roll, pitch, yaw_);
    odom_frame_ = msg.header.frame_id;
    have_odom_ = true;

    // Downsampled flown path (every 0.1 m)
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

  void onGoal(const geometry_msgs::msg::PoseStamped & goal_in)
  {
    if (!have_odom_) {
      RCLCPP_WARN(get_logger(), "Goal ignored: no odometry yet");
      return;
    }
    if (pos_.z() < takeoff_altitude_ - 0.3) {
      // On the ground the lidar sees the floor around the drone as obstacles;
      // take off first and plan once the map has been refreshed from altitude.
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

  // Computes goal_target_/goal_yaw_/goto_path_. Returns false if the goal is rejected.
  bool planGoal(const geometry_msgs::msg::PoseStamped & goal_in)
  {
    // Express the goal in the odometry (control) frame and in the map frame.
    geometry_msgs::msg::PoseStamped goal, goal_map;
    tf2::Transform odom_T_map;
    try {
      auto in = goal_in;
      in.header.stamp = rclcpp::Time(0);  // latest available transform
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
    // RViz 2D goals have z = 0 -> use goal_altitude in that case.
    const double z = goal_in.pose.position.z > 0.05 ? goal.pose.position.z : goal_altitude_;
    goal_target_ = tf2::Vector3(goal.pose.position.x, goal.pose.position.y, z);
    const auto & q = goal.pose.orientation;
    double r, p;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(r, p, goal_yaw_);

    // Route: A* on the 2D map (map frame), converted to the control frame.
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
        start_map.x(), start_map.y(), goal_map.pose.position.x, goal_map.pose.position.y, why);
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

  void onCommand(const std::string & cmd)
  {
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
        get_logger(), "Unknown command '%s' (takeoff | land | hover | mission)", cmd.c_str());
    }
  }

  // ------------------------------------------------------------------ planning
  // 2D grid from the 3D SLAM cloud: obstacle if any point lies in the height
  // band [obstacle_min_z, obstacle_max_z], free if only floor points, else unknown.
  void buildGrid()
  {
    cloud_dirty_ = false;
    if (!cloud_ || cloud_->width * cloud_->height == 0) {return;}
    const double res = grid_resolution_;
    double minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
    for (sensor_msgs::PointCloud2ConstIterator<float> x(*cloud_, "x"), y(*cloud_, "y");
      x != x.end(); ++x, ++y)
    {
      if (!std::isfinite(*x) || !std::isfinite(*y)) {continue;}
      minx = std::min(minx, double(*x)); maxx = std::max(maxx, double(*x));
      miny = std::min(miny, double(*y)); maxy = std::max(maxy, double(*y));
    }
    if (minx > maxx) {return;}
    minx -= 1.0; miny -= 1.0; maxx += 1.0; maxy += 1.0;
    const int w = static_cast<int>(std::ceil((maxx - minx) / res));
    const int h = static_cast<int>(std::ceil((maxy - miny) / res));

    auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    grid->header.frame_id = cloud_->header.frame_id.empty() ? map_frame_ : cloud_->header.frame_id;
    grid->header.stamp = now();
    grid->info.resolution = static_cast<float>(res);
    grid->info.width = w;
    grid->info.height = h;
    grid->info.origin.position.x = minx;
    grid->info.origin.position.y = miny;
    grid->info.origin.orientation.w = 1.0;
    grid->data.assign(static_cast<size_t>(w) * h, -1);
    for (sensor_msgs::PointCloud2ConstIterator<float> x(*cloud_, "x"), y(*cloud_, "y"),
      z(*cloud_, "z"); x != x.end(); ++x, ++y, ++z)
    {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
      const int cx = static_cast<int>((*x - minx) / res), cy = static_cast<int>((*y - miny) / res);
      if (cx < 0 || cy < 0 || cx >= w || cy >= h) {continue;}
      auto & cell = grid->data[cy * w + cx];
      if (*z >= obstacle_min_z_ && *z <= obstacle_max_z_) {
        cell = 100;
      } else if (*z < obstacle_min_z_ && cell != 100) {
        cell = 0;
      }
    }
    map_ = grid;
    grid_pub_->publish(*grid);
  }

  // A* on the 2D occupancy grid (8-connected), obstacles inflated by
  // inflation_radius, unknown cells treated as free (so goals into unexplored
  // space are allowed). Returns the intermediate corner points (map frame) of
  // a line-of-sight simplified route, excluding start and goal.
  std::optional<std::vector<std::pair<double, double>>> planPath(
    double sx, double sy, double gx, double gy, std::string & why) const
  {
    const auto & info = map_->info;
    const int w = static_cast<int>(info.width), h = static_cast<int>(info.height);
    const double res = info.resolution, ox = info.origin.position.x, oy = info.origin.position.y;
    auto toCell = [&](double x, double y, int & cx, int & cy) {
        cx = static_cast<int>(std::floor((x - ox) / res));
        cy = static_cast<int>(std::floor((y - oy) / res));
        return cx >= 0 && cy >= 0 && cx < w && cy < h;
      };
    int scx, scy, gcx, gcy;
    if (!toCell(gx, gy, gcx, gcy)) {
      why = "goal is outside the map";
      return std::nullopt;
    }
    if (!toCell(sx, sy, scx, scy)) {
      return std::vector<std::pair<double, double>>{};  // drone off-map: fly straight
    }

    // Inflate obstacles
    std::vector<uint8_t> blocked(static_cast<size_t>(w) * h, 0);
    const int r = static_cast<int>(std::ceil(inflation_radius_ / res));
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (map_->data[y * w + x] < 65) {continue;}
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            const int nx = x + dx, ny = y + dy;
            if (dx * dx + dy * dy <= r * r && nx >= 0 && ny >= 0 && nx < w && ny < h) {
              blocked[ny * w + nx] = 1;
            }
          }
        }
      }
    }
    const int start = scy * w + scx, goal = gcy * w + gcx;
    if (blocked[goal]) {
      why = "goal is on/too close to an obstacle (inflation " +
        std::to_string(inflation_radius_) + " m)";
      return std::nullopt;
    }
    // Let the drone leave an inflated zone it is already in.
    {
      std::vector<int> stack{start};
      std::vector<uint8_t> seen(blocked.size(), 0);
      while (!stack.empty()) {
        const int c = stack.back();
        stack.pop_back();
        if (seen[c] || !blocked[c]) {continue;}
        seen[c] = 1;
        const int cx = c % w, cy = c / w;
        if ((cx - scx) * (cx - scx) + (cy - scy) * (cy - scy) > (r + 1) * (r + 1)) {continue;}
        blocked[c] = 0;
        if (map_->data[c] >= 65) {continue;}  // never open real obstacles
        if (cx > 0) {stack.push_back(c - 1);}
        if (cx < w - 1) {stack.push_back(c + 1);}
        if (cy > 0) {stack.push_back(c - w);}
        if (cy < h - 1) {stack.push_back(c + w);}
      }
    }

    // A*
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> g(blocked.size(), inf);
    std::vector<int> parent(blocked.size(), -1);
    using QE = std::pair<float, int>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;
    auto heur = [&](int c) {
        return static_cast<float>(std::hypot(c % w - gcx, c / w - gcy));
      };
    g[start] = 0.0f;
    open.emplace(heur(start), start);
    static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    while (!open.empty()) {
      const auto [f, c] = open.top();
      open.pop();
      if (c == goal) {break;}
      if (f - heur(c) > g[c] + 1e-3f) {continue;}  // stale entry
      const int cx = c % w, cy = c / w;
      for (int k = 0; k < 8; ++k) {
        const int nx = cx + DX[k], ny = cy + DY[k];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {continue;}
        const int n = ny * w + nx;
        if (blocked[n]) {continue;}
        const float ng = g[c] + (k < 4 ? 1.0f : 1.41421f);
        if (ng < g[n]) {
          g[n] = ng;
          parent[n] = c;
          open.emplace(ng + heur(n), n);
        }
      }
    }
    if (parent[goal] < 0 && goal != start) {
      why = "no free route to the goal on the 2D map";
      return std::nullopt;
    }
    std::vector<int> cells;
    for (int c = goal; c != -1; c = parent[c]) {cells.push_back(c);}
    std::reverse(cells.begin(), cells.end());

    // Line-of-sight simplification
    auto lineFree = [&](int a, int b) {
        int x0 = a % w, y0 = a / w;
        const int x1 = b % w, y1 = b / w;
        const int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
        const int sx_ = x0 < x1 ? 1 : -1, sy_ = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true) {
          if (blocked[y0 * w + x0]) {return false;}
          if (x0 == x1 && y0 == y1) {return true;}
          const int e2 = 2 * err;
          if (e2 >= dy) {err += dy; x0 += sx_;}
          if (e2 <= dx) {err += dx; y0 += sy_;}
        }
      };
    std::vector<std::pair<double, double>> corners;
    size_t i = 0;
    while (i + 1 < cells.size()) {
      size_t j = cells.size() - 1;
      while (j > i + 1 && !lineFree(cells[i], cells[j])) {--j;}
      if (j == cells.size() - 1) {break;}
      corners.emplace_back(ox + (cells[j] % w + 0.5) * res, oy + (cells[j] / w + 0.5) * res);
      i = j;
    }
    return corners;
  }

  // ------------------------------------------------------------------ helpers
  void setState(State s)
  {
    state_ = s;
    publishState();
    RCLCPP_INFO(get_logger(), "State -> %s", stateName());
  }

  const char * stateName() const
  {
    static const char * names[] = {"IDLE", "TAKEOFF", "MISSION", "GOTO", "LANDING"};
    return names[static_cast<int>(state_)];
  }

  // Published on every change and once per second (so late subscribers,
  // e.g. "task status", always get the current state).
  void publishState()
  {
    std_msgs::msg::String msg;
    msg.data = stateName();
    state_pub_->publish(msg);
    last_state_pub_ = std::chrono::steady_clock::now();
  }

  void startTakeoff(State after)
  {
    after_takeoff_ = after;
    takeoff_target_ = tf2::Vector3(pos_.x(), pos_.y(), std::max(takeoff_altitude_, pos_.z()));
    setState(State::TAKEOFF);
  }

  // Propellers spin while the drone is flying or being moved, stop on the ground.
  void updatePropellers()
  {
    const bool airborne = have_odom_ && pos_.z() > land_altitude_ + 0.05;
    const bool spin = teleop_active_ || state_ != State::IDLE || airborne;
    std_msgs::msg::Float64 msg;
    msg.data = spin ? prop_spin_speed_ : 0.0;
    prop_pub_->publish(msg);
  }

  void startLanding()
  {
    // Aim slightly below the skids: the ground contact stops the descent.
    land_target_ = tf2::Vector3(pos_.x(), pos_.y(), land_altitude_ - 0.05);
    landing_check_time_.reset();
    setState(State::LANDING);
  }

  // Returns true when target reached. Publishes a body-frame velocity command.
  bool flyTo(const tf2::Vector3 & target, bool face_travel, std::optional<double> final_yaw = {})
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

    // Horizontal P control, saturated; slow down while turning hard.
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

  void publishZero() {cmd_pub_->publish(geometry_msgs::msg::Twist());}

  // ------------------------------------------------------------------ collision guard
  // Nearest obstacle per 10-degree sector around the drone (body frame), from
  // lidar points roughly at flight level. Floor/low objects below the drone and
  // ceilings far above are ignored.
  static constexpr int kSectors = 36;

  void onScan(const sensor_msgs::msg::PointCloud2 & msg)
  {
    std::array<float, kSectors> nearest;
    nearest.fill(std::numeric_limits<float>::infinity());
    for (sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
      x != x.end(); ++x, ++y, ++z)
    {
      // lidar_link is 0.10 m above base_link, same orientation
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

  // Remove the horizontal velocity component pointing at any close obstacle.
  void publishCmd(geometry_msgs::msg::Twist cmd)
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

  // ------------------------------------------------------------------ loop
  void controlLoop()
  {
    // Manual override has priority over everything.
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
        publishZero();  // gravity is off: zero velocity = hold position
        break;

      case State::TAKEOFF:
        flyTo(takeoff_target_, false);
        if ((takeoff_target_ - pos_).length() < 0.1) {
          if (after_takeoff_ == State::GOTO && pending_goal_) {
            // Hover briefly so the SLAM map is refreshed from altitude, then plan.
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
        // Landed when on the skids, or when the descent stalls (e.g. on a crate).
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

  // parameters
  std::vector<tf2::Vector3> waypoints_;
  bool run_mission_, face_travel_direction_, loop_mission_, auto_land_;
  double takeoff_altitude_, goal_altitude_, land_altitude_, max_speed_, max_vz_, max_yaw_rate_;
  double kp_xy_, kp_z_, kp_yaw_, tolerance_, start_delay_, teleop_timeout_;
  bool plan_paths_;
  double inflation_radius_, grid_resolution_, obstacle_min_z_, obstacle_max_z_;
  std::string map_frame_;

  // state
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
  std::array<float, 36> sector_range_{};
  std::optional<rclcpp::Time> last_scan_time_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
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

  // ROS
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr prop_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr grid_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneNode>());
  rclcpp::shutdown();
  return 0;
}
