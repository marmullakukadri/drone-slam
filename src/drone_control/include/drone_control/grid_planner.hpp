#ifndef DRONE_CONTROL__GRID_PLANNER_HPP_
#define DRONE_CONTROL__GRID_PLANNER_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace drone_control
{

using Route = std::vector<std::pair<double, double>>;

nav_msgs::msg::OccupancyGrid::SharedPtr buildOccupancyGrid(
  const sensor_msgs::msg::PointCloud2 & cloud, double resolution,
  double obstacle_min_z, double obstacle_max_z, const std::string & default_frame,
  const builtin_interfaces::msg::Time & stamp);

std::optional<Route> planPath(
  const nav_msgs::msg::OccupancyGrid & grid, double inflation_radius,
  double sx, double sy, double gx, double gy, std::string & why);

}

#endif
