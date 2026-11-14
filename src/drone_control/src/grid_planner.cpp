#include "drone_control/grid_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <queue>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace drone_control
{

nav_msgs::msg::OccupancyGrid::SharedPtr buildOccupancyGrid(
  const sensor_msgs::msg::PointCloud2 & cloud, double resolution,
  double obstacle_min_z, double obstacle_max_z, const std::string & default_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  if (cloud.width * cloud.height == 0) {return nullptr;}
  const double res = resolution;
  double minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
  for (sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y");
    x != x.end(); ++x, ++y)
  {
    if (!std::isfinite(*x) || !std::isfinite(*y)) {continue;}
    minx = std::min(minx, double(*x)); maxx = std::max(maxx, double(*x));
    miny = std::min(miny, double(*y)); maxy = std::max(maxy, double(*y));
  }
  if (minx > maxx) {return nullptr;}
  minx -= 1.0; miny -= 1.0; maxx += 1.0; maxy += 1.0;
  const int w = static_cast<int>(std::ceil((maxx - minx) / res));
  const int h = static_cast<int>(std::ceil((maxy - miny) / res));

  auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  grid->header.frame_id = cloud.header.frame_id.empty() ? default_frame : cloud.header.frame_id;
  grid->header.stamp = stamp;
  grid->info.resolution = static_cast<float>(res);
  grid->info.width = w;
  grid->info.height = h;
  grid->info.origin.position.x = minx;
  grid->info.origin.position.y = miny;
  grid->info.origin.orientation.w = 1.0;
  grid->data.assign(static_cast<size_t>(w) * h, -1);
  for (sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"),
    z(cloud, "z"); x != x.end(); ++x, ++y, ++z)
  {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
    const int cx = static_cast<int>((*x - minx) / res), cy = static_cast<int>((*y - miny) / res);
    if (cx < 0 || cy < 0 || cx >= w || cy >= h) {continue;}
    auto & cell = grid->data[cy * w + cx];
    if (*z >= obstacle_min_z && *z <= obstacle_max_z) {
      cell = 100;
    } else if (*z < obstacle_min_z && cell != 100) {
      cell = 0;
    }
  }
  return grid;
}

std::optional<Route> planPath(
  const nav_msgs::msg::OccupancyGrid & grid, double inflation_radius,
  double sx, double sy, double gx, double gy, std::string & why)
{
  const auto & info = grid.info;
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
    return Route{};
  }

  std::vector<uint8_t> blocked(static_cast<size_t>(w) * h, 0);
  const int r = static_cast<int>(std::ceil(inflation_radius / res));
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (grid.data[y * w + x] < 65) {continue;}
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
      std::to_string(inflation_radius) + " m)";
    return std::nullopt;
  }
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
      if (grid.data[c] >= 65) {continue;}
      if (cx > 0) {stack.push_back(c - 1);}
      if (cx < w - 1) {stack.push_back(c + 1);}
      if (cy > 0) {stack.push_back(c - w);}
      if (cy < h - 1) {stack.push_back(c + w);}
    }
  }

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
    if (f - heur(c) > g[c] + 1e-3f) {continue;}
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
  Route corners;
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

}
