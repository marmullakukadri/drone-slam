#include <gtest/gtest.h>

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "drone_control/grid_planner.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using drone_control::buildOccupancyGrid;
using drone_control::planPath;

namespace
{

nav_msgs::msg::OccupancyGrid emptyGrid(int w, int h, double res)
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = w;
  g.info.height = h;
  g.info.resolution = res;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(static_cast<size_t>(w) * h, 0);
  return g;
}

void wallAtColumn(nav_msgs::msg::OccupancyGrid & g, int col, int door_from, int door_to)
{
  for (int y = 0; y < static_cast<int>(g.info.height); ++y) {
    if (y >= door_from && y <= door_to) {continue;}
    g.data[y * g.info.width + col] = 100;
  }
}

sensor_msgs::msg::PointCloud2 cloud(const std::vector<std::array<float, 3>> & pts)
{
  sensor_msgs::msg::PointCloud2 c;
  c.header.frame_id = "map";
  sensor_msgs::PointCloud2Modifier mod(c);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.size());
  sensor_msgs::PointCloud2Iterator<float> x(c, "x"), y(c, "y"), z(c, "z");
  for (const auto & p : pts) {
    *x = p[0];
    *y = p[1];
    *z = p[2];
    ++x; ++y; ++z;
  }
  return c;
}

}

TEST(GridPlanner, StraightLineHasNoCorners)
{
  const auto g = emptyGrid(50, 50, 0.1);
  std::string why;
  const auto route = planPath(g, 0.3, 0.5, 0.5, 4.5, 4.5, why);
  ASSERT_TRUE(route.has_value()) << why;
  EXPECT_TRUE(route->empty());
}

TEST(GridPlanner, GoalOutsideMapIsRejected)
{
  const auto g = emptyGrid(50, 50, 0.1);
  std::string why;
  EXPECT_FALSE(planPath(g, 0.3, 0.5, 0.5, 9.0, 0.5, why).has_value());
  EXPECT_EQ(why, "goal is outside the map");
}

TEST(GridPlanner, GoalOnObstacleIsRejected)
{
  auto g = emptyGrid(50, 50, 0.1);
  wallAtColumn(g, 25, -1, -1);
  std::string why;
  EXPECT_FALSE(planPath(g, 0.3, 0.5, 0.5, 2.55, 2.5, why).has_value());
  EXPECT_NE(why.find("obstacle"), std::string::npos);
}

TEST(GridPlanner, ClosedWallHasNoRoute)
{
  auto g = emptyGrid(50, 50, 0.1);
  wallAtColumn(g, 25, -1, -1);
  std::string why;
  EXPECT_FALSE(planPath(g, 0.3, 0.5, 2.5, 4.5, 2.5, why).has_value());
  EXPECT_EQ(why, "no free route to the goal on the 2D map");
}

TEST(GridPlanner, RouteGoesThroughTheDoor)
{
  auto g = emptyGrid(50, 50, 0.1);
  wallAtColumn(g, 25, 35, 45);
  std::string why;
  const auto route = planPath(g, 0.2, 0.5, 0.5, 4.5, 0.5, why);
  ASSERT_TRUE(route.has_value()) << why;
  ASSERT_FALSE(route->empty());
  bool through_door = false;
  for (const auto & [x, y] : *route) {
    if (y >= 3.5 && y <= 4.6) {through_door = true;}
  }
  EXPECT_TRUE(through_door);
}

TEST(GridPlanner, BuildGridMarksFloorFreeAndWallsOccupied)
{
  const auto c = cloud({{0.05f, 0.05f, 0.0f}, {2.05f, 0.05f, 1.0f}, {2.05f, 2.05f, 5.0f}});
  const double res = 0.1;
  const auto g = buildOccupancyGrid(c, res, 0.2, 3.0, "map", builtin_interfaces::msg::Time());
  ASSERT_NE(g, nullptr);
  auto cell = [&](float x, float y) {
      const int cx = static_cast<int>((x - g->info.origin.position.x) / res);
      const int cy = static_cast<int>((y - g->info.origin.position.y) / res);
      return g->data[cy * g->info.width + cx];
    };
  EXPECT_EQ(cell(0.05f, 0.05f), 0);
  EXPECT_EQ(cell(2.05f, 0.05f), 100);
  EXPECT_EQ(cell(2.05f, 2.05f), -1);
  EXPECT_EQ(g->header.frame_id, "map");
}

TEST(GridPlanner, BuildGridFromEmptyCloudIsNull)
{
  EXPECT_EQ(
    buildOccupancyGrid(cloud({}), 0.1, 0.2, 3.0, "map", builtin_interfaces::msg::Time()),
    nullptr);
}
