cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(drone_node src/main.cpp src/drone_node.cpp src/grid_planner.cpp)
target_include_directories(drone_node PRIVATE include)
ament_target_dependencies(drone_node
  rclcpp geometry_msgs sensor_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS drone_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/drone_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_planner test/test_grid_planner.cpp src/grid_planner.cpp)
  target_include_directories(test_grid_planner PRIVATE include)
  ament_target_dependencies(test_grid_planner nav_msgs sensor_msgs)
endif()

ament_package()
