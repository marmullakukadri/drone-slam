cmake_minimum_required(VERSION 3.8)
project(drone_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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

add_executable(drone_node src/drone_node.cpp)

ament_target_dependencies(drone_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS drone_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_vnc.sh
  scripts/stop_sim.sh
  scripts/slam_evaluator.py
  scripts/map_saver.py
  scripts/map_filter.py
  scripts/drone_model.py
  scripts/initial_pose.py
  scripts/drone_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
