cmake_minimum_required(VERSION 3.8)
project(drone_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(slam_evaluator src/main.cpp src/slam_evaluator.cpp)
target_include_directories(slam_evaluator PRIVATE include)
ament_target_dependencies(slam_evaluator
  rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS slam_evaluator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_saver.py
  scripts/map_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
