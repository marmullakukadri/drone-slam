cmake_minimum_required(VERSION 3.8)
project(drone_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(initial_pose src/main.cpp src/initial_pose.cpp)
target_include_directories(initial_pose PRIVATE include)
ament_target_dependencies(initial_pose rclcpp geometry_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS initial_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
