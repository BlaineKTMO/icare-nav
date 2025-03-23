cmake_minimum_required(VERSION 3.8)
project(simple_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the executable
add_executable(dwa_nav_node src/dwa_nav_node.cpp)

# Link dependencies
ament_target_dependencies(dwa_nav_node
  rclcpp
  nav2_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  dwa_nav_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
