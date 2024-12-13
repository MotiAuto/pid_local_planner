cmake_minimum_required(VERSION 3.5)
project(pid_local_planner)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_pid_local_planner SHARED
  DIRECTORY src
)
target_include_directories(target_pid_local_planner
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_pid_local_planner
  PLUGIN "pid_local_planner::PIDLocalPlanner"
  EXECUTABLE pid_local_planner_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)