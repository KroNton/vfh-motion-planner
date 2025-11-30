cmake_minimum_required(VERSION 3.8)
project(motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)

# 1. Include directories BEFORE creating the library targets
include_directories(include)

add_library(vfh_controller_plugin SHARED
  src/vfh_controller.cpp
)

target_link_libraries(vfh_controller_plugin
  ${rclcpp_LIBRARIES}
  ${nav2_core_LIBRARIES}
  ${nav2_costmap_2d_LIBRARIES}
  ${nav2_util_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${angles_LIBRARIES}
)

# 2. CRITICAL: Uncomment this! 
# This macro registers the plugin XML with the ament index so ROS can find it.
pluginlib_export_plugin_description_file(nav2_core vfh_controller_plugin.xml)

install(TARGETS vfh_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Nodes (Optional, keeping your existing nodes)
add_executable(pd_motion_planner_node src/pd_motion_planner_node.cpp)
ament_target_dependencies(pd_motion_planner_node rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)

add_executable(pure_pursuit_motion_planner_node src/pure_pursuit_motion_planner_node.cpp)
ament_target_dependencies(pure_pursuit_motion_planner_node rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)

install(
  DIRECTORY include
  DESTINATION include
)

# 3. Installing the XML manually is fine, but the macro above is safer.
install(FILES vfh_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  pd_motion_planner_node
  pure_pursuit_motion_planner_node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(vfh_controller_plugin)
ament_export_dependencies(
  nav2_core
  rclcpp
  geometry_msgs
  nav2_costmap_2d
)
ament_package()