cmake_minimum_required(VERSION 3.8)
project(jetson_ackermann_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(car_msg REQUIRED)

include_directories(include)

add_library(jetson_ackermann_controller SHARED
  src/jetson_ackermann_controller.cpp
)

ament_target_dependencies(jetson_ackermann_controller
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
  pluginlib nav2_core nav2_util nav2_costmap_2d nav2_msgs
)

# 导出plugin描述
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS jetson_ackermann_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# 转换节点：cmd_vel -> /control_cmd VehicleState
add_executable(twist_to_vehicle_state_node src/twist_to_vehicle_state_node.cpp)
ament_target_dependencies(twist_to_vehicle_state_node
  rclcpp geometry_msgs tf2 car_msg
)
install(TARGETS twist_to_vehicle_state_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

