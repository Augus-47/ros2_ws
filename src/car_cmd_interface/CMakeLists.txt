cmake_minimum_required(VERSION 3.8)
project(car_cmd_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_executable(car_cmd_interface_node 
    src/vehicle_interface.cpp
)
ament_target_dependencies(car_cmd_interface_node 
    rclcpp 
    car_msg
    geometry_msgs
    std_msgs
)
include_directories(
  include
)

install(TARGETS 
  car_cmd_interface_node
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
