cmake_minimum_required(VERSION 3.8)
project(bump_go_tp5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# Version FSM
add_executable(bump_go_fsm src/bump_go_fsm.cpp)
ament_target_dependencies(bump_go_fsm
  rclcpp sensor_msgs geometry_msgs)

# Version BT
add_executable(bump_go_bt src/bump_go_bt.cpp)
ament_target_dependencies(bump_go_bt
  rclcpp sensor_msgs geometry_msgs behaviortree_cpp_v3)

install(TARGETS
  bump_go_fsm
  bump_go_bt
  DESTINATION lib/${PROJECT_NAME})

ament_package()
