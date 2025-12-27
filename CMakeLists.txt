cmake_minimum_required(VERSION 3.8)
project(pavbot_estop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(estop_node src/estop_node.cpp)

ament_target_dependencies(
  estop_node
  rclcpp
  std_msgs
)

install(
  TARGETS estop_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
