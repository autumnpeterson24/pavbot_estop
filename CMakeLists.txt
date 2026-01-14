cmake_minimum_required(VERSION 3.8)
project(pavbot_estop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pavbot_estop src/pavbot_estop.cpp)

ament_target_dependencies(
  pavbot_estop
  rclcpp
  std_msgs
)

install(
  TARGETS pavbot_estop
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
