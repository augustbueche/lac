cmake_minimum_required(VERSION 3.8)
project(ultrasonic_safety)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/ultrasonic_safety_node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  ultrasonic_safety
  DESTINATION lib/${PROJECT_NAME})

ament_package()
