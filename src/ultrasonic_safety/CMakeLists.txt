cmake_minimum_required(VERSION 3.8)

project(ultrasonic_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(include)

set(
  DEPENDENCIES
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"
  "visualization_msgs"
  "diagnostic_updater"
)

add_executable(ultrasonic_safety_node 
  src/ultrasonic_safety_node.cpp
)

ament_target_dependencies(ultrasonic_safety_node ${DEPENDENCIES})

install(TARGETS
  ultrasonic_safety_node
  DESTINATION lib/${PROJECT_NAME})




ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
