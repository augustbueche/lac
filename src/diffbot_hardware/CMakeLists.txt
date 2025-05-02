cmake_minimum_required(VERSION 3.5)
project(diffbot_hardware)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(serial REQUIRED)

include_directories(include)

# 1) Build your plugin
add_library(${PROJECT_NAME} SHARED
  src/DiffBotSystem.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  ament_index_cpp
  serial
)

# 2) Register the XML with pluginlib → writes ament_index entry
pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  plugin_description.xml
)

# 3) Install the shared library
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# 4) Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# 5) **Critical**: Install XML under plugins/ subfolder
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/plugins
)

ament_package()
