cmake_minimum_required(VERSION 3.5)
project(diffbot_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)

# Build your hardware plugin as a shared library
add_library(${PROJECT_NAME} SHARED
  src/DiffBotSystem.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  serial
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# *** THIS IS THE KEY CHANGE ***  
# Export (and install!) your plugin_description.xml under the
# hardware_interface package so that ros2_control_node’s loader
# (which uses ClassLoader<...>("hardware_interface", ...))
# can find it.
pluginlib_export_plugin_description_file(
  hardware_interface
  plugin_description.xml
)

ament_package()
