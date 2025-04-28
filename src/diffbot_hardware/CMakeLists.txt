cmake_minimum_required(VERSION 3.5)
project(diffbot_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)

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

# Install the shared library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# Install the plugin description XML
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export the plugin to the ament index so pluginlib can find it
pluginlib_export_plugin_description_file(
  diffbot_hardware
  plugin_description.xml
)

ament_package()
