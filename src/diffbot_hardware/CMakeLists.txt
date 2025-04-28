cmake_minimum_required(VERSION 3.5)
project(diffbot_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)

# 1) Build your hardware plugin as a shared library
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

# 2) Install the library into <install_prefix>/lib
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# 3) Install the plugin description XML to share/<pkg>
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# 4) Register it with pluginlib (this also indexes it for ament)
pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  plugin_description.xml
)

ament_package()
