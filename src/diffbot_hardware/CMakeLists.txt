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

# 2) Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# 3) Install your plugin XML for local debugging (optional)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# 4) ALSO install the same XML into the hardware_interface share
#    so ros2_control_node’s ClassLoader<...>("hardware_interface", …)
#    can actually find it. 
install(FILES plugin_description.xml
  DESTINATION share/hardware_interface
)

# 5) Register it with pluginlib’s index
pluginlib_export_plugin_description_file(
  hardware_interface
  plugin_description.xml
)

ament_package()
