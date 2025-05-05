cmake_minimum_required(VERSION 3.5)
project(diffbot_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(serial REQUIRED)  # Finds your local serial ROS package
find_package(rclcpp_lifecycle REQUIRED)

include_directories(
  include
)

# 1) Build the plugin (
# Tells CMake to compile src/DiffBotSystem.cpp into a shared library
# and name it diffbot_hardware)
add_library(${PROJECT_NAME} SHARED
  src/DiffBotSystem.cpp
)

#  Specify the include directory so that this package's headers can be found when using target_include_directories...
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
)


ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ament_index_cpp
  serial  # This adds include dirs & linking
)

# 2) Register pluginlib with plugin system so that it can be found and loaded at runtime
pluginlib_export_plugin_description_file(
  hardware_interface
  plugin_description.xml
)


# 3) Install shared lib
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# 4) Install headers under include/diffbot_hardware/ to global ROS include path
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# 5) Install plugin XML
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/plugins
)

# 6) Preparing hardware package for automated unit testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)

  ament_add_gmock(test_load_diffbot_hardware test/test_load_diffbot_hardware.cpp)
  if(TARGET test_load_diffbot_hardware)
    target_link_libraries(test_load_diffbot_hardware ${PROJECT_NAME})
  endif()
endif()

# Ensures that if another ROS2 package wants to link against the hardware 
# interface library, this will make sure that the correct compile and link flags are exported
ament_export_libraries(${PROJECT_NAME})

ament_package()
