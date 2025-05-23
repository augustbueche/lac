cmake_minimum_required(VERSION 3.5)
project(serial)

# Ensure C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable Position Independent Code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Required packages
find_package(ament_cmake REQUIRED)

# === Add serial library ===
add_library(${PROJECT_NAME}
  src/serial.cc
  include/serial/serial.h
  include/serial/v8stdint.h
)

# Platform-specific source files and linking
if(APPLE) # macOS
  find_library(IOKIT_LIBRARY IOKit)
  find_library(FOUNDATION_LIBRARY Foundation)
  target_sources(${PROJECT_NAME} PRIVATE
    src/impl/unix.cc
    src/impl/list_ports/list_ports_osx.cc
  )
  target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX) # Linux and others
  target_sources(${PROJECT_NAME} PRIVATE
    src/impl/unix.cc
    src/impl/list_ports/list_ports_linux.cc
  )
  target_link_libraries(${PROJECT_NAME} rt pthread)
elseif(WIN32) # Windows
  target_sources(${PROJECT_NAME} PRIVATE
    src/impl/win.cc
    src/impl/list_ports/list_ports_win.cc
  )
  target_link_libraries(${PROJECT_NAME} setupapi)
endif()

# === Export include directory and library ===
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# === Install library ===
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# === Install headers ===
install(DIRECTORY include/
  DESTINATION include
)

# === Export CMake config for ament ===
ament_package()

