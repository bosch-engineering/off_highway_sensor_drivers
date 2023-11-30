cmake_minimum_required(VERSION 3.16)
project(off_highway_premium_radar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -Wdeprecated)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)
find_package(io_context REQUIRED)
find_package(rclcpp REQUIRED)
# For default converter
find_package(diagnostic_updater REQUIRED)
find_package(off_highway_premium_radar_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  ASIO
  diagnostic_updater
  io_context
  off_highway_premium_radar_msgs
  pcl_conversions
  PCL
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# Library
add_library(${PROJECT_NAME} SHARED
  src/driver.cpp
  src/location_data_handler.cpp
  src/node.cpp
  src/pdu_definitions.cpp
  src/udp_socket.cpp
  src/converters/default_converter.cpp
)

ament_target_dependencies(${PROJECT_NAME} SYSTEM ${dependencies})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# Executable
add_executable(${PROJECT_NAME}_driver
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_driver ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}_driver SYSTEM ${dependencies})

target_include_directories(
  ${PROJECT_NAME}_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS
  cmake/${PROJECT_NAME}-extras.cmake
)
