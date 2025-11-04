cmake_minimum_required(VERSION 3.16)
project(off_highway_premium_radar_sample)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wdeprecated)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)
find_package(io_context REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# For default converter
find_package(diagnostic_updater REQUIRED)
find_package(off_highway_premium_radar_sample_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Library
add_library(${PROJECT_NAME} SHARED
  src/driver.cpp
  src/location_data_handler.cpp
  src/node.cpp
  src/pdu_definitions.cpp
  src/udp_socket.cpp
  src/converters/default_converter.cpp
)

# Get the ROS distribution name
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(ROS_DISTRO STREQUAL "humble" OR ROS_DISTRO STREQUAL "jazzy")
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${pcl_conversions_INCLUDE_DIRS}
  )
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${off_highway_premium_radar_sample_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${ASIO_LIBRARIES}
  diagnostic_updater::diagnostic_updater
  ${io_context_LIBRARIES}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# Use pcl_conversions only if available (for ROS Rolling and LTS ROS distros later than Jazzy)
if(TARGET pcl_conversions::pcl_conversions)
  target_link_libraries(${PROJECT_NAME} PUBLIC pcl_conversions::pcl_conversions)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ASIO_INCLUDE_DIRS}
  ${io_context_INCLUDE_DIRS}
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
ament_export_dependencies(
  ASIO
  diagnostic_updater
  io_context
  off_highway_premium_radar_sample_msgs
  pcl_conversions
  PCL
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# Executable
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "off_highway_premium_radar_sample::NodeWithDefaultConverter"
  EXECUTABLE driver
)

# Resources
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)
endif()

ament_package(CONFIG_EXTRAS
  cmake/${PROJECT_NAME}-extras.cmake
)
