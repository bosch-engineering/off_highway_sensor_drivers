cmake_minimum_required(VERSION 3.16)
project(off_highway_general_purpose_radar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wdeprecated)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types -Wno-missing-braces)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(off_highway_can REQUIRED)
find_package(off_highway_general_purpose_radar_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# Library to be used in unit tests
add_library(${PROJECT_NAME} SHARED
  src/receiver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Get the ROS distribution name
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(ROS_DISTRO STREQUAL "humble" OR ROS_DISTRO STREQUAL "jazzy")
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${pcl_conversions_INCLUDE_DIRS}
  )
endif()

target_link_libraries(off_highway_general_purpose_radar PUBLIC
      ${can_msgs_TARGETS}
      ${off_highway_general_purpose_radar_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      off_highway_can::off_highway_can
      rclcpp::rclcpp
      rclcpp_components::component
      rclcpp_components::component_manager
      sensor_msgs::sensor_msgs_library
)

# Use pcl_conversions only if available (for ROS Rolling and LTS ROS distros later than Jazzy)
if(TARGET pcl_conversions::pcl_conversions)
  target_link_libraries(${PROJECT_NAME} PUBLIC pcl_conversions::pcl_conversions)
endif()

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
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  can_msgs
  off_highway_can
  off_highway_general_purpose_radar_msgs
  pcl_conversions
  PCL
)

# Receiver
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "off_highway_general_purpose_radar::Receiver"
  EXECUTABLE receiver
)

install(TARGETS receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)
  find_package(pcl_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_radar_receiver_information test/test_receiver_information.cpp)
  target_include_directories(test_radar_receiver_information PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_radar_receiver_information ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_receiver_targets test/test_receiver_targets.cpp)
  target_include_directories(test_receiver_targets PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_receiver_targets ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_pcl_point_target test/test_pcl_point_target.cpp)
  target_include_directories(test_pcl_point_target PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_pcl_point_target ${PROJECT_NAME} pcl_ros::pcl_ros_tf)
endif()

ament_export_include_directories(
  include
)

ament_package()
