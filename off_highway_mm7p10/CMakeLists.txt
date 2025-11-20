cmake_minimum_required(VERSION 3.16)
project(off_highway_mm7p10)

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
find_package(tf2 REQUIRED)
find_package(can_msgs REQUIRED)
find_package(off_highway_can REQUIRED)
find_package(off_highway_mm7p10_msgs REQUIRED)

# Library to be used in unit tests
add_library(${PROJECT_NAME} SHARED
  src/receiver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
      ${can_msgs_TARGETS}
      ${off_highway_mm7p10_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      off_highway_can::off_highway_can
      rclcpp::rclcpp
      rclcpp_components::component
      rclcpp_components::component_manager
      sensor_msgs::sensor_msgs_library
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
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  can_msgs
  off_highway_can
  off_highway_mm7p10_msgs
)

# Receiver
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "off_highway_mm7p10::Receiver"
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
  find_package(tf2_geometry_msgs REQUIRED)

  ament_add_ros_isolated_gtest(test_imu_receiver test/test_imu_receiver.cpp)
  target_include_directories(test_imu_receiver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_imu_receiver ${PROJECT_NAME} ${tf2_geometry_msgs_TARGETS})

endif()

ament_package()
