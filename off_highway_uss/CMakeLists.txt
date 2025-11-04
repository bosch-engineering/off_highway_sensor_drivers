cmake_minimum_required(VERSION 3.16)
project(off_highway_uss)

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
find_package(off_highway_uss_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# Library to be used in unit tests
add_library(${PROJECT_NAME} SHARED
  src/interpolate_line.cpp
  src/receiver.cpp
  src/sender.cpp
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

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${can_msgs_TARGETS}
  ${off_highway_uss_msgs_TARGETS}
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
  off_highway_uss_msgs
  pcl_conversions
  PCL
)

# Receiver
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "off_highway_uss::Receiver"
  EXECUTABLE receiver
)

# Sender
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "off_highway_uss::Sender"
  EXECUTABLE sender
)


install(TARGETS receiver sender
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

  ament_add_ros_isolated_gtest(test_uss_receiver_information test/test_receiver_information.cpp)
  target_include_directories(test_uss_receiver_information PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_uss_receiver_information ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_receiver_objects test/test_receiver_objects.cpp)
  target_include_directories(test_receiver_objects PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_receiver_objects ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_receiver_echos test/test_receiver_echos.cpp)
  target_include_directories(test_receiver_echos PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_receiver_echos ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_receiver_range test/test_receiver_range.cpp)
  target_include_directories(test_receiver_range PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_receiver_range ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_sender test/test_sender.cpp)
  target_include_directories(test_sender PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_sender ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_pcl_point_object test/test_pcl_point_object.cpp)
  target_include_directories(test_pcl_point_object PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_pcl_point_object ${PROJECT_NAME} pcl_ros::pcl_ros_tf)
endif()

ament_package()
