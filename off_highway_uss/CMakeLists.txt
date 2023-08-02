cmake_minimum_required(VERSION 3.16)
project(off_highway_uss)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(off_highway_common REQUIRED)
find_package(off_highway_uss_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_library(${PROJECT_NAME} SHARED
  src/interpolate_line.cpp
  src/receiver.cpp
  src/sender.cpp
)

add_executable(receiver
  src/receiver_node.cpp
)

add_executable(sender
  src/sender_node.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_compile_features(receiver PUBLIC c_std_99 cxx_std_17)
target_compile_features(sender PUBLIC c_std_99 cxx_std_17)

target_link_libraries(receiver ${PROJECT_NAME})
target_link_libraries(sender ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} SYSTEM rclcpp sensor_msgs can_msgs off_highway_common off_highway_uss_msgs pcl_conversions PCL)
ament_target_dependencies(receiver rclcpp)
ament_target_dependencies(sender rclcpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(TARGETS
  receiver
  sender
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${PROJECT_NAME})

ament_package()
