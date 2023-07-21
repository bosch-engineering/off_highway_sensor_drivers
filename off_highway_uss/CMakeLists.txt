cmake_minimum_required(VERSION 3.16)
project(off_highway_uss)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(off_highway_common REQUIRED)
find_package(off_highway_uss_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

set(dependencies
  rclcpp
  sensor_msgs
  can_msgs
  off_highway_common
  off_highway_uss_msgs
  pcl_conversions
  PCL
)

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
ament_target_dependencies(${PROJECT_NAME} SYSTEM ${dependencies})

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
ament_export_dependencies(${dependencies})

# Receiver
add_executable(receiver
  src/receiver_node.cpp
)

target_include_directories(receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(receiver SYSTEM ${dependencies})
target_link_libraries(receiver ${PROJECT_NAME})

# Sender
add_executable(sender
  src/sender_node.cpp
)

target_include_directories(sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sender SYSTEM ${dependencies})
target_link_libraries(sender ${PROJECT_NAME})


install(TARGETS receiver sender
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_index_cpp REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(test_uss_receiver_information test/test_receiver_information.cpp)
  target_include_directories(test_uss_receiver_information PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_uss_receiver_information SYSTEM ament_index_cpp)
  target_link_libraries(test_uss_receiver_information ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_receiver_objects test/test_receiver_objects.cpp)
  target_include_directories(test_receiver_objects PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_receiver_objects SYSTEM ament_index_cpp)
  target_link_libraries(test_receiver_objects ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_receiver_echos test/test_receiver_echos.cpp)
  target_include_directories(test_receiver_echos PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_receiver_echos SYSTEM ament_index_cpp)
  target_link_libraries(test_receiver_echos ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_receiver_range test/test_receiver_range.cpp)
  target_include_directories(test_receiver_range PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_receiver_range SYSTEM ament_index_cpp)
  target_link_libraries(test_receiver_range ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_sender test/test_sender.cpp)
  target_include_directories(test_sender PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_sender SYSTEM ament_index_cpp)
  target_link_libraries(test_sender ${PROJECT_NAME})
endif()

ament_package()
