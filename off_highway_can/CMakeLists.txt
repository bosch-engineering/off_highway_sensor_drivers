cmake_minimum_required(VERSION 3.16)
project(off_highway_can)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wdeprecated)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-ffp-contract=fast)
endif()

option(COMPILE_DEBUG_LOG "Compile ROS debug logging output" OFF)

if(NOT ${COMPILE_DEBUG_LOG})
  add_compile_definitions(RCLCPP_LOG_MIN_SEVERITY=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(ros2_socketcan_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/can_message.cpp
  src/crc.cpp
  src/receiver.cpp
  src/sender.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp can_msgs diagnostic_updater ros2_socketcan_msgs)

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
ament_export_dependencies(rclcpp can_msgs diagnostic_updater ros2_socketcan_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Exclude FOSS
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE
    test/external/test_encode_decode.cpp
    include/off_highway_can/external/can_encode_decode_inl.h
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_encode_decode test/external/test_encode_decode.cpp)
  # Disable warnings since we won't change FOSS
  target_compile_options(test_encode_decode PRIVATE -Wno-sign-compare -Wno-unused-parameter)
  target_include_directories(test_encode_decode PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_encode_decode ${PROJECT_NAME})

  ament_add_gtest(test_encode_decode_by_round test/test_encode_decode_by_round.cpp)
  target_include_directories(test_encode_decode_by_round PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_encode_decode_by_round ${PROJECT_NAME})
endif()

ament_package()
