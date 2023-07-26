cmake_minimum_required(VERSION 3.16)
project(off_highway_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(COMPILE_DEBUG_LOG "Compile ROS debug logging output" OFF)

if(NOT ${COMPILE_DEBUG_LOG})
  add_compile_definitions(RCLCPP_LOG_MIN_SEVERITY=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/can_message.cpp
  src/crc.cpp
  src/receiver.cpp
  src/sender.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp can_msgs diagnostic_updater)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp can_msgs diagnostic_updater)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(
    linter_excludes
    include/off_highway_common/external/can_encode_decode_inl.h
  )

  ament_copyright(EXCLUDE ${linter_excludes})
  ament_cppcheck(
    EXCLUDE ${linter_excludes}
    LANGUAGE c++
  )
  ament_cpplint(EXCLUDE ${linter_excludes})
  ament_lint_cmake()
  ament_uncrustify(
    EXCLUDE ${linter_excludes}
    LANGUAGE c++
  )
endif()

ament_package()
