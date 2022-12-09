cmake_minimum_required(VERSION 3.16)
project(off_highway_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -Wno-unused-parameter -fPIC)
endif()

option(COMPILE_DEBUG_LOG "Compile ROS debug logging output" OFF)

if(NOT ${COMPILE_DEBUG_LOG})
  add_compile_definitions(ROSCONSOLE_MIN_SEVERITY=1)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    can_msgs
    diagnostic_updater
    roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    can_msgs
    diagnostic_updater
    roscpp
  DEPENDS
)

include_directories(
  include
)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/can_message.cpp
  src/crc.cpp
  src/helper.cpp
  src/receiver.cpp
  src/sender.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_encode_decode test/test_encode_decode.cpp)
endif()
