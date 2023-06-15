cmake_minimum_required(VERSION 3.16)
project(off_highway_general_purpose_radar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated)
endif()

find_package(PCL REQUIRED COMPONENTS common)

find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_updater
    roscpp
    off_highway_common
    off_highway_general_purpose_radar_msgs
    pcl_conversions
    pcl_ros
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    diagnostic_updater
    roscpp
    off_highway_common
    off_highway_general_purpose_radar_msgs
    pcl_conversions
    pcl_ros
  DEPENDS
)

include_directories(
  include
)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/receiver.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_receiver
  src/receiver_node.cpp
)

add_dependencies(${PROJECT_NAME}_receiver
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_receiver
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_receiver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_receiver test/test_receiver.test test/test_receiver.cpp)
  target_include_directories(test_receiver PUBLIC include ${catkin_INCLUDE_DIRS})
  target_link_libraries(test_receiver ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
