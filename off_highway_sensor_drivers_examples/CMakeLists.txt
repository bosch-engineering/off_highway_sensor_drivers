cmake_minimum_required(VERSION 3.16)
project(off_highway_sensor_drivers_examples)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/off_highway_radar_examples/extract_velocity.cpp
)

target_link_libraries(off_highway_sensor_drivers_examples PUBLIC
      ${geometry_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      rclcpp::rclcpp
      rclcpp_components::component
      rclcpp_components::component_manager
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "off_highway_sensor_drivers_examples::ExtractVelocity"
  EXECUTABLE extract_velocity
)

# Resources
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
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
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_components
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
