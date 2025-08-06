cmake_minimum_required(VERSION 3.16)
project(off_highway_premium_radar_sample_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wdeprecated)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DiagnosticsEthernetConfigurationInformation.msg"
  "msg/DoIpInformation.msg"
  "msg/EgoVehicleData.msg"
  "msg/EgoVehicleInput.msg"
  "msg/InterferenceIndicator.msg"
  "msg/LocationAttributes.msg"
  "msg/LocationAttributesHeader.msg"
  "msg/LocationAttributesPacket.msg"
  "msg/LocationDataHeader.msg"
  "msg/LocationDataPacket.msg"
  "msg/MeasurementCycleSyncData.msg"
  "msg/MisalignmentPacket.msg"
  "msg/OperationMode.msg"
  "msg/SensorBroadcast.msg"
  "msg/SensorBroadcastData.msg"
  "msg/SensorDtcInformation.msg"
  "msg/SensorEthernetConfigurationInformation.msg"
  "msg/SensorFeedback.msg"
  "msg/SensorFieldOfView.msg"
  "msg/SensorModulationPerformance.msg"
  "msg/SensorStateInformation.msg"
  "msg/Time.msg"
  "srv/MeasurementProgram.srv"
  "srv/SensorModeRequest.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
