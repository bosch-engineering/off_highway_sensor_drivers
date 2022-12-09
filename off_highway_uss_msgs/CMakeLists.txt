cmake_minimum_required(VERSION 3.16)
project(off_highway_uss_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
)

add_message_files(
  FILES
    DirectEcho.msg
    DirectEchos.msg
    Echo.msg
    Information.msg
    MaxDetectionRange.msg
    Object.msg
    Objects.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
)
