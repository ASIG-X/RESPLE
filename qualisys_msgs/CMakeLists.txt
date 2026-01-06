cmake_minimum_required(VERSION 3.5)
project(mocap4r2_msgs)

find_package(ament_cmake)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/Marker.msg"
  "msg/Markers.msg"
  "msg/ImusInfo.msg"
  "msg/RigidBody.msg"
  "msg/RigidBodies.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

#install(
#  FILES mocap4r2_bridge_mapping_rule.yaml
#  DESTINATION share/${PROJECT_NAME}
#)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)
ament_package()
