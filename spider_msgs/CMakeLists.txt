cmake_minimum_required(VERSION 3.8)
project(spider_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/IK.srv"
  "msg/ControlModeSystem.msg"


  DEPENDENCIES
  geometry_msgs)

ament_export_dependencies(rosidl_default_runtime geometry_msgs)
ament_package()

