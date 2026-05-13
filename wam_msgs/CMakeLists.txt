cmake_minimum_required(VERSION 3.8)
project(wam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/RTCartPos.msg
  msg/FtTorques.msg
  msg/RTOrtnPos.msg
  msg/RTOrtnVel.msg
  msg/RTPose.msg
  msg/TactilePressure.msg
  msg/TactilePressureArray.msg
  msg/RTCartVel.msg
  msg/RTJointPos.msg
  msg/RTJointVel.msg
  msg/EndpointState.msg
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
