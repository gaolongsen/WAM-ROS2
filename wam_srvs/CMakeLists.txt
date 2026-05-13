cmake_minimum_required(VERSION 3.8)
project(wam_srvs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/BHandFingerPos.srv
  srv/BHandFingerVel.srv
  srv/BHandGraspPos.srv
  srv/BHandGraspVel.srv
  srv/BHandSpreadPos.srv
  srv/BHandSpreadVel.srv
  srv/CartPosMove.srv
  srv/GravityComp.srv
  srv/Hold.srv
  srv/JointMove.srv
  srv/OrtnMove.srv
  srv/PoseMove.srv
  srv/LASERControl.srv
  srv/LEDControl.srv
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
