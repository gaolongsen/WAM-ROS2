cmake_minimum_required(VERSION 2.8.3)
project(bhand_srvs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
  trajectory_msgs
)

add_service_files(
  FILES
  FingerPosition.srv
  FingerVelocity.srv
  GraspPosition.srv
  GraspVelocity.srv
  SpreadPosition.srv
  SpreadVelocity.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime)

