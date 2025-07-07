cmake_minimum_required(VERSION 2.8.3)
project(bhand_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
  trajectory_msgs
)

add_message_files(
  FILES
  TactileState.msg
  TactileStateArray.msg
  FingerTipTorques.msg
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime)

