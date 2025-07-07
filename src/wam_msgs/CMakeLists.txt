cmake_minimum_required(VERSION 2.8.3)
project(wam_msgs)

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
  RTJointPositions.msg
  RTCartPosition.msg
  RTCartOrientation.msg
  RTCartPose.msg
  RTLinearVelocity.msg
  RTAngularVelocity.msg
  RTLinearandAngularVelocity.msg
  RTJointVelocities.msg
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime)

