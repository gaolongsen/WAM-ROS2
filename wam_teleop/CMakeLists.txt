cmake_minimum_required(VERSION 3.8)
project(wam_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wam_msgs REQUIRED)
find_package(wam_srvs REQUIRED)

add_executable(wam_joystick_teleop src/wam_joystick_teleop_ros2.cpp)
target_compile_features(wam_joystick_teleop PUBLIC cxx_std_17)
ament_target_dependencies(wam_joystick_teleop
  rclcpp
  sensor_msgs
  std_srvs
  wam_msgs
  wam_srvs
)

install(TARGETS wam_joystick_teleop DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
