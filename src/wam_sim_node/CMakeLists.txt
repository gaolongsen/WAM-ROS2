 cmake_minimum_required(VERSION 2.8.3)
 project(wam_sim_node)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
  moveit_commander moveit_core
  moveit_fake_controller_manager
  moveit_msgs
  moveit_planners_ompl
  moveit_ros_control_interface 
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  wam_srvs
  wam_msgs
  moveit_ros_visualization
  moveit_ros_warehouse
  moveit_setup_assistant
  moveit_simple_controller_manager
  rqt_moveit
  trajectory_msgs
  tf
  tf_conversions
  eigen_conversions
  geometry_msgs
  sensor_msgs
  std_srvs
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
   moveit_core
   roscpp
   moveit_commander moveit_core
   moveit_fake_controller_manager
   moveit_msgs
   geometry_msgs
   sensor_msgs
   moveit_planners_ompl
   moveit_ros_control_interface 
   moveit_ros_manipulation
   moveit_ros_move_group
   moveit_ros_perception
   moveit_ros_planning
   moveit_ros_planning_interface
   moveit_ros_robot_interaction
   moveit_ros_visualization
   moveit_ros_warehouse
   moveit_setup_assistant
   moveit_simple_controller_manager
   rqt_moveit
   tf
   tf_conversions
   eigen_conversions
   std_srvs
)
find_package(Barrett REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(
   -std=c++11
 )

add_executable(wam_sim_node  
    src/wam_sim_node.cpp
)

target_link_libraries(wam_sim_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

add_executable(bhand_sim_node  
    src/bhand_sim_node.cpp
)

target_link_libraries(bhand_sim_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)
