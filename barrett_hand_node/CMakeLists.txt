cmake_minimum_required(VERSION 3.8)
project(barrett_hand_node)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wam_msgs REQUIRED)
find_package(wam_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Barrett QUIET)

if(Barrett_FOUND)
  add_executable(barrett_hand_node src/barrett_hand_node_ros2.cpp)
  set_target_properties(barrett_hand_node PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
  )
  set(LOCAL_LIBBARRETT_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../libbarrett/include")
  if(EXISTS "${LOCAL_LIBBARRETT_INCLUDE}/barrett")
    target_include_directories(barrett_hand_node BEFORE PRIVATE ${LOCAL_LIBBARRETT_INCLUDE})
  endif()
  target_include_directories(barrett_hand_node PRIVATE
    ${BARRETT_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )
  ament_target_dependencies(barrett_hand_node
    geometry_msgs
    rclcpp
    sensor_msgs
    std_srvs
    wam_msgs
    wam_srvs
  )
  target_link_libraries(barrett_hand_node ${BARRETT_LIBRARIES} ${Boost_LIBRARIES})
  install(TARGETS barrett_hand_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "Barrett CMake package was not found. Skipping standalone BarrettHand executable; install libbarrett to build CAN-bus hand control.")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
