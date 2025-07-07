#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <iostream>
#include "bhand_sim_node.h"
#include "gazebo_publisher.h"


template <size_t DOF>
int bhand_main(int argc, char** argv) {
  ros::init(argc, argv, "bhand_sim_node");
  ros::NodeHandle n;
  //initialize gazebo client
  gazebo::client::setup(argc, argv);
  // Create gazebo node
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
	Gazebo_Publishers gazebo_publishers(n.advertise<bhand_msgs::TactileStateArray>("/bhand/TactileStates", 100));
  gazebo::transport::SubscriberPtr sub =
      node->Subscribe("~/bhand/palm/tactile", &Gazebo_Publishers::tactileSensorCb, &gazebo_publishers);
  moveit::planning_interface::MoveGroupInterface bhand_interface(
      "barrett_hand");
  bool standalone_bhand = true;
  auto joint_trajectory_publisher = n.advertise<trajectory_msgs::JointTrajectory>("/bhand_joint_trajectory_controller/command", 100);
  if (atoi(argv[1]) != 0) {
    standalone_bhand = false;
  }
  Bhand_Node<DOF> bhand_node(bhand_interface, joint_trajectory_publisher, standalone_bhand);
  ros::ServiceServer close_grasp_srv = n.advertiseService(
      "/bhand/closeGrasp", &Bhand_Node<DOF>::closeGraspCb, &bhand_node);
  ros::ServiceServer finger_position_srv = n.advertiseService(
      "/bhand/moveToFingerPositions", &Bhand_Node<DOF>::fingerPositionCb, &bhand_node);
  ros::ServiceServer grasp_position_srv = n.advertiseService(
      "/bhand/moveToGraspPosition", &Bhand_Node<DOF>::graspPositionCb, &bhand_node);
  ros::ServiceServer spread_position_cb = n.advertiseService(
      "/bhand/moveToSpreadPosition", &Bhand_Node<DOF>::spreadPositionCb, &bhand_node);
  ros::ServiceServer finger_velocity_cb = n.advertiseService(
      "/bhand/moveToFingerVelocities", &Bhand_Node<DOF>::fingerVelocityCb, &bhand_node);
  ros::ServiceServer grasp_velocity_cb = n.advertiseService(
      "/bhand/moveToGraspVelocity", &Bhand_Node<DOF>::graspVelocityCb, &bhand_node);
  ros::ServiceServer spread_velocity_srv = n.advertiseService(
      "/bhand/moveToSpreadVelocity", &Bhand_Node<DOF>::spreadVelocityCb, &bhand_node);
  ros::ServiceServer open_grasp_srv = n.advertiseService(
      "/bhand/openGrasp", &Bhand_Node<DOF>::openGraspCb, &bhand_node);
  ros::ServiceServer open_spread_srv = n.advertiseService(
      "/bhand/openSpread", &Bhand_Node<DOF>::openSpreadCb, &bhand_node);
  ros::ServiceServer close_spread_srv = n.advertiseService(
      "/bhand/closeSpread", &Bhand_Node<DOF>::closeSpreadCb, &bhand_node);
  ros::ServiceServer idle_srv = n.advertiseService(
      "/bhand/idle", &Bhand_Node<DOF>::idleCb, &bhand_node);
  ros::Subscriber cur_joint_pos_sub = n.subscribe("/joint_states", 100, &Bhand_Node<DOF>::currentJointPositionCb, &bhand_node);
  ros::Rate loop_rate(kPublishFrequency);
  while (ros::ok()) {
    gazebo_publishers.publishMsgs();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

int main(int argc, char** argv) {
  if (argc != 4) {
    ROS_FATAL("No DOF argument specified");
  } else {
    if (atoi(argv[1]) == 4) {
      // 4DOF WAM+BarrettHand
      ROS_INFO("Starting BarrettHand Simulation with 4DOF WAM ");
      return bhand_main<4>(argc, argv);
    } else if (atoi(argv[1]) == 7) {
      // 7DOF WAM+BarrettHand
      ROS_INFO("Starting BarrettHand Simulation with 7DOF WAM");
      return bhand_main<7>(argc, argv);
    } else if (atoi(argv[1]) == 0) {
      ROS_INFO("Starting standalone BarrettHand Simulation");
      return bhand_main<0>(argc, argv);
    }else {
      ROS_FATAL("Invalid DOF argument specified. Enter either 4 7 or 0");
    }
  }
}