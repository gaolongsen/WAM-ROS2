#include "wam_sim_node.h"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <iostream>

template <size_t DOF>
int wam_main(bool found_hand, bool found_fts) {
  ros::NodeHandle n;
  // Publisher for sending Joint States to simulated WAM
  // auto joint_position_publisher =
  // n.advertise<sensor_msgs::JointState>("/joint_states", 100);
  auto joint_trajectory_publisher =
      n.advertise<trajectory_msgs::JointTrajectory>(
          "/joint_trajectory_controller/command", 100);
  // Moveit planning interface for WAM.
  moveit::planning_interface::MoveGroupInterface wam_interface("wam");
  // Moveit planning interface for Bhand.
  Wam_Node<DOF> wam_node(wam_interface, joint_trajectory_publisher, found_hand);
  // AsyncSpinner with 2 threads for proper communication with Moveit Motion
  // Planning
  ros::AsyncSpinner spinner(2);
  spinner.start();
  // Robot State, used for IK and Jacobian
  wam_node.wam_state_ = wam_interface.getCurrentState();
  spinner.stop();
  // Loop Rate for ROS spin
  ros::Rate loop_rate(kPublishFrequency);
  // Wam Publishers
  wam_node.tool_pose_publisher_ =
      n.advertise<geometry_msgs::PoseStamped>("/wam/ToolPose", 100);
  wam_node.tool_velocity_publisher_ =
      n.advertise<geometry_msgs::Twist>("/wam/toolVelocity", 100);
  wam_node.joint_velocity_publisher_ =
      n.advertise<sensor_msgs::JointState>("/wam/jointVelocity", 100);
  /*Publisher for Trajectory Status. Checks if Velocity > 0 to determine if a
    trajectory is active. Publishes 'true' if a trajectory is active */
  wam_node.trajectory_status_publisher_ =
      n.advertise<std_msgs::Bool>("/wam/trajectoryStatus", 100);

  // Wam Services
  ros::ServiceServer joint_move_srv = n.advertiseService(
      "/wam/moveToJointPosition", &Wam_Node<DOF>::jointMoveCb, &wam_node);
  ros::ServiceServer cart_ortn_move_srv =
      n.advertiseService("/wam/moveToCartOrientation",
                         &Wam_Node<DOF>::cartOrientationCb, &wam_node);
  ros::ServiceServer cart_position_move_srv = n.advertiseService(
      "/wam/moveToCartPosition", &Wam_Node<DOF>::cartPositionMoveCb, &wam_node);
  ros::ServiceServer pose_move_srv = n.advertiseService(
      "/wam/moveToCartPose", &Wam_Node<DOF>::cartPoseMoveCb, &wam_node);
  ros::ServiceServer home_srv = n.advertiseService(
      "/wam/moveHome", &Wam_Node<DOF>::moveHomeCb, &wam_node);
  ros::ServiceServer idle_srv =
      n.advertiseService("/wam/idle", &Wam_Node<DOF>::idleCb, &wam_node);
  ros::ServiceServer hold_cart_position_srv = n.advertiseService(
      "/wam/holdCartPosition", &Wam_Node<DOF>::holdCartPositionCb, &wam_node);
  ros::ServiceServer hold_joint_pos_srv = n.advertiseService(
      "/wam/holdJointPosition", &Wam_Node<DOF>::holdJointPositionCb, &wam_node);
  ros::ServiceServer hold_cart_orientation_srv =
      n.advertiseService("/wam/holdCartOrientation",
                         &Wam_Node<DOF>::holdCartOrientationCb, &wam_node);
  ros::ServiceServer hold_cart_pose = n.advertiseService(
      "/wam/holdCartPose", &Wam_Node<DOF>::holdCartPoseCb, &wam_node);
  ros::ServiceServer gravity_comp_srv = n.advertiseService(
      "/wam/gravityCompensate", &Wam_Node<DOF>::gravityCompensateCb, &wam_node);
  ros::ServiceServer velocity_limit_srv = n.advertiseService(
      "/wam/setVelocityLimit", &Wam_Node<DOF>::setVelocityLimitCb, &wam_node);
  // FOR TESTING ONLY. Moves WAM Simulation to a Random Position
  ros::ServiceServer rand_pos_move_srv = n.advertiseService(
      "/wam/moveToRandomJointPosition", &Wam_Node<DOF>::randomMoveCb, &wam_node);
  // Wam Subscribers for Real-Time control
  ros::Subscriber cur_joint_pos_sub = n.subscribe(
      "/joint_states", 100, &Wam_Node<DOF>::currentJointPositionCb, &wam_node);
  ros::Subscriber rt_joint_position_sub =
      n.subscribe("/wam/RTJointPositionCMD", 100,
                  &Wam_Node<DOF>::rtJointPositionCb, &wam_node);
  ros::Subscriber rt_pose_velocty_sub =
      n.subscribe("/wam/RTLinearandAngularVelocityCMD", 1000,
                  &Wam_Node<DOF>::rtLinearAngularVelocityCb, &wam_node);
/*   ros::Subscriber rt_angular_velocity_sub =
      n.subscribe("/wam/RTAngularVelocityCMD", 100,
                  &Wam_Node<DOF>::rtAngularVelocityCb, &wam_node);
  ros::Subscriber rt_linear_velocity_sub =
      n.subscribe("/wam/RTLinearVelocityCMD", 100,
                  &Wam_Node<DOF>::rtLinearVelocityCb, &wam_node); */
  ros::Subscriber rt_cart_position_sub =
      n.subscribe("/wam/RTCartPositionCMD", 100,
                  &Wam_Node<DOF>::rtCartPositionCb, &wam_node);
  ros::Subscriber rt_cart_pose_sub = n.subscribe(
      "/wam/RTCartPoseCMD", 100, &Wam_Node<DOF>::rtCartPoseCb, &wam_node);
  ros::Subscriber rt_cart_orientation_sub =
      n.subscribe("/wam/RTCartOrientationCMD", 100,
                  &Wam_Node<DOF>::rtCartOrientationCb, &wam_node);
  ros::Subscriber rt_joint_velocity_sub =
      n.subscribe("/wam/RTJointVelocityCMD", 100,
                  &Wam_Node<DOF>::rtJointVelocityCb, &wam_node);
  //Publish thread, allows for publishing when blocking services are called
  std::thread publishThread(&Wam_Node<DOF>::publishWAM, &wam_node);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wam_node_sim");
  /*argv[1] specifies the configuration of WAM Simulation
    config types:
     1 -> 4 DOF link
     2 -> 4 DOF link with Barrett Hand
     3 -> 4 DOF link with Force/Torque Sensor (FTS not yet working with
    simulation) 4 -> 4 DOF link with Barrett Hand and Force/Torque Sensor 5 -> 7
    DOF link 6 -> 7 DOF link with Barrett Hand 7 -> 7 DOF link with Force/Torque
    Sensor 8 -> 7 DOF link with Barrett Hand and Force/Torque Sensorz */
  if (argc == 2) {
    int config = atoi(argv[1]);
    switch (config) {
      case 1:
        ROS_INFO("Starting node with 4DOF Link");
        return wam_main<4>(false, false);
        break;
      case 2:
        ROS_INFO("Starting node with 4DOF Link and Barrett Hand");
        return wam_main<4>(true, false);
        break;
      case 3:
        ROS_INFO("Starting node with 4DOF Link and Force/Torque Sensor");
        return wam_main<4>(false, true);
        break;
      case 4:
        ROS_INFO(
            "Starting node with 4DOF Link Barrett Hand and Force/Torque "
            "Sensor");
        return wam_main<4>(true, true);
        break;
      case 5:
        ROS_INFO("Starting node with 7DOF Link");
        return wam_main<7>(false, false);
        break;
      case 6:
        ROS_INFO("Starting node with 7DOF Link and Barrett Hand");
        return wam_main<7>(true, false);
        break;
      case 7:
        ROS_INFO("Starting node with 7DOF Link and Force/Torque Sensor");
        return wam_main<7>(false, true);
        break;
      case 8:
        ROS_INFO(
            "Starting node with 7DOF Link Barrett Hand and Force/Torque "
            "Sensor");
        return wam_main<7>(true, true);
        break;
      default:
        ROS_WARN("Invalid argument specified. Defaulting to 4DOF link");
        return wam_main<4>(false, false);
        break;
    }
  } else {
    ROS_WARN("No argument specified. Defaulting to 4DOF link");
  }
}