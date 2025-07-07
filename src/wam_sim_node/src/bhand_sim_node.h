#ifndef SRC_WAM_MOVEIT_NODE_SRC_BHAND_SIM_NODE_H_
#define SRC_WAM_MOVEIT_NODE_SRC_BHAND_SIM_NODE_H_
// Bhand Custom srvs
#include <thread>
#include "bhand_srvs/FingerPosition.h"
#include "bhand_srvs/FingerVelocity.h"
#include "bhand_srvs/GraspPosition.h"
#include "bhand_srvs/GraspVelocity.h"
#include "std_srvs/Trigger.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "bhand_srvs/SpreadPosition.h"
#include "bhand_srvs/SpreadVelocity.h"
#include "sensor_msgs/JointState.h"
#include <moveit/move_group_interface/move_group_interface.h>

const double kGoalTolerance = 0.01; //defines goal tolerance for moveit control
const double kPublishFrequency = 250;
double kMaxVelocityScalingFactor = 1;  //Set a scaling factor for optionally reducing the maximum joint velocity in moveit
std::vector<std::string> kBhandJointNames = {
    "bhand_j11_joint", "bhand_j12_joint", "bhand_j13_joint", "bhand_j21_joint",
    "bhand_j22_joint", "bhand_j23_joint", "bhand_j32_joint", "bhand_j33_joint"};
std::vector<double> kBhandUpperJointLimits = {3.14159265, 2.44346095, 2.44346095, 0, 2.44346095, 2.44346095, 2.44346095, 2.44346095};
std::vector<double> kBhandLowerJointLimits = {0, 0, 0, -3.14159265, 0, 0, 0, 0};

template <size_t DOF>
class Bhand_Node {
 public:
  /** Callback for /bhand/closeGrasp service.
   */
  bool closeGraspCb(std_srvs::Trigger::Request& request,
                    std_srvs::Trigger::Response& response);

  /** Callback for /bhand/closeSpread service.
   */
  bool closeSpreadCb(std_srvs::Trigger::Request& request,
                     std_srvs::Trigger::Response& response);

  /** Callback for /bhand/moveToFingerPosition service.
   */
  bool fingerPositionCb(bhand_srvs::FingerPosition::Request& request,
                        bhand_srvs::FingerPosition::Response& response);

  /** Callback for /bhand/moveToFingerVelocity service.
   */
  bool fingerVelocityCb(bhand_srvs::FingerVelocity::Request& request,
                        bhand_srvs::FingerVelocity::Response& response);

  /** Callback for /bhand/moveToGraspPosition service.
   */
  bool graspPositionCb(bhand_srvs::GraspPosition::Request& request,
                       bhand_srvs::GraspPosition::Response& response);

  /** Callback for /bhand/moveToGraspVelocity service.
   */
  bool graspVelocityCb(bhand_srvs::GraspVelocity::Request& request,
                       bhand_srvs::GraspVelocity::Response& response);

  /** Callback for /bhand/openGrasp service.
   */
  bool openGraspCb(std_srvs::Trigger::Request& request,
                   std_srvs::Trigger::Response& response);

  /** Callback for /bhand/openSpread service.
   */
  bool openSpreadCb(std_srvs::Trigger::Request& request,
                    std_srvs::Trigger::Response& response);

  /** Callback for /bhand/moveToSpreadPosition service.
   */
  bool spreadPositionCb(bhand_srvs::SpreadPosition::Request& request,
                        bhand_srvs::SpreadPosition::Response& response);

  /** Callback for /bhand/moveToSpreadVelocity service.
   */
  bool spreadVelocityCb(bhand_srvs::SpreadVelocity::Request& request,
                        bhand_srvs::SpreadVelocity::Response& response);
  /** Callback for /bhand/idle service.
   */
  bool idleCb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  // Subscriber callback to monitor /joint_states topic
  void currentJointPositionCb(const sensor_msgs::JointState::ConstPtr& msg);

  Bhand_Node(moveit::planning_interface::MoveGroupInterface& bhand_interface__,
             ros::Publisher bhand_joint_trajectory_publisher__, bool standalone_bhand);

 protected:
  /**function to set value of a joint indexed by name in a
   * sensor_msgs::JointState message */
  void setValue(sensor_msgs::JointState& joint_state, std::string joint_name, double value);

  /**function to check if a joint indexed by its name is within its upper and
   * lower limit*/
  bool checkJointLimits(sensor_msgs::JointState& joint_state,
                        std::string joint_name, double upper_limit,
                        double lower_limit);
  /**Represents index of Bhand joint states in the /joint_state topic.
   * If DOF is 4 or 7, Bhand is connected to WAM in simulation,
   * bhand_jp_index_ = 4 or 7 to ignore WAM joint states If no WAM present in 
   * simulation, bhand_jp_index = 0.
   */
  int bhand_jp_index_;
  // True if Bhand Velocity control is active. Set to false after Bhand/idle
  // called.
  bool velocity_status_;
  void jointVelocityThreadCb(std::vector<double> goal_velocity);
  trajectory_msgs::JointTrajectory setTrajectoryMsg(std::vector<double> goal_jp, std::vector<double> start_jp);
  sensor_msgs::JointState current_joint_position_;
  /**moveit planning interface for barrett hand*/
  moveit::planning_interface::MoveGroupInterface& bhand_interface_;
  ros::Publisher bhand_joint_trajectory_publisher_;
};

template <size_t DOF>
Bhand_Node<DOF>::Bhand_Node(
    moveit::planning_interface::MoveGroupInterface& bhand_interface__, ros::Publisher bhand_joint_trajectory_publisher__, bool standalone_bhand)
    : bhand_interface_(bhand_interface__), bhand_joint_trajectory_publisher_(bhand_joint_trajectory_publisher__) {
  bhand_interface_.setPoseReferenceFrame("bhand_base");
  bhand_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
  bhand_interface_.setGoalTolerance(kGoalTolerance);
  bhand_interface_.startStateMonitor();
  //Set bhand_jp_index_ based on DOF
  if (DOF == 4) {
    bhand_jp_index_ = 4;
  } else if (DOF == 7) {
    bhand_jp_index_ = 7;
  } else {
    bhand_jp_index_ = 0;
  }
  current_joint_position_.position.resize(kBhandJointNames.size());
  current_joint_position_.name.resize(kBhandJointNames.size());
  velocity_status_ = false;
  //if not standalone_bhand, wam_node will move bhand to startup position
  if (standalone_bhand) {
    sensor_msgs::JointState bhand_goal_joint_position;
    // move to open spread at startup
    bhand_goal_joint_position.position.resize(kBhandJointNames.size());
    bhand_goal_joint_position.name = kBhandJointNames;

    // bhand_j11_joint set to 3.14 and bhand_j21_joint set to -3.14
    bhand_goal_joint_position.position[0] = 3.14;
    bhand_goal_joint_position.position[3] = -3.14;
    bhand_interface_.setStartStateToCurrentState();
    bhand_interface_.setJointValueTarget(bhand_goal_joint_position);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    bhand_interface_.move();  // plan and move to target position
    spinner.stop();
  }
}

template <size_t DOF>
void Bhand_Node<DOF>::setValue(sensor_msgs::JointState& joint_state, std::string joint_name, double value) {
  int index = 0;
  while (joint_state.name[index].compare(joint_name) != 0) {
    index = index + 1;
  }
  joint_state.position[index] = value;
}


template <size_t DOF>
bool Bhand_Node<DOF>::checkJointLimits(sensor_msgs::JointState& joint_state,
                                       std::string joint_name,
                                       double upper_limit, double lower_limit) {
  int index = 0;
  while (joint_state.name[index].compare(joint_name) != 0) {
    index = index + 1;
  }
  if (joint_state.position[index] > upper_limit || joint_state.position[index] < lower_limit) {
    return false;
  } else {
    return true;
  }
}
// set current_joint_position_ to bhand joint states, ignoring WAM joint states
template <size_t DOF>
void Bhand_Node<DOF>::currentJointPositionCb(
    const sensor_msgs::JointState::ConstPtr& msg) {
  //Check if message has only WAM joint states
  if (msg->name.size() == DOF) {
    //Do nothing
  } else if (msg->name[0].compare(kBhandJointNames[0]) == 0) { //check if message has Bhand Joint states first
    // only Bhand joint states being published
    for (int i = 0; i < kBhandJointNames.size(); i++) {
      current_joint_position_.position[i] = msg->position[i];
      current_joint_position_.name[i] = msg->name[i];
    }
  } else { //both WAM and Bhand joint states published
    for (int i = 0; i < kBhandJointNames.size(); i++) {
      current_joint_position_.position[i] = msg->position[bhand_jp_index_ + i];
      current_joint_position_.name[i] = msg->name[bhand_jp_index_ + i];
    }
  }
}

template <size_t DOF>
trajectory_msgs::JointTrajectory Bhand_Node<DOF>::setTrajectoryMsg(std::vector<double> goal_jp, std::vector<double> start_jp) {
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.resize(kBhandJointNames.size());
  trajectory_msgs::JointTrajectoryPoint point1, point2;
  point1.positions.resize(kBhandJointNames.size());
  point2.positions.resize(kBhandJointNames.size());
  for (int i = 0; i < (int)kBhandJointNames.size(); i++) {
    joint_trajectory.joint_names[i] = kBhandJointNames[i];
    point1.positions[i] = start_jp[i];
    point2.positions[i] = goal_jp[i];
  }
  point1.time_from_start = ros::Duration(0);
  point2.time_from_start = ros::Duration(0.02);
  joint_trajectory.points.push_back(point1);
  joint_trajectory.points.push_back(point2);
  return joint_trajectory;
}

template <size_t DOF>
void Bhand_Node<DOF>::jointVelocityThreadCb(std::vector<double> goal_velocity) {
  // Check if limits reached, check if velocity status is true. If limits
  // reached set status to false
  bool limit_reached = false;
  sensor_msgs::JointState goal_jp;
  velocity_status_ = true;
  goal_jp.position.resize(kBhandJointNames.size());
  goal_jp.name.resize(kBhandJointNames.size());
  ros::Rate loop_rate(50);
  while (velocity_status_) {
    for (int i = 0; i < kBhandJointNames.size(); i++) {
      double position = 0;
      // Current Joint Position may not be in correct order. Find the correct
      // joint position
      for (int j = 0; j < kBhandJointNames.size(); j++) {
        if (current_joint_position_.name[j].compare(kBhandJointNames[i]) == 0) {
          position = current_joint_position_.position[j];
        }
      }
      // Apply goal velocity for 0.02 seconds (50Hz). Keep doing so until limit reached
      // or velocity_status_ set to false
      goal_jp.position.at(i) = position + (goal_velocity.at(i) * 0.02);
      goal_jp.name.at(i) = kBhandJointNames.at(i);
    }
    bhand_joint_trajectory_publisher_.publish(
        setTrajectoryMsg(goal_jp.position, current_joint_position_.position));
    for (int i = 0; i < kBhandJointNames.size(); i++) {
      if (!checkJointLimits(goal_jp, kBhandJointNames[i],
                            kBhandUpperJointLimits[i] + kGoalTolerance,
                            kBhandLowerJointLimits[i] - kGoalTolerance)) {
        limit_reached = true;
      }
    }
    loop_rate.sleep();
  }
}

template <size_t DOF>
bool Bhand_Node<DOF>::closeGraspCb(std_srvs::Trigger::Request& request,
                              std_srvs::Trigger::Response& response) {
  bhand_interface_.stop();
  velocity_status_ = false;
  ros::AsyncSpinner spinner(2);
  sensor_msgs::JointState goal_jp = current_joint_position_;
  //bhand_j12_joint, bhand_j22_joint, bhand_j32_joint to 2.39
  //bhand_j13_joint, bhand_j23_joint, bhand_j33_joint to 0.7975
  setValue(goal_jp, "bhand_j12_joint", 2.39);
  setValue(goal_jp, "bhand_j22_joint", 2.39);
  setValue(goal_jp, "bhand_j32_joint", 2.39);
  setValue(goal_jp, "bhand_j13_joint", 0.7975);
  setValue(goal_jp, "bhand_j23_joint", 0.7975);
  setValue(goal_jp, "bhand_j33_joint", 0.7975);
  
  spinner.start();
  bhand_interface_.setStartStateToCurrentState();
  bhand_interface_.setJointValueTarget(goal_jp);
  moveit::planning_interface::MoveGroupInterface::Plan bhand_plan;
  bool success = (bhand_interface_.plan(bhand_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("Closing BarrettHand Grasp");
    // execute plan in a non blocking manner. Allows for stopping trajectory by
    // calling /bhand/idle
    bhand_interface_.execute(bhand_plan);
    response.success = true;
  } else {
    ROS_ERROR("Could not close BarrettHand Grasp");
    response.success = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Bhand_Node<DOF>::closeSpreadCb(
    std_srvs::Trigger::Request& request,
    std_srvs::Trigger::Response& response) {
  bhand_interface_.stop();
  velocity_status_ = false;
  ros::AsyncSpinner spinner(2);
  sensor_msgs::JointState goal_jp = current_joint_position_;
  //bhand_j11_joint set to 3.14 and bhand_j21_joint set to -3.14
  setValue(goal_jp, "bhand_j11_joint", 3.14);
  setValue(goal_jp, "bhand_j21_joint", -3.14);
  spinner.start();
  bhand_interface_.setStartStateToCurrentState();
  bhand_interface_.setJointValueTarget(goal_jp);
  moveit::planning_interface::MoveGroupInterface::Plan bhand_plan;
  bool success = (bhand_interface_.plan(bhand_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("Closing BarrettHand Spread");
    // execute plan in a non blocking manner. Allows for stopping trajectory by
    // calling /bhand/idle
    bhand_interface_.execute(bhand_plan);
    response.success = true;
  } else {
    ROS_ERROR("Could not close BarrettHand Spread");
    response.success = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Bhand_Node<DOF>::fingerPositionCb(
    bhand_srvs::FingerPosition::Request& request,
    bhand_srvs::FingerPosition::Response& response) {
  bhand_interface_.stop();
  velocity_status_ = false;
  ros::AsyncSpinner spinner(2);
  sensor_msgs::JointState goal_jp = current_joint_position_;
  //bhand_j11_joint set to 3.14 and bhand_j21_joint set to -3.14
  setValue(goal_jp, "bhand_j12_joint", request.position[0]);
  setValue(goal_jp, "bhand_j22_joint", request.position[1]);
  setValue(goal_jp, "bhand_j32_joint", request.position[2]);
  spinner.start();
  bhand_interface_.setStartStateToCurrentState();
  bhand_interface_.setJointValueTarget(goal_jp);
  moveit::planning_interface::MoveGroupInterface::Plan bhand_plan;
  bool success = (bhand_interface_.plan(bhand_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("Moving BarrettHand to Finger Positions %.3f, %.3f, %.3f radians",
             request.position[0], request.position[1], request.position[2]);
    // execute plan in a non blocking manner. Allows for stopping trajectory by
    // calling /bhand/idle
    bhand_interface_.execute(bhand_plan);
    response.response = true;
  } else {
    ROS_ERROR("Could not move BarrettHand to Finger Positions %.3f, %.3f, %.3f radians",
             request.position[0], request.position[1], request.position[2]);
    response.response = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Bhand_Node<DOF>::fingerVelocityCb(
    bhand_srvs::FingerVelocity::Request& request,
    bhand_srvs::FingerVelocity::Response& response) {
  bhand_interface_.stop();
  velocity_status_ = false;  // stop any existing velocity control
  std::vector<double> goal_velocity;
  goal_velocity.resize(kBhandJointNames.size());
  if (request.velocity.size() == 3) {
    // bhand_j12_joint, bhand_j22_joint and bhand_j32_joint velocities set to requested value.
    goal_velocity[1] = request.velocity[0];
    goal_velocity[4] = request.velocity[1];
    goal_velocity[6] = request.velocity[2];
    velocity_status_ = true;
    ROS_INFO("Moving BarrettHand to Finger Velocities: %.3f, %.3f, %.3f rad/s",
             request.velocity[0], request.velocity[1], request.velocity[2]);
    std::thread jointVelocityThread(&Bhand_Node::jointVelocityThreadCb, this,
                                    goal_velocity);
    jointVelocityThread.detach();
    response.response = true;
  } else {
    ROS_ERROR("Invalid Finger Velocity Commands sent");
  }
  return true;
}

template <size_t DOF>
bool Bhand_Node<DOF>::graspPositionCb(
    bhand_srvs::GraspPosition::Request& request,
    bhand_srvs::GraspPosition::Response& response) {
  bhand_interface_.stop();
  velocity_status_ = false;
  ros::AsyncSpinner spinner(2);
  sensor_msgs::JointState goal_jp = current_joint_position_;
  //bhand_j11_joint set to 3.14 and bhand_j21_joint set to -3.14
  setValue(goal_jp, "bhand_j12_joint", request.position);
  setValue(goal_jp, "bhand_j22_joint", request.position);
  setValue(goal_jp, "bhand_j32_joint", request.position);
  setValue(goal_jp, "bhand_j13_joint", request.position*0.33343);
  setValue(goal_jp, "bhand_j23_joint", request.position*0.33343);
  setValue(goal_jp, "bhand_j33_joint", request.position*0.33343);
  spinner.start();
  bhand_interface_.setStartStateToCurrentState();
  bhand_interface_.setJointValueTarget(goal_jp);
  moveit::planning_interface::MoveGroupInterface::Plan bhand_plan;
  bool success = (bhand_interface_.plan(bhand_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("Moving BarrettHand Grasp: %.3f radians", request.position);
    // execute plan in a non blocking manner. Allows for stopping trajectory by
    // calling /bhand/idle
    bhand_interface_.execute(bhand_plan);
    response.response = true;
  } else {
    ROS_ERROR("Could not open BarrettHand Grasp");
    response.response = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Bhand_Node<DOF>::graspVelocityCb(
    bhand_srvs::GraspVelocity::Request& request,
    bhand_srvs::GraspVelocity::Response& response) {
  bhand_interface_.stop();
  velocity_status_ = false;  // stop any existing velocity control
  std::vector<double> goal_velocity;
  goal_velocity.resize(kBhandJointNames.size());
  // bhand_j12_joint, bhand_j22_joint and bhand_j32_joint velocities set to
  // requested value.
  goal_velocity[1] = request.velocity;
  goal_velocity[4] = request.velocity;
  goal_velocity[6] = request.velocity;
  // bhand_j13_joint, bhand_j23_joint and bhand_j33_joint velocities set to
  // requested value * 0.33343.
  goal_velocity[2] = request.velocity * 0.33343;
  goal_velocity[5] = request.velocity * 0.33343;
  goal_velocity[7] = request.velocity * 0.33343;
  velocity_status_ = true;
  std::thread jointVelocityThread(&Bhand_Node::jointVelocityThreadCb, this,
                                  goal_velocity);
  jointVelocityThread.detach();
  ROS_INFO("Moving BarrettHand Grasp Velocity: %.3f rad/s", request.velocity);
  return true;
}

template <size_t DOF>
bool Bhand_Node<DOF>::openGraspCb(std_srvs::Trigger::Request& request,
                                  std_srvs::Trigger::Response& response) {
  bhand_interface_.stop();
  velocity_status_ = false;
  ros::AsyncSpinner spinner(2);
  sensor_msgs::JointState goal_jp = current_joint_position_;
  //bhand_j12_joint, bhand_j22_joint, bhand_j32_joint to 0
  //bhand_j13_joint, bhand_j23_joint, bhand_j33_joint to 0
  setValue(goal_jp, "bhand_j12_joint", 0);
  setValue(goal_jp, "bhand_j22_joint", 0);
  setValue(goal_jp, "bhand_j32_joint", 0);
  setValue(goal_jp, "bhand_j13_joint", 0);
  setValue(goal_jp, "bhand_j23_joint", 0);
  setValue(goal_jp, "bhand_j33_joint", 0);
  spinner.start();
  bhand_interface_.setStartStateToCurrentState();
  bhand_interface_.setJointValueTarget(goal_jp);
  moveit::planning_interface::MoveGroupInterface::Plan bhand_plan;
  bool success = (bhand_interface_.plan(bhand_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("Opening BarrettHand Grasp");
    // execute plan in a non blocking manner. Allows for stopping trajectory by
    // calling /bhand/idle
    bhand_interface_.execute(bhand_plan);
    response.success = true;
  } else {
    ROS_ERROR("Could not open BarrettHand Grasp");
    response.success = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Bhand_Node<DOF>::openSpreadCb(std_srvs::Trigger::Request& request,
                                   std_srvs::Trigger::Response& response) {
  bhand_interface_.stop();
  velocity_status_ = false;
  ros::AsyncSpinner spinner(2);
  sensor_msgs::JointState goal_jp = current_joint_position_;
  //bhand_j11_joint and bhand_j21_joint set to 0
  setValue(goal_jp, "bhand_j11_joint", 0);
  setValue(goal_jp, "bhand_j21_joint", 0);
  spinner.start();
  bhand_interface_.setStartStateToCurrentState();
  bhand_interface_.setJointValueTarget(goal_jp);
  moveit::planning_interface::MoveGroupInterface::Plan bhand_plan;
  bool success = (bhand_interface_.plan(bhand_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("Opening BarrettHand Spread");
    // execute plan in a non blocking manner. Allows for stopping trajectory by
    // calling /bhand/idle
    bhand_interface_.execute(bhand_plan);
    response.success = true;
  } else {
    ROS_ERROR("Could not open BarrettHand Spread");
    response.success = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Bhand_Node<DOF>::spreadPositionCb(
    bhand_srvs::SpreadPosition::Request& request,
    bhand_srvs::SpreadPosition::Response& response) {
  bhand_interface_.stop();
  velocity_status_ = false;
  ros::AsyncSpinner spinner(2);
  sensor_msgs::JointState goal_jp = current_joint_position_;
  //bhand_j11_joint set to 3.14 and bhand_j21_joint set to -3.14
  setValue(goal_jp, "bhand_j11_joint", request.position);
  setValue(goal_jp, "bhand_j21_joint", -request.position);
  spinner.start();
  bhand_interface_.setStartStateToCurrentState();
  bhand_interface_.setJointValueTarget(goal_jp);
  moveit::planning_interface::MoveGroupInterface::Plan bhand_plan;
  bool success = (bhand_interface_.plan(bhand_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("Moving BarrettHand Spread: %.3f radians", request.position);
    // execute plan in a non blocking manner. Allows for stopping trajectory by
    // calling /bhand/idle
    bhand_interface_.execute(bhand_plan);
    response.response = true;
  } else {
    ROS_ERROR("Could not move BarrettHand Spread to %.3f radians", request.position);
    response.response = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Bhand_Node<DOF>::spreadVelocityCb(
    bhand_srvs::SpreadVelocity::Request& request,
    bhand_srvs::SpreadVelocity::Response& response) {
  bhand_interface_.stop();
  velocity_status_ = false;  // stop any existing velocity control
  std::vector<double> goal_velocity;
  goal_velocity.resize(kBhandJointNames.size());
  // bhand_j11_joint, bhand_j21_joint velocities set to requested value.
  goal_velocity[0] = request.velocity;
  goal_velocity[3] = request.velocity;
  velocity_status_ = true;
  std::thread jointVelocityThread(&Bhand_Node::jointVelocityThreadCb, this,
                                  goal_velocity);
  jointVelocityThread.detach();
  ROS_INFO("Moving BarrettHand Spread Velocity: %.3f rad/s", request.velocity);
  return true;
}

template <size_t DOF>
bool Bhand_Node<DOF>::idleCb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  bhand_interface_.stop();  // stop any trajectory execution, if one is active
  response.success = true;
  spinner.stop();
  return true;
}
#endif //SRC_WAM_MOVEIT_NODE_SRC_BHAND_SIM_NODE_H_