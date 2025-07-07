#ifndef SRC_WAM_MOVEIT_NODE_SRC_WAM_MOVEIT_NODE_
#define SRC_WAM_MOVEIT_NODE_SRC_WAM_MOVEIT_NODE_

#include <thread>
#include <fstream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_srvs/Empty.h" //for Random Pose Move service
#include "std_srvs/SetBool.h" 
#include "std_srvs/Trigger.h"
#include "std_msgs/Bool.h"

#include "wam_srvs/JointMove.h"
#include "wam_srvs/CartPositionMove.h"
#include "wam_srvs/CartPoseMove.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "wam_srvs/CartOrientationMove.h"
#include "wam_srvs/VelocityLimit.h"
//WAM msgs
#include "wam_msgs/RTJointPositions.h"
#include "wam_msgs/RTJointVelocities.h"
#include "wam_msgs/RTCartPosition.h"
#include "wam_msgs/RTCartOrientation.h"
#include "wam_msgs/RTCartPose.h"
#include "wam_msgs/RTAngularVelocity.h"
#include "wam_msgs/RTLinearVelocity.h"
#include "wam_msgs/RTLinearandAngularVelocity.h"

const double kGoalTolerance = 0.1; //defines goal tolerance for moveit control
double kMaxVelocityScalingFactor = 0.1;  //Set a scaling factor for optionally reducing the maximum joint velocity in moveit
//Paths for Zero Calibration Configuration files
const char kZeroCalConfigPath4[] = "/etc/barrett/calibration_data/wam4/zerocal.conf";
const char kZeroCalConfigPath7[] = "/etc/barrett/calibration_data/wam7w/zerocal.conf";
double kRtTimeout_s = 0.3; //timeout for realtime messages
const double kPublishFrequency = 50;
const double kMaxVelocity = 10; //Maximum joint velocity, as specified in srdf files
std::vector<std::string> kBhandJointNames = {
    "bhand_j11_joint", "bhand_j12_joint", "bhand_j13_joint", "bhand_j21_joint",
    "bhand_j22_joint", "bhand_j23_joint", "bhand_j32_joint", "bhand_j33_joint"};

/** Wam_Node class for controls wam_sim.
*/
template <size_t DOF>
class Wam_Node {
 public:
  robot_state::RobotStatePtr wam_state_;
  ros::Publisher tool_pose_publisher_;
  ros::Publisher tool_velocity_publisher_;
  ros::Publisher joint_velocity_publisher_;
  ros::Publisher joint_trajectory_publisher_;
  ros::ServiceClient bhand_open_grasp_client_;
  ros::ServiceClient bhand_close_spread_client_;
  /*  Publisher for trajectory_status. Trajectory Status is true if Joint Velocity > 0*/
  ros::Publisher trajectory_status_publisher_;

  /** Callback for /wam/moveToJointPosition service. 
   *  Sets the target for wam_interface_ to requested joint position and asyncronously executes it. 
   *  Trajectory can be terminated by calling /wam/idle.
   */
  bool jointMoveCb(wam_srvs::JointMove::Request& request, wam_srvs::JointMove::Response& response);

  /** Callback for /wam/moveToCartOrientation service. 
   *  Sets the target for wam_interface_ to requested cartesian orientation and asyncronously executes the trajectory. 
   *  Trajectory can be terminated by calling /wam/idle.
   */
  bool cartOrientationCb(wam_srvs::CartOrientationMove::Request& request, wam_srvs::CartOrientationMove::Response& response);

  /** Callback for /wam/moveToCartPostion service. 
   *  Sets the target for wam_interface_ to requested cartesian position and asyncronously executes the trajectory. 
   *  Trajectory can be terminated by calling /wam/idle.
   */
  bool cartPositionMoveCb(wam_srvs::CartPositionMove::Request& request, wam_srvs::CartPositionMove::Response& response);

  /** Callback for /wam/moveToCartPose service. 
   *  Sets the target for wam_interface_ to requested cartesian pose and asyncronously executes the trajectory. 
   *  Trajectory can be terminated by calling /wam/idle.
   */
  bool cartPoseMoveCb(wam_srvs::CartPoseMove::Request& request, wam_srvs::CartPoseMove::Response& response);

  /** Callback for /wam/moveToRandomPosition service. 
   *  Sets the target for wam_interface_ to a random joint position and asyncronously executes the trajectory.
   *  Trajectory can be terminated by calling /wam/idle.
   */
  bool randomMoveCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /** Callback for /wam/idle service. 
   *  Stops any trajectory execution, if active.
   */
  bool idleCb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

  /** Callback for /wam/holdCartPosition service. 
   *  Sets the target for wam_interface_ to current cartesian position. Stops any active trajectory.
   */
  bool holdCartPositionCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  /** Callback for /wam/holdCartPose service. 
   *  Sets the target for wam_interface_ to current cartesian pose. Stops any active trajectory.
   */
  bool holdCartPoseCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  /** Callback for /wam/holdJointPosition service. 
   *  Sets the target for wam_interface_ to current joint position. Stops any active trajectory.
   */
  bool holdJointPositionCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  /** Callback for /wam/holdCartOrientation service. 
   *  Sets the target for wam_interface_ to current cartesian orientation. Stops any active trajectory.
   */
  bool holdCartOrientationCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  /** Callback for /wam/gravityCompensate service. 
   *  Does nothing, present for compatibility with code written for wam_node.
   */
  bool gravityCompensateCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  /** Callback for /wam/moveHome service. 
   *  Sets the target for wam_interface_ to home_jp, and asyncronously executes trajectory.
   *  Trajectory can be terminated by calling /wam/idle.
   */
  bool moveHomeCb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

  /** Callback for /wam/setVelocityLimit service. 
   *  Sets joint velocity limit for wam_interface_ to requested value. Max velocity in SRDF files is 10rad/s.
   */
  bool setVelocityLimitCb(wam_srvs::VelocityLimit::Request& request, wam_srvs::VelocityLimit::Response& response);

  /** Callback for /wam/RTJointPositionCMD topic.
   *  Directly publishes requested joint position to the /joint_states topic, to visualize in RVIZ.
   */
  void rtJointPositionCb(const wam_msgs::RTJointPositions::ConstPtr& msg);

  /** Callback for /wam/RTLinearandAngularVelocityCMD topic.
   *  Converts Linear and Angular Velocity to Joint Velocity and sets Joint Position based on Joint Velocity for kRtTimeout_s.
   */
  void rtLinearAngularVelocityCb(const wam_msgs::RTLinearandAngularVelocity::ConstPtr& msg);

  /** Callback for /wam/RTAngularVelocityCMD topic.
   *  Converts Angular Velocity to Joint Velocity and sets Joint Position based on Joint Velocity for kRtTimeout_s.
   */
  void rtAngularVelocityCb(const wam_msgs::RTAngularVelocity::ConstPtr& msg);

  /** Callback for /wam/RTLinearVelocityCMD topic.
   *  Converts Linear Velocity to Joint Velocity and sets Joint Position based on Joint Velocity for kRtTimeout_s.
   */
  void rtLinearVelocityCb(const wam_msgs::RTLinearVelocity::ConstPtr& msg);

  /** Callback for /wam/RTCartPositionCMD topic.
   *  Uses inverse kinematics to convert cartesian position to joint position, and publishes
   *  it to the /joint_states topic to visualize in RVIZ.
   */
  void rtCartPositionCb(const wam_msgs::RTCartPosition::ConstPtr& msg);

  /** Callback for /wam/RTCartOrientationCMD topic.
   *  Uses inverse kinematics to convert cartesian orientation to joint position, and publishes
   *  it to the /joint_states topic to visualize in RVIZ.
   */
  void rtCartOrientationCb(const wam_msgs::RTCartOrientation::ConstPtr& msg);

  /** Callback for /wam/RTCartPoseCMD topic.
   *  Uses inverse kinematics to convert cartesian pose to joint position, and publishes
   *  it to the /joint_states topic to visualize in RVIZ.
   */
  void rtCartPoseCb(const wam_msgs::RTCartPose::ConstPtr& msg);

  /** Callback for /wam/RTJointVelocityCMD topic.
   *  Updates joint positions based on joint velocity for kRtTimeout_s time, and publishes it to the /joint_states
   *  topic to visualize in RVIZ.
   */
  void rtJointVelocityCb(const wam_msgs::RTJointVelocities::ConstPtr& msg);

  /** Subscriber Callback function for Current Joint Position. Updates current_joint_position_ at each step.*/
  void currentJointPositionCb(const sensor_msgs::JointState::ConstPtr& msg);

  /** Returns Tool Velocity. 
   * Tool Velocity = (current_tool_position_-previous_tool_position_)/(current_velocity_pub_time_-previous_velocity_pub_time_) */
  geometry_msgs::Twist calcToolVelocity();

  /** Returns Joint Velocity.
   * Joint Velocity = (current_joint_position_-previous_joint_position_)/(current_velocity_pub_time_-previous_velocity_pub_time_) */
  sensor_msgs::JointState calcJointVelocity();

  /**Publishes current Tool Pose, Joint Velocity and Tool Velocity */
  void publishWAM();
  /** Constructor for Wam_Node. Moves WAM to Home Joint Positions, since URDF
   * starts with all joint values at 0
   */
  Wam_Node(
    moveit::planning_interface::MoveGroupInterface& wam_interface__,
    ros::Publisher joint_trajectory_publisher__, bool found_hand);
 private:
 /**Function to convert goal joint state to a trajectory with 2 points containing start and goal joint state for ROS_CONTROL with Gazebo*/
  trajectory_msgs::JointTrajectory setTrajectoryMsg(std::vector<double> goal_jp, std::vector<double> start_jp, double end_time);

  /**Inverts quaternion and changes the frame of reference of the message
   * Inverting quaternion is necessary, since the WAM's orientation frames are flipped
   * Changing the frame of reference is important, since moveit plans wrt World frame, and WAM plans wrt base_link frame*/
  geometry_msgs::PoseStamped worldToBaseTransform(geometry_msgs::PoseStamped msg, std::string frame1, std::string frame2);
  geometry_msgs::PoseStamped baseToWorldTransform(geometry_msgs::PoseStamped msg, std::string frame1, std::string frame2);
  /** Parses the Zero Cal config file and returns Joint Positions for Home pose.*/ 
  std::vector<double> parseHomeData();
  /**moveit planning interface for WAM*/
  moveit::planning_interface::MoveGroupInterface& wam_interface_;
  geometry_msgs::Twist tool_velocity_;
  geometry_msgs::PoseStamped current_tool_pose_;
  sensor_msgs::JointState home_jp_;
  ros::Time previous_velocity_pub_time_, current_velocity_pub_time_;
  /** current and previous joint position, tool orientations and tool positions for tool and joint velocity publishers */
  std::vector<double> current_tool_rpy_, previous_tool_rpy_;
  geometry_msgs::Point current_tool_position_, previous_tool_position_;
  /** Indicates if first_publish_ has been made. */ 
  bool first_publish_;
  sensor_msgs::JointState current_joint_position_, previous_joint_position_;
  /** Indicates whether RT Velocity command is active. 
   * If /wam/idle is called, this is set to false to stop RT trajectory. */
  bool rt_velocity_status_;
  //Checks if first RT message has been received
  bool first_rt_msg_;
  /**True if a trajectory is currently active. */
  bool trajectory_status_;
  bool found_hand_;
  /**Callback for joint Veloity Move Thread, used for Real-Time joint and cartesian velocity commands.
    Updates joint positions based on a goal velocity for a set amount of time.
    rt_goal_velocity_, rt_trajectory_duration_, update_rt_ have to be set before calling.*/
  void jointVelocityThreadCb();
  void cartVelocityThreadCb();
  /** Goal Velocity to apply to the WAM. */
  std::vector<double> rt_goal_velocity_;
  /** Goal Twist to apply to the WAM. */
  std::vector<double> rt_goal_twist_;
  /** Time over which to apply joint velocity. */
  double rt_trajectory_duration_;
  /** Notify if goal velocity has been updated. */
  bool update_rt_;
};

template <size_t DOF>
Wam_Node<DOF>::Wam_Node(
    moveit::planning_interface::MoveGroupInterface& wam_interface__,
    ros::Publisher joint_trajectory_publisher__, bool found_hand)
    : wam_interface_(wam_interface__),
      joint_trajectory_publisher_(joint_trajectory_publisher__), found_hand_(found_hand) {
  wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
  wam_interface_.setGoalTolerance(kGoalTolerance);
  wam_interface_.startStateMonitor();
  wam_interface_.setEndEffectorLink("ee");
  wam_interface_.setPlanningTime(10);
  first_publish_ = false;
  rt_velocity_status_ = false;
  first_rt_msg_ = true;
  trajectory_status_ = false;
  previous_tool_rpy_.resize(3);
  current_tool_rpy_.resize(3);
  // move hand to close spread at startup
  if (found_hand_) {
    ros::NodeHandle n;
    //Subscribe to open grasp and close spread client. Called before moving WAM home.
    bhand_open_grasp_client_ = n.serviceClient<std_srvs::Trigger>("/bhand/openGrasp");
    bhand_close_spread_client_ = n.serviceClient<std_srvs::Trigger>("/bhand/closeSpread");
    moveit::planning_interface::MoveGroupInterface bhand_interface("barrett_hand");
    sensor_msgs::JointState bhand_goal_joint_position;
    bhand_goal_joint_position.position.resize(kBhandJointNames.size());
    bhand_goal_joint_position.name = kBhandJointNames;

    // bhand_j11_joint set to 3.14 and bhand_j21_joint set to -3.14
    bhand_goal_joint_position.position[0] = 3.14;
    bhand_goal_joint_position.position[3] = -3.14;
    bhand_interface.setStartStateToCurrentState();
    bhand_interface.setJointValueTarget(bhand_goal_joint_position);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    bhand_interface.move();  // plan and move to target position
    spinner.stop();
  }
  current_joint_position_.position.resize(DOF);
  current_joint_position_.velocity.resize(DOF);
  current_joint_position_.name.resize(DOF);
  home_jp_.position = parseHomeData();
  home_jp_.velocity.resize(DOF);
  home_jp_.effort.resize(DOF);
  home_jp_.name.resize(DOF);

  for (int i = 0; i < (int)DOF; i++) {
    home_jp_.name.at(i) = "q" + std::to_string(i + 1);
    home_jp_.velocity.at(i) = 0;
    home_jp_.effort.at(i) = 0;
  }
  wam_interface_.setJointValueTarget(home_jp_.position);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  wam_interface_.move(); //plan and move to target position
  if (found_hand_) {
    std::vector<double> start_jp;
    start_jp = home_jp_.position;
    start_jp[3] -=0.35;
    wam_interface_.setJointValueTarget(start_jp);
    wam_interface_.move(); //plan and move to target position
    sensor_msgs::JointState bhand_goal_joint_position;
    bhand_goal_joint_position.position.resize(kBhandJointNames.size());
    bhand_goal_joint_position.name = kBhandJointNames;
    bhand_goal_joint_position.position[0] = 0;
    bhand_goal_joint_position.position[3] = 0;
    moveit::planning_interface::MoveGroupInterface bhand_interface("barrett_hand");
    bhand_interface.setJointValueTarget(bhand_goal_joint_position);
    bhand_interface.move();  // plan and move to target position
    spinner.stop();
  }
}

template <size_t DOF>
trajectory_msgs::JointTrajectory Wam_Node<DOF>::setTrajectoryMsg(std::vector<double> goal_jp, std::vector<double> start_jp, double end_time) {
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.resize(DOF);
  trajectory_msgs::JointTrajectoryPoint point1, point2;
  point1.positions.resize(DOF);
  point2.positions.resize(DOF);
  for (int i = 0; i < (int)DOF; i++) {
    joint_trajectory.joint_names[i] = "q" + std::to_string(i+1);
    point1.positions[i] = start_jp[i];
    point2.positions[i] = goal_jp[i];
  }
  point1.time_from_start = ros::Duration(0);
  point2.time_from_start = ros::Duration(end_time);
  joint_trajectory.points.push_back(point1);
  joint_trajectory.points.push_back(point2);
  return joint_trajectory;
}

template <size_t DOF>
geometry_msgs::PoseStamped Wam_Node<DOF>::worldToBaseTransform(
    geometry_msgs::PoseStamped msg, std::string frame1, std::string frame2) {
  /**tf listener and buffer to transform pose message frames*/
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener transform_listener_(tfBuffer);
  geometry_msgs::TransformStamped base_transform;
  base_transform = tfBuffer.lookupTransform(frame1, frame2, ros::Time(0),
                                            ros::Duration(1.0));
  tf2::doTransform(msg, msg, base_transform);
  //Invert Quaternion
  tf2::Quaternion quat, quat_inv;
  tf2::convert(msg.pose.orientation, quat);
  quat_inv = quat.inverse();
  quat_inv.normalize();
  tf2::convert(quat_inv, msg.pose.orientation);
  return msg;
}

template <size_t DOF>
geometry_msgs::PoseStamped Wam_Node<DOF>::baseToWorldTransform(
    geometry_msgs::PoseStamped msg, std::string frame1, std::string frame2) {
  //Invert Quaternion
  tf2::Quaternion quat, quat_inv;
  tf2::convert(msg.pose.orientation, quat);
  quat_inv = quat.inverse();
  quat_inv.normalize();
  tf2::convert(quat_inv, msg.pose.orientation);
  /**tf listener and buffer to transform pose message frames*/
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener transform_listener_(tfBuffer);
  geometry_msgs::TransformStamped base_transform;
  base_transform = tfBuffer.lookupTransform(frame1, frame2, ros::Time(0),
                                            ros::Duration(1.0));
  tf2::doTransform(msg, msg, base_transform);
  return msg;
}

template <size_t DOF>
std::vector<double> Wam_Node<DOF>::parseHomeData() {
  std::ifstream file;
  if ((int)DOF == 7) {
      file.open(kZeroCalConfigPath7);
    } else {
    file.open(kZeroCalConfigPath4);
  }
  std::vector<double> home_jp;
  home_jp.resize(DOF);
  if (file.is_open()) {
    std::string line;
    getline(file, line);
    line.erase(0, 9); //remove "home = (" from the line
    line.erase(line.length() - 3, line.length() - 1); //remove " );" from the line
    std::transform(line.begin(), line.end(), line.begin(),
                   [](char ch) { return ch == ',' ? ' ' : ch; }); //replace commas with spaces
    const char *cur = line.c_str();
    const char *next = cur;
    for (int i = 0; i < (int)DOF; ++i) {
      home_jp[i] = strtod(cur, (char **)&next); //convert string to double
      if (cur == next) {
        std::cout << "Error reading zerocal.conf file!" << std::endl;
      } else {
        cur = next;
      }
    }
  } else {
    std::cout << "Could not open file" << std::endl;
  }
  return home_jp;
}

template <size_t DOF>
bool Wam_Node<DOF>::jointMoveCb(wam_srvs::JointMove::Request& request,
                   wam_srvs::JointMove::Response& response) {
  if (request.joint_state.position.size() != DOF) {
    ROS_ERROR("Commanded Joint State != DOF");
    response.response = false;
    return false;
  }
  wam_interface_.stop();  //stop any active trajectory
  ros::AsyncSpinner spinner(2);
  spinner.start();
  wam_interface_.setStartStateToCurrentState();
  wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
  wam_interface_.setJointValueTarget(request.joint_state.position);
  moveit::planning_interface::MoveGroupInterface::Plan wam_plan;
  bool success = (wam_interface_.plan(wam_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO_STREAM("Moving WAM to Joint Positions:\n" << request.joint_state);
    //execute plan in a non blocking manner. Allows for stopping trajectory by calling /wam/idle
    wam_interface_.execute(wam_plan);
    response.response = true;
  } else {
    ROS_ERROR_STREAM("Could not move WAM to Joint Positions:\n" << request.joint_state);
    response.response = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Wam_Node<DOF>::cartOrientationCb(wam_srvs::CartOrientationMove::Request& request,
                                 wam_srvs::CartOrientationMove::Response& response) {
  response.response = true;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  wam_interface_.setStartStateToCurrentState();
  wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
  //Invert orientation message and change frame
  geometry_msgs::PoseStamped goal_pose = current_tool_pose_;
  goal_pose.pose.orientation = request.orientation;
  //Convert base_link frame to world frame for Moveit
  goal_pose = baseToWorldTransform(goal_pose, "world", "base_link");
  wam_interface_.setApproximateJointValueTarget(goal_pose);
  moveit::planning_interface::MoveGroupInterface::Plan wam_plan;
  bool success = (wam_interface_.plan(wam_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO_STREAM("Moving WAM to Cartesian Orientation: \n"
                    << request.orientation);
    wam_interface_.asyncExecute(
        wam_plan);  // execute plan in a non blocking manner. Allows for
                    // stopping trajectory
    response.response = true;
  } else {
    ROS_ERROR("Could not move WAM to Cartesian Orientation");
    response.response = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Wam_Node<DOF>::cartPositionMoveCb(wam_srvs::CartPositionMove::Request& request,
                                wam_srvs::CartPositionMove::Response& response) {
  response.response = true;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  wam_interface_.setStartStateToCurrentState();
  wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
  //Convert position message frame from base_link frame to world frame
  geometry_msgs::PoseStamped goal_pose = current_tool_pose_;
  goal_pose.pose.position = request.position;
  goal_pose = baseToWorldTransform(goal_pose, "world", "base_link");
  wam_interface_.setApproximateJointValueTarget(goal_pose);
  moveit::planning_interface::MoveGroupInterface::Plan wam_plan;
  bool success = (wam_interface_.plan(wam_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO_STREAM("Moving WAM to Cartesian Position: \n" << request.position);
    //execute plan in a non blocking manner. Allows for stopping trajectory by calling /wam/idle
    wam_interface_.asyncExecute(wam_plan);
    response.response = true;
  } else {
    ROS_ERROR("Could not move WAM to Cartesian Position");
    response.response = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Wam_Node<DOF>::cartPoseMoveCb(wam_srvs::CartPoseMove::Request& request,
                            wam_srvs::CartPoseMove::Response& response) {
  wam_interface_.stop();  //stop any active trajectory
  ros::AsyncSpinner spinner(2);
  spinner.start();
  wam_interface_.setStartStateToCurrentState();
  wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
  //Convert frame from base_link to world and invert quaternion
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.pose = request.pose;
  goal_pose = baseToWorldTransform(goal_pose, "world", "base_link");
  wam_interface_.setApproximateJointValueTarget(goal_pose, "ee");
  moveit::planning_interface::MoveGroupInterface::Plan wam_plan;
  bool success = (wam_interface_.plan(wam_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO_STREAM("Moving WAM to Cartesian Pose: \n" << request.pose);
    //execute plan in a non blocking manner. Allows for stopping trajectory by calling /wam/idle
    wam_interface_.asyncExecute(wam_plan);
    response.response = true;
  } else {
    ROS_ERROR("Could not move WAM to Cartesian Pose");
    response.response = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Wam_Node<DOF>::randomMoveCb(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response) {
  wam_interface_.stop();  //stop any active trajectory
  ros::AsyncSpinner spinner(2);
  spinner.start();
  wam_interface_.setStartStateToCurrentState();
  wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
  wam_interface_.setRandomTarget();
  moveit::planning_interface::MoveGroupInterface::Plan wam_plan;
  bool success = (wam_interface_.plan(wam_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("Moving WAM to Random Position");
    //execute plan in a non blocking manner. Allows for stopping trajectory by calling /wam/idle
    wam_interface_.asyncExecute(wam_plan);
  } else {
    ROS_ERROR("Could not move WAM to Random Position");
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Wam_Node<DOF>::idleCb(std_srvs::Trigger::Request& request,
                           std_srvs::Trigger::Response& response) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  rt_velocity_status_ = false;
  wam_interface_.stop();  // stop any trajectory execution, if one is active
  response.success = true;
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Wam_Node<DOF>::holdCartPositionCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
  wam_interface_.stop();  //stop any active trajectory
  if (request.data) {
    ros::AsyncSpinner spinner(2);
    spinner.start();
    wam_interface_.setStartStateToCurrentState();
    wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
    wam_interface_.setJointValueTarget(current_joint_position_);
    moveit::planning_interface::MoveGroupInterface::Plan wam_plan;
    bool success = (wam_interface_.plan(wam_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO("Holding Cartesian Position");
      // execute plan in a non blocking manner. Allows for stopping trajectory
      // by calling /wam/idle
      wam_interface_.asyncExecute(wam_plan);
      response.success = true;
    } else {
      ROS_ERROR("Could not Hold Cartesian Position");
      response.success = false;
    }
    spinner.stop();
    return true;
  } else {
    ROS_INFO("Releasing Cartesian Position hold");
    response.success = true;
  }
}

template <size_t DOF>
bool Wam_Node<DOF>::holdCartPoseCb(std_srvs::SetBool::Request& request,
                                      std_srvs::SetBool::Response& response) {
  wam_interface_.stop();  //stop any active trajectory
  if (request.data) {
    ros::AsyncSpinner spinner(2);
    spinner.start();
    wam_interface_.setStartStateToCurrentState();
    wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
    wam_interface_.setJointValueTarget(current_joint_position_);
    moveit::planning_interface::MoveGroupInterface::Plan wam_plan;
    bool success = (wam_interface_.plan(wam_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO("Holding Cartesian Pose");
      //execute plan in a non blocking manner. Allows for stopping trajectory by calling /wam/idle
      wam_interface_.asyncExecute(wam_plan);
      response.success = true;
    } else {
      ROS_ERROR("Could not hold Cartesian Pose");
      response.success = false;
    }
    spinner.stop();
    return true;
  } else {
    ROS_INFO("Releasing Cartesian Pose hold");
    response.success = true;
  }
}

template <size_t DOF>
bool Wam_Node<DOF>::holdJointPositionCb(std_srvs::SetBool::Request& request,
                            std_srvs::SetBool::Response& response) {
  wam_interface_.stop();  //stop any active trajectory
  if (request.data) {
    ros::AsyncSpinner spinner(2);
    spinner.start();
    wam_interface_.setStartStateToCurrentState();
    wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
    wam_interface_.setJointValueTarget(current_joint_position_);
    moveit::planning_interface::MoveGroupInterface::Plan wam_plan;
    bool success = (wam_interface_.plan(wam_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO("Holding Joint Position");
      //execute plan in a non blocking manner. Allows for stopping trajectory by calling /wam/idle
      wam_interface_.asyncExecute(wam_plan);
      response.success = true;
    } else {
      ROS_ERROR("Could not hold Joint Position");
      response.success = false;
    }
    spinner.stop();
    return true;
  } else {
    ROS_INFO("Releasing Joint Position hold");
    response.success = true;
  }
}

template <size_t DOF>
bool Wam_Node<DOF>::holdCartOrientationCb(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  wam_interface_.stop();  //stop any active trajectory
  if (request.data) {
    ros::AsyncSpinner spinner(2);
    spinner.start();
    wam_interface_.setStartStateToCurrentState();
    wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
    std::vector<double> current_rpy = wam_interface_.getCurrentRPY();
    wam_interface_.setRPYTarget(current_rpy[0], current_rpy[1], current_rpy[2]);
    moveit::planning_interface::MoveGroupInterface::Plan wam_plan;
    bool success = (wam_interface_.plan(wam_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO("Holding Cartesian Orientation");
      //execute plan in a non blocking manner. Allows for stopping trajectory by calling /wam/idle
      wam_interface_.asyncExecute(wam_plan);
      response.success = true;
    } else {
      ROS_ERROR("Could not hold Cartesian Orientation");
      response.success = false;
    }
    spinner.stop();
    return true;
  } else {
    ROS_INFO("Releasing Cartesian Orientation hold");
    response.success = true;
  }
}

template <size_t DOF>                              
bool Wam_Node<DOF>::gravityCompensateCb(std_srvs::SetBool::Request& request,
                     std_srvs::SetBool::Response& response) {
  if (request.data) {
    ROS_INFO("Gravity Compensation turned on");
    response.success = true;
  } else {
    ROS_INFO("Gravity Compensation turned off");
    response.success = true;
  }
}

template <size_t DOF>
bool Wam_Node<DOF>::moveHomeCb(std_srvs::Trigger::Request& request,
                               std_srvs::Trigger::Response& response) {
  wam_interface_.stop();  // stop any active trajectory
  ros::AsyncSpinner spinner(2);
  spinner.start();
  wam_interface_.setStartStateToCurrentState();
  wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
  wam_interface_.setJointValueTarget(home_jp_);
  moveit::planning_interface::MoveGroupInterface::Plan wam_plan;
  bool success = (wam_interface_.plan(wam_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // Open grasp and close spread before moving WAM home
  if (found_hand_) {
    std_srvs::Trigger open_grasp;
    std_srvs::Trigger close_spread;
    bhand_open_grasp_client_.call(open_grasp);
    bhand_close_spread_client_.call(close_spread);
  }
  if (success) {
    ROS_INFO("Moving WAM Home");
    // execute plan in a non blocking manner. Allows for stopping trajectory by
    // calling /wam/idle
    wam_interface_.asyncExecute(wam_plan);
    response.success = true;
  } else {
    ROS_ERROR("Could not move WAM Home");
    response.success = false;
  }
  spinner.stop();
  return true;
}

template <size_t DOF>
bool Wam_Node<DOF>::setVelocityLimitCb(wam_srvs::VelocityLimit::Request& request, wam_srvs::VelocityLimit::Response& response) {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Setting WAM Velocity Limit");
  /** Max velocity limit in SRDF files is 10rad/s. Set kMaxVelocityScalingFactor
   *  such that kMaxVelocity*kMaxVelocityScalingFactor = request.velocity_limit
  */
  kMaxVelocityScalingFactor = (request.velocity_limit)/kMaxVelocity;
  wam_interface_.setMaxVelocityScalingFactor(kMaxVelocityScalingFactor);
  spinner.stop();
  return true;
}

/* Returns Tool Velocity.
   Tool Velocity = (current_tool_position_-previous_tool_position_)/(current_velocity_pub_time_-previous_velocity_pub_time_) */
template <size_t DOF>
geometry_msgs::Twist Wam_Node<DOF>::calcToolVelocity() {
  geometry_msgs::Twist tool_velocity;
  tool_velocity.linear.x = (current_tool_position_.x - previous_tool_position_.x) /
                          (current_velocity_pub_time_.toSec() - previous_velocity_pub_time_.toSec());
  tool_velocity.linear.y = (current_tool_position_.y - previous_tool_position_.y) /
                          (current_velocity_pub_time_.toSec() - previous_velocity_pub_time_.toSec());
  tool_velocity.linear.z = (current_tool_position_.z - previous_tool_position_.z) /
                          (current_velocity_pub_time_.toSec() - previous_velocity_pub_time_.toSec());
  tool_velocity.angular.x = (current_tool_rpy_.at(0) - previous_tool_rpy_.at(0)) /
                           (current_velocity_pub_time_.toSec() - previous_velocity_pub_time_.toSec());
  tool_velocity.angular.y = (current_tool_rpy_.at(1) - previous_tool_rpy_.at(1)) /
                           (current_velocity_pub_time_.toSec() - previous_velocity_pub_time_.toSec());
  tool_velocity.angular.z = (current_tool_rpy_.at(2) - previous_tool_rpy_.at(2)) /
                           (current_velocity_pub_time_.toSec() - previous_velocity_pub_time_.toSec());
  return tool_velocity;
}

/** Returns Joint Velocity. 
 * Joint Velocity = (current_joint_position_-previous_joint_position_)/(current_velocity_pub_time_-previous_velocity_pub_time_)
 * Also checks if joint velocity limit is exceeded*/
template <size_t DOF>
sensor_msgs::JointState Wam_Node<DOF>::calcJointVelocity() {
  sensor_msgs::JointState joint_velocity;
  joint_velocity.velocity.resize(DOF);
  int no_zeros = 0; //check how many joint velocities are 0. If all are 0, then set trajectory_status_ to false.
  for (int i = 0; i < (int)DOF; i++) {
    joint_velocity.velocity.at(i) = (current_joint_position_.position.at(i) -
                                     previous_joint_position_.position.at(i)) /
                                    (current_velocity_pub_time_.toSec() -
                                     previous_velocity_pub_time_.toSec());
    if (isinf(joint_velocity.velocity.at(i)) || isnan(joint_velocity.velocity.at(i))) {
      joint_velocity.velocity.at(i) = 0;
    }
    /* Check if joint Velocity is greater than limit + 0.1. If true, throw error */
    if (!isinf(joint_velocity.velocity.at(i)) &&
        !isnan(joint_velocity.velocity.at(i)) && (
            joint_velocity.velocity.at(i) >
            ((kMaxVelocityScalingFactor * kMaxVelocity) + 0.4))) {
      ROS_ERROR("Velocity Limit reached");
    }
    if (!isinf(joint_velocity.velocity.at(i)) &&
        !isnan(joint_velocity.velocity.at(i)) &&
                (joint_velocity.velocity.at(i) == 0)) {
      no_zeros = no_zeros + 1;
    }
  }
  if (no_zeros == DOF) {
    trajectory_status_ = false;
  } else {
    trajectory_status_ = true;
  }
  return joint_velocity;
}

template <size_t DOF>
void Wam_Node<DOF>::publishWAM() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (ros::ok()) {
    current_tool_pose_ = worldToBaseTransform(wam_interface_.getCurrentPose(), "base_link", "world");
   //current_tool_pose_ = wam_interface_.getCurrentPose();
    tool_pose_publisher_.publish(current_tool_pose_);
    /*start with velocities at 0, and update previous tool and joint positions
     * and publish times to calculate velocity at each time step.*/
    if (!first_publish_) {
      first_publish_ = true;
      previous_velocity_pub_time_ = ros::Time::now();
      previous_joint_position_ = current_joint_position_;
      previous_tool_position_ = current_tool_pose_.pose.position;
      //convert quaternion to RPY for Angular Velocity calculations
      tf2::Quaternion quat;
      tf2::convert(current_tool_pose_.pose.orientation, quat);
      tf2::Matrix3x3(quat).getRPY(previous_tool_rpy_[0], previous_tool_rpy_[1], previous_tool_rpy_[2]);
      sensor_msgs::JointState wam_velocity;
      wam_velocity.velocity.resize(DOF);
      for (int i = 0; i < (int)DOF; i++) {
        wam_velocity.velocity.at(i) = 0;
      }
      tool_velocity_.linear.x = 0;
      tool_velocity_.linear.y = 0;
      tool_velocity_.linear.z = 0;
      tool_velocity_.angular.x = 0;
      tool_velocity_.angular.y = 0;
      tool_velocity_.angular.z = 0;
      tool_velocity_publisher_.publish(tool_velocity_);
    } else {
      current_velocity_pub_time_ = ros::Time::now();
      current_tool_position_ = current_tool_pose_.pose.position;
      //convert quaternion to RPY for Angular Velocity calculations
      tf2::Quaternion quat;
      tf2::convert(current_tool_pose_.pose.orientation, quat);
      tf2::Matrix3x3(quat).getRPY(current_tool_rpy_[0], current_tool_rpy_[1], current_tool_rpy_[2]);
      tool_velocity_publisher_.publish(calcToolVelocity());
      joint_velocity_publisher_.publish(calcJointVelocity());
      previous_velocity_pub_time_ = current_velocity_pub_time_;
      previous_tool_position_ = current_tool_position_;
      previous_joint_position_ = current_joint_position_;
      previous_tool_rpy_ = current_tool_rpy_;
    }
  }
  spinner.stop();
}

template <size_t DOF>
void Wam_Node<DOF>::currentJointPositionCb(const sensor_msgs::JointState::ConstPtr& msg) {
    int index = 0;
    while (msg->name[index].compare("q1") != 0) { //Remove BarrettHand mesages, if present
      index = index + 1;
    }
    for (int i = 0; i < (int)DOF; i++) {
      current_joint_position_.position[i] = msg->position[index + i];
    }
}

template <size_t DOF>
void Wam_Node<DOF>::rtJointPositionCb(const wam_msgs::RTJointPositions::ConstPtr& msg) {
  sensor_msgs::JointState joint_position_msg;
  trajectory_msgs::JointTrajectory joint_traj_msg;
  trajectory_msgs::JointTrajectoryPoint point1, point2;
  if (msg->joint_states.size() != DOF) {
    ROS_ERROR("Invalid number of Joint Positions received. Please enter %lu Joint Positions", DOF);
    return;
  } else {
    joint_trajectory_publisher_.publish(setTrajectoryMsg(msg->joint_states, current_joint_position_.position, 0.0001));
  } 
}

template <size_t DOF>
void Wam_Node<DOF>::rtCartPositionCb(const wam_msgs::RTCartPosition::ConstPtr& msg) {
  wam_state_->setJointGroupPositions("wam", current_joint_position_.position);
  wam_state_->update();  // update all link transforms
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.pose.position = msg->point;
  goal_pose.pose.orientation = current_tool_pose_.pose.orientation;
  // Convert frame from base_link to world, and invert quaternion
  goal_pose = baseToWorldTransform(goal_pose, "world", "base_link");
  wam_interface_.setApproximateJointValueTarget(goal_pose, "ee");
  robot_state::RobotState goal_state = wam_interface_.getJointValueTarget();
  std::vector<double> goal_jp;
  goal_state.copyJointGroupPositions("wam", goal_jp);
  joint_trajectory_publisher_.publish(
      setTrajectoryMsg(goal_jp, current_joint_position_.position, 0.0001));
}

template <size_t DOF>
void Wam_Node<DOF>::rtCartOrientationCb(
    const wam_msgs::RTCartOrientation::ConstPtr& msg) {
  wam_state_->setJointGroupPositions("wam", current_joint_position_.position);
  wam_state_->update(); //update all link transforms
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.pose.position = current_tool_pose_.pose.position;
  goal_pose.pose.orientation = msg->orientation;
  // Convert frame from base_link to world, and invert quaternion
  goal_pose = baseToWorldTransform(goal_pose, "world", "base_link");
  wam_interface_.setApproximateJointValueTarget(goal_pose, "ee");
  robot_state::RobotState goal_state = wam_interface_.getJointValueTarget();
  std::vector<double> goal_jp;
  goal_state.copyJointGroupPositions("wam", goal_jp);
  joint_trajectory_publisher_.publish(
      setTrajectoryMsg(goal_jp, current_joint_position_.position, 0.0001));
}

template <size_t DOF>
void Wam_Node<DOF>::rtCartPoseCb(const wam_msgs::RTCartPose::ConstPtr& msg) {
/*   wam_state_->setJointGroupPositions("wam", current_joint_position_.position);
  wam_state_->update();  // update all link transforms */
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.pose.position = msg->point;
  goal_pose.pose.orientation = msg->orientation;
  // Convert frame from base_link to world, and invert quaternion
  goal_pose = baseToWorldTransform(goal_pose, "world", "base_link");
  wam_interface_.setApproximateJointValueTarget(goal_pose, "ee");
  robot_state::RobotState goal_state = wam_interface_.getJointValueTarget();
  std::vector<double> goal_jp;
  goal_state.copyJointGroupPositions("wam", goal_jp);
  joint_trajectory_publisher_.publish(
      setTrajectoryMsg(goal_jp, current_joint_position_.position, 0.0001));
}

/** Updates Joint Positions based on Joint Velocity, for kRtTimeout_s time. 
 *  Publishes joint positions at kPublishFrequency. 
 *  no_points = (kPublishFrequency * rt_trajectory_duration_).
 *  Joint Position messages are published no_points times, or until update_rt_ is set to true.
 *  goal_jp = current_joint_position_ + (rt_goal_velocity_ * rt_trajectory_duration_)
 *  step_size = (goal_jp - current_joint_position_) / no_points.
 *  timestep = 1/kPublishFrequency
 *  At each timestep, current joint position is increased by step_size, and no_points is reduced by 1.
 */
template <size_t DOF>
void Wam_Node<DOF>::jointVelocityThreadCb() {
  update_rt_ = true;
  while (update_rt_) {
    update_rt_ = false; //only executes while loop once, unless user sets update_rt_ to update goal values
    rt_velocity_status_ = true; //set to false if /wam/idle called
    sensor_msgs::JointState goal_jp;
    sensor_msgs::JointState jp_msg;
    goal_jp.position.resize(DOF);
    jp_msg.position.resize(DOF);
    jp_msg.velocity.resize(DOF);
    jp_msg.effort.resize(DOF);
    jp_msg.name.resize(DOF);
    std::vector<double> step_size;
    int no_points = kPublishFrequency * rt_trajectory_duration_;
    ros::Rate loop_rate(kPublishFrequency);
    for (int i = 0; i < (int)DOF; i++) {
      goal_jp.position.at(i) = current_joint_position_.position.at(i) + (rt_goal_velocity_.at(i) * rt_trajectory_duration_);
      step_size.push_back((goal_jp.position.at(i) - current_joint_position_.position.at(i)) / no_points);
    }
    joint_trajectory_publisher_.publish(setTrajectoryMsg(goal_jp.position, current_joint_position_.position, kRtTimeout_s));
  }
  first_rt_msg_ = true;
}

template <size_t DOF>
void Wam_Node<DOF>::rtJointVelocityCb(const wam_msgs::RTJointVelocities::ConstPtr& msg) {
  if (msg->velocities.size() != (int)DOF) {
    ROS_ERROR("Invalid Joint Velocity command. Please enter %lu Joint Velocities.",DOF);
    return;
  }
  if (first_rt_msg_) {
    first_rt_msg_ = false;
    rt_trajectory_duration_ = kRtTimeout_s;
    rt_goal_velocity_ = msg->velocities;
    std::thread jointVelocityThread(&Wam_Node::jointVelocityThreadCb,this);
    jointVelocityThread.detach();
  } else {
    update_rt_ = true;
    rt_goal_velocity_ = msg->velocities;
  }
}

/** Updates Cartesian Poses based on Goal Linear and Angular velocities, for kRtTimeout_s time. 
 *  Publishes joint positions at kPublishFrequency. 
 *  no_points = (kPublishFrequency * rt_trajectory_duration_).
 *  Joint Position messages are published no_points times, or until update_rt_ is set to true.
 *  goal_pose = current_pose + (rt_goal_velocity_ * rt_trajectory_duration_)
 *  step_size = (goal_pose - current_pose) / no_points.
 *  timestep = 1/kPublishFrequency
 *  At each timestep, current joint position is increased by step_size, and no_points is reduced by 1.
 */
template <size_t DOF>
void Wam_Node<DOF>::cartVelocityThreadCb() {
  update_rt_ = true;
  while (update_rt_) {
    update_rt_ = false; //only executes while loop once, unless user sets update_rt_ to update goal values
    rt_velocity_status_ = true; //set to false if /wam/idle called
    std::vector<double> step_size;
    std::vector<double> goal_rpy;
    geometry_msgs::PoseStamped goal_pose;
    //Positions and RPY
    goal_rpy.resize(3);
    int no_points = kPublishFrequency * rt_trajectory_duration_;
    ros::Rate loop_rate(kPublishFrequency);
    goal_pose.pose.position.x = current_tool_pose_.pose.position.x + (rt_goal_twist_[0] * rt_trajectory_duration_);
    goal_pose.pose.position.y = current_tool_pose_.pose.position.y + (rt_goal_twist_[1] * rt_trajectory_duration_);
    goal_pose.pose.position.z = current_tool_pose_.pose.position.z + (rt_goal_twist_[2] * rt_trajectory_duration_);
    for (int i = 0; i < 3; i++) {
      goal_rpy.at(i) = current_tool_rpy_.at(i) + (rt_goal_twist_.at(i+3) * rt_trajectory_duration_);
    }
    tf2::Quaternion goal_quaternion;
    goal_quaternion.setRPY( goal_rpy.at(0), goal_rpy.at(1), goal_rpy.at(2));
    goal_pose.pose.orientation = tf2::toMsg(goal_quaternion);
    goal_pose = baseToWorldTransform(goal_pose, "world", "base_link");
    wam_interface_.setApproximateJointValueTarget(goal_pose.pose, "ee");
    robot_state::RobotState goal_state = wam_interface_.getJointValueTarget();
    std::vector<double> goal_jp;
    goal_state.copyJointGroupPositions("wam", goal_jp);
    joint_trajectory_publisher_.publish(setTrajectoryMsg(goal_jp, current_joint_position_.position, kRtTimeout_s));
  }
  first_rt_msg_ = true;
}
/** Converts Linear and Angular Velocity to Joint Velocity using a pseudoinverse of current jacobian
 * qdot = pseudoinverse(jacobian) * rt_goal_twist_.
*/
template <size_t DOF>
void Wam_Node<DOF>::rtLinearAngularVelocityCb( 
    const wam_msgs::RTLinearandAngularVelocity::ConstPtr& msg) {
  if (first_rt_msg_) {
    first_rt_msg_ = false;
    rt_trajectory_duration_ = kRtTimeout_s;
    //convert direction and magnitude to a vector
    rt_goal_twist_.resize(6);
    rt_goal_twist_[0] = msg->linear_velocity_direction[0] * msg->linear_velocity_magnitude;
    rt_goal_twist_[1] = msg->linear_velocity_direction[1] * msg->linear_velocity_magnitude;
    rt_goal_twist_[2] = msg->linear_velocity_direction[2] * msg->linear_velocity_magnitude;
    rt_goal_twist_[3] = msg->angular_velocity_direction[0] * msg->angular_velocity_magnitude;
    rt_goal_twist_[4] = msg->angular_velocity_direction[1] * msg->angular_velocity_magnitude;
    rt_goal_twist_[5] = msg->angular_velocity_direction[2] * msg->angular_velocity_magnitude;
    std::thread cartVelocityThread(&Wam_Node::cartVelocityThreadCb,this);
    cartVelocityThread.detach();
  } else {
    update_rt_ = true;
    //convert direction and magnitude to a vector
    rt_goal_twist_[0] = msg->linear_velocity_direction[0] * msg->linear_velocity_magnitude;
    rt_goal_twist_[1] = msg->linear_velocity_direction[1] * msg->linear_velocity_magnitude;
    rt_goal_twist_[2] = msg->linear_velocity_direction[2] * msg->linear_velocity_magnitude;
    rt_goal_twist_[3] = msg->angular_velocity_direction[0] * msg->angular_velocity_magnitude;
    rt_goal_twist_[4] = msg->angular_velocity_direction[1] * msg->angular_velocity_magnitude;
    rt_goal_twist_[5] = msg->angular_velocity_direction[2] * msg->angular_velocity_magnitude;
  }
}

/** Converts Angular Velocity to Joint Velocity using a pseudoinverse of current jacobian
 * qdot = pseudoinverse(jacobian) * rt_goal_twist_.
*/

template <size_t DOF>
void Wam_Node<DOF>::rtAngularVelocityCb(const wam_msgs::RTAngularVelocity::ConstPtr& msg) {
  if (first_rt_msg_) {
    first_rt_msg_ = false;
    rt_trajectory_duration_ = kRtTimeout_s;
    //convert direction and magnitude to a vector
    rt_goal_twist_[0] = 0;
    rt_goal_twist_[1] = 0;
    rt_goal_twist_[2] = 0;
    rt_goal_twist_[3] = msg->direction[0] * msg->magnitude;
    rt_goal_twist_[4] = msg->direction[1] * msg->magnitude;
    rt_goal_twist_[5] = msg->direction[2] * msg->magnitude;
    std::thread cartVelocityThread(&Wam_Node::cartVelocityThreadCb,this);
    cartVelocityThread.detach();
  } else {
    update_rt_ = true;
    //convert direction and magnitude to a vector
    rt_goal_twist_[0] = 0;
    rt_goal_twist_[1] = 0;
    rt_goal_twist_[2] = 0;
    rt_goal_twist_[3] = msg->direction[0] * msg->magnitude;
    rt_goal_twist_[4] = msg->direction[1] * msg->magnitude;
    rt_goal_twist_[5] = msg->direction[2] * msg->magnitude;
  }
}

/** Converts Linear Velocity to Joint Velocity using a pseudoinverse of current jacobian.
 * qdot = pseudoinverse(jacobian) * rt_goal_twist_.
*/
template <size_t DOF>
void Wam_Node<DOF>::rtLinearVelocityCb(const wam_msgs::RTLinearVelocity::ConstPtr& msg) {
  if (first_rt_msg_) {
    first_rt_msg_ = false;
    rt_trajectory_duration_ = kRtTimeout_s;
    //convert direction and magnitude to a vector
    rt_goal_twist_[0] = msg->direction[0] * msg->magnitude;
    rt_goal_twist_[1] = msg->direction[1] * msg->magnitude;
    rt_goal_twist_[2] = msg->direction[2] * msg->magnitude;
    rt_goal_twist_[3] = 0;
    rt_goal_twist_[4] = 0;
    rt_goal_twist_[5] = 0;
    std::thread cartVelocityThread(&Wam_Node::cartVelocityThreadCb,this);
    cartVelocityThread.detach();
  } else {
    update_rt_ = true;
    //convert direction and magnitude to a vector
    rt_goal_twist_[0] = msg->direction[0] * msg->magnitude;
    rt_goal_twist_[1] = msg->direction[1] * msg->magnitude;
    rt_goal_twist_[2] = msg->direction[2] * msg->magnitude;
    rt_goal_twist_[3] = 0;
    rt_goal_twist_[4] = 0;
    rt_goal_twist_[5] = 0;
  }
}
#endif //SRC_WAM_MOVEIT_NODE_SRC_WAM_MOVEIT_NODE_