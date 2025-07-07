#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <thread>
#include <stdlib.h>
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include "wam_msgs/RTJointPositions.h"
#include "wam_msgs/RTLinearandAngularVelocity.h"
#include "wam_msgs/RTCartPose.h"
#include "wam_msgs/RTJointVelocities.h"
#include "wam_srvs/JointMove.h"
wam_srvs::JointMove setServiceCall(
    sensor_msgs::JointState::ConstPtr start_joint_position) {
  wam_srvs::JointMove wam_jp_srv;
  if (start_joint_position->position.size() == 4 ||
      start_joint_position->position.size() == 7) {
    // no bhand joint states, only wam
    wam_jp_srv.request.joint_state.position = start_joint_position->position;
  } else {
    // bhand and wam joint states
    // Avoid Bhand joint positions. Bhand joint states or wam states could be first
    int index = 0;
    if (start_joint_position->name[0].compare("q1") == 0) {
      // wam joint states first
      while (start_joint_position->name[index].compare("bhand_j11_joint") != 0) {
               std::cout << start_joint_position->name[index] << std::endl;
        wam_jp_srv.request.joint_state.position.push_back(
            start_joint_position->position[index]);
        index = index + 1;
      }
    } else {
      // bhand joint states first, avoid them
      while (start_joint_position->name[0].compare("q1") != 0) {
        index = index + 1;
      }
      for (int i = index; i < start_joint_position->position.size(); i++) {
        wam_jp_srv.request.joint_state.position.push_back(
            start_joint_position->position[i]);
      }
    }
  }
  return wam_jp_srv;
}

void playJointPositions(std::string bag_name, ros::Publisher rt_joint_position_publisher,
          ros::ServiceClient joint_position_move_client) {
  rosbag::Bag wam_bag;
  wam_bag.open(bag_name);
  ros::Time current_time = ros::Time::now();
  ros::Time prevTime = current_time;
  ros::Time current_msg_time;
  ros::Time prev_msg_time;
  bool first_check = true;
  bool second_check = true;
  std::vector<double> prev_msg;
  ros::Rate loop_rate(500);
  for (rosbag::MessageInstance const message_instance : rosbag::View(wam_bag)) {
    ros::Time current_time = ros::Time::now();
    current_msg_time = message_instance.getTime();
    if (first_check) {
      sensor_msgs::JointState::ConstPtr start_joint_position = message_instance.instantiate<sensor_msgs::JointState>();
      wam_srvs::JointMove wam_jp_srv = setServiceCall(start_joint_position);
      ROS_INFO_STREAM("Moving WAM to start pose " << *start_joint_position);
      if (!joint_position_move_client.call(wam_jp_srv)) { //move to start position at first check
      } else if (!wam_jp_srv.response.response) {
        ROS_ERROR("Could not move to start Pose 2");
        exit(0);
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));
      ROS_INFO("Moved WAM to start Pose");
      first_check = false;
      prev_msg_time = current_msg_time;
    }
    wam_msgs::RTJointPositions::ConstPtr rt_bag_msg =
        message_instance.instantiate<wam_msgs::RTJointPositions>();
    if (rt_bag_msg != nullptr) {
/*       while ((current_time - prevTime).toSec() <
             (current_msg_time - prev_msg_time).toSec()) {  // wait until (current_msg_time - prev_msg_time) has passed
        current_time = ros::Time::now();
      } */
      prev_msg_time = current_msg_time;
      prevTime = current_time;
      rt_joint_position_publisher.publish(rt_bag_msg);
      prev_msg = rt_bag_msg->joint_states;
      loop_rate.sleep();
    }
  }
  wam_bag.close();
}

void playJointVelocities(std::string bag_name, ros::Publisher rt_jv_pub,
          ros::ServiceClient joint_position_move_client) {
  rosbag::Bag wam_bag;
  wam_bag.open(bag_name);
  ros::Time current_time = ros::Time::now();
  ros::Time prevTime = current_time;
  ros::Time current_msg_time;
  ros::Time prev_msg_time;
  bool first_check = true;
  for (rosbag::MessageInstance const message_instance : rosbag::View(wam_bag)) {
    ros::Time current_time = ros::Time::now();
    current_msg_time = message_instance.getTime();
    if (first_check) { //move to start position at first check
      sensor_msgs::JointState::ConstPtr start_joint_position = message_instance.instantiate<sensor_msgs::JointState>();
      wam_srvs::JointMove wam_jp_srv = setServiceCall(start_joint_position);
      ROS_INFO_STREAM("Moving WAM to start pose " << *start_joint_position);
      if (!joint_position_move_client.call(wam_jp_srv)) { //move to start position at first check
      } else if (!wam_jp_srv.response.response) {
        ROS_ERROR("Could not move to start Pose");
        exit(0);
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));
      first_check = false;
      prev_msg_time = current_msg_time;
    }
    wam_msgs::RTJointVelocities::ConstPtr rt_bag_msg =
        message_instance.instantiate<wam_msgs::RTJointVelocities>();
    if (rt_bag_msg != nullptr) {
      while ((current_time - prevTime).toSec() <
             (current_msg_time - prev_msg_time).toSec()) {  //wait until (current_msg_time - prev_msg_time) has passed.
        current_time = ros::Time::now();
      }
      prev_msg_time = current_msg_time;
      prevTime = current_time;
      rt_jv_pub.publish(rt_bag_msg);
    }
  }
  wam_bag.close();
}

void playToolPoses(std::string bag_name, ros::Publisher rt_tp_pub,
          ros::ServiceClient joint_position_move_client) {
  rosbag::Bag wam_bag;
  wam_bag.open(bag_name);
  ros::Time current_time = ros::Time::now();
  ros::Time prevTime = current_time;
  ros::Time current_msg_time;
  ros::Time prev_msg_time;
  bool first_check = true;
  for (rosbag::MessageInstance const message_instance : rosbag::View(wam_bag)) {
    ros::Time current_time = ros::Time::now();
    current_msg_time = message_instance.getTime();
    if (first_check) { //move to start position at first check
      sensor_msgs::JointState::ConstPtr start_joint_position = message_instance.instantiate<sensor_msgs::JointState>();
      wam_srvs::JointMove wam_jp_srv = setServiceCall(start_joint_position);
      ROS_INFO_STREAM("Moving WAM to start pose " << *start_joint_position);
      if (!joint_position_move_client.call(wam_jp_srv)) { //move to start position at first check
      } else if (!wam_jp_srv.response.response) {
        ROS_ERROR("Could not move to start Pose");
        exit(0);
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));
      first_check = false;
      prev_msg_time = current_msg_time;
    }
    wam_msgs::RTCartPose::ConstPtr rt_bag_msg =
        message_instance.instantiate<wam_msgs::RTCartPose>();
    if (rt_bag_msg != nullptr) {
      while ((current_time - prevTime).toSec() <
             (current_msg_time - prev_msg_time)
                 .toSec()) {  // wait until (current_msg_time - prev_msg_time)
                              // has passed.
        current_time = ros::Time::now();
      }
      prev_msg_time = current_msg_time;
      prevTime = current_time;
      rt_tp_pub.publish(rt_bag_msg);
    }
  }
  wam_bag.close();
}
void playToolVelocities(std::string bag_name, ros::Publisher rt_tv_pub,
          ros::ServiceClient joint_position_move_client) {
  rosbag::Bag wam_bag;
  wam_bag.open(bag_name);
  ros::Time current_time = ros::Time::now();
  ros::Time prevTime = current_time;
  ros::Time current_msg_time;
  ros::Time prev_msg_time;
  bool first_check = true;
  for (rosbag::MessageInstance const message_instance : rosbag::View(wam_bag)) {
    ros::Time current_time = ros::Time::now();
    current_msg_time = message_instance.getTime();
    if (first_check) { //move to start position at first check
      sensor_msgs::JointState::ConstPtr start_joint_position = message_instance.instantiate<sensor_msgs::JointState>();
      wam_srvs::JointMove wam_jp_srv = setServiceCall(start_joint_position);
      ROS_INFO_STREAM("Moving WAM to start pose " << *start_joint_position);
      if (!joint_position_move_client.call(wam_jp_srv)) { //move to start position at first check
      } else if (!wam_jp_srv.response.response) {
        ROS_ERROR("Could not move to start Pose");
        exit(0);
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));
      first_check = false;
      prev_msg_time = current_msg_time;
    }
    wam_msgs::RTLinearandAngularVelocity::ConstPtr rt_bag_msg =
        message_instance
            .instantiate<wam_msgs::RTLinearandAngularVelocity>();
    if (rt_bag_msg != nullptr) {
      while ((current_time - prevTime).toSec() <
             (current_msg_time - prev_msg_time)
                 .toSec()) {  // wait until (current_msg_time - prev_msg_time)
                              // has passed.
        current_time = ros::Time::now();
      }
      prev_msg_time = current_msg_time;
      prevTime = current_time;
      rt_tv_pub.publish(*rt_bag_msg);
    }
  }
  wam_bag.close();
}
int getBagInfo(std::string ros_bag_name) {
  rosbag::Bag wam_bag;
  wam_bag.open(ros_bag_name);
  rosbag::View wam_bag_view(wam_bag);
  std::vector<const rosbag::ConnectionInfo *> connection_info = wam_bag_view.getConnections();
  std::string topic = connection_info.at(1)->topic; //get RT message topic, and use that to determine how the trajectory was recorded
  int option;
  if (topic.compare("/wam/RTJointPositionCMD") == 0) {
    ROS_INFO("Rosbag recorded using Joint Positions");
    option = 0;
  } else if (topic.compare("/wam/RTJointVelocityCMD") == 0) {
    ROS_INFO("Rosbag recorded using Joint Velocities");
    option = 3;
  } else if (topic.compare("/wam/RTCartPoseCMD") == 0) {
    ROS_INFO("Rosbag recorded using Cartesian Poses");
    option = 1;
  } else if (topic.compare("/wam/RTLinearandAngularVelocityCMD") == 0) {
    ROS_INFO("Rosbag recorded using Linear and Angular Velocities");
    option = 2;
  }
  wam_bag.close();
  return option;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wam_play");
  ros::NodeHandle n;
  std::string input;
  ros::Rate loop_rate(500);
  std::string ros_bag_name = "wam.bag";
  if (argc > 1) {
    ros_bag_name = argv[1];
  }
  ros_bag_name = "wam_rosbags/" + ros_bag_name;
  std::cout << "Name:" << ros_bag_name << std::endl;
  int option = getBagInfo(ros_bag_name); //jp:0, tp: 1, tv: 2, jv: 3
  ROS_INFO("Press any key to start Playing");
  getline(std::cin, input);
  ROS_INFO("Press ctrl-c to stop Playing");
  switch (option) {
    case 0:
      playJointPositions(
          ros_bag_name,
          n.advertise<wam_msgs::RTJointPositions>("/wam/RTJointPositionCMD", 1000),
          n.serviceClient<wam_srvs::JointMove>("/wam/moveToJointPosition"));
          break;
    case 1:
      playToolPoses(
          ros_bag_name,
          n.advertise<wam_msgs::RTCartPose>("/wam/RTCartPoseCMD", 1000),
          n.serviceClient<wam_srvs::JointMove>("/wam/moveToJointPosition"));
          break;
    case 2:
      playToolVelocities(
          ros_bag_name,
          n.advertise<wam_msgs::RTLinearandAngularVelocity>(
              "/wam/RTLinearandAngularVelocityCMD", 1000),
          n.serviceClient<wam_srvs::JointMove>("/wam/moveToJointPosition"));
          break;
    case 3:
      playJointVelocities(
          ros_bag_name,
          n.advertise<wam_msgs::RTJointVelocities>("/wam/RTJointVelocityCMD", 1000),
          n.serviceClient<wam_srvs::JointMove>("/wam/moveToJointPosition"));
          break;
  }
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}