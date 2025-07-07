#ifndef SRC_WAM_DEMOS_SRC_TEACH_
#define SRC_WAM_DEMOS_SRC_TEACH_
//Set ratelimits very high. Negates the effects of systems::RateLimiter
const double kRateLimits = 500;
using namespace std::chrono; 
class Teach {
 private:
  rosbag::Bag wam_bag_;
  //command messages
  wam_msgs::RTJointPositions rt_joint_position_cmd_;
  wam_msgs::RTJointVelocities rt_joint_velocity_cmd_;
  wam_msgs::RTCartPose wam_cart_pose_cmd_;
  wam_msgs::RTLinearandAngularVelocity wam_tool_velocity_cmd_;
  
  sensor_msgs::JointState current_joint_position_;
  //indicators for recording rosbags
  bool joint_position_teach_, tool_pose_teach_, tool_velocity_teach_, joint_velocity_teach_;
  std::string bag_name_;
  bool teaching_;
  /**True if first message has not been written. 
   * First message contains start JP */
  bool first_write_;
  ros::ServiceClient idle_srv_;
  int option_;
 public:
  void wamJointPositionCb(const sensor_msgs::JointState::ConstPtr& msg);
  void wamJointVelocityCb(const sensor_msgs::JointState::ConstPtr& msg);
  void wamToolPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg); 
  void wamToolVelocityCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void startTeaching();
  void stopTeaching();
  Teach(std::string bag_name,
                 ros::ServiceClient idle_srv, int option) {
    bag_name_ = bag_name;
    option_ = option;
    joint_position_teach_ = tool_pose_teach_ = tool_velocity_teach_ = joint_velocity_teach_ = false;
    teaching_ = false;
    idle_srv_ = idle_srv;
  }
};

//callback function for /joint_states topic. If joint_position_teach is set, write the joint states to wam_bag_
void Teach::wamJointPositionCb(
    const sensor_msgs::JointState::ConstPtr& msg) {
  current_joint_position_ = *msg;
  if (joint_position_teach_) {
    int size = 0;
    int start = 0;
    if (msg->position.size() == 4 || msg->position.size() == 7) {
      size = msg->position.size();
      start = 0;
    } else {
      //Avoid bhand joint states
      size = msg->position.size() - 8;
      if (msg->name[0].compare("q1") == 0) {
        //Fields from 0 to "size" are WAM joint states
        start = 0;
      } else {
        //Fields from 8 to "size + 8" are WAM joint states
        start = 8;
      }
    }
    rt_joint_position_cmd_.joint_states.resize(size);
    rt_joint_position_cmd_.rate_limits.resize(size);
    for (int i = start; i < (start+size); i++) {
      rt_joint_position_cmd_.joint_states.at(i-start) = msg->position.at(i);
      rt_joint_position_cmd_.rate_limits.at(i-start) = kRateLimits;
    }
    wam_bag_.write("/wam/RTJointPositionCMD", ros::Time::now(), rt_joint_position_cmd_);
  }
}

//callback function for /wam/jointVelocity topic. If joint_velocity_teach_ is set, write the joint velocity to wam_bag_
void Teach::wamJointVelocityCb(
    const sensor_msgs::JointState::ConstPtr& msg) {
  if (joint_velocity_teach_) {
    rt_joint_velocity_cmd_.velocities.resize(msg->velocity.size());
    for (int i = 0; i < msg->velocity.size(); i++) {
      rt_joint_velocity_cmd_.velocities.at(i) = msg->velocity.at(i);
    }
    wam_bag_.write("/wam/RTJointVelocityCMD", ros::Time::now(), rt_joint_velocity_cmd_);
  }
}

//callback function for /wam/toolPose topic. If tool_pose_teach_ is set, write the tool pose to wam_bag_
void Teach::wamToolPoseCb(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (tool_pose_teach_) {
    wam_cart_pose_cmd_.point = msg->pose.position;
    wam_cart_pose_cmd_.orientation = msg->pose.orientation;
    wam_bag_.write("/wam/RTCartPoseCMD", ros::Time::now(), wam_cart_pose_cmd_);
    for (int i = 0; i < 3; i++) {
        wam_cart_pose_cmd_.position_rate_limits.at(i) = kRateLimits * 0.1;
        wam_cart_pose_cmd_.orientation_rate_limits.at(i) = kRateLimits * 0.1;
    }
    wam_cart_pose_cmd_.orientation_rate_limits.at(3) = kRateLimits * 0.1;
  }
}

//callback function for /wam/toolVelocity topic. If tool_velocity_teach_ is set, write the tool velocity to wam_bag_
void Teach::wamToolVelocityCb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  if (tool_velocity_teach_) {
    wam_tool_velocity_cmd_.linear_velocity_magnitude =
        sqrt((msg->twist.linear.x * msg->twist.linear.x) + (msg->twist.linear.y * msg->twist.linear.y) +
             (msg->twist.linear.z * msg->twist.linear.z));
    wam_tool_velocity_cmd_.angular_velocity_magnitude = sqrt((msg->twist.angular.x * msg->twist.angular.x) +
                                             (msg->twist.angular.y * msg->twist.angular.y) +
                                             (msg->twist.angular.z * msg->twist.angular.z));
    if (wam_tool_velocity_cmd_.angular_velocity_magnitude != 0) {
      wam_tool_velocity_cmd_.angular_velocity_direction[0] =
          (msg->twist.angular.x) / wam_tool_velocity_cmd_.angular_velocity_magnitude;
      wam_tool_velocity_cmd_.angular_velocity_direction[1] =
          (msg->twist.angular.x) / wam_tool_velocity_cmd_.angular_velocity_magnitude;
      wam_tool_velocity_cmd_.angular_velocity_direction[2] =
          (msg->twist.angular.x) / wam_tool_velocity_cmd_.angular_velocity_magnitude;
    } else {
      wam_tool_velocity_cmd_.angular_velocity_direction[0] = 0;
      wam_tool_velocity_cmd_.angular_velocity_direction[1] = 0;
      wam_tool_velocity_cmd_.angular_velocity_direction[2] = 0;
    }
    if (wam_tool_velocity_cmd_.linear_velocity_magnitude != 0) {
      wam_tool_velocity_cmd_.linear_velocity_direction[0] =
          (msg->twist.linear.x) / wam_tool_velocity_cmd_.linear_velocity_magnitude;
      wam_tool_velocity_cmd_.linear_velocity_direction[1] =
          (msg->twist.linear.y) / wam_tool_velocity_cmd_.linear_velocity_magnitude;
      wam_tool_velocity_cmd_.linear_velocity_direction[2] =
          (msg->twist.linear.z) / wam_tool_velocity_cmd_.linear_velocity_magnitude;
    } else {
      wam_tool_velocity_cmd_.linear_velocity_direction[0] = 0;
      wam_tool_velocity_cmd_.linear_velocity_direction[1] = 0;
      wam_tool_velocity_cmd_.linear_velocity_direction[2] = 0;
    }
    wam_bag_.write("/wam/RTLinearandAngularVelocityCMD", ros::Time::now(),
                  wam_tool_velocity_cmd_);
  }
}

//Callback of teach_thread. Idle wam and start teaching. Spin until teaching_ is set to false.
void Teach::startTeaching() {
  wam_bag_.open(bag_name_, rosbag::bagmode::Write);
  teaching_ = true;
  /*Write current_joint_position_ to a random topic. Will be used by the play program to move the WAM to the start position */
  wam_bag_.write("startJP", current_joint_position_.header.stamp, current_joint_position_); 
  switch (option_) {
    case 0:
      joint_position_teach_ = true;
      break;
    case 1:
      tool_pose_teach_ = true;
      break;
    case 2:
      tool_velocity_teach_ = true;
      break;
    case 3:
      joint_velocity_teach_ = true;
      break;
  }
  std_srvs::Trigger idle_srv_call;
  if (!idle_srv_.call(idle_srv_call)) {
    ROS_ERROR("Could not idle WAM");
    exit(0);
  } else if (!idle_srv_call.response.success) {
    ROS_ERROR("Could not idle WAM");
    exit(0);
  }
  ros::Rate loop_rate(500);
  while (ros::ok() && teaching_) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Teach::stopTeaching() {
  joint_position_teach_ = tool_pose_teach_ = tool_velocity_teach_ = joint_velocity_teach_ = false;
  wam_bag_.close();
  teaching_ = false;
}
#endif // SRC_WAM_DEMOS_SRC_TEACH_