#ifndef SRC_WAM_MOVEIT_NODE_SRC_GAZEBO_PUBLISHER_H_
#define SRC_WAM_MOVEIT_NODE_SRC_GAZEBO_PUBLISHER_H_
// Class to Publish Gazebo messages to ROS Topics
#include "bhand_msgs/TactileState.h"
#include "bhand_msgs/TactileStateArray.h"
class Gazebo_Publishers {
  public:
		void tactileSensorCb(ConstTactilePtr &msg);
		Gazebo_Publishers(ros::Publisher tactile_state_publisher);
		void publishMsgs();
	private:
		ros::Publisher tactile_state_publisher_;
		bhand_msgs::TactileState tactile_state_msg_, prev_tactile_msg_;
		bhand_msgs::TactileStateArray tactile_state_array_;
};

Gazebo_Publishers::Gazebo_Publishers(ros::Publisher tactile_state_publisher) : tactile_state_publisher_(tactile_state_publisher) {
	tactile_state_array_.tactile_states.resize(1);
}

void Gazebo_Publishers::tactileSensorCb(ConstTactilePtr &msg) {
	const google::protobuf::RepeatedPtrField<std::string> collision_names = msg->collision_name();
	std::fill(tactile_state_msg_.tactile_state.begin(), tactile_state_msg_.tactile_state.end(), 0); //set all pressures to 0
	auto pressure_it = msg->pressure().begin();
	for (auto it = collision_names.begin(); it < collision_names.end(); it++) {
		std::string collision_name = *it;
		collision_name.erase(0,26); //remove "robot::base::pressure_pad_" from the message
		int collision_index = std::stoi(collision_name); //convert string to int
		tactile_state_msg_.tactile_state[collision_index-1] = *pressure_it;
		pressure_it++;
	}
}

void Gazebo_Publishers::publishMsgs() {
	if (prev_tactile_msg_.tactile_state == tactile_state_msg_.tactile_state) { //no new Gazebo message recieved. Current Pressures are 0
		std::fill(tactile_state_msg_.tactile_state.begin(), tactile_state_msg_.tactile_state.end(), 0); //set all pressures to 0
	}
	tactile_state_array_.tactile_states.at(0) = tactile_state_msg_;
	tactile_state_publisher_.publish(tactile_state_array_);
	prev_tactile_msg_ = tactile_state_msg_;
}

#endif //SRC_WAM_MOVEIT_NODE_SRC_GAZEBO_PUBLISHER_H_