/*
 Copyright 2019 Barrett Technology <support@barrett.com>

 ROS 2 port for standalone BarrettHand control with libbarrett.
 */

#include <unistd.h>

#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/wrench.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "wam_msgs/msg/ft_torques.hpp"
#include "wam_msgs/msg/tactile_pressure.hpp"
#include "wam_msgs/msg/tactile_pressure_array.hpp"
#include "wam_srvs/srv/b_hand_finger_pos.hpp"
#include "wam_srvs/srv/b_hand_finger_vel.hpp"
#include "wam_srvs/srv/b_hand_grasp_pos.hpp"
#include "wam_srvs/srv/b_hand_grasp_vel.hpp"
#include "wam_srvs/srv/b_hand_spread_pos.hpp"
#include "wam_srvs/srv/b_hand_spread_vel.hpp"

#include <barrett/detail/stl_utils.h>
#include <barrett/math.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>

static const int PUBLISH_FREQ = 250;

using namespace barrett;

class BarrettHandNode
{
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

protected:
  Hand * hand;
  ForceTorqueSensor * fts;
  std::shared_ptr<rclcpp::Node> node_;
  sensor_msgs::msg::JointState bhand_joint_state;
  wam_msgs::msg::TactilePressureArray tactileStates;
  wam_msgs::msg::TactilePressure tactileState;
  geometry_msgs::msg::Wrench fts_state;
  wam_msgs::msg::FtTorques ftTorque_state;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr bhand_joint_state_pub;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fts_pub;
  rclcpp::Publisher<wam_msgs::msg::TactilePressureArray>::SharedPtr tps_pub;
  rclcpp::Publisher<wam_msgs::msg::FtTorques>::SharedPtr fingerTs_pub;
  cf_type cf;
  ct_type ct;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr hand_initialize_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr hand_open_grsp_srv, hand_close_grsp_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr hand_open_sprd_srv, hand_close_sprd_srv;
  rclcpp::Service<wam_srvs::srv::BHandFingerPos>::SharedPtr hand_fngr_pos_srv;
  rclcpp::Service<wam_srvs::srv::BHandFingerVel>::SharedPtr hand_fngr_vel_srv;
  rclcpp::Service<wam_srvs::srv::BHandGraspPos>::SharedPtr hand_grsp_pos_srv;
  rclcpp::Service<wam_srvs::srv::BHandGraspVel>::SharedPtr hand_grsp_vel_srv;
  rclcpp::Service<wam_srvs::srv::BHandSpreadPos>::SharedPtr hand_sprd_pos_srv;
  rclcpp::Service<wam_srvs::srv::BHandSpreadVel>::SharedPtr hand_sprd_vel_srv;

public:
  BarrettHandNode() : hand(nullptr), fts(nullptr) {}

  void init(ProductManager & pm);
  std::shared_ptr<rclcpp::Node> node() { return node_; }

  bool handInitialize(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
  bool handOpenGrasp(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
  bool handCloseGrasp(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
  bool handOpenSpread(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
  bool handCloseSpread(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
  bool handFingerPos(const std::shared_ptr<wam_srvs::srv::BHandFingerPos::Request>, std::shared_ptr<wam_srvs::srv::BHandFingerPos::Response>);
  bool handGraspPos(const std::shared_ptr<wam_srvs::srv::BHandGraspPos::Request>, std::shared_ptr<wam_srvs::srv::BHandGraspPos::Response>);
  bool handSpreadPos(const std::shared_ptr<wam_srvs::srv::BHandSpreadPos::Request>, std::shared_ptr<wam_srvs::srv::BHandSpreadPos::Response>);
  bool handFingerVel(const std::shared_ptr<wam_srvs::srv::BHandFingerVel::Request>, std::shared_ptr<wam_srvs::srv::BHandFingerVel::Response>);
  bool handGraspVel(const std::shared_ptr<wam_srvs::srv::BHandGraspVel::Request>, std::shared_ptr<wam_srvs::srv::BHandGraspVel::Response>);
  bool handSpreadVel(const std::shared_ptr<wam_srvs::srv::BHandSpreadVel::Request>, std::shared_ptr<wam_srvs::srv::BHandSpreadVel::Response>);
  void publishHand();
  void publishFTS();
};

void BarrettHandNode::init(ProductManager & pm)
{
  node_ = std::make_shared<rclcpp::Node>("barrett_hand_node");
  hand = pm.getHand();
  if (hand == nullptr) {
    throw std::runtime_error("No BarrettHand was discovered by libbarrett.");
  }

  if (hand->hasFingertipTorqueSensors()) {
    fingerTs_pub = node_->create_publisher<wam_msgs::msg::FtTorques>("bhand/finger_tip_states", 1);
    if (hand->hasTactSensors()) {
      tps_pub = node_->create_publisher<wam_msgs::msg::TactilePressureArray>("bhand/tactile_states", 1);
      RCLCPP_INFO(node_->get_logger(), "Barrett Hand with Fingertip Torque and Tactile Sensors");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Barrett Hand with Fingertip Sensors");
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "Barrett Hand with no sensors");
  }

  if (pm.foundForceTorqueSensor()) {
    RCLCPP_INFO(node_->get_logger(), "Force/Torque sensor");
    fts = pm.getForceTorqueSensor();
    fts->tare();
    fts_pub = node_->create_publisher<geometry_msgs::msg::Wrench>("bhand/fts_states", 1);
  }

  usleep(500000);
  hand->initialize();
  hand->update();

  bhand_joint_state_pub = node_->create_publisher<sensor_msgs::msg::JointState>("bhand/joint_states", 1);
  hand_initialize_srv = node_->create_service<std_srvs::srv::Empty>("bhand/initialize", std::bind(&BarrettHandNode::handInitialize, this, std::placeholders::_1, std::placeholders::_2));
  hand_open_grsp_srv = node_->create_service<std_srvs::srv::Empty>("bhand/open_grasp", std::bind(&BarrettHandNode::handOpenGrasp, this, std::placeholders::_1, std::placeholders::_2));
  hand_close_grsp_srv = node_->create_service<std_srvs::srv::Empty>("bhand/close_grasp", std::bind(&BarrettHandNode::handCloseGrasp, this, std::placeholders::_1, std::placeholders::_2));
  hand_open_sprd_srv = node_->create_service<std_srvs::srv::Empty>("bhand/open_spread", std::bind(&BarrettHandNode::handOpenSpread, this, std::placeholders::_1, std::placeholders::_2));
  hand_close_sprd_srv = node_->create_service<std_srvs::srv::Empty>("bhand/close_spread", std::bind(&BarrettHandNode::handCloseSpread, this, std::placeholders::_1, std::placeholders::_2));
  hand_fngr_pos_srv = node_->create_service<wam_srvs::srv::BHandFingerPos>("bhand/finger_pos", std::bind(&BarrettHandNode::handFingerPos, this, std::placeholders::_1, std::placeholders::_2));
  hand_grsp_pos_srv = node_->create_service<wam_srvs::srv::BHandGraspPos>("bhand/grasp_pos", std::bind(&BarrettHandNode::handGraspPos, this, std::placeholders::_1, std::placeholders::_2));
  hand_sprd_pos_srv = node_->create_service<wam_srvs::srv::BHandSpreadPos>("bhand/spread_pos", std::bind(&BarrettHandNode::handSpreadPos, this, std::placeholders::_1, std::placeholders::_2));
  hand_fngr_vel_srv = node_->create_service<wam_srvs::srv::BHandFingerVel>("bhand/finger_vel", std::bind(&BarrettHandNode::handFingerVel, this, std::placeholders::_1, std::placeholders::_2));
  hand_grsp_vel_srv = node_->create_service<wam_srvs::srv::BHandGraspVel>("bhand/grasp_vel", std::bind(&BarrettHandNode::handGraspVel, this, std::placeholders::_1, std::placeholders::_2));
  hand_sprd_vel_srv = node_->create_service<wam_srvs::srv::BHandSpreadVel>("bhand/spread_vel", std::bind(&BarrettHandNode::handSpreadVel, this, std::placeholders::_1, std::placeholders::_2));

  const char * bhand_jnts[] = {"inner_f1", "inner_f2", "inner_f3", "spread", "outer_f1", "outer_f2", "outer_f3"};
  bhand_joint_state.name.assign(bhand_jnts, bhand_jnts + 7);
  bhand_joint_state.position.resize(7);
  tactileState.pressure.resize(24);
  tactileState.normalized_pressure.resize(24);
  tactileStates.tactile_pressures.resize(4);
  ftTorque_state.torque.resize(4);
}

bool BarrettHandNode::handInitialize(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Initializing the BarrettHand");
  hand->initialize();
  return true;
}

bool BarrettHandNode::handOpenGrasp(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Opening the BarrettHand Grasp");
  hand->open(Hand::GRASP, false);
  return true;
}

bool BarrettHandNode::handCloseGrasp(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Closing the BarrettHand Grasp");
  hand->close(Hand::GRASP, false);
  return true;
}

bool BarrettHandNode::handOpenSpread(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Opening the BarrettHand Spread");
  hand->open(Hand::SPREAD, false);
  return true;
}

bool BarrettHandNode::handCloseSpread(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Closing the BarrettHand Spread");
  hand->close(Hand::SPREAD, false);
  return true;
}

bool BarrettHandNode::handFingerPos(const std::shared_ptr<wam_srvs::srv::BHandFingerPos::Request> req, std::shared_ptr<wam_srvs::srv::BHandFingerPos::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Moving BarrettHand to Finger Positions: %.3f, %.3f, %.3f radians", req->radians[0], req->radians[1], req->radians[2]);
  hand->trapezoidalMove(Hand::jp_type(req->radians[0], req->radians[1], req->radians[2], 0.0), Hand::GRASP, false);
  return true;
}

bool BarrettHandNode::handGraspPos(const std::shared_ptr<wam_srvs::srv::BHandGraspPos::Request> req, std::shared_ptr<wam_srvs::srv::BHandGraspPos::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Moving BarrettHand Grasp: %.3f radians", req->radians);
  hand->trapezoidalMove(Hand::jp_type(req->radians), Hand::GRASP, false);
  return true;
}

bool BarrettHandNode::handSpreadPos(const std::shared_ptr<wam_srvs::srv::BHandSpreadPos::Request> req, std::shared_ptr<wam_srvs::srv::BHandSpreadPos::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Moving BarrettHand Spread: %.3f radians", req->radians);
  hand->trapezoidalMove(Hand::jp_type(req->radians), Hand::SPREAD, false);
  return true;
}

bool BarrettHandNode::handFingerVel(const std::shared_ptr<wam_srvs::srv::BHandFingerVel::Request> req, std::shared_ptr<wam_srvs::srv::BHandFingerVel::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Moving BarrettHand Finger Velocities: %.3f, %.3f, %.3f m/s", req->velocity[0], req->velocity[1], req->velocity[2]);
  hand->velocityMove(Hand::jv_type(req->velocity[0], req->velocity[1], req->velocity[2], 0.0), Hand::GRASP);
  return true;
}

bool BarrettHandNode::handGraspVel(const std::shared_ptr<wam_srvs::srv::BHandGraspVel::Request> req, std::shared_ptr<wam_srvs::srv::BHandGraspVel::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Moving BarrettHand Grasp: %.3f m/s", req->velocity);
  hand->velocityMove(Hand::jv_type(req->velocity), Hand::GRASP);
  return true;
}

bool BarrettHandNode::handSpreadVel(const std::shared_ptr<wam_srvs::srv::BHandSpreadVel::Request> req, std::shared_ptr<wam_srvs::srv::BHandSpreadVel::Response>)
{
  RCLCPP_INFO(node_->get_logger(), "Moving BarrettHand Spread: %.3f m/s", req->velocity);
  usleep(5000);
  hand->velocityMove(Hand::jv_type(req->velocity), Hand::SPREAD);
  return true;
}

void BarrettHandNode::publishFTS()
{
  fts->update();
  cf = math::saturate(fts->getForce(), 99.99);
  ct = math::saturate(fts->getTorque(), 9.999);
  fts_state.force.x = cf[0];
  fts_state.force.y = cf[1];
  fts_state.force.z = cf[2];
  fts_state.torque.x = ct[0];
  fts_state.torque.y = ct[1];
  fts_state.torque.z = ct[2];
  fts_pub->publish(fts_state);
}

void BarrettHandNode::publishHand()
{
  hand->update(Hand::S_POSITION | Hand::S_FINGERTIP_TORQUE | Hand::S_TACT_TOP10);
  std::vector<int> fingerTip = hand->getFingertipTorque();
  Hand::jp_type hi = hand->getInnerLinkPosition();
  Hand::jp_type ho = hand->getOuterLinkPosition();

  if (hand->hasTactSensors()) {
    std::vector<TactilePuck *> tps = hand->getTactilePucks();
    for (unsigned i = 0; i < tps.size(); i++) {
      TactilePuck::v_type pressures(tps[i]->getTactileData());
      for (int j = 0; j < pressures.size(); j++) {
        int value = static_cast<int>(pressures[j] * 256.0) / 102;
        tactileState.pressure[j] = pressures[j];
        int c = 0;
        int chunk;
        for (int z = 4; z >= 0; --z) {
          chunk = (value <= 7) ? value : 7;
          value -= chunk;
          switch (chunk) {
            case 0: c += 1; break;
            case 1: c += 2; break;
            case 3: c += 2; break;
            default: c += 4; break;
          }
          switch (chunk - 4) {
            case 3: c += 4; break;
            case 2: c += 3; break;
            case 1: c += 2; break;
            case 0: c += 1; break;
            default: break;
          }
        }
        tactileState.normalized_pressure[j] = c - 5;
      }
      tactileStates.tactile_pressures[i] = tactileState;
    }
    tps_pub->publish(tactileStates);
  }
  if (hand->hasFingertipTorqueSensors()) {
    for (unsigned i = 0; i < fingerTip.size(); i++) {
      ftTorque_state.torque[i] = fingerTip[i];
    }
    fingerTs_pub->publish(ftTorque_state);
  }
  for (size_t i = 0; i < 4; i++) {
    bhand_joint_state.position[i] = hi[i];
  }
  for (size_t j = 0; j < 3; j++) {
    bhand_joint_state.position[j + 4] = ho[j];
  }
  bhand_joint_state.header.stamp = node_->now();
  bhand_joint_state_pub->publish(bhand_joint_state);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ProductManager pm;
  BarrettHandNode barrett_hand_node;
  barrett_hand_node.init(pm);
  rclcpp::Rate pub_rate(PUBLISH_FREQ);

  while (rclcpp::ok()) {
    barrett_hand_node.publishHand();
    if (pm.foundForceTorqueSensor()) {
      barrett_hand_node.publishFTS();
    }
    rclcpp::spin_some(barrett_hand_node.node());
    pub_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
