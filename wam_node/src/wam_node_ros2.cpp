/*
 Copyright 2019 Barrett Technology <support@barrett.com>

 ROS 2 port for Barrett WAM control with libbarrett.
 */

#include <unistd.h>

#include <cmath>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

#include <boost/tuple/tuple.hpp>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "wam_msgs/msg/ft_torques.hpp"
#include "wam_msgs/msg/rt_cart_pos.hpp"
#include "wam_msgs/msg/rt_cart_vel.hpp"
#include "wam_msgs/msg/rt_joint_pos.hpp"
#include "wam_msgs/msg/rt_joint_vel.hpp"
#include "wam_msgs/msg/rt_ortn_vel.hpp"
#include "wam_msgs/msg/tactile_pressure.hpp"
#include "wam_msgs/msg/tactile_pressure_array.hpp"
#include "wam_srvs/srv/b_hand_finger_pos.hpp"
#include "wam_srvs/srv/b_hand_finger_vel.hpp"
#include "wam_srvs/srv/b_hand_grasp_pos.hpp"
#include "wam_srvs/srv/b_hand_grasp_vel.hpp"
#include "wam_srvs/srv/b_hand_spread_pos.hpp"
#include "wam_srvs/srv/b_hand_spread_vel.hpp"
#include "wam_srvs/srv/cart_pos_move.hpp"
#include "wam_srvs/srv/gravity_comp.hpp"
#include "wam_srvs/srv/hold.hpp"
#include "wam_srvs/srv/joint_move.hpp"
#include "wam_srvs/srv/ortn_move.hpp"
#include "wam_srvs/srv/pose_move.hpp"

#include <barrett/detail/stl_utils.h>
#include <barrett/math.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/systems/wam.h>
#include <barrett/units.h>

#define BARRETT_SMF_CONFIGURE_PM
#include <barrett/standard_main_function.h>

static const int WAM_CONTROL_RATE = 500;
static const int WAM_PUBLISH_FREQ = 500;
static const int FT_PUBLISH_FREQ = 500;
static const int BH_PUBLISH_FREQ = 40;
static const int SAFETY_MODE_FREQ = 10;
static const double SPEED = 0.03;

using namespace barrett;

bool configure_pm(int argc, char ** argv, ::ProductManager & pm)
{
  (void)argc;
  (void)argv;
  pm.getExecutionManager(1.0 / WAM_CONTROL_RATE);
  return true;
}

template<typename T1, typename T2, typename OutputType>
class Multiplier : public systems::System, public systems::SingleOutput<OutputType>
{
public:
  Input<T1> input1;
  Input<T2> input2;

  explicit Multiplier(std::string sysName = "Multiplier")
  : systems::System(sysName), systems::SingleOutput<OutputType>(this), input1(this), input2(this)
  {
  }

  ~Multiplier() override
  {
    mandatoryCleanUp();
  }

protected:
  OutputType data;

  void operate() override
  {
    data = input1.getValue() * input2.getValue();
    this->outputValue->setData(&data);
  }

private:
  DISALLOW_COPY_AND_ASSIGN(Multiplier);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ToQuaternion : public systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond>
{
public:
  Eigen::Quaterniond outputQuat;

  explicit ToQuaternion(std::string sysName = "ToQuaternion")
  : systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond>(sysName)
  {
  }

  ~ToQuaternion() override
  {
    mandatoryCleanUp();
  }

protected:
  void operate() override
  {
    const math::Vector<3>::type & inputRPY = input.getValue();
    tf2::Quaternion q;
    q.setRPY(inputRPY[0], inputRPY[1], inputRPY[2]);
    outputQuat.x() = q.x();
    outputQuat.y() = q.y();
    outputQuat.z() = q.z();
    outputQuat.w() = q.w();
    this->outputValue->setData(&outputQuat);
  }

private:
  DISALLOW_COPY_AND_ASSIGN(ToQuaternion);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

math::Vector<3>::type toRPY(Eigen::Quaterniond inquat)
{
  math::Vector<3>::type newRPY;
  tf2::Quaternion q(inquat.x(), inquat.y(), inquat.z(), inquat.w());
  tf2::Matrix3x3(q).getRPY(newRPY[0], newRPY[1], newRPY[2]);
  return newRPY;
}

template<size_t DOF>
class WamNode
{
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

protected:
  bool cart_vel_status, ortn_vel_status, jnt_vel_status;
  bool jnt_pos_status, cart_pos_status, ortn_pos_status, new_rt_cmd;
  double cart_vel_mag, ortn_vel_mag;
  double home_velocity, home_acceleration;
  systems::Wam<DOF> & wam;
  Hand * hand;
  ForceTorqueSensor * fts;
  jp_type jp_cmd, jp_home;
  jp_type rt_jp_cmd, rt_jp_rl;
  jv_type rt_jv_cmd;
  cp_type cp_cmd, rt_cv_cmd;
  cp_type rt_cp_cmd, rt_cp_rl;
  cf_type cf;
  ct_type ct;
  Eigen::Quaterniond ortn_cmd, rt_op_cmd, rt_op_rl;
  pose_type pose_cmd;
  math::Vector<3>::type rt_ortn_cmd;
  systems::ExposedOutput<Eigen::Quaterniond> orientationSetPoint, current_ortn;
  systems::ExposedOutput<cp_type> cart_dir, current_cart_pos, cp_track;
  systems::ExposedOutput<math::Vector<3>::type> rpy_cmd, current_rpy_ortn;
  systems::ExposedOutput<jv_type> jv_track;
  systems::ExposedOutput<jp_type> jp_track;
  systems::TupleGrouper<cp_type, Eigen::Quaterniond> rt_pose_cmd;
  systems::Summer<cp_type> cart_pos_sum;
  systems::Summer<math::Vector<3>::type> ortn_cmd_sum;
  systems::Ramp ramp;
  systems::RateLimiter<jp_type> jp_rl;
  systems::RateLimiter<cp_type> cp_rl;
  Multiplier<double, cp_type, cp_type> mult_linear;
  Multiplier<double, math::Vector<3>::type, math::Vector<3>::type> mult_angular;
  ToQuaternion to_quat, to_quat_print;
  Eigen::Quaterniond ortn_print;
  rclcpp::Time last_cart_vel_msg_time, last_ortn_vel_msg_time, last_jnt_vel_msg_time;
  rclcpp::Time last_jnt_pos_msg_time, last_cart_pos_msg_time, last_ortn_pos_msg_time;
  rclcpp::Duration rt_msg_timeout;

  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Subscription<wam_msgs::msg::RTCartVel>::SharedPtr cart_vel_sub;
  rclcpp::Subscription<wam_msgs::msg::RTOrtnVel>::SharedPtr ortn_vel_sub;
  rclcpp::Subscription<wam_msgs::msg::RTJointVel>::SharedPtr jnt_vel_sub;
  rclcpp::Subscription<wam_msgs::msg::RTJointPos>::SharedPtr jnt_pos_sub;
  rclcpp::Subscription<wam_msgs::msg::RTCartPos>::SharedPtr cart_pos_sub;

  sensor_msgs::msg::JointState wam_joint_state, bhand_joint_state;
  wam_msgs::msg::FtTorques ftTorque_state;
  wam_msgs::msg::TactilePressureArray tactileStates;
  wam_msgs::msg::TactilePressure tactileState;
  geometry_msgs::msg::PoseStamped wam_pose;
  geometry_msgs::msg::Wrench fts_state;
  std_msgs::msg::Bool move_is_done;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wam_joint_state_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wam_move_state_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr bhand_joint_state_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr wam_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fts_pub;
  rclcpp::Publisher<wam_msgs::msg::TactilePressureArray>::SharedPtr tps_pub;
  rclcpp::Publisher<wam_msgs::msg::FtTorques>::SharedPtr fingerTs_pub;

  rclcpp::Service<wam_srvs::srv::GravityComp>::SharedPtr gravity_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr go_home_srv;
  rclcpp::Service<wam_srvs::srv::Hold>::SharedPtr hold_jpos_srv, hold_cpos_srv, hold_ortn_srv;
  rclcpp::Service<wam_srvs::srv::JointMove>::SharedPtr joint_move_srv;
  rclcpp::Service<wam_srvs::srv::PoseMove>::SharedPtr pose_move_srv;
  rclcpp::Service<wam_srvs::srv::CartPosMove>::SharedPtr cart_move_srv;
  rclcpp::Service<wam_srvs::srv::OrtnMove>::SharedPtr ortn_move_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr hand_open_grsp_srv, hand_close_grsp_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr hand_open_sprd_srv, hand_close_sprd_srv;
  rclcpp::Service<wam_srvs::srv::BHandFingerPos>::SharedPtr hand_fngr_pos_srv;
  rclcpp::Service<wam_srvs::srv::BHandFingerVel>::SharedPtr hand_fngr_vel_srv;
  rclcpp::Service<wam_srvs::srv::BHandGraspPos>::SharedPtr hand_grsp_pos_srv;
  rclcpp::Service<wam_srvs::srv::BHandGraspVel>::SharedPtr hand_grsp_vel_srv;
  rclcpp::Service<wam_srvs::srv::BHandSpreadPos>::SharedPtr hand_sprd_pos_srv;
  rclcpp::Service<wam_srvs::srv::BHandSpreadVel>::SharedPtr hand_sprd_vel_srv;

public:
  explicit WamNode(systems::Wam<DOF> & wam_)
  : cart_vel_status(false),
    ortn_vel_status(false),
    jnt_vel_status(false),
    jnt_pos_status(false),
    cart_pos_status(false),
    ortn_pos_status(false),
    new_rt_cmd(false),
    cart_vel_mag(SPEED),
    ortn_vel_mag(SPEED),
    home_velocity(0.10),
    home_acceleration(0.10),
    wam(wam_),
    hand(nullptr),
    fts(nullptr),
    ramp(nullptr, SPEED),
    rt_msg_timeout(rclcpp::Duration::from_seconds(0.3))
  {
  }

  void init(ProductManager & pm);

  bool gravity(
    const std::shared_ptr<wam_srvs::srv::GravityComp::Request> req,
    std::shared_ptr<wam_srvs::srv::GravityComp::Response> res);
  bool goHome(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  bool holdJPos(
    const std::shared_ptr<wam_srvs::srv::Hold::Request> req,
    std::shared_ptr<wam_srvs::srv::Hold::Response> res);
  bool holdCPos(
    const std::shared_ptr<wam_srvs::srv::Hold::Request> req,
    std::shared_ptr<wam_srvs::srv::Hold::Response> res);
  bool holdOrtn(
    const std::shared_ptr<wam_srvs::srv::Hold::Request> req,
    std::shared_ptr<wam_srvs::srv::Hold::Response> res);
  bool jointMove(
    const std::shared_ptr<wam_srvs::srv::JointMove::Request> req,
    std::shared_ptr<wam_srvs::srv::JointMove::Response> res);
  bool poseMove(
    const std::shared_ptr<wam_srvs::srv::PoseMove::Request> req,
    std::shared_ptr<wam_srvs::srv::PoseMove::Response> res);
  bool cartMove(
    const std::shared_ptr<wam_srvs::srv::CartPosMove::Request> req,
    std::shared_ptr<wam_srvs::srv::CartPosMove::Response> res);
  bool ortnMove(
    const std::shared_ptr<wam_srvs::srv::OrtnMove::Request> req,
    std::shared_ptr<wam_srvs::srv::OrtnMove::Response> res);
  bool handOpenGrasp(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  bool handCloseGrasp(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  bool handOpenSpread(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  bool handCloseSpread(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  bool handFingerPos(
    const std::shared_ptr<wam_srvs::srv::BHandFingerPos::Request> req,
    std::shared_ptr<wam_srvs::srv::BHandFingerPos::Response> res);
  bool handGraspPos(
    const std::shared_ptr<wam_srvs::srv::BHandGraspPos::Request> req,
    std::shared_ptr<wam_srvs::srv::BHandGraspPos::Response> res);
  bool handSpreadPos(
    const std::shared_ptr<wam_srvs::srv::BHandSpreadPos::Request> req,
    std::shared_ptr<wam_srvs::srv::BHandSpreadPos::Response> res);
  bool handFingerVel(
    const std::shared_ptr<wam_srvs::srv::BHandFingerVel::Request> req,
    std::shared_ptr<wam_srvs::srv::BHandFingerVel::Response> res);
  bool handGraspVel(
    const std::shared_ptr<wam_srvs::srv::BHandGraspVel::Request> req,
    std::shared_ptr<wam_srvs::srv::BHandGraspVel::Response> res);
  bool handSpreadVel(
    const std::shared_ptr<wam_srvs::srv::BHandSpreadVel::Request> req,
    std::shared_ptr<wam_srvs::srv::BHandSpreadVel::Response> res);
  void cartVelCB(const wam_msgs::msg::RTCartVel::SharedPtr msg);
  void ortnVelCB(const wam_msgs::msg::RTOrtnVel::SharedPtr msg);
  void jntVelCB(const wam_msgs::msg::RTJointVel::SharedPtr msg);
  void jntPosCB(const wam_msgs::msg::RTJointPos::SharedPtr msg);
  void cartPosCB(const wam_msgs::msg::RTCartPos::SharedPtr msg);
  void publishWam(ProductManager & pm);
  void publishHand();
  void publishFTS();
  void updateRT(ProductManager & pm);
  std::shared_ptr<rclcpp::Node> node() { return node_; }
};

template<size_t DOF>
void WamNode<DOF>::init(ProductManager & pm)
{
  node_ = std::make_shared<rclcpp::Node>("wam_node");
  const bool auto_gravity_comp = node_->declare_parameter<bool>("auto_gravity_comp", true);
  const bool initialize_hand_on_startup =
    node_->declare_parameter<bool>("initialize_hand_on_startup", false);
  const bool hand_clearance_move_on_startup =
    node_->declare_parameter<bool>("hand_clearance_move_on_startup", false);
  home_velocity = node_->declare_parameter<double>("home_velocity", 0.10);
  home_acceleration = node_->declare_parameter<double>("home_acceleration", 0.10);
  const auto stale_time = node_->now() - rclcpp::Duration::from_seconds(1.0);
  last_cart_vel_msg_time = stale_time;
  last_ortn_vel_msg_time = stale_time;
  last_jnt_vel_msg_time = stale_time;
  last_jnt_pos_msg_time = stale_time;
  last_cart_pos_msg_time = stale_time;
  last_ortn_pos_msg_time = stale_time;

  pm.getExecutionManager()->startManaging(ramp);

  RCLCPP_INFO(node_->get_logger(), "\n%zu-DOF WAM", DOF);
  jp_home = wam.getJointPositions();

  if (pm.foundForceTorqueSensor()) {
    RCLCPP_INFO(node_->get_logger(), "Force/Torque sensor");
    fts = pm.getForceTorqueSensor();
    fts->tare();
    fts_pub = node_->create_publisher<geometry_msgs::msg::Wrench>("fts/fts_states", 1);
  }

  if (pm.foundHand()) {
    hand = pm.getHand();
    RCLCPP_INFO(node_->get_logger(), "Barrett Hand");
    if (hand->hasFingertipTorqueSensors()) {
      RCLCPP_INFO(node_->get_logger(), "...with Fingertip Sensors");
      fingerTs_pub = node_->create_publisher<wam_msgs::msg::FtTorques>("bhand/finger_tip_states", 1);
    }
    if (hand->hasTactSensors()) {
      RCLCPP_INFO(node_->get_logger(), "...with Tactile Sensors");
      tps_pub = node_->create_publisher<wam_msgs::msg::TactilePressureArray>("bhand/tactile_states", 1);
    }

    if (initialize_hand_on_startup) {
      pm.getSafetyModule()->setTorqueLimit(3.0);

      if (hand_clearance_move_on_startup) {
        jp_type jp_init = wam.getJointPositions();
        jp_init[3] -= 0.35;
        RCLCPP_WARN(
          node_->get_logger(),
          "Moving WAM joint 4 by -0.35 rad before BarrettHand initialization.");
        usleep(500000);
        wam.moveTo(jp_init);
        usleep(500000);
      }

      hand->initialize();
      hand->update();
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "BarrettHand automatic initialization is disabled; set initialize_hand_on_startup:=true to enable it.");
    }

    bhand_joint_state_pub = node_->create_publisher<sensor_msgs::msg::JointState>("bhand/joint_states", 1);

    hand_open_grsp_srv = node_->create_service<std_srvs::srv::Empty>(
      "bhand/open_grasp", std::bind(&WamNode<DOF>::handOpenGrasp, this, std::placeholders::_1, std::placeholders::_2));
    hand_close_grsp_srv = node_->create_service<std_srvs::srv::Empty>(
      "bhand/close_grasp", std::bind(&WamNode<DOF>::handCloseGrasp, this, std::placeholders::_1, std::placeholders::_2));
    hand_open_sprd_srv = node_->create_service<std_srvs::srv::Empty>(
      "bhand/open_spread", std::bind(&WamNode<DOF>::handOpenSpread, this, std::placeholders::_1, std::placeholders::_2));
    hand_close_sprd_srv = node_->create_service<std_srvs::srv::Empty>(
      "bhand/close_spread", std::bind(&WamNode<DOF>::handCloseSpread, this, std::placeholders::_1, std::placeholders::_2));
    hand_fngr_pos_srv = node_->create_service<wam_srvs::srv::BHandFingerPos>(
      "bhand/finger_pos", std::bind(&WamNode<DOF>::handFingerPos, this, std::placeholders::_1, std::placeholders::_2));
    hand_grsp_pos_srv = node_->create_service<wam_srvs::srv::BHandGraspPos>(
      "bhand/grasp_pos", std::bind(&WamNode<DOF>::handGraspPos, this, std::placeholders::_1, std::placeholders::_2));
    hand_sprd_pos_srv = node_->create_service<wam_srvs::srv::BHandSpreadPos>(
      "bhand/spread_pos", std::bind(&WamNode<DOF>::handSpreadPos, this, std::placeholders::_1, std::placeholders::_2));
    hand_fngr_vel_srv = node_->create_service<wam_srvs::srv::BHandFingerVel>(
      "bhand/finger_vel", std::bind(&WamNode<DOF>::handFingerVel, this, std::placeholders::_1, std::placeholders::_2));
    hand_grsp_vel_srv = node_->create_service<wam_srvs::srv::BHandGraspVel>(
      "bhand/grasp_vel", std::bind(&WamNode<DOF>::handGraspVel, this, std::placeholders::_1, std::placeholders::_2));
    hand_sprd_vel_srv = node_->create_service<wam_srvs::srv::BHandSpreadVel>(
      "bhand/spread_vel", std::bind(&WamNode<DOF>::handSpreadVel, this, std::placeholders::_1, std::placeholders::_2));

    const char * bhand_jnts[] = {"inner_f1", "inner_f2", "inner_f3", "spread", "outer_f1", "outer_f2", "outer_f3"};
    bhand_joint_state.name.assign(bhand_jnts, bhand_jnts + 7);
    bhand_joint_state.position.resize(7);
    tactileState.pressure.resize(24);
    tactileState.normalized_pressure.resize(24);
    tactileStates.tactile_pressures.resize(4);
    ftTorque_state.torque.resize(4);
  }

  if (auto_gravity_comp) {
    RCLCPP_WARN(node_->get_logger(), "Enabling WAM gravity compensation on startup.");
    wam.gravityCompensate(true);
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "WAM gravity compensation is disabled on startup; call /wam/gravity_comp or set auto_gravity_comp:=true.");
  }

  const char * wam_jnts[] = {"wam_j1", "wam_j2", "wam_j3", "wam_j4", "wam_j5", "wam_j6", "wam_j7"};
  wam_joint_state.name.assign(wam_jnts, wam_jnts + DOF);
  wam_joint_state.position.resize(DOF);
  wam_joint_state.velocity.resize(DOF);
  wam_joint_state.effort.resize(DOF);

  wam_joint_state_pub = node_->create_publisher<sensor_msgs::msg::JointState>("wam/joint_states", 1);
  wam_move_state_pub = node_->create_publisher<std_msgs::msg::Bool>("wam/move_is_done", 1);
  wam_pose_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("wam/pose", 1);

  cart_vel_sub = node_->create_subscription<wam_msgs::msg::RTCartVel>(
    "wam/cart_vel_cmd", 1, std::bind(&WamNode<DOF>::cartVelCB, this, std::placeholders::_1));
  ortn_vel_sub = node_->create_subscription<wam_msgs::msg::RTOrtnVel>(
    "wam/ortn_vel_cmd", 1, std::bind(&WamNode<DOF>::ortnVelCB, this, std::placeholders::_1));
  jnt_vel_sub = node_->create_subscription<wam_msgs::msg::RTJointVel>(
    "wam/jnt_vel_cmd", 1, std::bind(&WamNode<DOF>::jntVelCB, this, std::placeholders::_1));
  jnt_pos_sub = node_->create_subscription<wam_msgs::msg::RTJointPos>(
    "wam/jnt_pos_cmd", 1, std::bind(&WamNode<DOF>::jntPosCB, this, std::placeholders::_1));
  cart_pos_sub = node_->create_subscription<wam_msgs::msg::RTCartPos>(
    "wam/cart_pos_cmd", 1, std::bind(&WamNode<DOF>::cartPosCB, this, std::placeholders::_1));

  gravity_srv = node_->create_service<wam_srvs::srv::GravityComp>(
    "wam/gravity_comp", std::bind(&WamNode<DOF>::gravity, this, std::placeholders::_1, std::placeholders::_2));
  go_home_srv = node_->create_service<std_srvs::srv::Empty>(
    "wam/go_home", std::bind(&WamNode<DOF>::goHome, this, std::placeholders::_1, std::placeholders::_2));
  hold_jpos_srv = node_->create_service<wam_srvs::srv::Hold>(
    "wam/hold_joint_pos", std::bind(&WamNode<DOF>::holdJPos, this, std::placeholders::_1, std::placeholders::_2));
  hold_cpos_srv = node_->create_service<wam_srvs::srv::Hold>(
    "wam/hold_cart_pos", std::bind(&WamNode<DOF>::holdCPos, this, std::placeholders::_1, std::placeholders::_2));
  hold_ortn_srv = node_->create_service<wam_srvs::srv::Hold>(
    "wam/hold_ortn", std::bind(&WamNode<DOF>::holdOrtn, this, std::placeholders::_1, std::placeholders::_2));
  joint_move_srv = node_->create_service<wam_srvs::srv::JointMove>(
    "wam/joint_move", std::bind(&WamNode<DOF>::jointMove, this, std::placeholders::_1, std::placeholders::_2));
  pose_move_srv = node_->create_service<wam_srvs::srv::PoseMove>(
    "wam/pose_move", std::bind(&WamNode<DOF>::poseMove, this, std::placeholders::_1, std::placeholders::_2));
  cart_move_srv = node_->create_service<wam_srvs::srv::CartPosMove>(
    "wam/cart_move", std::bind(&WamNode<DOF>::cartMove, this, std::placeholders::_1, std::placeholders::_2));
  ortn_move_srv = node_->create_service<wam_srvs::srv::OrtnMove>(
    "wam/ortn_move", std::bind(&WamNode<DOF>::ortnMove, this, std::placeholders::_1, std::placeholders::_2));
}

template<size_t DOF>
bool WamNode<DOF>::gravity(
  const std::shared_ptr<wam_srvs::srv::GravityComp::Request> req,
  std::shared_ptr<wam_srvs::srv::GravityComp::Response> res)
{
  (void)res;
  wam.gravityCompensate(req->gravity);
  RCLCPP_INFO(node_->get_logger(), "Gravity Compensation Request: %s", req->gravity ? "true" : "false");
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::goHome(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Returning to Home Position");
  if (hand != nullptr) {
    hand->open(Hand::GRASP, true);
    hand->close(Hand::SPREAD, true);
  }
  if (!wam.isGravityCompensated()) {
    RCLCPP_WARN(node_->get_logger(), "Enabling gravity compensation before /wam/go_home.");
    wam.gravityCompensate(true);
  }

  const jp_type current_jp = wam.getJointPositions();
  const jp_type home_jp = wam.getHomePosition();
  std::ostringstream current_ss;
  std::ostringstream target_ss;
  for (size_t i = 0; i < DOF; i++) {
    if (i > 0) {
      current_ss << ", ";
      target_ss << ", ";
    }
    current_ss << current_jp[i];
    target_ss << home_jp[i];
  }
  RCLCPP_INFO(node_->get_logger(), "Current Joint Pose: [%s]", current_ss.str().c_str());
  RCLCPP_INFO(node_->get_logger(), "Home Joint Pose: [%s]", target_ss.str().c_str());
  RCLCPP_INFO(
    node_->get_logger(), "Moving home with velocity %.3f and acceleration %.3f",
    home_velocity, home_acceleration);
  wam.moveTo(home_jp, false, home_velocity, home_acceleration);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::holdJPos(
  const std::shared_ptr<wam_srvs::srv::Hold::Request> req,
  std::shared_ptr<wam_srvs::srv::Hold::Response> res)
{
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Joint Position Hold request: %s", req->hold ? "true" : "false");
  if (req->hold) {
    wam.moveTo(wam.getJointPositions());
  } else {
    wam.idle();
  }
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::holdCPos(
  const std::shared_ptr<wam_srvs::srv::Hold::Request> req,
  std::shared_ptr<wam_srvs::srv::Hold::Response> res)
{
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Cartesian Position Hold request: %s", req->hold ? "true" : "false");
  if (req->hold) {
    wam.moveTo(wam.getToolPosition());
  } else {
    wam.idle();
  }
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::holdOrtn(
  const std::shared_ptr<wam_srvs::srv::Hold::Request> req,
  std::shared_ptr<wam_srvs::srv::Hold::Response> res)
{
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Orientation Hold request: %s", req->hold ? "true" : "false");
  if (req->hold) {
    orientationSetPoint.setValue(wam.getToolOrientation());
    wam.trackReferenceSignal(orientationSetPoint.output);
  } else {
    wam.idle();
  }
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::jointMove(
  const std::shared_ptr<wam_srvs::srv::JointMove::Request> req,
  std::shared_ptr<wam_srvs::srv::JointMove::Response> res)
{
  (void)res;
  if (req->joints.size() != DOF) {
    RCLCPP_INFO(
      node_->get_logger(), "Request Failed: %zu-DOF request received, must be %zu-DOF",
      req->joints.size(), DOF);
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Moving Robot to Commanded Joint Pose");
  jp_type current_jp = wam.getJointPositions();
  std::ostringstream current_ss;
  std::ostringstream target_ss;
  for (size_t i = 0; i < DOF; i++) {
    jp_cmd[i] = req->joints[i];
    if (i > 0) {
      current_ss << ", ";
      target_ss << ", ";
    }
    current_ss << current_jp[i];
    target_ss << jp_cmd[i];
  }
  RCLCPP_INFO(node_->get_logger(), "Current Joint Pose: [%s]", current_ss.str().c_str());
  RCLCPP_INFO(node_->get_logger(), "Target Joint Pose: [%s]", target_ss.str().c_str());
  if (!wam.isGravityCompensated()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "WAM gravity compensation is disabled; gravity-loaded joints may not track joint_move commands.");
  }
  wam.moveTo(jp_cmd, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::poseMove(
  const std::shared_ptr<wam_srvs::srv::PoseMove::Request> req,
  std::shared_ptr<wam_srvs::srv::PoseMove::Response> res)
{
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Moving Robot to Commanded Pose");
  cp_cmd[0] = req->pose.position.x;
  cp_cmd[1] = req->pose.position.y;
  cp_cmd[2] = req->pose.position.z;
  ortn_cmd.x() = req->pose.orientation.x;
  ortn_cmd.y() = req->pose.orientation.y;
  ortn_cmd.z() = req->pose.orientation.z;
  ortn_cmd.w() = req->pose.orientation.w;
  pose_cmd = boost::make_tuple(cp_cmd, ortn_cmd);
  wam.moveTo(pose_cmd, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::cartMove(
  const std::shared_ptr<wam_srvs::srv::CartPosMove::Request> req,
  std::shared_ptr<wam_srvs::srv::CartPosMove::Response> res)
{
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Moving Robot to Commanded Cartesian Position");
  for (int i = 0; i < 3; i++) {
    cp_cmd[i] = req->position[i];
  }
  wam.moveTo(cp_cmd, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::ortnMove(
  const std::shared_ptr<wam_srvs::srv::OrtnMove::Request> req,
  std::shared_ptr<wam_srvs::srv::OrtnMove::Response> res)
{
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Moving Robot to Commanded End Effector Orientation");
  ortn_cmd.x() = req->orientation[0];
  ortn_cmd.y() = req->orientation[1];
  ortn_cmd.z() = req->orientation[2];
  ortn_cmd.w() = req->orientation[3];
  wam.moveTo(ortn_cmd, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::handOpenGrasp(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Opening the BarrettHand Grasp");
  hand->open(Hand::GRASP, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::handCloseGrasp(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Closing the BarrettHand Grasp");
  hand->close(Hand::GRASP, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::handOpenSpread(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Opening the BarrettHand Spread");
  hand->open(Hand::SPREAD, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::handCloseSpread(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Closing the BarrettHand Spread");
  hand->close(Hand::SPREAD, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::handFingerPos(
  const std::shared_ptr<wam_srvs::srv::BHandFingerPos::Request> req,
  std::shared_ptr<wam_srvs::srv::BHandFingerPos::Response> res)
{
  (void)res;
  RCLCPP_INFO(
    node_->get_logger(), "Moving BarrettHand to Finger Positions: %.3f, %.3f, %.3f radians",
    req->radians[0], req->radians[1], req->radians[2]);
  hand->trapezoidalMove(Hand::jp_type(req->radians[0], req->radians[1], req->radians[2], 0.0), Hand::GRASP, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::handGraspPos(
  const std::shared_ptr<wam_srvs::srv::BHandGraspPos::Request> req,
  std::shared_ptr<wam_srvs::srv::BHandGraspPos::Response> res)
{
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Moving BarrettHand Grasp: %.3f radians", req->radians);
  hand->trapezoidalMove(Hand::jp_type(req->radians), Hand::GRASP, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::handSpreadPos(
  const std::shared_ptr<wam_srvs::srv::BHandSpreadPos::Request> req,
  std::shared_ptr<wam_srvs::srv::BHandSpreadPos::Response> res)
{
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Moving BarrettHand Spread: %.3f radians", req->radians);
  hand->trapezoidalMove(Hand::jp_type(req->radians), Hand::SPREAD, false);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::handFingerVel(
  const std::shared_ptr<wam_srvs::srv::BHandFingerVel::Request> req,
  std::shared_ptr<wam_srvs::srv::BHandFingerVel::Response> res)
{
  (void)res;
  RCLCPP_INFO(
    node_->get_logger(), "Moving BarrettHand Finger Velocities: %.3f, %.3f, %.3f m/s",
    req->velocity[0], req->velocity[1], req->velocity[2]);
  hand->velocityMove(Hand::jv_type(req->velocity[0], req->velocity[1], req->velocity[2], 0.0), Hand::GRASP);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::handGraspVel(
  const std::shared_ptr<wam_srvs::srv::BHandGraspVel::Request> req,
  std::shared_ptr<wam_srvs::srv::BHandGraspVel::Response> res)
{
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Moving BarrettHand Grasp: %.3f m/s", req->velocity);
  hand->velocityMove(Hand::jv_type(req->velocity), Hand::GRASP);
  return true;
}

template<size_t DOF>
bool WamNode<DOF>::handSpreadVel(
  const std::shared_ptr<wam_srvs::srv::BHandSpreadVel::Request> req,
  std::shared_ptr<wam_srvs::srv::BHandSpreadVel::Response> res)
{
  (void)res;
  RCLCPP_INFO(node_->get_logger(), "Moving BarrettHand Spread: %.3f m/s", req->velocity);
  usleep(5000);
  hand->velocityMove(Hand::jv_type(req->velocity), Hand::SPREAD);
  return true;
}

template<size_t DOF>
void WamNode<DOF>::cartVelCB(const wam_msgs::msg::RTCartVel::SharedPtr msg)
{
  if (cart_vel_status) {
    for (size_t i = 0; i < 3; i++) {
      rt_cv_cmd[i] = msg->direction[i];
    }
    new_rt_cmd = true;
    if (msg->magnitude != 0) {
      cart_vel_mag = msg->magnitude;
    }
  }
  last_cart_vel_msg_time = node_->now();
}

template<size_t DOF>
void WamNode<DOF>::ortnVelCB(const wam_msgs::msg::RTOrtnVel::SharedPtr msg)
{
  if (ortn_vel_status) {
    for (size_t i = 0; i < 3; i++) {
      rt_ortn_cmd[i] = msg->angular[i];
    }
    new_rt_cmd = true;
    if (msg->magnitude != 0) {
      ortn_vel_mag = msg->magnitude;
    }
  }
  last_ortn_vel_msg_time = node_->now();
}

template<size_t DOF>
void WamNode<DOF>::jntVelCB(const wam_msgs::msg::RTJointVel::SharedPtr msg)
{
  if (msg->velocities.size() != DOF) {
    RCLCPP_INFO(node_->get_logger(), "Commanded Joint Velocities != DOF of WAM");
    return;
  }
  if (jnt_vel_status) {
    for (size_t i = 0; i < DOF; i++) {
      rt_jv_cmd[i] = msg->velocities[i];
    }
    new_rt_cmd = true;
  }
  last_jnt_vel_msg_time = node_->now();
}

template<size_t DOF>
void WamNode<DOF>::jntPosCB(const wam_msgs::msg::RTJointPos::SharedPtr msg)
{
  if (msg->joints.size() != DOF) {
    RCLCPP_INFO(node_->get_logger(), "Commanded Joint Positions != DOF of WAM");
    return;
  }
  if (jnt_pos_status) {
    for (size_t i = 0; i < DOF; i++) {
      rt_jp_cmd[i] = msg->joints[i];
      rt_jp_rl[i] = msg->rate_limits[i];
    }
    new_rt_cmd = true;
  }
  last_jnt_pos_msg_time = node_->now();
}

template<size_t DOF>
void WamNode<DOF>::cartPosCB(const wam_msgs::msg::RTCartPos::SharedPtr msg)
{
  if (cart_pos_status) {
    for (size_t i = 0; i < 3; i++) {
      rt_cp_cmd[i] = msg->position[i];
      rt_cp_rl[i] = msg->rate_limits[i];
    }
    new_rt_cmd = true;
  }
  last_cart_pos_msg_time = node_->now();
}

template<size_t DOF>
void WamNode<DOF>::publishWam(ProductManager & pm)
{
  (void)pm;
  jp_type jp = wam.getJointPositions();
  jt_type jt = wam.getJointTorques();
  jv_type jv = wam.getJointVelocities();
  cp_type cp_pub = wam.getToolPosition();
  Eigen::Quaterniond to_pub = wam.getToolOrientation();
  move_is_done.data = wam.moveIsDone();

  for (size_t i = 0; i < DOF; i++) {
    wam_joint_state.position[i] = jp[i];
    wam_joint_state.velocity[i] = jv[i];
    wam_joint_state.effort[i] = jt[i];
  }
  wam_joint_state.header.stamp = node_->now();
  wam_joint_state_pub->publish(wam_joint_state);
  wam_move_state_pub->publish(move_is_done);

  wam_pose.header.stamp = node_->now();
  wam_pose.pose.position.x = cp_pub[0];
  wam_pose.pose.position.y = cp_pub[1];
  wam_pose.pose.position.z = cp_pub[2];
  wam_pose.pose.orientation.w = to_pub.w();
  wam_pose.pose.orientation.x = to_pub.x();
  wam_pose.pose.orientation.y = to_pub.y();
  wam_pose.pose.orientation.z = to_pub.z();
  wam_pose_pub->publish(wam_pose);
}

template<size_t DOF>
void WamNode<DOF>::publishHand()
{
  hand->update(Hand::S_POSITION | Hand::S_FINGERTIP_TORQUE | Hand::S_TACT_TOP10);
  std::vector<TactilePuck *> tps = hand->getTactilePucks();
  std::vector<int> fingerTip = hand->getFingertipTorque();
  Hand::jp_type hi = hand->getInnerLinkPosition();
  Hand::jp_type ho = hand->getOuterLinkPosition();

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
          case 0: c = c + 1; break;
          case 1: c = c + 2; break;
          case 2: c = c + 3; break;
          default: c = c + 4; break;
        }
        switch (chunk - 4) {
          case 3: c = c + 4; break;
          case 2: c = c + 3; break;
          case 1: c = c + 2; break;
          case 0: c = c + 1; break;
          default: c = c + 0; break;
        }
      }
      tactileState.normalized_pressure[j] = c - 5;
    }
    tactileStates.tactile_pressures[i] = tactileState;
  }
  for (unsigned i = 0; i < fingerTip.size(); i++) {
    ftTorque_state.torque[i] = fingerTip[i];
  }
  for (size_t i = 0; i < 4; i++) {
    bhand_joint_state.position[i] = hi[i];
  }
  for (size_t j = 0; j < 3; j++) {
    bhand_joint_state.position[j + 4] = ho[j];
  }
  bhand_joint_state.header.stamp = node_->now();
  bhand_joint_state_pub->publish(bhand_joint_state);
  if (hand->hasTactSensors()) {
    tps_pub->publish(tactileStates);
  }
  if (hand->hasFingertipTorqueSensors()) {
    fingerTs_pub->publish(ftTorque_state);
  }
}

template<size_t DOF>
void WamNode<DOF>::publishFTS()
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

template<size_t DOF>
void WamNode<DOF>::updateRT(ProductManager & pm)
{
  (void)pm;
  const auto now = node_->now();

  if (last_cart_vel_msg_time + rt_msg_timeout > now) {
    if (!cart_vel_status) {
      cart_dir.setValue(cp_type(0.0, 0.0, 0.0));
      current_cart_pos.setValue(wam.getToolPosition());
      current_ortn.setValue(wam.getToolOrientation());
      systems::forceConnect(ramp.output, mult_linear.input1);
      systems::forceConnect(cart_dir.output, mult_linear.input2);
      systems::forceConnect(mult_linear.output, cart_pos_sum.getInput(0));
      systems::forceConnect(current_cart_pos.output, cart_pos_sum.getInput(1));
      systems::forceConnect(cart_pos_sum.output, rt_pose_cmd.getInput<0>());
      systems::forceConnect(current_ortn.output, rt_pose_cmd.getInput<1>());
      ramp.setSlope(cart_vel_mag);
      ramp.stop();
      ramp.setOutput(0.0);
      ramp.start();
      wam.trackReferenceSignal(rt_pose_cmd.output);
    } else if (new_rt_cmd) {
      ramp.reset();
      ramp.setSlope(cart_vel_mag);
      cart_dir.setValue(rt_cv_cmd);
      current_cart_pos.setValue(wam.tpoTpController.referenceInput.getValue());
    }
    cart_vel_status = true;
    new_rt_cmd = false;
  } else if (last_ortn_vel_msg_time + rt_msg_timeout > now) {
    if (!ortn_vel_status) {
      rpy_cmd.setValue(math::Vector<3>::type(0.0, 0.0, 0.0));
      current_cart_pos.setValue(wam.getToolPosition());
      current_rpy_ortn.setValue(toRPY(wam.getToolOrientation()));
      systems::forceConnect(ramp.output, mult_angular.input1);
      systems::forceConnect(rpy_cmd.output, mult_angular.input2);
      systems::forceConnect(mult_angular.output, ortn_cmd_sum.getInput(0));
      systems::forceConnect(current_rpy_ortn.output, ortn_cmd_sum.getInput(1));
      systems::forceConnect(ortn_cmd_sum.output, to_quat.input);
      systems::forceConnect(current_cart_pos.output, rt_pose_cmd.getInput<0>());
      systems::forceConnect(to_quat.output, rt_pose_cmd.getInput<1>());
      ramp.setSlope(ortn_vel_mag);
      ramp.stop();
      ramp.setOutput(0.0);
      ramp.start();
      wam.trackReferenceSignal(rt_pose_cmd.output);
    } else if (new_rt_cmd) {
      ramp.reset();
      ramp.setSlope(ortn_vel_mag);
      rpy_cmd.setValue(rt_ortn_cmd);
      current_rpy_ortn.setValue(toRPY(wam.tpoToController.referenceInput.getValue()));
    }
    ortn_vel_status = true;
    new_rt_cmd = false;
  } else if (last_jnt_vel_msg_time + rt_msg_timeout > now) {
    if (!jnt_vel_status) {
      jv_type jv_start;
      for (size_t i = 0; i < DOF; i++) {
        jv_start[i] = 0.0;
      }
      jv_track.setValue(jv_start);
      wam.trackReferenceSignal(jv_track.output);
    } else if (new_rt_cmd) {
      jv_track.setValue(rt_jv_cmd);
    }
    jnt_vel_status = true;
    new_rt_cmd = false;
  } else if (last_jnt_pos_msg_time + rt_msg_timeout > now) {
    if (!jnt_pos_status) {
      jp_type jp_start = wam.getJointPositions();
      jp_track.setValue(jp_start);
      jp_rl.setLimit(rt_jp_rl);
      systems::forceConnect(jp_track.output, jp_rl.input);
      wam.trackReferenceSignal(jp_rl.output);
    } else if (new_rt_cmd) {
      jp_track.setValue(rt_jp_cmd);
      jp_rl.setLimit(rt_jp_rl);
    }
    jnt_pos_status = true;
    new_rt_cmd = false;
  } else if (last_cart_pos_msg_time + rt_msg_timeout > now) {
    if (!cart_pos_status) {
      cp_track.setValue(wam.getToolPosition());
      current_ortn.setValue(wam.getToolOrientation());
      cp_rl.setLimit(rt_cp_rl);
      systems::forceConnect(cp_track.output, cp_rl.input);
      systems::forceConnect(cp_rl.output, rt_pose_cmd.getInput<0>());
      systems::forceConnect(current_ortn.output, rt_pose_cmd.getInput<1>());
      wam.trackReferenceSignal(rt_pose_cmd.output);
    } else if (new_rt_cmd) {
      cp_track.setValue(rt_cp_cmd);
      cp_rl.setLimit(rt_cp_rl);
    }
    cart_pos_status = true;
    new_rt_cmd = false;
  } else if (cart_vel_status || ortn_vel_status || jnt_vel_status || jnt_pos_status || cart_pos_status) {
    wam.moveTo(wam.getJointPositions());
    cart_vel_status = ortn_vel_status = jnt_vel_status = jnt_pos_status = cart_pos_status = ortn_pos_status = false;
  }
}

template<size_t DOF>
int wam_main(int argc, char ** argv, ProductManager & pm, systems::Wam<DOF> & wam)
{
  uint32_t bh_ctr = 0;
  uint32_t ft_ctr = 0;
  uint32_t safety_ctr = 0;
  uint32_t ft_cts_per_loop = WAM_PUBLISH_FREQ / FT_PUBLISH_FREQ;
  uint32_t bh_cts_per_loop = WAM_PUBLISH_FREQ / BH_PUBLISH_FREQ;
  uint32_t safety_cts_per_loop = WAM_PUBLISH_FREQ / SAFETY_MODE_FREQ;

  rclcpp::init(argc, argv);
  WamNode<DOF> wam_node(wam);
  wam_node.init(pm);
  rclcpp::Rate pub_rate(WAM_PUBLISH_FREQ);

  while (rclcpp::ok()) {
    if (++ft_ctr >= ft_cts_per_loop) {
      ft_ctr = 0;
      if (pm.getForceTorqueSensor()) {
        wam_node.publishFTS();
      }
    }

    if (++bh_ctr >= bh_cts_per_loop) {
      bh_ctr = 0;
      if (pm.getHand()) {
        wam_node.publishHand();
      }
    }

    rclcpp::spin_some(wam_node.node());
    wam_node.publishWam(pm);
    wam_node.updateRT(pm);

    if (++safety_ctr >= safety_cts_per_loop) {
      safety_ctr = 0;
      if (pm.getSafetyModule()->getMode() != SafetyModule::ACTIVE) {
        break;
      }
    }

    pub_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
