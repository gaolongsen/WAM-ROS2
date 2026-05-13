#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/empty.hpp"
#include "wam_msgs/msg/rt_cart_vel.hpp"
#include "wam_msgs/msg/rt_ortn_vel.hpp"
#include "wam_srvs/srv/b_hand_grasp_vel.hpp"
#include "wam_srvs/srv/b_hand_spread_vel.hpp"
#include "wam_srvs/srv/hold.hpp"

static const int CNTRL_FREQ = 50;

class WamTeleop : public rclcpp::Node
{
public:
  WamTeleop()
  : Node("wam_teleop"),
    grsp_publish(false),
    sprd_publish(false),
    cart_publish(false),
    ortn_publish(false),
    home_publish(false),
    hold_publish(false),
    home_st(false),
    hold_st(false),
    ortn_mode(false),
    bh_cmd_st(0),
    hold_enabled(false)
  {
    init();
  }

  void update();

private:
  bool grsp_publish, sprd_publish, cart_publish, ortn_publish, home_publish, hold_publish;
  bool home_st, hold_st, ortn_mode;
  int bh_cmd_st;
  int deadman_btn, guardian_deadman_btn, gpr_open_btn, gpr_close_btn;
  int sprd_open_btn, sprd_close_btn, ortn_btn, home_btn, hold_btn;
  int axis_x, axis_y, axis_z, axis_r, axis_p, axis_yaw;
  double max_grsp_vel, max_sprd_vel, cart_mag, ortn_mag;
  double req_xdir, req_ydir, req_zdir, req_rdir, req_pdir, req_yawdir;
  double bh_grsp_vel, bh_sprd_vel;
  bool hold_enabled;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Client<wam_srvs::srv::BHandGraspVel>::SharedPtr grasp_vel_srv;
  rclcpp::Client<wam_srvs::srv::BHandSpreadVel>::SharedPtr spread_vel_srv;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr go_home_srv;
  rclcpp::Client<wam_srvs::srv::Hold>::SharedPtr hold_srv;
  rclcpp::Publisher<wam_msgs::msg::RTCartVel>::SharedPtr cart_vel_pub;
  rclcpp::Publisher<wam_msgs::msg::RTOrtnVel>::SharedPtr ortn_vel_pub;
  wam_msgs::msg::RTCartVel cart_vel;
  wam_msgs::msg::RTOrtnVel ortn_vel;

  void init();
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
};

void WamTeleop::init()
{
  deadman_btn = declare_parameter("deadman_button", 10);
  guardian_deadman_btn = declare_parameter("guardian_deadman_button", 11);
  gpr_open_btn = declare_parameter("gripper_open_button", 12);
  gpr_close_btn = declare_parameter("gripper_close_button", 14);
  sprd_open_btn = declare_parameter("spread_open_button", 13);
  sprd_close_btn = declare_parameter("spread_close_button", 15);
  ortn_btn = declare_parameter("orientation_control_button", 8);
  home_btn = declare_parameter("go_home_button", 0);
  hold_btn = declare_parameter("hold_joints_button", 3);
  max_grsp_vel = declare_parameter("grasp_max_velocity", 1.0);
  max_sprd_vel = declare_parameter("spread_max_velocity", 1.0);
  cart_mag = declare_parameter("cartesian_magnitude", 0.05);
  ortn_mag = declare_parameter("orientation_magnitude", 1.0);
  axis_x = declare_parameter("cartesian_x_axis", 3);
  axis_y = declare_parameter("cartesian_y_axis", 2);
  axis_z = declare_parameter("cartesian_z_axis", 1);
  axis_r = declare_parameter("orientation_roll_axis", 3);
  axis_p = declare_parameter("orientation_pitch_axis", 2);
  axis_yaw = declare_parameter("orientation_yaw_axis", 1);

  joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&WamTeleop::joyCallback, this, std::placeholders::_1));
  grasp_vel_srv = create_client<wam_srvs::srv::BHandGraspVel>("bhand/grasp_vel");
  spread_vel_srv = create_client<wam_srvs::srv::BHandSpreadVel>("bhand/spread_vel");
  go_home_srv = create_client<std_srvs::srv::Empty>("wam/go_home");
  hold_srv = create_client<wam_srvs::srv::Hold>("wam/hold_joint_pos");
  cart_vel_pub = create_publisher<wam_msgs::msg::RTCartVel>("wam/cart_vel_cmd", 1);
  ortn_vel_pub = create_publisher<wam_msgs::msg::RTOrtnVel>("wam/ortn_vel_cmd", 1);
}

void WamTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  grsp_publish = sprd_publish = cart_publish = ortn_publish = home_publish = hold_publish = ortn_mode = false;

  const auto button_available = [&](int idx) {
    return idx >= 0 && static_cast<size_t>(idx) < joy_msg->buttons.size();
  };
  const auto axis_available = [&](int idx) {
    return idx >= 0 && static_cast<size_t>(idx) < joy_msg->axes.size();
  };

  if (!button_available(deadman_btn) || !joy_msg->buttons[deadman_btn]) {
    return;
  }
  if (button_available(guardian_deadman_btn) && joy_msg->buttons[guardian_deadman_btn]) {
    return;
  }

  ortn_mode = button_available(ortn_btn) && joy_msg->buttons[ortn_btn];

  if (button_available(gpr_open_btn) && joy_msg->buttons[gpr_open_btn]) {
    bh_grsp_vel = -max_grsp_vel;
    grsp_publish = true;
  } else if (button_available(gpr_close_btn) && joy_msg->buttons[gpr_close_btn]) {
    bh_grsp_vel = max_grsp_vel;
    grsp_publish = true;
  }

  if (button_available(sprd_open_btn) && joy_msg->buttons[sprd_open_btn]) {
    bh_sprd_vel = -max_sprd_vel;
    sprd_publish = true;
  } else if (button_available(sprd_close_btn) && joy_msg->buttons[sprd_close_btn]) {
    bh_sprd_vel = max_sprd_vel;
    sprd_publish = true;
  }

  const bool home_pressed = button_available(home_btn) && joy_msg->buttons[home_btn];
  home_publish = home_pressed && !home_st;
  home_st = home_pressed;

  const bool hold_pressed = button_available(hold_btn) && joy_msg->buttons[hold_btn];
  hold_publish = hold_pressed && !hold_st;
  hold_st = hold_pressed;

  req_xdir = axis_available(axis_x) ? joy_msg->axes[axis_x] : 0.0;
  req_ydir = axis_available(axis_y) ? joy_msg->axes[axis_y] : 0.0;
  req_zdir = axis_available(axis_z) ? joy_msg->axes[axis_z] : 0.0;
  req_rdir = axis_available(axis_r) ? -joy_msg->axes[axis_r] : 0.0;
  req_pdir = axis_available(axis_p) ? -joy_msg->axes[axis_p] : 0.0;
  req_yawdir = axis_available(axis_yaw) ? joy_msg->axes[axis_yaw] : 0.0;

  if (!ortn_mode) {
    cart_publish = std::abs(req_xdir) > 0.25 || std::abs(req_ydir) > 0.25 || std::abs(req_zdir) > 0.25;
  } else {
    ortn_publish = std::abs(req_rdir) > 0.25 || std::abs(req_pdir) > 0.25 || std::abs(req_yawdir) > 0.25;
  }
}

void WamTeleop::update()
{
  if (grsp_publish && bh_cmd_st == 0 && !sprd_publish && !cart_publish && !ortn_publish) {
    auto request = std::make_shared<wam_srvs::srv::BHandGraspVel::Request>();
    request->velocity = bh_grsp_vel;
    grasp_vel_srv->async_send_request(request);
    bh_cmd_st = 1;
  } else if (sprd_publish && bh_cmd_st == 0 && !grsp_publish && !cart_publish && !ortn_publish) {
    auto request = std::make_shared<wam_srvs::srv::BHandSpreadVel::Request>();
    request->velocity = bh_sprd_vel;
    spread_vel_srv->async_send_request(request);
    bh_cmd_st = 2;
  } else if (bh_cmd_st != 0 && !grsp_publish && !sprd_publish && !cart_publish && !ortn_publish) {
    if (bh_cmd_st == 1) {
      auto request = std::make_shared<wam_srvs::srv::BHandGraspVel::Request>();
      request->velocity = 0.0;
      grasp_vel_srv->async_send_request(request);
    }
    if (bh_cmd_st == 2) {
      auto request = std::make_shared<wam_srvs::srv::BHandSpreadVel::Request>();
      request->velocity = 0.0;
      spread_vel_srv->async_send_request(request);
    }
    bh_cmd_st = 0;
  }

  if (hold_publish && !cart_publish && !ortn_publish && !grsp_publish && !sprd_publish && !home_publish) {
    hold_enabled = !hold_enabled;
    auto request = std::make_shared<wam_srvs::srv::Hold::Request>();
    request->hold = hold_enabled;
    hold_srv->async_send_request(request);
  }

  if (home_publish && !hold_publish && !cart_publish && !grsp_publish && !sprd_publish && !ortn_publish) {
    go_home_srv->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  }

  if (cart_publish && !ortn_publish && !grsp_publish && !sprd_publish && !home_publish && !hold_publish) {
    cart_vel.direction[0] = req_xdir;
    cart_vel.direction[1] = req_ydir;
    cart_vel.direction[2] = req_zdir;
    cart_vel.magnitude = cart_mag;
    cart_vel_pub->publish(cart_vel);
  }

  if (ortn_publish && !cart_publish && !grsp_publish && !sprd_publish && !home_publish && !hold_publish) {
    ortn_vel.angular[0] = req_rdir;
    ortn_vel.angular[1] = req_pdir;
    ortn_vel.angular[2] = req_yawdir;
    ortn_vel.magnitude = ortn_mag;
    ortn_vel_pub->publish(ortn_vel);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto wam_teleop = std::make_shared<WamTeleop>();
  rclcpp::Rate pub_rate(CNTRL_FREQ);

  while (rclcpp::ok()) {
    rclcpp::spin_some(wam_teleop);
    wam_teleop->update();
    pub_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
