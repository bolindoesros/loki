/**
 * @file controller.cpp
 * @brief Cascaded PID controller implementation for the Loki AUV.
 */

#include "loki_control/controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace loki
{

// ── Constructor ───────────────────────────────────────────────────────────────

ControllerNode::ControllerNode()
: Node("loki_controller")
{
  // ── Parameters ─────────────────────────────────────────────────────────────
  max_pitch_cmd_    = declare_parameter("max_pitch_cmd",  15.0);
  double alpha      = declare_parameter("alpha",           0.7);

  auto load_pid = [&](const std::string & ns) -> PIDParams {
    PIDParams p;
    p.kp         = declare_parameter(ns + ".kp",          0.0);
    p.ki         = declare_parameter(ns + ".ki",          0.0);
    p.kd         = declare_parameter(ns + ".kd",          0.0);
    p.alpha      = alpha;
    p.antiwindup = declare_parameter(ns + ".antiwindup",  0.0);
    p.output_max = declare_parameter(ns + ".output_max",  400.0);
    p.output_min = declare_parameter(ns + ".output_min", -400.0);
    return p;
  };

  speed_pid_.set_params(load_pid("speed"));
  yaw_pid_.set_params(load_pid("yaw"));
  depth_pid_.set_params(load_pid("depth"));
  pitch_pid_.set_params(load_pid("pitch"));

  // ── QoS ────────────────────────────────────────────────────────────────────
  auto qos = rclcpp::QoS(10).reliable();

  // ── Subscriptions ───────────────────────────────────────────────────────────
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", qos,
    std::bind(&ControllerNode::on_odometry, this, std::placeholders::_1));

  target_depth_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/target/depth", qos,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      target_depth_ = msg->data; });

  target_heading_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/target/heading", qos,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      target_heading_ = msg->data; });

  target_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/target/speed", qos,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      target_speed_ = msg->data; });

  target_moving_mass_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/target/moving_mass", qos,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      target_moving_mass_ = std::clamp(msg->data, 0.0, 1.0); });

  // ── Command publishers ──────────────────────────────────────────────────────
  thruster_pub_    = create_publisher<std_msgs::msg::Int32>  ("/cmd/thruster",    qos);
  elevator_pub_    = create_publisher<std_msgs::msg::Int32>  ("/cmd/elevator",    qos);
  rudder_pub_      = create_publisher<std_msgs::msg::Int32>  ("/cmd/rudder",      qos);
  moving_mass_pub_ = create_publisher<std_msgs::msg::Float64>("/cmd/moving_mass", qos);
  arm_state_pub_   = create_publisher<std_msgs::msg::Bool>   ("/system/arm_state",qos);

  // ── Monitor publishers ──────────────────────────────────────────────────────
  mon_target_depth_pub_   = create_publisher<std_msgs::msg::Float64>("/monitor/target/depth",        qos);
  mon_target_heading_pub_ = create_publisher<std_msgs::msg::Float64>("/monitor/target/heading",      qos);
  mon_target_speed_pub_   = create_publisher<std_msgs::msg::Float64>("/monitor/target/speed",        qos);
  mon_target_mass_pub_    = create_publisher<std_msgs::msg::Float64>("/monitor/target/moving_mass",  qos);

  // ── Arm service ─────────────────────────────────────────────────────────────
  arm_srv_ = create_service<std_srvs::srv::SetBool>(
    "/system/arm",
    std::bind(&ControllerNode::on_arm, this,
      std::placeholders::_1, std::placeholders::_2));

  // ── 100 Hz control loop ─────────────────────────────────────────────────────
  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&ControllerNode::control_loop, this));

  last_time_ = now();
  RCLCPP_INFO(get_logger(), "Loki controller ready — call /system/arm to start");
}

// ── Arm callback ──────────────────────────────────────────────────────────────

void ControllerNode::on_arm(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  const std_srvs::srv::SetBool::Response::SharedPtr res)
{
  is_armed_ = req->data;

  if (is_armed_) {
    // Capture current state as setpoints on arm
    target_heading_     = current_heading_;
    target_depth_       = current_depth_;
    target_speed_       = 0.0;
    target_moving_mass_ = 0.5;

    // Reset all integrators — prevents pre-arm windup
    speed_pid_.reset();
    yaw_pid_.reset();
    depth_pid_.reset();
    pitch_pid_.reset();

    res->message = "Armed";
    RCLCPP_INFO(get_logger(), "ARMED");
  } else {
    res->message = "Disarmed";
    RCLCPP_INFO(get_logger(), "DISARMED");
  }

  res->success = true;

  auto msg = std_msgs::msg::Bool();
  msg.data  = is_armed_;
  arm_state_pub_->publish(msg);
}

// ── Odometry callback ─────────────────────────────────────────────────────────

void ControllerNode::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Depth — positive downward
  current_depth_ = -msg->pose.pose.position.z;

  // Forward speed
  current_speed_ = msg->twist.twist.linear.x;

  // Attitude from quaternion
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_pitch_      = pitch * 180.0 / M_PI;
  current_heading_    = yaw   * 180.0 / M_PI;
  current_pitch_rate_ = msg->twist.twist.angular.y * 180.0 / M_PI;

  if (current_heading_ < 0.0) current_heading_ += 360.0;
}

// ── Control loop ──────────────────────────────────────────────────────────────

void ControllerNode::control_loop()
{
  // ── Timing ──────────────────────────────────────────────────────────────────
  auto   t  = now();
  double dt = std::clamp((t - last_time_).seconds(), 1e-4, 0.05);
  last_time_ = t;

  // ── Always publish setpoint echoes for Foxglove ──────────────────────────────
  auto pub_f = [](rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & p, double v) {
    std_msgs::msg::Float64 msg; msg.data = v; p->publish(msg);
  };
  pub_f(mon_target_depth_pub_,   target_depth_);
  pub_f(mon_target_heading_pub_, target_heading_);
  pub_f(mon_target_speed_pub_,   target_speed_);
  pub_f(mon_target_mass_pub_,    target_moving_mass_);

  // ── Neutral outputs when disarmed ────────────────────────────────────────────
  if (!is_armed_) {
    auto i = std_msgs::msg::Int32();   i.data = 1500;
    auto f = std_msgs::msg::Float64(); f.data = 0.5;
    thruster_pub_->publish(i);
    elevator_pub_->publish(i);
    rudder_pub_->publish(i);
    moving_mass_pub_->publish(f);
    return;
  }

  // ── Speed ────────────────────────────────────────────────────────────────────
  double speed_effort = speed_pid_.compute(dt, target_speed_ - current_speed_);

  // ── Yaw ──────────────────────────────────────────────────────────────────────
  double yaw_effort = yaw_pid_.compute(
    dt, wrap_angle(target_heading_ - current_heading_));

  // ── Depth outer loop → desired pitch ─────────────────────────────────────────
  double desired_pitch = depth_pid_.compute(dt, target_depth_ - current_depth_);
  desired_pitch        = std::clamp(desired_pitch, -max_pitch_cmd_, 0.0);

  // ── Pitch inner loop → elevator ───────────────────────────────────────────────
  // Uses pitch rate from odometry as explicit derivative — cleaner than
  // differencing pitch angle (PX4 uuv_att_control pattern)
  double pitch_effort = pitch_pid_.compute(
    dt,
    desired_pitch - current_pitch_,
    -current_pitch_rate_);

  // ── Moving mass — operator controlled passthrough ─────────────────────────────
  auto mm_msg = std_msgs::msg::Float64();
  mm_msg.data = target_moving_mass_;
  moving_mass_pub_->publish(mm_msg);

  // ── Publish PWM ───────────────────────────────────────────────────────────────
  auto t_msg = std_msgs::msg::Int32(); t_msg.data = effort_to_pwm(speed_effort);
  auto e_msg = std_msgs::msg::Int32(); e_msg.data = effort_to_pwm(pitch_effort);
  auto r_msg = std_msgs::msg::Int32(); r_msg.data = effort_to_pwm(yaw_effort, true);

  thruster_pub_->publish(t_msg);
  elevator_pub_->publish(e_msg);
  rudder_pub_->publish(r_msg);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

int ControllerNode::effort_to_pwm(double effort, bool invert)
{
  int pwm = 1500 + (invert ? -1 : 1) * static_cast<int>(effort);
  return std::clamp(pwm, 1100, 1900);
}

double ControllerNode::wrap_angle(double deg)
{
  while (deg >  180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

}  // namespace loki
