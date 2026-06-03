/*
 * @file controller.cpp
 * @brief Cascaded PID controller for loki auv.
 */

#include "loki_control/controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace loki
{

ControllerNode::ControllerNode()
: Node("loki_controller")
{
  // fallback params
  max_pitch_cmd_ = declare_parameter("max_pitch_cmd", 20.0);
  double alpha   = declare_parameter("alpha",          0.7);

  speed_pid_.update_params(load_pid("speed", alpha));
  yaw_pid_.update_params(load_pid("yaw",     alpha));
  depth_pid_.update_params(load_pid("depth", alpha));
  pitch_pid_.update_params(load_pid("pitch", alpha));

  auto qos = rclcpp::QoS(10).reliable();

  // Subscriptions
  odom_sub_               = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", qos, std::bind(&ControllerNode::on_odometry, this, std::placeholders::_1));
  target_depth_sub_       = create_subscription<std_msgs::msg::Float64>("/target/depth", qos, std::bind(&ControllerNode::on_target_depth, this, std::placeholders::_1));
  target_heading_sub_     = create_subscription<std_msgs::msg::Float64>("/target/heading", qos, std::bind(&ControllerNode::on_target_heading, this, std::placeholders::_1));
  target_speed_sub_       = create_subscription<std_msgs::msg::Float64>("/target/speed", qos, std::bind(&ControllerNode::on_target_speed, this, std::placeholders::_1));
  target_moving_mass_sub_ = create_subscription<std_msgs::msg::Float64>("/target/moving_mass", qos, std::bind(&ControllerNode::on_target_moving_mass, this, std::placeholders::_1));

  // Publishers
  thruster_pub_    = create_publisher<std_msgs::msg::Int32>  ("/cmd/thruster",     qos);
  elevator_pub_    = create_publisher<std_msgs::msg::Int32>  ("/cmd/elevator",     qos);
  rudder_pub_      = create_publisher<std_msgs::msg::Int32>  ("/cmd/rudder",       qos);
  moving_mass_pub_ = create_publisher<std_msgs::msg::Float64>("/cmd/moving_mass",  qos);
  arm_state_pub_   = create_publisher<std_msgs::msg::Bool>   ("/system/arm_state", qos);

  // Monitor publishers
  mon_target_depth_pub_   = create_publisher<std_msgs::msg::Float64>("/monitor/target/depth",       qos);
  mon_target_heading_pub_ = create_publisher<std_msgs::msg::Float64>("/monitor/target/heading",     qos);
  mon_target_speed_pub_   = create_publisher<std_msgs::msg::Float64>("/monitor/target/speed",       qos);
  mon_target_mass_pub_    = create_publisher<std_msgs::msg::Float64>("/monitor/target/moving_mass", qos);
  mon_desired_pitch_pub_  = create_publisher<std_msgs::msg::Float64>("/monitor/desired_pitch",      qos);

  // Arm Service
  arm_srv_ = create_service<std_srvs::srv::SetBool>("/system/arm",
    std::bind(&ControllerNode::on_arm, this, std::placeholders::_1, std::placeholders::_2));

  timer_ = create_wall_timer(std::chrono::milliseconds(10),
    std::bind(&ControllerNode::control_loop, this));

  last_time_      = now();
  last_odom_time_ = now();
  RCLCPP_INFO(get_logger(), "Loki controller ready!");
}

void ControllerNode::on_target_depth(const std_msgs::msg::Float64::SharedPtr msg){
  target_depth_ = msg->data;
}

void ControllerNode::on_target_heading(const std_msgs::msg::Float64::SharedPtr msg){
  target_heading_ = msg->data;
}

void ControllerNode::on_target_speed(const std_msgs::msg::Float64::SharedPtr msg){
  target_speed_ = msg->data;
}

void ControllerNode::on_target_moving_mass(const std_msgs::msg::Float64::SharedPtr msg){
  target_moving_mass_ = std::clamp(msg->data, 0.0, 1.0);
}

void ControllerNode::on_arm(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  const std_srvs::srv::SetBool::Response::SharedPtr res)
{
  is_armed_ = req->data;

  if (is_armed_) {
    target_heading_     = current_heading_; // locks to current heading
    target_depth_       = current_depth_;   // locks to current depth
    target_speed_       = 0.0;
    target_moving_mass_ = 0.0;

    speed_pid_.reset_controller();
    yaw_pid_.reset_controller();
    depth_pid_.reset_controller();
    pitch_pid_.reset_controller();

    res->message = "Armed";
    RCLCPP_INFO(get_logger(), "ARMED");
  } else {
    res->message = "Disarmed";
    RCLCPP_INFO(get_logger(), "DISARMED");
  }

  res->success = true;
  auto msg  = std_msgs::msg::Bool();
  msg.data  = is_armed_;
  arm_state_pub_->publish(msg);
}

void ControllerNode::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_time_ = now();

  current_depth_ = msg->pose.pose.position.z;
  current_speed_ = msg->twist.twist.linear.x;

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

  if (current_heading_ < 0.0) current_heading_ += 360.0; // heading correction to [0, 360]
}

void ControllerNode::control_loop()
{
  auto   t  = now();
  double dt = std::clamp((t - last_time_).seconds(), 1e-4, 0.05);
  last_time_ = t;
  double desired_pitch = 0.0;

  publish_f64(mon_target_depth_pub_,   target_depth_);
  publish_f64(mon_target_heading_pub_, target_heading_);
  publish_f64(mon_target_speed_pub_,   target_speed_);
  publish_f64(mon_target_mass_pub_,    target_moving_mass_);

  if ((now() - last_odom_time_).seconds() > 0.5) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry timeout — disarming");
    is_armed_ = false;
  }

  if (!is_armed_) {
    auto neutral_pwm = std_msgs::msg::Int32();   neutral_pwm.data = 1500;
    auto zero_mass   = std_msgs::msg::Float64(); zero_mass.data   = 0.0;
    thruster_pub_->publish(neutral_pwm);
    elevator_pub_->publish(neutral_pwm);
    rudder_pub_->publish(neutral_pwm);
    moving_mass_pub_->publish(zero_mass);
    publish_f64(mon_desired_pitch_pub_, 0.0);  // disarmed, no pitch command
    return;
  }

  double speed_effort = speed_pid_.compute_control(dt, target_speed_ - current_speed_);
  double yaw_effort   = yaw_pid_.compute_control(dt, wrap_angle(target_heading_ - current_heading_));

  double depth_err = target_depth_ - current_depth_;
  desired_pitch    = depth_pid_.compute_control(dt, depth_err);
  desired_pitch    = std::clamp(desired_pitch, -max_pitch_cmd_, max_pitch_cmd_);
  publish_f64(mon_desired_pitch_pub_, desired_pitch);  // update with real value

  double pitch_effort = pitch_pid_.compute_control(
      dt, desired_pitch - current_pitch_, -current_pitch_rate_);

  auto mm_msg = std_msgs::msg::Float64();
  mm_msg.data = target_moving_mass_;
  moving_mass_pub_->publish(mm_msg);

  auto t_msg = std_msgs::msg::Int32(); t_msg.data = effort_to_pwm(speed_effort);
  auto e_msg = std_msgs::msg::Int32(); e_msg.data = effort_to_pwm(pitch_effort, true);
  auto r_msg = std_msgs::msg::Int32(); r_msg.data = effort_to_pwm(yaw_effort,   true);

  thruster_pub_->publish(t_msg);
  elevator_pub_->publish(e_msg);
  rudder_pub_->publish(r_msg);
}

PIDParams ControllerNode::load_pid(const std::string & ns, double alpha)
{
  PIDParams p;
  p.Kp_gains                = declare_parameter(ns + ".kp",         0.0);
  p.Ki_gains                = declare_parameter(ns + ".ki",         0.0);
  p.Kd_gains                = declare_parameter(ns + ".kd",         0.0);
  p.alpha                   = alpha;
  p.antiwindup_cte          = declare_parameter(ns + ".antiwindup", 0.0);
  p.upper_output_saturation = declare_parameter(ns + ".output_max", 400.0);
  p.lower_output_saturation = declare_parameter(ns + ".output_min", -400.0);
  return p;
}

void ControllerNode::publish_f64(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub, double value)
{
  std_msgs::msg::Float64 msg;
  msg.data = value;
  pub->publish(msg);
}

int ControllerNode::effort_to_pwm(double effort, bool invert)
{
  if (invert) return std::clamp(1500 - static_cast<int>(effort), 1100, 1900);
  return std::clamp(1500 + static_cast<int>(effort), 1100, 1900);
}

double ControllerNode::wrap_angle(double deg)
{
  while (deg >  180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

}  // namespace loki