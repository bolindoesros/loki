#include <memory>
#include <string>
#include "tracker650_ros_driver/tracker650_republisher.hpp"
#include "tracker650_ros_driver/tracker650_parser.hpp"

/**
 * @brief Callback for raw DVL sentence messages.
 *
 * This function is called automatically whenever the republisher receives
 * a new raw DVL packet from the receiver node on `/dvl/raw_data`.
 *
 * Pipeline:
 * 1. Extract raw NMEA-style sentence from the ROS String message.
 * 2. Parse the sentence using Tracker650Parser.
 * 3. Convert parsed DVL velocity into TwistWithCovarianceStamped.
 * 4. Publish the processed velocity message to `/dvl/twist_stamped`.
 *
 * @param msg Shared pointer to the received ROS String message.
 *            The actual raw DVL sentence is stored in `msg->data`.
 */
void Tracker650Republisher::rawDvlCallBack(const std_msgs::msg::String::SharedPtr msg)
{
  //Subscribe
  std::string raw_data = msg->data;

  RCLCPP_INFO(this->get_logger(), "Received: %s", raw_data.c_str());

  Tracker650Parser::DvlVelocity velocity;   

  if(!parser_.parseDVPDX(raw_data, velocity))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to Parse DVL Packet");
    return;
  }

  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp = this->now();
  twist_msg.header.frame_id = "dvl_link";
  twist_msg.twist.twist.linear.x = velocity.vx;
  twist_msg.twist.twist.linear.y = velocity.vy;
  twist_msg.twist.twist.linear.z = velocity.vz;

    twist_msg.twist.covariance[0] = 0.01;   // vx
  twist_msg.twist.covariance[7] = 0.01;   // vy
  twist_msg.twist.covariance[14] = 0.01;  // vz

  twist_msg.twist.covariance[21] = 999999.0;
  twist_msg.twist.covariance[28] = 999999.0;
  twist_msg.twist.covariance[35] = 999999.0;
  
  twist_pub_->publish(twist_msg);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // auto node = std::make_shared<Tracker650Republisher>();
  // node->republisherloop();
  // the following code is not used because loop is handled by ROS.
  // spin() waits for /dvl/raw_data messages. Automatically calls RawDvlCallback(). callback parses and pub.
  rclcpp::spin(std::make_shared<Tracker650Republisher>());

  rclcpp::shutdown();
  return 0;
}


