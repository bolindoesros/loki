#include <chrono>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "auv_interfaces/msg/esp_packet.hpp"

using namespace std::chrono_literals;

class RosToEspPublisher : public rclcpp::Node
{
public:
  RosToEspPublisher()
  : Node("ros_to_esp"), seq_(0)
  {
    this->declare_parameter<double>("publish_hz", 2.0);

    double publish_hz = this->get_parameter("publish_hz").as_double();
    if (publish_hz <= 0.0) {
      publish_hz = 2.0;
    }

    auto period_ms = static_cast<int>(1000.0 / publish_hz);
    if (period_ms < 1) {
      period_ms = 1;
    }

    publisher_ = this->create_publisher<auv_interfaces::msg::EspPacket>(
      "/pc_to_esp_cmd", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&RosToEspPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "ros_to_esp started at %.2f Hz", publish_hz);
  }

private:
  void timer_callback()
  {
    auv_interfaces::msg::EspPacket msg;

    msg.seq = seq_;
    msg.cmd[0] = static_cast<int8_t>(seq_ % 128);
    msg.cmd[1] = 1;
    msg.cmd[2] = 2;
    msg.cmd[3] = 3;
    msg.cmd[4] = 4;
    msg.cmd[5] = 5;

    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
      "Published seq=%u cmd=[%d,%d,%d,%d,%d,%d]",
      msg.seq,
      msg.cmd[0], msg.cmd[1], msg.cmd[2],
      msg.cmd[3], msg.cmd[4], msg.cmd[5]);

    seq_++;
  }

  rclcpp::Publisher<auv_interfaces::msg::EspPacket>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t seq_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosToEspPublisher>());
  rclcpp::shutdown();
  return 0;
}