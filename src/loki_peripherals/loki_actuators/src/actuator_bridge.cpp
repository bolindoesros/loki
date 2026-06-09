#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <loki_msgs/msg/esp_packet.hpp>

static int8_t pwm_to_int8(int pwm)
{
    int val = (pwm - 1500) / 4;
    return static_cast<int8_t>(std::max(-100, std::min(100, val)));
}

class HwBridge : public rclcpp::Node
{
public:
    HwBridge() : Node("hw_bridge")
    {
        auto qos = rclcpp::QoS(10).reliable();

        esp_pub_ = create_publisher<loki_msgs::msg::EspPacket>("/pc_to_esp_cmd", qos);

        thruster_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/cmd/thruster", qos,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { thruster_ = msg->data; });

        elevator_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/cmd/elevator", qos,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { elevator_ = msg->data; });

        rudder_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/cmd/rudder", qos,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { rudder_ = msg->data; });

        moving_mass_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/cmd/moving_mass", qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { moving_mass_ = msg->data; });

        // Publish at 20 Hz
        timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                   [this]() { publish(); });

        RCLCPP_INFO(get_logger(), "actutaor bridge ready to publish commands");
    }

private:
    void publish()
    {
        loki_msgs::msg::EspPacket pkt;
        pkt.seq    = seq_++;
        pkt.cmd[0] = pwm_to_int8(thruster_);
        pkt.cmd[1] = pwm_to_int8(elevator_);
        pkt.cmd[2] = pwm_to_int8(rudder_);
        pkt.cmd[3] = static_cast<int8_t>(std::max(0, std::min(100, static_cast<int>(moving_mass_ * 100))));
        pkt.cmd[4] = 0;  // reserved
        pkt.cmd[5] = 0;  // reserved
        esp_pub_->publish(pkt);
    }

    rclcpp::Publisher<loki_msgs::msg::EspPacket>::SharedPtr esp_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   thruster_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   elevator_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   rudder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr moving_mass_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int    thruster_    = 1500;
    int    elevator_    = 1500;
    int    rudder_      = 1500;
    double moving_mass_ = 0.0;
    uint32_t seq_       = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HwBridge>());
    rclcpp::shutdown();
    return 0;
}
