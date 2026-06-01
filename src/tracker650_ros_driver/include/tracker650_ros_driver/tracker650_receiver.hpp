#include <iostream>
#include <sstream> 
#include <string>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

//Define C++ class behaving like ROS2 Node
class Tracker650Receiver : public rclcpp::Node
{
public:
    // sets ROS2 Node name as tracker650_receiver
    Tracker650Receiver() : Node("tracker650_receiver")
    {
        RCLCPP_INFO(this->get_logger(), "Receiver started");
        //this creates a publisher object which publishes standard string to /dvl/raw_data with queue size 10 (default).
        //Moreover, it assigns access to raw_pub_ smart pointer.
        //Hence, to publish, simply use raw_pub_->publish(raw_msg); which means to use the raw_pub_ pointer to access publisher object
        // Call/access publisher object's publish() function and send the raw_msg.
        raw_pub_ = this->create_publisher<std_msgs::msg::String>("dvl/raw_data", 10);
    }
    bool UDPInitialize();
    std::string readDVL();
    void publishRawDVL(std::string line);
    void receiveLoop();

private:  
    static constexpr int UDP_PORT = 27000;
    static constexpr int BUFFER_SIZE = 2048;

    int sockfd{};
    sockaddr_in server_addr{};

    char buffer[BUFFER_SIZE];
    //This creates a smart pointer 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
};