#include <memory>
#include <string>
#include "tracker650_ros_driver/tracker650_receiver.hpp"

bool Tracker650Receiver::UDPInitialize()
{
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  
  if (sockfd < 0) 
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
    return false;
  }

  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(UDP_PORT);

  if (bind(sockfd, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to BIND UDP socket");
      close(sockfd);
      return false;
  }

  RCLCPP_INFO(this->get_logger(), "Listening on UDP port");
  return true;
}

std::string Tracker650Receiver::readDVL()
{
  std::memset(buffer, 0, BUFFER_SIZE);
  
  ssize_t bytes_received = recvfrom(
            sockfd,
            buffer,
            BUFFER_SIZE - 1,
            0,
            nullptr,
            nullptr
        );

  if (bytes_received < 0) {
    RCLCPP_WARN(this->get_logger(), "recvfrom failed");
    return "";
  }

  std::string packet(buffer, bytes_received);
  RCLCPP_INFO(this->get_logger(), "Receiver Received from UDP Port");

  return packet;
}

void Tracker650Receiver::publishRawDVL(std::string line)
{
  std_msgs::msg::String raw_msg;
  raw_msg.data = line;
  raw_pub_->publish(raw_msg);
}

void Tracker650Receiver::receiveLoop()
{
  if (!UDPInitialize()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP");
    return;
  }

  while (rclcpp::ok())
  {
  std::string packet = readDVL(); //receives raw $DVDX String

  //Although std::string packet is enough to read one line. Fail safe is to read incase 1 packet is multiple lines.
  //To do that, we can use std::getline() function that reads entire line from stream like source.
  if (packet.empty())
  {
    continue;
  }
  
  std::stringstream packet_stream(packet); //initialize string stream pre-loaded with contents of packet.
  //string stream turns string into  fake input as if its being typed by keyboard/file input
  std::string line;
  //get line reads from packet_stream, writes to line. delimiter is by default new line char.
  while (std::getline(packet_stream, line, '\n'))
  {
    if (line.empty())
    {
      continue;
    }

    publishRawDVL(line);
  }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Tracker650Receiver>(); //create a tracker receiver node
  node->receiveLoop(); //access receiveLoop() function inside node which inherits Tracker650Receiver methods. start blocking UDP receive-and-pub Loop
  
  rclcpp::shutdown();
  return 0;
}

