
#include "Socket.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto socket = std::make_shared<Robot::Socket>();
  rclcpp::spin(socket);
  rclcpp::shutdown();
  return 0;
}