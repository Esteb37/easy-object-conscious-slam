#include "Follower.hpp"
#include "Socket.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto socket = std::make_shared<Robot::Socket>();
  auto follower = std::make_shared<Robot::Follower>();
  rclcpp::spin(socket);
  rclcpp::spin(follower);
  rclcpp::shutdown();
  return 0;
}