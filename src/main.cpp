
#include "Capture.hpp"
#include "Socket.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto capture = std::make_shared<Robot::Capture>();
  rclcpp::spin(capture);
  rclcpp::shutdown();
  return 0;
}