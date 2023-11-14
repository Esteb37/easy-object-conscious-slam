#pragma once

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

#define Transform geometry_msgs::msg::TransformStamped
#define Pose geometry_msgs::msg::PoseStamped
#define Velocity geometry_msgs::msg::Twist
#define Lidar sensor_msgs::msg::LaserScan
#define Float32 std_msgs::msg::Float32

using namespace rclcpp;

static const char *
timestamp()
{
  long int time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  long int year = time / 31536000;
  time = time % 31536000;
  long int month = time / 2592000;
  time = time % 2592000;
  long int day = time / 86400;
  time = time % 86400;
  long int hour = time / 3600;
  time = time % 3600;
  long int minute = time / 60;
  time = time % 60;
  long int second = time;
  time = time % 1;
  long int millisecond = time * 1000;
  std::string timestamp = std::to_string(year) + "-" + std::to_string(month) + "-" + std::to_string(day) + " " + std::to_string(hour) + ":" + std::to_string(minute) + ":" + std::to_string(second) + ":" + std::to_string(millisecond);
  return timestamp.c_str();
}

#define LOG(NODE, MSG) RCLCPP_INFO(NODE->get_logger(), "%d %s() %s", __LINE__, __FUNCTION__, MSG)

#define LOGN(NODE, MSG) RCLCPP_INFO(NODE->get_logger(), "%d %s() %f", __LINE__, __FUNCTION__, (double)MSG)
