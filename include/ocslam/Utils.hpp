#pragma once

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <webots/robot.h>
const char *timestamp()
{
  long int time = wb_robot_get_time();
  // format the timestamp as year, month, day, hour, minute, second, millisecond
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

#define LOG(MSG) RCLCPP_INFO(node_->get_logger(), "%d %s() %s", __LINE__, __FUNCTION__, MSG)

#define LOGN(MSG) RCLCPP_INFO(node_->get_logger(), "%d %s() %f", __LINE__, __FUNCTION__, MSG)
