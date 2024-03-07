#ifndef CAPTURE_HPP
#define CAPTURE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "Utils.hpp"

namespace Robot
{

  class Capture : public Node
  {
  public:
    Capture() : Node("Capture")
    {
      setup();
    }

  private:
    void setup();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    std::string camera_topic_;
    std::string save_folder_;
    int image_count_ = 0;
  };

}
#endif