#include "Capture.hpp"
#include "Utils.hpp"

#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

namespace Robot
{

  void Capture::setup()
  {

    camera_topic_ = "/Robot/Astra_rgb/image_color";
    save_folder_ = "/home/esteb37/ocslam/resource/images"; // Replace with the desired save folder

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, rclcpp::SensorDataQoS().reliable(),
        std::bind(&Capture::imageCallback, this, std::placeholders::_1));

    LOG(this, "Capture initialized");
  }

  void Capture::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Image received");

    try
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      cv::imwrite(save_folder_ + "/image_" + std::to_string(image_count_++) + ".png", cv_ptr->image);
      RCLCPP_INFO(this->get_logger(), "Image saved");
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
    }

    // wait 1 seconds
    std::this_thread::sleep_for(std::chrono::seconds(1));

    image_count_++;
  }

} // namespace Robot
