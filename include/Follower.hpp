#ifndef FOLLOWER_HPP
#define FOLLOWER_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "Utils.hpp"

namespace Robot
{

  class Follower : public Node
  {
  public:
    Follower() : Node("Follower")
    {
      setup();

      while (rclcpp::ok())
      {
        run();
      }
    }

  private:
    void setup();
    void run();

    void poseCallback(const Pose::SharedPtr msg)
    {
      pose_ = *msg;
    }

    void goalCallback(const Pose::SharedPtr msg)
    {
      goal_ = *msg;
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      lidar_ = *msg;
    }

    Subscription<Pose>::SharedPtr
        poseSubscription_;
    Subscription<Pose>::SharedPtr
        goalSubscription_;
    Subscription<Lidar>::SharedPtr
        lidarSubscription_;
    Publisher<Velocity>::SharedPtr
        velocityPublisher_;

    Pose pose_;
    Pose goal_;
    Velocity velocity_;
    Lidar lidar_;
  };

} // namespace Robot

#endif