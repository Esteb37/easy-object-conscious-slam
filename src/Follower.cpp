#include "Follower.hpp"
#include "Utils.hpp"

namespace Robot
{

  void Follower::setup()
  {
    poseSubscription_ = this->create_subscription<Pose>(
        "/pose", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Follower::poseCallback, this, std::placeholders::_1));
    goalSubscription_ = this->create_subscription<Pose>(
        "/goal", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Follower::goalCallback, this, std::placeholders::_1));
    lidarSubscription_ = this->create_subscription<Lidar>(
        "/scan", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Follower::lidarCallback, this, std::placeholders::_1));

    LOG(this, "Follower initialized");
  }

  void Follower::run()
  {
  }

} // namespace Robot
