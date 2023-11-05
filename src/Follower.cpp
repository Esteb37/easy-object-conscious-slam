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
        "/lidar", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Follower::lidarCallback, this, std::placeholders::_1));

    LOG(this, "Follower initialized");
  }

  void Follower::run()
  {
  }

} // namespace Robot

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Robot::Follower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}