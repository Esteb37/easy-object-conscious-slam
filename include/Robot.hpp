#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <webots/motor.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

#include "Utils.hpp"

#define TRACK_WIDTH 0.178
#define WHEEL_RADIUS 0.033
#define PPR 4096
#define GEAR_RATIO 1.0

namespace Robot
{

  class Robot : public webots_ros2_driver::PluginInterface
  {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node,
              std::unordered_map<std::string, std::string> &parameters) override;

  private:
    void cmdVelCallback(const Velocity::SharedPtr msg);
    void lidarCallback(const Lidar::SharedPtr msg);

    void setupTopics();
    void setupWebots();
    void setupPose();

    void updateOdometry();
    void broadcast();

    Subscription<Velocity>::SharedPtr
        cmdVelSubscription_;
    Subscription<Lidar>::SharedPtr
        lidarSubscription_;
    Publisher<Odom>::SharedPtr
        odomPublisher_;
    Publisher<Lidar>::SharedPtr
        lidarPublisher_;

    WbDeviceTag rightMotor_;
    WbDeviceTag leftMotor_;
    WbDeviceTag rightPositionSensor_;
    WbDeviceTag leftPositionSensor_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped odomTransform_;
    geometry_msgs::msg::TransformStamped lidarTransform_;

    Velocity cmdVelMsg_;
    Lidar lidar_;

    Velocity currentVelocity_;
    Odom currentPose_;
    webots_ros2_driver::WebotsNode *node_;

    Clock::SharedPtr clock_;

    Time lastTime_;
    float lastRightWheelPosition_ = 0;
    float lastLeftWheelPosition_ = 0;
    bool firstReading_ = true;
  };
} // namespace Robot
#endif