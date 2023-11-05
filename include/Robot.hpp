#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <webots/motor.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

#include "Utils.hpp"

#define TRACK_WIDTH 0.08
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
    void updateOdometry();

    void setRightVelocity(float linear, float angular)
    {
      wb_motor_set_velocity(rightMotor_, (linear - angular * TRACK_WIDTH) / WHEEL_RADIUS);
    }

    void setLeftVelocity(float linear, float angular)
    {
      wb_motor_set_velocity(leftMotor_, (linear + angular * TRACK_WIDTH) / WHEEL_RADIUS);
    }

    Subscription<Velocity>::SharedPtr
        cmdVelSubscription_;
    Publisher<Pose>::SharedPtr
        posePublisher_;
    Publisher<Velocity>::SharedPtr
        velocityPublisher_;
    Publisher<Float32>::SharedPtr
        leftWheelPublisher_;
    Publisher<Float32>::SharedPtr
        rightWheelPublisher_;

    WbDeviceTag rightMotor_;
    WbDeviceTag leftMotor_;
    WbDeviceTag rightPositionSensor_;
    WbDeviceTag leftPositionSensor_;

    Velocity cmdVelMsg_;

    Velocity currentVelocity_;
    Pose currentPose_;
    webots_ros2_driver::WebotsNode *node_;

    Clock::SharedPtr clock_;

    Time lastTime_;
    float lastRightWheelPosition_;
    float lastLeftWheelPosition_;
  };
} // namespace Robot
#endif