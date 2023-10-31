#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <webots/motor.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025

#define LINEAR_SPEED(right, left) ((right + left) * WHEEL_RADIUS / 2)
#define ANGULAR_SPEED(right, left) ((right - left) * WHEEL_RADIUS / (2 * HALF_DISTANCE_BETWEEN_WHEELS))

namespace Robot
{

  typedef geometry_msgs::msg::Vector3 Vector3;
  typedef geometry_msgs::msg::Twist Velocity;
  typedef geometry_msgs::msg::Pose Pose;

  class Drivetrain : public webots_ros2_driver::PluginInterface
  {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node,
              std::unordered_map<std::string, std::string> &parameters) override;

    Pose getPose()
    {
      return currentPose_;
    }
    Velocity getVelocity()
    {
      return currentVelocity_;
    }

    void setPose(Pose pose)
    {
      currentPose_ = pose;
    }

    void setVelocity(Velocity velocity)
    {
      currentVelocity_ = velocity;
    }
    void setLinearVelocity(Vector3 velocity)
    {
      currentVelocity_.linear = velocity;
    }
    void setAngularVelocity(Vector3 velocity)
    {
      currentVelocity_.angular = velocity;
    }
    void setRightVelocity(float linear, float angular)
    {
      wb_motor_set_velocity(rightMotor_, (linear - angular * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS);
    }

    void setLeftVelocity(float linear, float angular)
    {
      wb_motor_set_velocity(leftMotor_, (linear + angular * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS);
    }

    void driveTo(Pose pose);
    void driveMeters(float distance);
    void turnDegrees(float angle);
    void driveSeconds(float seconds);
    void turnSeconds(float seconds);

  private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void publishPose();
    void publishVelocity();
    void updateOdometry();

    rclcpp::Subscription<Velocity>::SharedPtr
        cmdVelSubscription_;
    rclcpp::Publisher<Pose>::SharedPtr
        posePublisher_;
    rclcpp::Publisher<Velocity>::SharedPtr
        velocityPublisher_;

    WbDeviceTag rightMotor_;
    WbDeviceTag leftMotor_;

    Velocity cmdVelMsg_;

    Velocity currentVelocity_;
    Pose currentPose_;
    webots_ros2_driver::WebotsNode *node_;
  };
} // namespace Robot
#endif