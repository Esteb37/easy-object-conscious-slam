#include "ocslam/Drivetrain.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

// function that logs the current timestamp, filename, function name and line number

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
#define LOG()           \
  printf("%s %s:%s:%d", \
         timestamp(),   \
         __FILE__,      \
         __func__,      \
         __LINE__);

namespace Robot
{

  void Drivetrain::init(
      webots_ros2_driver::WebotsNode *node,
      std::unordered_map<std::string, std::string> &parameters)
  {
    node_ = node;

    rightMotor_ = wb_robot_get_device("right wheel motor");
    leftMotor_ = wb_robot_get_device("left wheel motor");

    wb_motor_set_position(rightMotor_, INFINITY);
    wb_motor_set_velocity(rightMotor_, 0.0);

    wb_motor_set_position(leftMotor_, INFINITY);
    wb_motor_set_velocity(leftMotor_, 0.0);

    cmdVelSubscription_ = node->create_subscription<Velocity>(
        "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Drivetrain::cmdVelCallback, this, std::placeholders::_1));

    posePublisher_ = node->create_publisher<Pose>("/pose", rclcpp::SensorDataQoS().reliable());

    velocityPublisher_ = node->create_publisher<Velocity>("/velocity", rclcpp::SensorDataQoS().reliable());
  }

  void Drivetrain::cmdVelCallback(
      const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmdVelMsg_.linear = msg->linear;
    cmdVelMsg_.angular = msg->angular;
  }

  void Drivetrain::step()
  {
    updateOdometry();
    publishPose();
    publishVelocity();
    setRightVelocity(cmdVelMsg_.linear.x, cmdVelMsg_.angular.z);
    setLeftVelocity(cmdVelMsg_.linear.x, cmdVelMsg_.angular.z);
  }

  void Drivetrain::updateOdometry()
  {

    auto rightMotorSpeed = wb_motor_get_velocity(rightMotor_);
    auto leftMotorSpeed = wb_motor_get_velocity(leftMotor_);

    currentVelocity_.linear.x = LINEAR_SPEED(rightMotorSpeed, leftMotorSpeed);
    currentVelocity_.angular.z = ANGULAR_SPEED(rightMotorSpeed, leftMotorSpeed);
    currentPose_.position.x += currentVelocity_.linear.x * std::cos(currentPose_.orientation.x);
    currentPose_.position.y += currentVelocity_.linear.x * std::sin(currentPose_.orientation.y);
    currentPose_.orientation.z += currentVelocity_.angular.z;
  }

  void Drivetrain::publishPose()
  {
    posePublisher_->publish(currentPose_);
  }

  void Drivetrain::publishVelocity()
  {
    velocityPublisher_->publish(currentVelocity_);
  }
} // namespace Robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(Robot::Drivetrain,
                       webots_ros2_driver::PluginInterface)