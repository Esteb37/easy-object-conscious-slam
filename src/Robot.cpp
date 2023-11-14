#include "Robot.hpp"

#include <cstdio>
#include <functional>
#include <webots/position_sensor.h>
#include <webots/robot.h>

// function that logs the current timestamp, filename, function name and line number

namespace Robot
{

  void Robot::init(
      webots_ros2_driver::WebotsNode *node,
      std::unordered_map<std::string, std::string> &parameters)
  {
    node_ = node;

    // Setup WEBOTS motors
    rightMotor_ = wb_robot_get_device("right wheel motor");
    leftMotor_ = wb_robot_get_device("left wheel motor");

    wb_motor_set_position(rightMotor_, INFINITY);
    wb_motor_set_position(leftMotor_, INFINITY);
    wb_motor_set_velocity(rightMotor_, 0.0);
    wb_motor_set_velocity(leftMotor_, 0.0);

    // Setup WEBOTS encoders
    auto TIME_STEP = wb_robot_get_basic_time_step();
    rightPositionSensor_ = wb_motor_get_position_sensor(rightMotor_);
    leftPositionSensor_ = wb_motor_get_position_sensor(leftMotor_);
    wb_position_sensor_enable(rightPositionSensor_, TIME_STEP);
    wb_position_sensor_enable(leftPositionSensor_, TIME_STEP);

    // Setup subscriptions and publishers
    cmdVelSubscription_ = node->create_subscription<Velocity>(
        "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Robot::cmdVelCallback, this, std::placeholders::_1));

    lidarSubscription_ = node->create_subscription<Lidar>(
        "/Robot/LDS_01", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Robot::lidarCallback, this, std::placeholders::_1));

    posePublisher_ = node->create_publisher<Pose>("/pose", rclcpp::SensorDataQoS().reliable());

    velocityPublisher_ = node->create_publisher<Velocity>("/velocity", rclcpp::SensorDataQoS().reliable());

    rightWheelPublisher_ = node->create_publisher<Float32>("/right_wheel", rclcpp::SensorDataQoS().reliable());

    leftWheelPublisher_ = node->create_publisher<Float32>("/left_wheel", rclcpp::SensorDataQoS().reliable());

    lidarPublisher_ = node->create_publisher<Lidar>("/scan", rclcpp::SensorDataQoS().reliable());

    // Setup initial pose
    clock_ = node->get_clock();
    lastTime_ = clock_->now();
    lastRightWheelPosition_ = 0.0;
    lastLeftWheelPosition_ = 0.0;
    transform_.header.frame_id = "odom";
    transform_.child_frame_id = "base_link";

    currentPose_.header.frame_id = "odom";

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    LOG(node_, "Robot initialized");
  }

  void Robot::cmdVelCallback(
      const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmdVelMsg_.linear = msg->linear;
    cmdVelMsg_.angular = msg->angular;
  }

  void Robot::lidarCallback(
      const Lidar::SharedPtr msg)
  {
    lidar_ = *msg;
  }

  void Robot::step()
  {
    updateOdometry();
    setRightVelocity(cmdVelMsg_.linear.x, cmdVelMsg_.angular.z);
    setLeftVelocity(cmdVelMsg_.linear.x, cmdVelMsg_.angular.z);
  }

  void Robot::updateOdometry()
  {
    auto currentTime = clock_->now();
    auto dt = currentTime - lastTime_;
    lastTime_ = currentTime;

    float rightWheelPosition = wb_position_sensor_get_value(rightPositionSensor_);
    float leftWheelPosition = wb_position_sensor_get_value(leftPositionSensor_);

    if (firstReading_)
    {
      lastRightWheelPosition_ = rightWheelPosition;
      lastLeftWheelPosition_ = leftWheelPosition;
      firstReading_ = false;
    }

    float rightWheelSpeed = (rightWheelPosition - lastRightWheelPosition_) / dt.seconds();
    float leftWheelSpeed = (leftWheelPosition - lastLeftWheelPosition_) / dt.seconds();

    lastRightWheelPosition_ = rightWheelPosition;
    lastLeftWheelPosition_ = leftWheelPosition;

    currentVelocity_.linear.x = (rightWheelSpeed + leftWheelSpeed) * WHEEL_RADIUS / 2.0;
    currentVelocity_.angular.z = (rightWheelSpeed - leftWheelSpeed) * WHEEL_RADIUS / TRACK_WIDTH;

    currentPose_.pose.position.x += currentVelocity_.linear.x * dt.seconds() * cos(currentPose_.pose.orientation.z);

    currentPose_.pose.position.y += currentVelocity_.linear.x * dt.seconds() * sin(currentPose_.pose.orientation.z);

    currentPose_.pose.orientation.z += currentVelocity_.angular.z * dt.seconds();

    auto msg = Float32();

    msg.data = rightWheelSpeed;
    rightWheelPublisher_->publish(msg);
    msg.data = leftWheelSpeed;
    leftWheelPublisher_->publish(msg);

    currentPose_.header.stamp = currentTime;

    lidar_.header.frame_id = "world";
    lidar_.header.stamp = currentTime;

    tf_broadcaster_->sendTransform(transform_);

    lidarPublisher_->publish(lidar_);
    posePublisher_->publish(currentPose_);
    velocityPublisher_->publish(currentVelocity_);
  }

} // namespace Robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(Robot::Robot,
                       webots_ros2_driver::PluginInterface)