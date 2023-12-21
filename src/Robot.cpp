#include "Robot.hpp"

#include <cstdio>
#include <functional>
#include <tf2/utils.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

// function that logs the current timestamp, filename, function name and line number

namespace Robot
{

  void Robot::init(
      webots_ros2_driver::WebotsNode *node,
      std::unordered_map<std::string, std::string> &parameters)
  {
    try
    {
      node_ = node;

      setupTopics();
      setupWebots();
      setupPose();

      LOG(node_, "Robot initialized");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Robot::init() failed: %s", e.what());
    }
  }

  void Robot::step()
  {
    updateOdometry();
    broadcast();

    // Drive simulation
    wb_motor_set_velocity(rightMotor_, (cmdVelMsg_.linear.x + cmdVelMsg_.angular.z * TRACK_WIDTH) / WHEEL_RADIUS);
    wb_motor_set_velocity(leftMotor_, (cmdVelMsg_.linear.x - cmdVelMsg_.angular.z * TRACK_WIDTH) / WHEEL_RADIUS);
  }

  void Robot::cmdVelCallback(const Velocity::SharedPtr msg)
  {
    cmdVelMsg_.linear = msg->linear;
    cmdVelMsg_.angular = msg->angular;
  }

  void Robot::lidarCallback(const Lidar::SharedPtr msg)
  {
    lidar_ = *msg;
  }

  void Robot::setupTopics()
  {
    cmdVelSubscription_ = node_->create_subscription<Velocity>(
        "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Robot::cmdVelCallback, this, std::placeholders::_1));
    lidarSubscription_ = node_->create_subscription<Lidar>(
        "/Robot/LDS_01", rclcpp::SensorDataQoS().reliable(),
        std::bind(&Robot::lidarCallback, this, std::placeholders::_1));

    odomPublisher_ = node_->create_publisher<Odom>("/odom", rclcpp::SensorDataQoS().reliable());

    lidar_.header.frame_id = "LDS_01";
    lidarPublisher_ = node_->create_publisher<Lidar>("/scan", rclcpp::SensorDataQoS().reliable());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    odomTransform_.header.frame_id = "odom";
    odomTransform_.child_frame_id = "base_footprint";
  }

  void Robot::setupWebots()
  {
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
  }

  void Robot::setupPose()
  {
    clock_ = node_->get_clock();
    lastTime_ = clock_->now();
    lastRightWheelPosition_ = 0.0;
    lastLeftWheelPosition_ = 0.0;

    currentPose_.header.frame_id = "odom";
    currentPose_.child_frame_id = "base_footprint";
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

    // Calculate wheel velocities
    float rightWheelVelocityRadS = (rightWheelPosition - lastRightWheelPosition_) / dt.seconds();
    float leftWheelVelocityRadS = (leftWheelPosition - lastLeftWheelPosition_) / dt.seconds();

    // Calculate robot velocity
    float linearVelocity = (rightWheelVelocityRadS + leftWheelVelocityRadS) * WHEEL_RADIUS / 2.0f;
    float angularVelocity = (rightWheelVelocityRadS - leftWheelVelocityRadS) * WHEEL_RADIUS / TRACK_WIDTH;

    // Calculate robot pose
    float deltaTheta = angularVelocity * dt.seconds();

    // Update robot heading
    tf2::Quaternion prevQuat;
    prevQuat.setX(currentPose_.pose.pose.orientation.x);
    prevQuat.setY(currentPose_.pose.pose.orientation.y);
    prevQuat.setZ(currentPose_.pose.pose.orientation.z);
    prevQuat.setW(currentPose_.pose.pose.orientation.w);

    float prevHeading = tf2::impl::getYaw(prevQuat);
    float newHeading = prevHeading + deltaTheta;

    tf2::Quaternion newQuat;
    newQuat.setRPY(0.0, 0.0, newHeading);
    currentPose_.pose.pose.orientation.x = newQuat.x();
    currentPose_.pose.pose.orientation.y = newQuat.y();
    currentPose_.pose.pose.orientation.z = newQuat.z();
    currentPose_.pose.pose.orientation.w = newQuat.w();

    // Update robot position
    float deltaX = linearVelocity * cos(newHeading) * dt.seconds();
    float deltaY = linearVelocity * sin(newHeading) * dt.seconds();

    currentPose_.pose.pose.position.x += deltaX;
    currentPose_.pose.pose.position.y += deltaY;

    currentPose_.twist.twist.linear.x = linearVelocity;
    currentPose_.twist.twist.angular.z = angularVelocity;

    // Update last wheel position
    lastRightWheelPosition_ = rightWheelPosition;
    lastLeftWheelPosition_ = leftWheelPosition;
  }

  void Robot::broadcast()
  {

    auto currentTime = clock_->now();

    currentPose_.header.stamp = currentTime;
    lidar_.header.stamp = currentTime;

    lidarPublisher_->publish(lidar_);
    odomPublisher_->publish(currentPose_);

    odomTransform_.header.stamp = currentTime;
    odomTransform_.transform.translation.x = currentPose_.pose.pose.position.x;
    odomTransform_.transform.translation.y = currentPose_.pose.pose.position.y;
    odomTransform_.transform.translation.z = 0.0;
    odomTransform_.transform.rotation = currentPose_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(odomTransform_);
  }

} // namespace Robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(Robot::Robot,
                       webots_ros2_driver::PluginInterface)