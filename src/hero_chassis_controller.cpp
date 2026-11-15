#include "hero_chassis_controller/hero_chassis_controller.h"

#include <algorithm>
#include <cmath>
#include <exception>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hero_chassis_controller {

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort_interface,
    ros::NodeHandle& root_node_handle,
    ros::NodeHandle& controller_node_handle) {
  if (!loadParameters(controller_node_handle) ||
      !loadJointHandles(effort_interface, controller_node_handle) ||
      !initializePidControllers(controller_node_handle)) {
    return false;
  }

  kinematics_.reset(
      new MecanumKinematics(wheel_radius_, wheel_base_, wheel_track_));
  if (!kinematics_->isValid()) {
    ROS_ERROR("Invalid mecanum geometry: radius, base and track must be positive");
    return false;
  }

  VelocityCommand initial_command;
  command_buffer_.writeFromNonRT(initial_command);

  command_subscriber_ = root_node_handle.subscribe(
      "/cmd_vel", 1, &HeroChassisController::commandCallback, this);

  transform_listener_.reset(
      new tf2_ros::TransformListener(transform_buffer_));
  odometry_publisher_.reset(
      new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(
          root_node_handle, "/odom", 4));
  transform_publisher_.reset(
      new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(
          root_node_handle, "/tf", 4));
  transform_publisher_->msg_.transforms.resize(1);

  ROS_INFO_STREAM("Hero chassis controller initialized in '"
                  << command_frame_ << "' command frame");
  return true;
}

bool HeroChassisController::loadParameters(
    ros::NodeHandle& controller_node_handle) {
  controller_node_handle.param("wheel_radius", wheel_radius_, 0.07625);
  controller_node_handle.param("wheel_base", wheel_base_, 0.4);
  controller_node_handle.param("wheel_track", wheel_track_, 0.4);
  controller_node_handle.param("command_frame", command_frame_,
                               std::string("base_link"));
  controller_node_handle.param("odom_frame", odom_frame_,
                               std::string("odom"));
  controller_node_handle.param("base_frame", base_frame_,
                               std::string("base_link"));
  controller_node_handle.param("command_timeout", command_timeout_, 0.5);
  double publish_rate = 0.0;
  controller_node_handle.param("publish_rate", publish_rate, 50.0);
  controller_node_handle.param("maximum_wheel_speed", maximum_wheel_speed_,
                               50.0);
  controller_node_handle.param("maximum_wheel_effort", maximum_wheel_effort_,
                               5.0);
  controller_node_handle.param("linear_acceleration_limit",
                               linear_acceleration_limit_, 1.0);
  controller_node_handle.param("angular_acceleration_limit",
                               angular_acceleration_limit_, 2.0);

  const std::array<double, 9> numeric_parameters = {
      wheel_radius_, wheel_base_, wheel_track_, command_timeout_, publish_rate,
      maximum_wheel_speed_, maximum_wheel_effort_,
      linear_acceleration_limit_, angular_acceleration_limit_};
  for (const double value : numeric_parameters) {
    if (!std::isfinite(value)) {
      ROS_ERROR("Controller parameters must be finite");
      return false;
    }
  }

  if (command_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
      wheel_radius_ <= 0.0 || wheel_base_ <= 0.0 || wheel_track_ <= 0.0 ||
      command_timeout_ < 0.0 || publish_rate <= 0.0 ||
      maximum_wheel_speed_ <= 0.0 || maximum_wheel_effort_ <= 0.0 ||
      linear_acceleration_limit_ < 0.0 ||
      angular_acceleration_limit_ < 0.0) {
    ROS_ERROR("Controller parameters contain an invalid frame, limit, or geometry");
    return false;
  }

  publish_period_ = 1.0 / publish_rate;
  return true;
}

bool HeroChassisController::loadJointHandles(
    hardware_interface::EffortJointInterface* effort_interface,
    ros::NodeHandle& controller_node_handle) {
  const std::array<std::string, kWheelCount> parameter_names = {
      "joints/front_left", "joints/front_right", "joints/rear_left",
      "joints/rear_right"};
  const std::array<std::string, kWheelCount> default_joint_names = {
      "left_front_wheel_joint", "right_front_wheel_joint",
      "left_back_wheel_joint", "right_back_wheel_joint"};

  try {
    for (std::size_t index = 0; index < parameter_names.size(); ++index) {
      std::string joint_name;
      controller_node_handle.param(parameter_names[index], joint_name,
                                   default_joint_names[index]);
      wheel_joints_[index] = effort_interface->getHandle(joint_name);
    }
  } catch (const hardware_interface::HardwareInterfaceException& error) {
    ROS_ERROR_STREAM("Unable to get wheel joint handle: " << error.what());
    return false;
  }
  return true;
}

bool HeroChassisController::initializePidControllers(
    ros::NodeHandle& controller_node_handle) {
  const std::array<std::string, kWheelCount> pid_names = {
      "front_left", "front_right", "rear_left", "rear_right"};
  for (std::size_t index = 0; index < pid_names.size(); ++index) {
    ros::NodeHandle pid_node(controller_node_handle,
                             "pids/" + pid_names[index]);
    if (!wheel_pid_[index].init(pid_node)) {
      ROS_ERROR_STREAM("Failed to initialize PID for " << pid_names[index]);
      return false;
    }
  }
  return true;
}

void HeroChassisController::starting(const ros::Time& time) {
  limited_command_ = ChassisVelocity{};
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;
  last_publish_time_ = time;
  for (auto& pid : wheel_pid_) {
    pid.reset();
  }
}

void HeroChassisController::update(const ros::Time& time,
                                   const ros::Duration& period) {
  const double period_seconds = period.toSec();
  if (!std::isfinite(period_seconds) || period_seconds <= 0.0) {
    for (auto& joint : wheel_joints_) {
      joint.setCommand(0.0);
    }
    return;
  }

  VelocityCommand command = *command_buffer_.readFromRT();
  ChassisVelocity requested_velocity;
  const bool command_is_fresh =
      command.valid && time >= command.received_at &&
      (command_timeout_ == 0.0 ||
       (time - command.received_at).toSec() <= command_timeout_);
  if (command_is_fresh) {
    requested_velocity = command.velocity;
  }

  const ChassisVelocity limited_velocity =
      limitAcceleration(requested_velocity, period_seconds);
  const WheelVelocities target_wheel_velocity =
      limitWheelSpeed(kinematics_->toWheelVelocities(limited_velocity));
  const std::array<double, kWheelCount> targets = {
      target_wheel_velocity.front_left, target_wheel_velocity.front_right,
      target_wheel_velocity.rear_left, target_wheel_velocity.rear_right};

  for (std::size_t index = 0; index < targets.size(); ++index) {
    const double measured_velocity = wheel_joints_[index].getVelocity();
    if (!std::isfinite(measured_velocity)) {
      wheel_pid_[index].reset();
      wheel_joints_[index].setCommand(0.0);
      continue;
    }

    const double velocity_error = targets[index] - measured_velocity;
    const double raw_effort =
        wheel_pid_[index].computeCommand(velocity_error, period);
    if (!std::isfinite(raw_effort)) {
      wheel_pid_[index].reset();
      wheel_joints_[index].setCommand(0.0);
      continue;
    }

    const double limited_effort =
        std::max(-maximum_wheel_effort_,
                 std::min(maximum_wheel_effort_, raw_effort));
    wheel_joints_[index].setCommand(limited_effort);
  }

  updateOdometry(time, period_seconds);
}

void HeroChassisController::stopping(const ros::Time&) {
  for (std::size_t index = 0; index < wheel_joints_.size(); ++index) {
    wheel_joints_[index].setCommand(0.0);
    wheel_pid_[index].reset();
  }
}

void HeroChassisController::commandCallback(
    const geometry_msgs::TwistConstPtr& message) {
  ChassisVelocity requested;
  requested.forward = message->linear.x;
  requested.left = message->linear.y;
  requested.yaw_rate = message->angular.z;

  if (!std::isfinite(requested.forward) || !std::isfinite(requested.left) ||
      !std::isfinite(requested.yaw_rate)) {
    ROS_WARN_THROTTLE(1.0, "Ignoring non-finite /cmd_vel command");
    return;
  }

  if (command_frame_ != base_frame_) {
    geometry_msgs::Vector3Stamped linear_in;
    geometry_msgs::Vector3Stamped angular_in;
    linear_in.header.frame_id = command_frame_;
    angular_in.header.frame_id = command_frame_;
    linear_in.header.stamp = ros::Time(0);
    angular_in.header.stamp = ros::Time(0);
    linear_in.vector.x = requested.forward;
    linear_in.vector.y = requested.left;
    angular_in.vector.z = requested.yaw_rate;

    try {
      const auto linear_out = transform_buffer_.transform(
          linear_in, base_frame_, ros::Duration(0.05));
      const auto angular_out = transform_buffer_.transform(
          angular_in, base_frame_, ros::Duration(0.05));
      requested.forward = linear_out.vector.x;
      requested.left = linear_out.vector.y;
      requested.yaw_rate = angular_out.vector.z;
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(1.0, "Cannot transform /cmd_vel from %s to %s: %s",
                        command_frame_.c_str(), base_frame_.c_str(),
                        error.what());
      return;
    }
  }

  if (!std::isfinite(requested.forward) || !std::isfinite(requested.left) ||
      !std::isfinite(requested.yaw_rate)) {
    ROS_WARN_THROTTLE(1.0, "Ignoring invalid transformed /cmd_vel command");
    return;
  }

  VelocityCommand command;
  command.velocity = requested;
  command.received_at = ros::Time::now();
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
}

ChassisVelocity HeroChassisController::limitAcceleration(
    const ChassisVelocity& requested, double period_seconds) {
  const double linear_change = linear_acceleration_limit_ * period_seconds;
  const double angular_change = angular_acceleration_limit_ * period_seconds;

  limited_command_.forward =
      linear_acceleration_limit_ == 0.0
          ? requested.forward
          : moveToward(limited_command_.forward, requested.forward,
                       linear_change);
  limited_command_.left =
      linear_acceleration_limit_ == 0.0
          ? requested.left
          : moveToward(limited_command_.left, requested.left, linear_change);
  limited_command_.yaw_rate =
      angular_acceleration_limit_ == 0.0
          ? requested.yaw_rate
          : moveToward(limited_command_.yaw_rate, requested.yaw_rate,
                       angular_change);
  return limited_command_;
}

WheelVelocities HeroChassisController::limitWheelSpeed(
    const WheelVelocities& requested) const {
  const double largest_speed =
      std::max({std::abs(requested.front_left),
                std::abs(requested.front_right),
                std::abs(requested.rear_left),
                std::abs(requested.rear_right)});
  if (largest_speed <= maximum_wheel_speed_) {
    return requested;
  }

  const double scale = maximum_wheel_speed_ / largest_speed;
  WheelVelocities limited;
  limited.front_left = requested.front_left * scale;
  limited.front_right = requested.front_right * scale;
  limited.rear_left = requested.rear_left * scale;
  limited.rear_right = requested.rear_right * scale;
  return limited;
}

void HeroChassisController::updateOdometry(const ros::Time& time,
                                           double period_seconds) {
  WheelVelocities measured_wheel_velocity;
  measured_wheel_velocity.front_left =
      wheel_joints_[kFrontLeft].getVelocity();
  measured_wheel_velocity.front_right =
      wheel_joints_[kFrontRight].getVelocity();
  measured_wheel_velocity.rear_left =
      wheel_joints_[kRearLeft].getVelocity();
  measured_wheel_velocity.rear_right =
      wheel_joints_[kRearRight].getVelocity();
  const ChassisVelocity measured_velocity =
      kinematics_->toChassisVelocity(measured_wheel_velocity);

  const double cos_yaw = std::cos(odom_yaw_);
  const double sin_yaw = std::sin(odom_yaw_);
  odom_x_ += (measured_velocity.forward * cos_yaw -
              measured_velocity.left * sin_yaw) *
             period_seconds;
  odom_y_ += (measured_velocity.forward * sin_yaw +
              measured_velocity.left * cos_yaw) *
             period_seconds;
  odom_yaw_ = normalizeYaw(odom_yaw_ +
                           measured_velocity.yaw_rate * period_seconds);

  if (time < last_publish_time_) {
    last_publish_time_ = time;
  }
  if ((time - last_publish_time_).toSec() >= publish_period_) {
    publishOdometry(time, measured_velocity);
    last_publish_time_ = time;
  }
}

void HeroChassisController::publishOdometry(
    const ros::Time& time, const ChassisVelocity& measured_velocity) {
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odom_yaw_);
  orientation.normalize();
  const geometry_msgs::Quaternion orientation_message =
      tf2::toMsg(orientation);

  if (odometry_publisher_->trylock()) {
    auto& message = odometry_publisher_->msg_;
    message.header.stamp = time;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    message.pose.pose.position.x = odom_x_;
    message.pose.pose.position.y = odom_y_;
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation = orientation_message;
    message.twist.twist.linear.x = measured_velocity.forward;
    message.twist.twist.linear.y = measured_velocity.left;
    message.twist.twist.angular.z = measured_velocity.yaw_rate;
    odometry_publisher_->unlockAndPublish();
  }

  if (transform_publisher_->trylock()) {
    auto& transform = transform_publisher_->msg_.transforms.front();
    transform.header.stamp = time;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odom_x_;
    transform.transform.translation.y = odom_y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = orientation_message;
    transform_publisher_->unlockAndPublish();
  }
}

double HeroChassisController::moveToward(double current, double target,
                                         double maximum_change) {
  if (target > current) {
    return std::min(target, current + maximum_change);
  }
  return std::max(target, current - maximum_change);
}

double HeroChassisController::normalizeYaw(double yaw) {
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
