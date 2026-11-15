#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_

#include <array>
#include <memory>
#include <string>

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "hero_chassis_controller/mecanum_kinematics.h"

namespace hero_chassis_controller {

class HeroChassisController
    : public controller_interface::Controller<
          hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;

  bool init(hardware_interface::EffortJointInterface* effort_interface,
            ros::NodeHandle& root_node_handle,
            ros::NodeHandle& controller_node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  enum WheelIndex {
    kFrontLeft = 0,
    kFrontRight = 1,
    kRearLeft = 2,
    kRearRight = 3,
    kWheelCount = 4
  };

  struct VelocityCommand {
    ChassisVelocity velocity;
    ros::Time received_at;
    bool valid{false};
  };

  bool loadParameters(ros::NodeHandle& controller_node_handle);
  bool loadJointHandles(
      hardware_interface::EffortJointInterface* effort_interface,
      ros::NodeHandle& controller_node_handle);
  bool initializePidControllers(ros::NodeHandle& controller_node_handle);
  void commandCallback(const geometry_msgs::TwistConstPtr& message);
  ChassisVelocity limitAcceleration(const ChassisVelocity& requested,
                                    double period_seconds);
  WheelVelocities limitWheelSpeed(
      const WheelVelocities& requested) const;
  void updateOdometry(const ros::Time& time, double period_seconds);
  void publishOdometry(const ros::Time& time,
                       const ChassisVelocity& measured_velocity);
  static double moveToward(double current, double target,
                           double maximum_change);
  static double normalizeYaw(double yaw);

  std::array<hardware_interface::JointHandle, kWheelCount> wheel_joints_;
  std::array<control_toolbox::Pid, kWheelCount> wheel_pid_;
  std::unique_ptr<MecanumKinematics> kinematics_;

  ros::Subscriber command_subscriber_;
  realtime_tools::RealtimeBuffer<VelocityCommand> command_buffer_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>
      odometry_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>>
      transform_publisher_;

  tf2_ros::Buffer transform_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;

  std::string command_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double wheel_radius_{0.0};
  double wheel_base_{0.0};
  double wheel_track_{0.0};
  double command_timeout_{0.0};
  double publish_period_{0.0};
  double maximum_wheel_speed_{0.0};
  double maximum_wheel_effort_{0.0};
  double linear_acceleration_limit_{0.0};
  double angular_acceleration_limit_{0.0};

  ChassisVelocity limited_command_;
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  ros::Time last_publish_time_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_
