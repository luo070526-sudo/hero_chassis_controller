#include "hero_chassis_controller/mecanum_kinematics.h"

#include <cmath>

namespace hero_chassis_controller {
namespace {

bool isFinite(const ChassisVelocity& velocity) {
  return std::isfinite(velocity.forward) && std::isfinite(velocity.left) &&
         std::isfinite(velocity.yaw_rate);
}

bool isFinite(const WheelVelocities& velocity) {
  return std::isfinite(velocity.front_left) &&
         std::isfinite(velocity.front_right) &&
         std::isfinite(velocity.rear_left) &&
         std::isfinite(velocity.rear_right);
}

}  // namespace

MecanumKinematics::MecanumKinematics(double wheel_radius, double wheel_base,
                                     double wheel_track)
    : wheel_radius_(wheel_radius),
      rotation_lever_arm_(0.5 * (wheel_base + wheel_track)) {}

bool MecanumKinematics::isValid() const {
  return std::isfinite(wheel_radius_) &&
         std::isfinite(rotation_lever_arm_) && wheel_radius_ > 0.0 &&
         rotation_lever_arm_ > 0.0;
}

WheelVelocities MecanumKinematics::toWheelVelocities(
    const ChassisVelocity& chassis_velocity) const {
  WheelVelocities result;
  if (!isValid() || !isFinite(chassis_velocity)) {
    return result;
  }

  const double rotation = rotation_lever_arm_ * chassis_velocity.yaw_rate;
  result.front_left =
      (chassis_velocity.forward - chassis_velocity.left - rotation) /
      wheel_radius_;
  result.front_right =
      (chassis_velocity.forward + chassis_velocity.left + rotation) /
      wheel_radius_;
  result.rear_left =
      (chassis_velocity.forward + chassis_velocity.left - rotation) /
      wheel_radius_;
  result.rear_right =
      (chassis_velocity.forward - chassis_velocity.left + rotation) /
      wheel_radius_;
  if (!isFinite(result)) {
    return WheelVelocities{};
  }
  return result;
}

ChassisVelocity MecanumKinematics::toChassisVelocity(
    const WheelVelocities& wheel_velocities) const {
  ChassisVelocity result;
  if (!isValid() || !isFinite(wheel_velocities)) {
    return result;
  }

  const double linear_scale = wheel_radius_ * 0.25;
  result.forward =
      linear_scale * (wheel_velocities.front_left +
                      wheel_velocities.front_right +
                      wheel_velocities.rear_left +
                      wheel_velocities.rear_right);
  result.left =
      linear_scale * (-wheel_velocities.front_left +
                      wheel_velocities.front_right +
                      wheel_velocities.rear_left -
                      wheel_velocities.rear_right);
  result.yaw_rate =
      linear_scale * (-wheel_velocities.front_left +
                      wheel_velocities.front_right -
                      wheel_velocities.rear_left +
                      wheel_velocities.rear_right) /
      rotation_lever_arm_;
  if (!isFinite(result)) {
    return ChassisVelocity{};
  }
  return result;
}

}  // namespace hero_chassis_controller
