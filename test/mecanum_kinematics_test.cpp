#include "hero_chassis_controller/mecanum_kinematics.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

bool nearlyEqual(double first, double second, double tolerance = 1e-9) {
  return std::abs(first - second) <= tolerance;
}

bool checkRoundTrip(const hero_chassis_controller::MecanumKinematics& model,
                    const hero_chassis_controller::ChassisVelocity& expected) {
  const auto wheel_speeds = model.toWheelVelocities(expected);
  const auto actual = model.toChassisVelocity(wheel_speeds);
  return nearlyEqual(actual.forward, expected.forward) &&
         nearlyEqual(actual.left, expected.left) &&
         nearlyEqual(actual.yaw_rate, expected.yaw_rate);
}

}  // namespace

int main() {
  const hero_chassis_controller::MecanumKinematics model(0.07625, 0.4,
                                                          0.4);
  if (!model.isValid()) {
    std::cerr << "valid model was rejected\n";
    return 1;
  }

  if (!checkRoundTrip(model, {1.0, 0.0, 0.0}) ||
      !checkRoundTrip(model, {0.0, 1.0, 0.0}) ||
      !checkRoundTrip(model, {0.0, 0.0, 1.0}) ||
      !checkRoundTrip(model, {0.8, -0.35, 0.6})) {
    std::cerr << "forward and inverse kinematics are inconsistent\n";
    return 1;
  }

  const auto left_motion =
      model.toWheelVelocities({0.0, 1.0, 0.0});
  if (!(left_motion.front_left < 0.0 &&
        left_motion.front_right > 0.0 &&
        left_motion.rear_left > 0.0 &&
        left_motion.rear_right < 0.0)) {
    std::cerr << "lateral wheel directions do not match the model\n";
    return 1;
  }

  const auto forward_motion =
      model.toWheelVelocities({1.0, 0.0, 0.0});
  if (!(forward_motion.front_left > 0.0 &&
        forward_motion.front_right > 0.0 &&
        forward_motion.rear_left > 0.0 &&
        forward_motion.rear_right > 0.0)) {
    std::cerr << "forward wheel directions do not match the model\n";
    return 1;
  }

  const auto counter_clockwise_motion =
      model.toWheelVelocities({0.0, 0.0, 1.0});
  if (!(counter_clockwise_motion.front_left < 0.0 &&
        counter_clockwise_motion.front_right > 0.0 &&
        counter_clockwise_motion.rear_left < 0.0 &&
        counter_clockwise_motion.rear_right > 0.0)) {
    std::cerr << "rotational wheel directions do not match the model\n";
    return 1;
  }

  const hero_chassis_controller::MecanumKinematics invalid_model(0.0, 0.4,
                                                                  0.4);
  if (invalid_model.isValid()) {
    std::cerr << "invalid model was accepted\n";
    return 1;
  }

  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_chassis =
      model.toWheelVelocities({infinity, 0.0, 0.0});
  if (!nearlyEqual(invalid_chassis.front_left, 0.0) ||
      !nearlyEqual(invalid_chassis.front_right, 0.0) ||
      !nearlyEqual(invalid_chassis.rear_left, 0.0) ||
      !nearlyEqual(invalid_chassis.rear_right, 0.0)) {
    std::cerr << "non-finite chassis command was accepted\n";
    return 1;
  }

  const auto invalid_wheels =
      model.toChassisVelocity({nan, 0.0, 0.0, 0.0});
  if (!nearlyEqual(invalid_wheels.forward, 0.0) ||
      !nearlyEqual(invalid_wheels.left, 0.0) ||
      !nearlyEqual(invalid_wheels.yaw_rate, 0.0)) {
    std::cerr << "non-finite wheel feedback was accepted\n";
    return 1;
  }

  std::cout << "mecanum kinematics tests passed\n";
  return 0;
}
