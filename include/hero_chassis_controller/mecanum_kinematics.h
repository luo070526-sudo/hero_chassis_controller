#ifndef HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H_
#define HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H_

namespace hero_chassis_controller {

struct ChassisVelocity {
  double forward{0.0};
  double left{0.0};
  double yaw_rate{0.0};
};

struct WheelVelocities {
  double front_left{0.0};
  double front_right{0.0};
  double rear_left{0.0};
  double rear_right{0.0};
};

class MecanumKinematics {
 public:
  MecanumKinematics(double wheel_radius, double wheel_base,
                    double wheel_track);

  bool isValid() const;
  WheelVelocities toWheelVelocities(
      const ChassisVelocity& chassis_velocity) const;
  ChassisVelocity toChassisVelocity(
      const WheelVelocities& wheel_velocities) const;

 private:
  double wheel_radius_;
  double rotation_lever_arm_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_MECANUM_KINEMATICS_H_
