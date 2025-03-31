#pragma once

namespace rover0_controller
{
namespace mecanum_kinematics
{
struct WheelAngularVelocity
{
  double front_left;
  double front_right;
  double rear_left;
  double rear_right;
};

struct Twist
{
  double linear_x;
  double linear_y;
  double angular_z;
};

class MecanumKinematics final
{
public:
  /**
   * @brief Constructs a MecanumKinematics object with specified wheel parameters.
   *
   * @param wheel_radius Radius of the wheels (meters).
   * @param wheel_base Distance between front and rear wheels (meters).
   * @param track_width Distance between left and right wheels (meters).
   */
  explicit MecanumKinematics(double wheel_radius, double wheel_base, double track_width);

  /**
   * @brief Default destructor.
   */
  ~MecanumKinematics() = default;

  /**
   * @brief Computes the wheel angular velocities given a desired robot twist.
   *
   * Inverse Kinematics: Twist -> Wheel Angular Velocities
   *
   * @param twist Desired robot twist (linear and angular velocities).
   * @return WheelAngularVelocity Struct containing angular velocities for each wheel (rad/s).
   */
  WheelAngularVelocity inverse(Twist const&) const;

  /**
   * @brief Computes the robot's twist given the current wheel angular velocities.
   *
   * Forward Kinematics: Wheel Angular Velocities -> Twist
   *
   * @param wheel_velocities Current wheel angular velocities (rad/s).
   * @return Twist Struct containing the robot's linear and angular velocities (m/s, rad/s).
   */
  Twist forward(WheelAngularVelocity const&) const;

private:
  double wheel_radius_; ///< Radius of the wheels (meters).
  double wheel_base_;   ///< Distance between front and rear wheels (meters).
  double track_width_;  ///< Distance between left and right wheels (meters).
};
}
}
