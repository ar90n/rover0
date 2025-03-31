#pragma once

#include <cstddef>

#include "mecanum_kinematics.hpp"
#include "rcpputils/rolling_mean_accumulator.hpp"

#include "rover0_controller/mecanum_kinematics.hpp"

namespace rover0_controller
{
class WheelOdometry final
{
public:
  explicit WheelOdometry(
    double wheel_radius,
    double wheel_base,
    double track_width,
    size_t rolling_window_size
  );
  ~WheelOdometry() = default;

  bool update(
    double front_left_wheel_velocity,
    double front_right_wheel_velocity,
    double rear_left_wheel_velocity,
    double rear_right_wheel_velocity,
    double dt
  );

  double getX() const;
  double getY() const;
  double getHeading() const;
  double getLinearX() const;
  double getLinearY() const;
  double getAngular() const;

private:
  mecanum_kinematics::MecanumKinematics const mecanum_kinematics_;

  double                                    x_;
  double                                    y_;
  double                                    heading_;
  rcpputils::RollingMeanAccumulator<double> linear_x_accumulator_;
  rcpputils::RollingMeanAccumulator<double> linear_y_accumulator_;
  rcpputils::RollingMeanAccumulator<double> angular_z_accumulator_;
};
}
