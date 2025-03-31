#include <cmath>

#include "rover0_controller/wheel_odometry.hpp"

namespace
{
struct DeltaPosition
{
  double x;
  double y;
};

DeltaPosition calc_global_delta_position_by_linear_movement(
  DeltaPosition local_delta_pos,
  double        heading,
  double        delta_heading
)
{
  double const new_heading = heading + 0.5 * delta_heading;
  double const cos_value   = cos(new_heading);
  double const sin_value   = sin(new_heading);
  double const delta_x     = local_delta_pos.x * cos_value - local_delta_pos.y * sin_value; // meters
  double const delta_y     = local_delta_pos.x * sin_value + local_delta_pos.y * cos_value; // meters
  return DeltaPosition{ delta_x, delta_y };
}

DeltaPosition calc_global_delta_position_by_arc_movement(
  DeltaPosition local_delta_pos,
  double        heading,
  double        delta_heading
)
{
  double const cos_heading       = std::cos(heading);
  double const sin_heading       = std::sin(heading);
  double const cos_delta_heading = std::cos(delta_heading);
  double const sin_delta_heading = std::sin(delta_heading);

  double const local_delta_norm = std::hypot(local_delta_pos.x, local_delta_pos.y);
  if (local_delta_norm < 1e-12)
  {
    return DeltaPosition{ 0.0, 0.0 };
  }

  double const r       = local_delta_norm / delta_heading;
  double const cos_vel = local_delta_pos.x / local_delta_norm;
  double const sin_vel = local_delta_pos.y / local_delta_norm;

  double const S0 = sin_heading * cos_vel + cos_heading * sin_vel;
  double const C0 = cos_heading * cos_vel - sin_heading * sin_vel;
  double const S1 = S0 * cos_delta_heading + C0 * sin_delta_heading;
  double const C1 = C0 * cos_delta_heading - S0 * sin_delta_heading;

  double const delta_x = r * (S1 - S0);
  double const delta_y = -r * (C1 - C0);
  return DeltaPosition{ delta_x, delta_y };
}

DeltaPosition calc_global_delta_position(DeltaPosition local_delta_pos, double heading, double delta_heading)
{
  if (fabs(delta_heading) < 1e-6)
  {
    return calc_global_delta_position_by_linear_movement(local_delta_pos, heading, delta_heading);
  }

  return calc_global_delta_position_by_arc_movement(local_delta_pos, heading, delta_heading);
}
}

rover0_controller::WheelOdometry::WheelOdometry(
    double wheel_radius,
    double wheel_base,
    double track_width,
    size_t rolling_window_size
)
    : mecanum_kinematics_{rover0_controller::mecanum_kinematics::MecanumKinematics{wheel_radius, wheel_base, track_width}},
      x_{0.0},
      y_{0.0},
      heading_{0.0},
      linear_x_accumulator_{rcpputils::RollingMeanAccumulator<double>{rolling_window_size}},
      linear_y_accumulator_{rcpputils::RollingMeanAccumulator<double>{rolling_window_size}},
      angular_z_accumulator_{rcpputils::RollingMeanAccumulator<double>{rolling_window_size}}
{
}

bool rover0_controller::WheelOdometry::update(
  double front_left_wheel_angular_velocity,
  double front_right_wheel_angular_velocity,
  double rear_left_wheel_angular_velocity,
  double rear_right_wheel_angular_velocity,
  double dt
)
{
  rover0_controller::mecanum_kinematics::Twist const twist = mecanum_kinematics_.forward(
    rover0_controller::mecanum_kinematics::WheelAngularVelocity{ front_left_wheel_angular_velocity,
                                                                 front_right_wheel_angular_velocity,
                                                                 rear_left_wheel_angular_velocity,
                                                                 rear_right_wheel_angular_velocity }
  );
  double const        delta_heading   = twist.angular_z * dt;
  DeltaPosition const local_delta_pos = DeltaPosition{ twist.linear_x * dt, twist.linear_y * dt };
  DeltaPosition const delta_pos       = calc_global_delta_position(local_delta_pos, heading_, delta_heading);

  x_       += delta_pos.x;
  y_       += delta_pos.y;
  heading_ += delta_heading;
  linear_x_accumulator_.accumulate(twist.linear_x);
  linear_y_accumulator_.accumulate(twist.linear_y);
  angular_z_accumulator_.accumulate(twist.angular_z);
  return true;
}

double rover0_controller::WheelOdometry::getX() const
{
  return x_;
}
double rover0_controller::WheelOdometry::getY() const
{
  return y_;
}
double rover0_controller::WheelOdometry::getHeading() const
{
  return heading_;
}
double rover0_controller::WheelOdometry::getLinearX() const
{
  return linear_x_accumulator_.getRollingMean();
}
double rover0_controller::WheelOdometry::getLinearY() const
{
  return linear_y_accumulator_.getRollingMean();
}
double rover0_controller::WheelOdometry::getAngular() const
{
  return angular_z_accumulator_.getRollingMean();
}
