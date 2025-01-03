#include <cmath>

#include "rover0_controller/mecanum_kinematics.hpp"

namespace rover0_controller::mecanum_kinematics
{

MecanumKinematics::MecanumKinematics(double wheel_radius, double wheel_base, double track_width)
    : wheel_radius_(wheel_radius),
      wheel_base_(wheel_base),
      track_width_(track_width)
{
}

WheelAngularVelocity MecanumKinematics::inverse(Twist const& twist) const
{
    double const r = (wheel_base_ + track_width_) / 2;
    double const front_left = (twist.linear_x - twist.linear_y - r * twist.angular_z) / wheel_radius_;
    double const front_right = (twist.linear_x + twist.linear_y + r * twist.angular_z) / wheel_radius_;
    double const rear_left = (twist.linear_x + twist.linear_y - r * twist.angular_z) / wheel_radius_;
    double const rear_right = (twist.linear_x - twist.linear_y + r * twist.angular_z) / wheel_radius_;
    return WheelAngularVelocity{
        front_left, front_right, rear_left, rear_right
    };
}

Twist MecanumKinematics::forward(WheelAngularVelocity const& wheel_velocities) const
{
    double const r = (wheel_base_ + track_width_) / 2;
    double const linear_x =  0.25 * wheel_radius_ * (wheel_velocities.front_left +
                                              wheel_velocities.front_right +
                                              wheel_velocities.rear_left +
                                              wheel_velocities.rear_right);
    double const linear_y =  0.25 * wheel_radius_ * (-wheel_velocities.front_left +
                                              wheel_velocities.front_right +
                                              wheel_velocities.rear_left -
                                              wheel_velocities.rear_right);
    double const angular_z = 0.25 * wheel_radius_ * (-wheel_velocities.front_left +
                                              wheel_velocities.front_right -
                                              wheel_velocities.rear_left +
                                              wheel_velocities.rear_right) / r;
    return Twist{
        linear_x,
        linear_y,
        angular_z
    };
}

} // namespace mecanum_kinematics
