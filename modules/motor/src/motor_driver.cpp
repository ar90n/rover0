#include "motor_driver.hpp"

namespace {
static constexpr float MAX_TICS_PER_SEC{ 480.0f };
static constexpr float PID_OUTPUT_RESCALE_COEFF{ 1.0f / MAX_TICS_PER_SEC };
}

MotorDriver::MotorDriver(float pid_kp, float pid_ki, float pid_kd, uint32_t interval_ms)
  : interval_ms(interval_ms)
  , motor_pid(mamePID::pi_d<
              float>(pid_kp, pid_ki, pid_kd, static_cast<float>(interval_ms) / 1000.0, -MAX_TICS_PER_SEC, MAX_TICS_PER_SEC))
{
}

void MotorDriver::update(int16_t delta_tics)
{
  if (is_emergency) {
    return;
  }

  accumulated_tics_      += delta_tics;
  estimated_tics_per_sec  = estimate_tics_per_sec(delta_tics);

  drive_power_ += PID_OUTPUT_RESCALE_COEFF * motor_pid.calculate(target_tics_per_sec, estimated_tics_per_sec);
  drive_power_  = std::clamp(drive_power_, -1.0f, 1.0f);
}

void MotorDriver::set_target_tics_per_sec(float tics_per_sec)
{
  target_tics_per_sec = tics_per_sec;
}

float MotorDriver::drive_power() const
{
  if (is_emergency) {
    return 0;
  }

  return drive_power_;
}

int16_t MotorDriver::accumulated_tics() const
{
  return accumulated_tics_;
}

void MotorDriver::clear_accumulated_tics()
{
  accumulated_tics_ = 0;
}

void MotorDriver::emergency()
{
  is_emergency = true;
  drive_power_ = 0;
  motor_pid.reset();
}

void MotorDriver::release()
{
  is_emergency = false;
}

float MotorDriver::estimate_tics_per_sec(int16_t delta_tics) const
{
  static constexpr float estimated_weight{ 7.0 };
  static constexpr float cur_weight{ 1.0 };
  static constexpr float total_weight{ estimated_weight + cur_weight };

  float const cur_tics_per_sec{ 1000.0f * static_cast<float>(delta_tics) / static_cast<float>(interval_ms) };
  return (estimated_weight * estimated_tics_per_sec + cur_weight * cur_tics_per_sec) / total_weight;
}