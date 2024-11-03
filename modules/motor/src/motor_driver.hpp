#pragma once

#include <cstdint>

#include "mamePID.hpp"

class MotorDriver
{
public:
  MotorDriver(float pid_kp, float pid_ki, float pid_kd, uint32_t interval_msg);
  ~MotorDriver() = default;

  void    update(int16_t delta_tics);
  void    set_target_tics_per_sec(float tics_per_sec);
  float   drive_power() const;
  int16_t accumulated_tics() const;
  void    clear_accumulated_tics();

  void emergency();
  void release();

private:
  using PIDType = decltype(mamePID::pi_d<float>(0.0, 0.0, 0.0, 0.0));

  uint32_t const interval_ms;
  PIDType        motor_pid;
  bool           is_emergency{ false };
  float          drive_power_{ 0 };
  float          target_tics_per_sec{ 0 };
  float          estimated_tics_per_sec{ 0 };
  int16_t        accumulated_tics_{ 0 };

  float estimate_tics_per_sec(int16_t delta_tics) const;
};