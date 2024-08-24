#pragma once

#include "config.hpp"
#include "encoder.hpp"
#include "gpio.hpp"
#include "imu.hpp"
#include "motor.hpp"

namespace device {
using GpioLED                 = Gpio<Config::LED_PIN>;
using GpioESW                 = Gpio<Config::ESW_PIN, GPIO_FUNC_SIO, GPIO_IN>;
using RearLeftRotaryEncoder   = RotaryEncoder<Config::REAR_LEFT_ENCODER_PIN, PIO0_BASE, 0>;
using RearRightRotaryEncoder  = RotaryEncoder<Config::REAR_RIGHT_ENCODER_PIN, PIO0_BASE, 1>;
using FrontLeftRotaryEncoder  = RotaryEncoder<Config::FRONT_LEFT_ENCODER_PIN, PIO0_BASE, 2>;
using FrontRightRotaryEncoder = RotaryEncoder<Config::FRONT_RIGHT_ENCODER_PIN, PIO0_BASE, 3>;
using RearLeftMotor =
  Motor<Config::REAR_LEFT_FORWARD_PIN, Config::REAR_LEFT_BACKWARD_PIN, RearLeftRotaryEncoder>;
using RearRightMotor =
  Motor<Config::REAR_RIGHT_FORWARD_PIN, Config::REAR_RIGHT_BACKWARD_PIN, RearRightRotaryEncoder>;
using FrontLeftMotor =
  Motor<Config::FRONT_LEFT_FORWARD_PIN, Config::FRONT_LEFT_BACKWARD_PIN, FrontLeftRotaryEncoder>;
using FrontRightMotor =
  Motor<Config::FRONT_RIGHT_FORWARD_PIN, Config::FRONT_RIGHT_BACKWARD_PIN, FrontRightRotaryEncoder>;
using IMU = imu::IMU_6050<Config::I2C_IMU, Config::I2C_IMU_SDA, Config::I2C_IMU_SCL>;

} // namespace device