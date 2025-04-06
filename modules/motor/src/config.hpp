#pragma once

#include "pico/binary_info.h"

struct Config
{
  static constexpr uint  LED_PIN                          = 25;
  static constexpr uint  ESW_PIN                          = 28;
  static constexpr uint  REAR_LEFT_FORWARD_PIN            = 4;
  static constexpr uint  REAR_LEFT_BACKWARD_PIN           = 5;
  static constexpr uint  REAR_LEFT_ENCODER_PIN            = 0;
  static constexpr uint  REAR_RIGHT_FORWARD_PIN           = 2;
  static constexpr uint  REAR_RIGHT_BACKWARD_PIN          = 3;
  static constexpr uint  REAR_RIGHT_ENCODER_PIN           = 1;
  static constexpr uint  FRONT_LEFT_FORWARD_PIN           = 10;
  static constexpr uint  FRONT_LEFT_BACKWARD_PIN          = 11;
  static constexpr uint  FRONT_LEFT_ENCODER_PIN           = 15;
  static constexpr uint  FRONT_RIGHT_FORWARD_PIN          = 12;
  static constexpr uint  FRONT_RIGHT_BACKWARD_PIN         = 13;
  static constexpr uint  FRONT_RIGHT_ENCODER_PIN          = 14;
  static constexpr uint  UART_CONTROL                     = 0;
  static constexpr uint  UART_CONTROL_TX_PIN              = 16;
  static constexpr uint  UART_CONTROL_RX_PIN              = 17;
  static constexpr uint  UART_BUFFER_SIZE                 = 1024;
  static constexpr uint  I2C_IMU                          = 0;
  static constexpr uint  I2C_IMU_SDA                      = 20;
  static constexpr uint  I2C_IMU_SCL                      = 21;
  static constexpr uint  I2C_IMU_BAUDRATE                 = 400 * 1000;
  static constexpr uint  EMERGENCY_WORKER_INTERVAL_MS     = 10;
  static constexpr uint  LED_HEARTBEAT_WORKER_INTERVAL_MS = 500;
  static constexpr uint  MOTOR_CONTROL_WORKER_INTERVAL_MS = 10;
  static constexpr uint  IMU_CONTROL_WORKER_INTERVAL_MS   = 20;
  static constexpr uint  INTERCORE_FIFO_SIZE              = 1024;
  static constexpr uint  QUEUE_SIZE                       = 1024;
  static constexpr float MOTOR_DRIVER_PID_KP              = 0.2;
  static constexpr float MOTOR_DRIVER_PID_KI              = 0.005;
  static constexpr float MOTOR_DRIVER_PID_KD              = 0.030;
};
