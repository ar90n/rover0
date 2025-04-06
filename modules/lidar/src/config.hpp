#pragma once

#include <cstdint>

struct Config
{
  static constexpr uint32_t LED_PIN             = 25;
  static constexpr uint32_t PWM_PIN             = 7;
  static constexpr uint32_t UART_LIDAR          = 1;
  static constexpr uint32_t UART_LIDAR_TX_PIN   = 0;
  static constexpr uint32_t UART_LIDAR_RX_PIN   = 1;
  static constexpr uint32_t UART_CONTROL        = 0;
  static constexpr uint32_t UART_CONTROL_TX_PIN = 4;
  static constexpr uint32_t UART_CONTROL_RX_PIN = 5;
  static constexpr uint32_t UART_BUFFER_SIZE    = 4096;
  static constexpr uint32_t INTERCORE_FIOF_SIZE = 1024;
  static constexpr uint32_t ROS_DOMAIN_ID       = 104;
  static constexpr uint32_t LIDAR_RPM           = 270;
  static constexpr uint32_t UROS_TIMEOUT_MS     = 1000;
  static constexpr uint32_t UROS_ATTEMPTS       = 120;
};
