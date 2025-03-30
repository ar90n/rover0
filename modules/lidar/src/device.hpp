#pragma once

#include "buffered_intercore_fifo.hpp"
#include "config.hpp"
#include "gpio.hpp"
#include "uart.hpp"

namespace device {
using UartControl = Uart<
  Config::UART_LIDAR,
  Config::UART_CONTROL_TX_PIN,
  Config::UART_CONTROL_RX_PIN,
  Config::UART_BUFFER_SIZE>;
using UartLidar =
  Uart<Config::UART_CONTROL, Config::UART_LIDAR_RX_PIN, Config::UART_LIDAR_RX_PIN, Config::UART_BUFFER_SIZE>;
using GpioPWM = Gpio<Config::PWM_PIN, GPIO_FUNC_PWM>;
using Core0IntercoreFIFO = BufferedIntercoreFIFO<0, Config::INTERCORE_FIOF_SIZE>;
}