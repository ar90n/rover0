#include "config.hpp"
#include "gpio.hpp"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "task.hpp"
#include "uart.hpp"
#include "xv11lidar.h"

namespace
{
using GpioPWM = Gpio<Config::PWM_PIN, GPIO_FUNC_PWM>;
using GpioLED = Gpio<Config::LED_PIN>;
using UartLidar =
  Uart<Config::UART_CONTROL, Config::UART_LIDAR_RX_PIN, Config::UART_LIDAR_RX_PIN, Config::UART_BUFFER_SIZE>;

auto gpio_led   = GpioLED::instance();
auto gpio_pwm   = GpioPWM::instance();
auto uart_lidar = UartLidar::instance();

// LiDAR data processing functions
xv11::ReturnType read_byte_from_serial()
{
  if (!uart_lidar.has_data())
  {
    return std::make_pair(false, static_cast<uint8_t>(0));
  }
  return std::make_pair(true, uart_lidar.read());
}

void write_pwm_value(float pwm)
{
  gpio_pwm.write(pwm);
}

xv11::Lidar lidar(read_byte_from_serial, write_pwm_value, time_us_32, Config::LIDAR_RPM);

void heartbeat_task()
{
  static bool led_state{ false };
  gpio_led.write(led_state);
  led_state = !led_state;
}

void lidar_motor_control_task()
{
  lidar.apply_motor_pid();
}

void lidar_fetch_task()
{
  xv11::DataPacket packet;
  while (lidar.process(&packet))
  {
    // Send data to Core0 via FIFO
    // First word: angle_quad (16 bits) + first distance (16 bits)
    uint32_t word1 = (static_cast<uint32_t>(packet.angle_quad) << 16) | (packet.distances[0] & 0xFFFF);
    multicore_fifo_push_blocking(word1);

    // Second word: second distance (16 bits) + third distance (16 bits)
    uint32_t word2 = (static_cast<uint32_t>(packet.distances[1]) << 16) | (packet.distances[2] & 0xFFFF);
    multicore_fifo_push_blocking(word2);

    // Third word: fourth distance (16 bits) + first intensity (16 bits)
    uint32_t word3 = (static_cast<uint32_t>(packet.distances[3]) << 16) | (packet.signals[0] & 0xFFFF);
    multicore_fifo_push_blocking(word3);

    // Fourth word: second intensity (16 bits) + third intensity (16 bits)
    uint32_t word4 = (static_cast<uint32_t>(packet.signals[1]) << 16) | (packet.signals[2] & 0xFFFF);
    multicore_fifo_push_blocking(word4);

    // Fifth word: fourth intensity (16 bits)
    uint32_t word5 = (static_cast<uint32_t>(packet.signals[3]) << 16);
    multicore_fifo_push_blocking(word5);

    // Sixth word: timestamp (32 bits)
    uint32_t word6 = packet.timestamp_us;
    multicore_fifo_push_blocking(word6);
  }
}

void comm_entry()
{
  multicore_fifo_clear_irq();

  uart_lidar.init(115200);

  TaskRunner{ create_scheduled_task<500>(heartbeat_task),
              create_scheduled_task<10>(lidar_fetch_task),
              create_scheduled_task<50>(lidar_motor_control_task) }
    .run_forever();
}

} // namespace

extern "C" void comm_main()
{
  comm_entry();
}
