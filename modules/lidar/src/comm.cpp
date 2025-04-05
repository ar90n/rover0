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
    size_t const   base_index{ (4 * packet.angle_quad) % 360 };
    uint32_t const words[]{
      (static_cast<uint32_t>(base_index + 0) << 16) | (packet.distances[0] & 0xFFFF),
      (static_cast<uint32_t>(base_index + 1) << 16) | (packet.distances[1] & 0xFFFF),
      (static_cast<uint32_t>(base_index + 2) << 16) | (packet.distances[2] & 0xFFFF),
      (static_cast<uint32_t>(base_index + 3) << 16) | (packet.distances[3] & 0xFFFF),
      (0xFFFF0000 | (static_cast<uint32_t>(packet.timestamp_us) & 0xFFFF)),
    };

    for (auto const& word : words)
    {
      multicore_fifo_push_blocking(word);
    }
  }
}

void comm_entry()
{
  multicore_fifo_clear_irq();

  uart_lidar.init(115200);

  TaskRunner{ create_scheduled_task<500>(heartbeat_task),
              create_scheduled_task<1>(lidar_fetch_task),
              create_scheduled_task<10>(lidar_motor_control_task) }
    .run_forever(0);
}

} // namespace

extern "C" void comm_main()
{
  comm_entry();
}
