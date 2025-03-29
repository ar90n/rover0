#include "config.hpp"
#include "gpio.hpp"
#include "pico/async_context.h"
#include "pico/async_context_poll.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "task.hpp"
#include <stdio.h>

namespace {
using GpioLED = Gpio<Config::LED_PIN>;
auto gpio_led = GpioLED::instance();

void heartbeat_task()
{
  static bool led_state{ false };
  gpio_led.write(led_state);
  led_state = !led_state;
}

void comm_entry()
{
  multicore_fifo_clear_irq();

  async_context_poll_t async_context;
  async_context_poll_init_with_defaults(&async_context);

  invoke_as_task<500>(async_context, heartbeat_task);

  poll(async_context);
}

} // namespace

extern "C" void comm_main()
{
  comm_entry();
}
