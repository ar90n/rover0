#include "config.hpp"
#include "gpio.hpp"
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

  TaskContext task_context;
  
  schedule_task<500>(task_context, heartbeat_task);

  run_task_loop(task_context, 10); // Explicitly set 10ms poll interval
}

} // namespace

extern "C" void comm_main()
{
  comm_entry();
}
