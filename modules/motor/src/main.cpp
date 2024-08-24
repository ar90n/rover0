#include <string_view>

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "calibrate.hpp"
#include "comm_proc.hpp"
#include "logger.hpp"
#include "main_proc.hpp"

void printf_write(std::string_view msg)
{
  printf("%s", msg.data());
}

int main()
{
  stdio_init_all();
#ifndef NDEBUG
  logger::init(printf_write);
#endif

  if (calibrate::should_calibrate()) {
    logger::init(printf_write);
    calibrate::calibrate();
    return 0;
  }

  // There isn't a strict rule that you must always call sleep_ms() before calling multicore_launch_core1
  sleep_ms(1000);
  multicore_launch_core1(comm_proc::run);

  main_proc::run();
  return 0;
}
