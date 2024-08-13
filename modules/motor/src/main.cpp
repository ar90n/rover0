#include <string_view>

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "comm_proc.hpp"
#include "logger.hpp"
#include "main_proc.hpp"

void printf_write(std::string_view msg)
{
  printf("%s", msg.data());
}

int main()
{
//  stdio_init_all();
//#ifndef NDEBUG
//  logger::init(printf_write);
//#endif

  // There isn't a strict rule that you must always call sleep_ms() before calling multicore_launch_core1
  sleep_ms(1000);
  multicore_launch_core1(comm_proc::run);

  main_proc::run();
  return 0;
}

// 200Hz
// * 10 = 2000Hz  (10 pt per cycle)
// * 5 = 10000Hz (5 inst per pt)kkkkkkkkk

// 125000000 / 10000 = 12500

// 255 / 10 = 25.5 (cycle per capture)
// 25.5 * 2 = 51 (edges per capture)
// 255 - 51 = 204 (cycle per pwm)
// 204 as hex = 0xCC

//  10000 / 20 = 500 (cycle per capture)
// / 5 = 100 (edges per capture)

// 255 / 100 = 2.55 (cycle per capture)
// 2.55 * 2 = 5.1 (edges per capture)
// 255 - 5.1 = 249 (cycle per pwm)
// 249 as hex = 0xF9

// 200Hz
// * 20 = 4000Hz  (20 pt per cycle)
// * 5 = 20000Hz (5 inst per pt)

// 125000000 / 20000 = 6250

// 255 / 20 = 12.75 (cycle per capture)
// 12.75 * 2 = 25.5 (edges per capture)
// 255 - 25.5 = 204 (cycle per pwm)
// 204 as hex = 0xCC

//  10000 / 20 = 500 (cycle per capture)
// / 5 = 100 (edges per capture)

// 255 / 100 = 2.55 (cycle per capture)
// 2.55 * 2 = 5.1 (edges per capture)
// 255 - 5.1 = 249 (cycle per pwm)
// 249 as hex = 0xF9

// 256 / 12500 = 0.02048
// 1 / 0.02048 = 48.828125

// imu pos 46mm 13mm

// void main2() {
//     logger::log("Entered core1 (core={})\r\n", get_core_num());
//
//     while(1) {
//         logger::log("ping (core={})\r\n", get_core_num());
//         sleep_ms(1000);
//     }
// }
//
// int main() {
//   UartControl::instance().init(115200);
//   logger::init(uart_write);
//
//     // There isn't a strict rule that you must always call sleep_ms()
//     // after stdio_init_all(). However, in some cases, it can be a helpful
//     // precautionary measure to ensure that the UART has properly
//     // initialized and is ready to transmit data without any issues.
//     sleep_ms(100);
//
//     logger::log("Entered core0 (core={})\r\n", get_core_num());
//
//     multicore_launch_core1(core1_entry);
//
//     while(1) {
//         logger::log("ping (core={})\r\n", get_core_num());
//         sleep_ms(1000);
//     }
// }