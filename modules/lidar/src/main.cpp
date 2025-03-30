#include <time.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>

#include <pico/multicore.h>
#include <pico/stdlib.h>

#include "buffered_intercore_fifo.hpp"
#include "config.hpp"
#include "device.hpp"
#include "gpio.hpp"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "transport.hpp"
#include "uart.hpp"
#include "xv11lidar.h"

#ifdef __cplusplus
extern "C"
{
#endif

  int clock_gettime(clockid_t unused, struct timespec* tp)
  {
    uint64_t m  = time_us_64();
    tp->tv_sec  = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
  }

#ifdef __cplusplus
}
#endif

#define RCCHECK(fn)                                                                                          \
  do {                                                                                                       \
    rcl_ret_t temp_rc = fn;                                                                                  \
    if ((temp_rc != RCL_RET_OK)) {                                                                           \
      error_loop();                                                                                          \
    }                                                                                                        \
  } while (0)
#define RCSOFTCHECK(fn)                                                                                      \
  do {                                                                                                       \
    rcl_ret_t temp_rc = fn;                                                                                  \
    if ((temp_rc != RCL_RET_OK)) {                                                                           \
      /* Log or handle error if needed in the future */                                                      \
    }                                                                                                        \
  } while (0)

using uRosTransport = UartTransport<device::UartControl>;

auto uart_ctrl      = device::UartControl::instance();
auto intercore_fifo = device::Core0IntercoreFIFO::instance();

namespace {
float      distances[360];
float      intensities[360];
uint32_t   last_timestamp_us = 0;
static int sec               = 0;
float      duration          = 0;

// Function to receive LiDAR data from Core1 via FIFO
void lidar_receive_task(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer == NULL) {
    return;
  }

  // Check if there's data in the FIFO
  while (6 <= intercore_fifo.size()) {
    // Read the first word: angle_quad (16 bits) + first distance (16 bits)
    uint32_t word1      = intercore_fifo.read();
    uint16_t angle_quad = word1 >> 16;
    uint16_t distance1  = word1 & 0xFFFF;

    // Read the second word: second distance (16 bits) + third distance (16 bits)
    uint32_t word2     = intercore_fifo.read();
    uint16_t distance2 = word2 >> 16;
    uint16_t distance3 = word2 & 0xFFFF;

    // Read the third word: fourth distance (16 bits) + first intensity (16 bits)
    uint32_t word3      = intercore_fifo.read();
    uint16_t distance4  = word3 >> 16;
    uint16_t intensity1 = word3 & 0xFFFF;

    // Read the fourth word: second intensity (16 bits) + third intensity (16 bits)
    uint32_t word4      = intercore_fifo.read();
    uint16_t intensity2 = word4 >> 16;
    uint16_t intensity3 = word4 & 0xFFFF;

    // Read the fifth word: fourth intensity (16 bits)
    uint32_t word5      = intercore_fifo.read();
    uint16_t intensity4 = word5 >> 16;

    // Read the sixth word: timestamp (32 bits)
    uint32_t word6        = intercore_fifo.read();
    uint32_t timestamp_us = word6;

    // Calculate offset in the arrays
    size_t const offset        = (4 * angle_quad) % 360;
    float const  distanceScale = 1.0 / 1000.0f;

    // Store the data in the arrays
    distances[offset]     = distanceScale * distance1;
    distances[offset + 1] = distanceScale * distance2;
    distances[offset + 2] = distanceScale * distance3;
    distances[offset + 3] = distanceScale * distance4;

    intensities[offset]     = intensity1;
    intensities[offset + 1] = intensity2;
    intensities[offset + 2] = intensity3;
    intensities[offset + 3] = intensity4;

    // Calculate duration for time_increment
    if (last_timestamp_us > 0) {
      duration = (timestamp_us - last_timestamp_us) / 1000000.0 / 4.0;
    }
    last_timestamp_us = timestamp_us;
  }
}

// Core1 entry function declaration (defined in comm.cpp)
extern "C" void comm_main();

rcl_publisher_t             publisher;
sensor_msgs__msg__LaserScan msg;
float                       range_data[360];
float                       intensity_data[360];
void                        publish_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer == NULL) {
    return;
  }

  rosidl_runtime_c__String__assign(&msg.header.frame_id, "lidar");
  msg.ranges.data          = range_data;
  msg.ranges.capacity      = 360;
  msg.ranges.size          = 360;
  msg.intensities.data     = intensity_data;
  msg.intensities.capacity = 360;
  msg.intensities.size     = 360;
  msg.header.stamp.sec     = sec++;
  msg.angle_min            = 3.14 / 1;
  msg.angle_max            = -3.14 / 1;
  msg.angle_increment      = -2 * 3.14 / 360;
  msg.time_increment       = duration;
  msg.scan_time            = msg.time_increment * 360;
  msg.range_min            = 0.15;
  msg.range_max            = 6.0;

  for (int i = 0; i < 360; i++) {
    msg.ranges.data[i]      = distances[i];
    msg.intensities.data[i] = intensities[i];
  }

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}
} // namespace

// Error handle loop
void error_loop()
{
  while (1) {
    sleep_ms(100);
  }
}

extern Gpio<Config::LED_PIN> gpio_led;

int main()
{
  intercore_fifo.init();

  // Launch Core1 for LiDAR control
  sleep_ms(1000);
  multicore_launch_core1(comm_main);

  uRosTransport trns(uart_ctrl, 230400);
  rmw_uros_set_custom_transport(
    true, &trns, uRosTransport::open, uRosTransport::close, uRosTransport::write, uRosTransport::read
  );

  // Wait for agent successful ping for 2 minutes.
  auto ret = rmw_uros_ping_agent(Config::UROS_TIMEOUT_MS, Config::UROS_ATTEMPTS);

  // create init_options
  rcl_allocator_t allocator{ rcl_get_default_allocator() };
  auto            init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  if (0 < Config::ROS_DOMAIN_ID) {
    RCCHECK(rcl_init_options_set_domain_id(&init_options, static_cast<size_t>(Config::ROS_DOMAIN_ID)));
  }

  rclc_support_t support;
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "lidar", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"
  ));

  // create publish_timer,
  rcl_timer_t        publish_timer;
  const unsigned int publish_timer_timeout = 60.0 / Config::LIDAR_RPM * 1000;
  RCCHECK(rclc_timer_init_default(
    &publish_timer, &support, RCL_MS_TO_NS(publish_timer_timeout), publish_timer_callback
  ));

  // create lidar_receive_timer
  rcl_timer_t        lidar_receive_timer;
  const unsigned int lidar_receive_timeout = 5; // Check more frequently for FIFO data
  RCCHECK(rclc_timer_init_default(
    &lidar_receive_timer, &support, RCL_MS_TO_NS(lidar_receive_timeout), lidar_receive_task
  ));

  // create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &lidar_receive_timer));

  while (true) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  }
  return 0;
}
