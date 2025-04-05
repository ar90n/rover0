#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <time.h>

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
  do                                                                                                         \
  {                                                                                                          \
    rcl_ret_t temp_rc = fn;                                                                                  \
    if ((temp_rc != RCL_RET_OK))                                                                             \
    {                                                                                                        \
      error_loop();                                                                                          \
    }                                                                                                        \
  } while (0)
#define RCSOFTCHECK(fn)                                                                                      \
  do                                                                                                         \
  {                                                                                                          \
    rcl_ret_t temp_rc = fn;                                                                                  \
    if ((temp_rc != RCL_RET_OK))                                                                             \
    {                                                                                                        \
      /* Log or handle error if needed in the future */                                                      \
    }                                                                                                        \
  } while (0)

using uRosTransport = UartTransport<device::UartControl>;

auto uart_ctrl      = device::UartControl::instance();
auto intercore_fifo = device::Core0IntercoreFIFO::instance();

namespace
{
float      distances[360]    = { 0 };
float      intensities[360]  = { 0 };
uint32_t   last_timestamp_us = 0;
static int sec               = 0;
float      duration          = 0;

// Function to receive LiDAR data from Core1 via FIFO
void lidar_receive_task(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer == NULL)
  {
    return;
  }

  while (intercore_fifo.has_data())
  {
    uint32_t const word  = intercore_fifo.read();
    uint16_t const index = (word >> 16) & 0xFFFF;
    uint16_t const value = word & 0xFFFF;

    constexpr float const dst_scale = 1.0 / 1000.0f;
    if (0 <= index && index < 360)
    {
      distances[index] = value * dst_scale;
    }
    else if (index == 0xFFFF) // 0xFFFF means timestamp
    {
      uint32_t const timestamp_us = value;
      if (last_timestamp_us > 0)
      {
        duration = (timestamp_us - last_timestamp_us) / 1000000.0 / 4.0;
      }
      last_timestamp_us = timestamp_us;
    }
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

  if (timer == NULL)
  {
    return;
  }

  rosidl_runtime_c__String__assign(&msg.header.frame_id, "rover0_lidar_link");
  int64_t time_ns = rmw_uros_epoch_nanos();
  msg.header.stamp.sec = (int32_t)(time_ns / 1000000000);
  msg.header.stamp.nanosec = (int32_t)(time_ns % 1000000000);
  msg.ranges.data          = range_data;
  msg.ranges.capacity      = 360;
  msg.ranges.size          = 360;
  msg.intensities.data     = intensity_data;
  msg.intensities.capacity = 360;
  msg.intensities.size     = 360;
  msg.angle_min            = -std::numbers::pi;
  msg.angle_max            = std::numbers::pi;
  msg.angle_increment      = (msg.angle_max - msg.angle_min) / (360 - 1);
  msg.time_increment       = duration;
  msg.scan_time            = msg.time_increment * 360;
  msg.range_min            = 0.15;
  msg.range_max            = 6.0;

  for (int i = 0; i < 360; i++)
  {
    msg.ranges.data[i] = distances[i];
  }

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}
} // namespace

// Error handle loop
void error_loop()
{
  while (1)
  {
    sleep_ms(100);
  }
}

int main()
{
  // Launch Core1 for LiDAR control
  sleep_ms(1000);
  multicore_launch_core1(comm_main);

  intercore_fifo.init();

  uRosTransport trns(uart_ctrl, 230400);
  rmw_uros_set_custom_transport(
    true,
    &trns,
    uRosTransport::open,
    uRosTransport::close,
    uRosTransport::write,
    uRosTransport::read
  );

  // Wait for agent successful ping for 2 minutes.
  auto ret = rmw_uros_ping_agent(Config::UROS_TIMEOUT_MS, Config::UROS_ATTEMPTS);

  // create init_options
  rcl_allocator_t allocator{ rcl_get_default_allocator() };
  auto            init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  if (0 < Config::ROS_DOMAIN_ID)
  {
    RCCHECK(rcl_init_options_set_domain_id(&init_options, static_cast<size_t>(Config::ROS_DOMAIN_ID)));
  }

  rclc_support_t support;
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RCCHECK(rmw_uros_sync_session(1000));

  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "lidar", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan"
  ));

  // create publish_timer,
  rcl_timer_t        publish_timer;
  const unsigned int publish_timer_timeout = 60.0 / Config::LIDAR_RPM * 1000;
  RCCHECK(rclc_timer_init_default(
    &publish_timer,
    &support,
    RCL_MS_TO_NS(publish_timer_timeout),
    publish_timer_callback
  ));

  // create lidar_receive_timer
  rcl_timer_t        lidar_receive_timer;
  const unsigned int lidar_receive_timeout = 200;
  RCCHECK(rclc_timer_init_default(
    &lidar_receive_timer,
    &support,
    RCL_US_TO_NS(lidar_receive_timeout),
    lidar_receive_task
  ));

  // create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &lidar_receive_timer));

  rclc_executor_spin(&executor);

  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_node_fini(&node));

  return 0;
}
