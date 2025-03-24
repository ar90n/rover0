#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/string.h>

#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>

#include "gpio.hpp"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "transport.hpp"
#include "uart.hpp"
#include "xv11lidar.h"
#include <pico/stdlib.h>

#include "pico/stdlib.h"
#include <time.h>

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
    }                                                                                                        \
  } while (0)

struct Config
{
  static constexpr uint LED_PIN             = 25;
  static constexpr uint PWM_PIN             = 7;
  static constexpr uint UART_LIDAR          = 1;
  static constexpr uint UART_LIDAR_TX_PIN   = 0;
  static constexpr uint UART_LIDAR_RX_PIN   = 1;
  static constexpr uint UART_CONTROL        = 0;
  static constexpr uint UART_CONTROL_TX_PIN = 4;
  static constexpr uint UART_CONTROL_RX_PIN = 5;
  static constexpr uint UART_BUFFER_SIZE    = 1024;
  static constexpr uint ROS_DOMAIN_ID       = 104;
  static constexpr uint LIDAR_RPM           = 270;
  static constexpr uint UROS_TIMEOUT_MS     = 1000;
  static constexpr uint UROS_ATTEMPTS       = 120;
};

using UartControl = Uart<
  Config::UART_LIDAR,
  Config::UART_CONTROL_TX_PIN,
  Config::UART_CONTROL_RX_PIN,
  Config::UART_BUFFER_SIZE>;
using UartLidar =
  Uart<Config::UART_CONTROL, Config::UART_LIDAR_RX_PIN, Config::UART_LIDAR_RX_PIN, Config::UART_BUFFER_SIZE>;
using GpioLED       = Gpio<Config::LED_PIN>;
using GpioPWM       = Gpio<Config::PWM_PIN, GPIO_FUNC_PWM>;
using uRosTransport = UartTransport<UartControl>;

auto uart_lidar = UartLidar::instance();
auto uart_ctrl  = UartControl::instance();
auto gpio_led   = GpioLED::instance();
auto gpio_pwm   = GpioPWM::instance();

namespace {
float      distances[360];
float      intensities[360];
uint32_t   distances_index   = 0;
uint32_t   last_timestamp_us = 0;
static int sec               = 0;
float      duration          = 0;

xv11::ReturnType read_byte_from_serial()
{
  if (!uart_lidar.has_data()) {
    return std::make_pair(false, static_cast<uint8_t>(0));
  }
  return std::make_pair(true, uart_lidar.read());
}

void write_pwm_value(float pwm)
{
  gpio_pwm.write(pwm);
}

xv11::Lidar lidar(read_byte_from_serial, write_pwm_value, time_us_32, Config::LIDAR_RPM);

void fetch_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer == NULL) {
    return;
  }

  xv11::DataPacket packet;
  while (lidar.process(&packet)) {
    size_t const offset        = 4 * packet.angle_quad;
    float const  distanceScale = 1.0 / 1000.0f;
    for (int i = 0; i < 4; i++) {
      distances[offset + i]   = distanceScale * packet.distances[i];
      intensities[offset + i] = packet.signals[i];
    }

    duration          = (packet.timestamp_us - last_timestamp_us) / 1000000.0 / 4.0;
    last_timestamp_us = packet.timestamp_us;
  }
}

void pwm_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer == NULL) {
    return;
  }

  lidar.apply_motor_pid();
}

void heartbeat_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) {
    return;
  }

  static bool led_state = false;
  gpio_led.write(led_state);
  led_state = !led_state;
}

rcl_publisher_t             publisher;
sensor_msgs__msg__LaserScan msg;
rcl_publisher_t             publisher2;
std_msgs__msg__String       msg2;
float                       range_data[360];
float                       intensity_data[360];
void                        publish_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer == NULL) {
    return;
  }
  auto oo1 = uart_lidar.get_queue_size();

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

int main()
{
  uart_lidar.init(115200);

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

  // create fetch_timer,
  rcl_timer_t        fetch_timer;
  const unsigned int fetch_timer_timeout = 1;
  RCCHECK(
    rclc_timer_init_default(&fetch_timer, &support, RCL_MS_TO_NS(fetch_timer_timeout), fetch_timer_callback)
  );

  // create pwm_timer,
  rcl_timer_t        pwm_timer;
  const unsigned int pwm_timer_timeout = 25;
  RCCHECK(rclc_timer_init_default(&pwm_timer, &support, RCL_MS_TO_NS(pwm_timer_timeout), pwm_timer_callback));

  // create heartbeat_timer,
  rcl_timer_t        heartbeat_timer;
  const unsigned int heartbeat_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &heartbeat_timer, &support, RCL_MS_TO_NS(heartbeat_timeout), heartbeat_timer_callback
  ));

  // create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &fetch_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &pwm_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &heartbeat_timer));

  gpio_pwm.write(1.0f);
  while (true) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));
  }
  return 0;
}