#include <atomic>
#include <format>
#include <variant>

#include "hardware/irq.h"
#include "pico/async_context_poll.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "config.hpp"
#include "device.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "motor_driver.hpp"
#include "queue.hpp"
#include "task.hpp"

namespace {

auto gpio_led          = device::GpioLED::instance();
auto gpio_esw          = device::GpioESW::instance();
auto motor_rear_left   = device::RearLeftMotor::instance();
auto motor_rear_right  = device::RearRightMotor::instance();
auto motor_front_left  = device::FrontLeftMotor::instance();
auto motor_front_right = device::FrontRightMotor::instance();
auto imu_              = device::IMU::instance();
auto intercore_fifo    = device::Core0IntercoreFIFO::instance();

struct IMUStore
{
  int16_t accel_x = 0;
  int16_t accel_y = 0;
  int16_t accel_z = 0;
  int16_t gyro_x  = 0;
  int16_t gyro_y  = 0;
  int16_t gyro_z  = 0;
  int16_t temp    = 0;
} imu_store;

MotorDriver motor_rear_left_driver(
  Config::MOTOR_DRIVER_PID_KP,
  Config::MOTOR_DRIVER_PID_KI,
  Config::MOTOR_DRIVER_PID_KD,
  Config::MOTOR_CONTROL_WORKER_INTERVAL_MS
);
MotorDriver motor_rear_right_driver(
  Config::MOTOR_DRIVER_PID_KP,
  Config::MOTOR_DRIVER_PID_KI,
  Config::MOTOR_DRIVER_PID_KD,
  Config::MOTOR_CONTROL_WORKER_INTERVAL_MS
);
MotorDriver motor_front_left_driver(
  Config::MOTOR_DRIVER_PID_KP,
  Config::MOTOR_DRIVER_PID_KI,
  Config::MOTOR_DRIVER_PID_KD,
  Config::MOTOR_CONTROL_WORKER_INTERVAL_MS
);
MotorDriver motor_front_right_driver(
  Config::MOTOR_DRIVER_PID_KP,
  Config::MOTOR_DRIVER_PID_KI,
  Config::MOTOR_DRIVER_PID_KD,
  Config::MOTOR_CONTROL_WORKER_INTERVAL_MS
);

template<typename T>
void push_msg(T const& msg)
{
  if (!multicore_fifo_wready()) {
    return;
  }
  multicore_fifo_push_blocking(serialize(msg));
}

std::optional<message::RxMsg> pop_rx_msg()
{
  if (!intercore_fifo.has_data()) {
    return std::nullopt;
  }

  uint32_t const bytes{ intercore_fifo.read() };
  return message::parse_rx_msg(bytes);
}

template<unsigned Interval>
async_at_time_worker_t create_motor_control_worker()
{
  return task::create_scheduled_worker_in_ms<Interval>(
    [](async_context_t* context, async_at_time_worker_t* worker) {
      motor_rear_left_driver.update(motor_rear_left.get_encoder_value().value_or(0));
      motor_rear_left.drive(motor_rear_left_driver.drive_power());

      motor_front_left_driver.update(motor_front_left.get_encoder_value().value_or(0));
      motor_front_left.drive(motor_front_left_driver.drive_power());

      motor_rear_right_driver.update(motor_rear_right.get_encoder_value().value_or(0));
      motor_rear_right.drive(motor_rear_right_driver.drive_power());

      motor_front_right_driver.update(motor_front_right.get_encoder_value().value_or(0));
      motor_front_right.drive(motor_front_right_driver.drive_power());
    }
  );
}

template<unsigned Interval>
async_at_time_worker_t create_imu_control_worker()
{
  return task::create_scheduled_worker_in_ms<Interval>(
    [](async_context_t* context, async_at_time_worker_t* worker) {
      auto const ret    = imu_.read();
      imu_store.accel_x = ret.accel.x;
      imu_store.accel_y = ret.accel.y;
      imu_store.accel_z = ret.accel.z;
      imu_store.gyro_x  = ret.gyro.x;
      imu_store.gyro_y  = ret.gyro.y;
      imu_store.gyro_z  = ret.gyro.z;
      imu_store.temp    = ret.temp;
    }
  );
}

template<unsigned Interval>
async_at_time_worker_t create_led_heartbeat_worker()
{
  return task::create_scheduled_worker_in_ms<Interval>(
    [](async_context_t* context, async_at_time_worker_t* worker) {
      static bool led{ false };
      gpio_led.write(led);
      led = !led;
    }
  );
}

template<unsigned Interval>
async_at_time_worker_t create_emergency_worker()
{
  return task::create_scheduled_worker_in_ms<Interval>(
    [](async_context_t* context, async_at_time_worker_t* worker) {
      auto const esw = gpio_esw.read();
      if (esw) {
        motor_front_left_driver.emergency();
        motor_front_right_driver.emergency();
        motor_rear_left_driver.emergency();
        motor_rear_right_driver.emergency();
      } else {
        motor_front_left_driver.release();
        motor_front_right_driver.release();
        motor_rear_left_driver.release();
        motor_rear_right_driver.release();
      }
    }
  );
}

struct MsgVisitor
{
  void operator()(message::MotorMsg const& msg) const
  {
    // msg.value is in q7 format
    float const tics_per_sec{ static_cast<float>(msg.value) / 128.0f };
    switch (msg.param) {
      case message::MotorDevice::REAR_LEFT:
        motor_rear_left_driver.set_target_tics_per_sec(tics_per_sec);
        break;
      case message::MotorDevice::REAR_RIGHT:
        motor_rear_right_driver.set_target_tics_per_sec(tics_per_sec);
        break;
      case message::MotorDevice::FRONT_LEFT:
        motor_front_left_driver.set_target_tics_per_sec(tics_per_sec);
        break;
      case message::MotorDevice::FRONT_RIGHT:
        motor_front_right_driver.set_target_tics_per_sec(tics_per_sec);
        break;
    }
  }
  void operator()(message::EncoderMsg const& msg) const
  {
    switch (msg.param) {
      case message::MotorDevice::REAR_LEFT: {
        push_msg(message::EncoderMsg{ message::MotorDevice::REAR_LEFT,
                                      motor_rear_left_driver.accumulated_tics() });
        motor_rear_left_driver.clear_accumulated_tics();
      } break;
      case message::MotorDevice::REAR_RIGHT: {
        push_msg(message::EncoderMsg{ message::MotorDevice::REAR_RIGHT,
                                      motor_rear_right_driver.accumulated_tics() });
        motor_rear_right_driver.clear_accumulated_tics();
      } break;
      case message::MotorDevice::FRONT_LEFT: {
        push_msg(message::EncoderMsg{ message::MotorDevice::FRONT_LEFT,
                                      motor_front_left_driver.accumulated_tics() });
        motor_front_left_driver.clear_accumulated_tics();
      } break;
      case message::MotorDevice::FRONT_RIGHT: {
        push_msg(message::EncoderMsg{ message::MotorDevice::FRONT_RIGHT,
                                      motor_front_right_driver.accumulated_tics() });
        motor_front_right_driver.clear_accumulated_tics();
      } break;
    }
  }
  void operator()(message::ImuMsg const& msg) const
  {
    switch (msg.param) {
      case message::ImuData::ACCEL_X:
        push_msg(message::ImuMsg{ message::ImuData::ACCEL_X, imu_store.accel_x });
        break;
      case message::ImuData::ACCEL_Y:
        push_msg(message::ImuMsg{ message::ImuData::ACCEL_Y, imu_store.accel_y });
        break;
      case message::ImuData::ACCEL_Z:
        push_msg(message::ImuMsg{ message::ImuData::ACCEL_Z, imu_store.accel_z });
        break;
      case message::ImuData::GYRO_X:
        push_msg(message::ImuMsg{ message::ImuData::GYRO_X, imu_store.gyro_x });
        break;
      case message::ImuData::GYRO_Y:
        push_msg(message::ImuMsg{ message::ImuData::GYRO_Y, imu_store.gyro_y });
        break;
      case message::ImuData::GYRO_Z:
        push_msg(message::ImuMsg{ message::ImuData::GYRO_Z, imu_store.gyro_z });
        break;
      case message::ImuData::TEMP:
        push_msg(message::ImuMsg{ message::ImuData::TEMP, imu_store.temp });
        break;
    }
  }
};

void handle_msg()
{
  while (true) {
    auto const msg{ pop_rx_msg() };
    if (!msg.has_value()) {
      break;
    }
    std::visit(MsgVisitor{}, msg.value());
  }
}
}

namespace main_proc {
int run()
{
  ::imu_.init(Config::I2C_IMU_BAUDRATE);
  ::intercore_fifo.init();

  async_context_poll_t context;
  async_context_poll_init_with_defaults(&context);

  async_at_time_worker_t emergency_worker = create_emergency_worker<Config::EMERGENCY_WORKER_INTERVAL_MS>();
  async_context_add_at_time_worker_in_ms(&context.core, &emergency_worker, 0);

  async_at_time_worker_t led_heartbeat_worker =
    create_led_heartbeat_worker<Config::LED_HEARTBEAT_WORKER_INTERVAL_MS>();
  async_context_add_at_time_worker_in_ms(&context.core, &led_heartbeat_worker, 0);

  async_at_time_worker_t motor_control_worker =
    create_motor_control_worker<Config::MOTOR_CONTROL_WORKER_INTERVAL_MS>();
  async_context_add_at_time_worker_in_ms(&context.core, &motor_control_worker, 0);

  async_at_time_worker_t imu_control_worker =
    create_imu_control_worker<Config::IMU_CONTROL_WORKER_INTERVAL_MS>();
  async_context_add_at_time_worker_in_ms(&context.core, &imu_control_worker, 0);

  while (1) {
    handle_msg();
    async_context_poll(&context.core);
  }

  return 0;
}
}
