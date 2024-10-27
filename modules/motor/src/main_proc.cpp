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

std::atomic<int16_t> motor_rear_left_encoder_value   = 0;
std::atomic<int16_t> motor_rear_right_encoder_value  = 0;
std::atomic<int16_t> motor_front_left_encoder_value  = 0;
std::atomic<int16_t> motor_front_right_encoder_value = 0;
std::atomic<int16_t> imu_accel_x                     = 0;
std::atomic<int16_t> imu_accel_y                     = 0;
std::atomic<int16_t> imu_accel_z                     = 0;
std::atomic<int16_t> imu_gyro_x                      = 0;
std::atomic<int16_t> imu_gyro_y                      = 0;
std::atomic<int16_t> imu_gyro_z                      = 0;
std::atomic<int16_t> imu_temp                        = 0;

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
  if (!multicore_fifo_rvalid()) {
    return std::nullopt;
  }

  uint32_t const bytes{ multicore_fifo_pop_blocking() };
  return message::parse_rx_msg(bytes);
}

template<unsigned Interval>
async_at_time_worker_t create_motor_encoder_worker()
{
  return task::create_scheduled_worker_in_ms<Interval>(
    [](async_context_t* context, async_at_time_worker_t* worker) {
      if (auto const cnt = motor_rear_left.get_encoder_value(); cnt.has_value()) {
        motor_rear_left_encoder_value += cnt.value();
      }
      if (auto const cnt = motor_front_left.get_encoder_value(); cnt.has_value()) {
        motor_front_left_encoder_value += cnt.value();
      }
      if (auto const cnt = motor_rear_right.get_encoder_value(); cnt.has_value()) {
        motor_rear_right_encoder_value += cnt.value();
      }
      if (auto const cnt = motor_front_right.get_encoder_value(); cnt.has_value()) {
        motor_front_right_encoder_value += cnt.value();
      }
    }
  );
}

template<unsigned Interval>
async_at_time_worker_t create_imu_worker()
{
  return task::create_scheduled_worker_in_ms<Interval>(
    [](async_context_t* context, async_at_time_worker_t* worker) {
      auto const ret = imu_.read();
      ::imu_accel_x.store(ret.accel.x);
      ::imu_accel_y.store(ret.accel.y);
      ::imu_accel_z.store(ret.accel.z);
      ::imu_gyro_x.store(ret.gyro.x);
      ::imu_gyro_y.store(ret.gyro.y);
      ::imu_gyro_z.store(ret.gyro.z);
      ::imu_temp.store(ret.temp);
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
        motor_rear_left.emergency();
        motor_rear_right.emergency();
        motor_front_left.emergency();
        motor_front_right.emergency();
      } else {
        motor_rear_left.release();
        motor_rear_right.release();
        motor_front_left.release();
        motor_front_right.release();
      }
    }
  );
}

struct MsgVisitor
{
  void operator()(message::MotorMsg const& msg) const
  {
    auto const drive_power = msg.value / 32768.0;
    switch (msg.param) {
      case message::MotorDevice::REAR_LEFT:
        motor_rear_left.drive(drive_power);
        break;
      case message::MotorDevice::REAR_RIGHT:
        motor_rear_right.drive(drive_power);
        break;
      case message::MotorDevice::FRONT_LEFT:
        motor_front_left.drive(drive_power);
        break;
      case message::MotorDevice::FRONT_RIGHT:
        motor_front_right.drive(drive_power);
        break;
    }
  }
  void operator()(message::EncoderMsg const& msg) const
  {
    switch (msg.param) {
      case message::MotorDevice::REAR_LEFT: {
        push_msg(message::EncoderMsg{ message::MotorDevice::REAR_LEFT,
                                      ::motor_rear_left_encoder_value.exchange(0) });
      } break;
      case message::MotorDevice::REAR_RIGHT: {
        push_msg(message::EncoderMsg{ message::MotorDevice::REAR_RIGHT,
                                      ::motor_rear_right_encoder_value.exchange(0) });
      } break;
      case message::MotorDevice::FRONT_LEFT: {
        push_msg(message::EncoderMsg{ message::MotorDevice::FRONT_LEFT,
                                      ::motor_front_left_encoder_value.exchange(0) });
      } break;
      case message::MotorDevice::FRONT_RIGHT: {
        push_msg(message::EncoderMsg{ message::MotorDevice::FRONT_RIGHT,
                                      ::motor_front_right_encoder_value.exchange(0) });
      } break;
    }
  }
  void operator()(message::ImuMsg const& msg) const
  {
    switch (msg.param) {
      case message::ImuData::ACCEL_X:
        push_msg(message::ImuMsg{ message::ImuData::ACCEL_X, ::imu_accel_x.load() });
        break;
      case message::ImuData::ACCEL_Y:
        push_msg(message::ImuMsg{ message::ImuData::ACCEL_Y, ::imu_accel_y.load() });
        break;
      case message::ImuData::ACCEL_Z:
        push_msg(message::ImuMsg{ message::ImuData::ACCEL_Z, ::imu_accel_z.load() });
        break;
      case message::ImuData::GYRO_X:
        push_msg(message::ImuMsg{ message::ImuData::GYRO_X, ::imu_gyro_x.load() });
        break;
      case message::ImuData::GYRO_Y:
        push_msg(message::ImuMsg{ message::ImuData::GYRO_Y, ::imu_gyro_y.load() });
        break;
      case message::ImuData::GYRO_Z:
        push_msg(message::ImuMsg{ message::ImuData::GYRO_Z, ::imu_gyro_z.load() });
        break;
      case message::ImuData::TEMP:
        push_msg(message::ImuMsg{ message::ImuData::TEMP, ::imu_temp.load() });
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

  async_context_poll_t context;
  async_context_poll_init_with_defaults(&context);

  async_at_time_worker_t emergency_worker = create_emergency_worker<10>();
  async_context_add_at_time_worker_in_ms(&context.core, &emergency_worker, 0);

  async_at_time_worker_t led_heartbeat_worker = create_led_heartbeat_worker<500>();
  async_context_add_at_time_worker_in_ms(&context.core, &led_heartbeat_worker, 0);

  async_at_time_worker_t motor_encoder_worker = create_motor_encoder_worker<1>();
  async_context_add_at_time_worker_in_ms(&context.core, &motor_encoder_worker, 0);

  async_at_time_worker_t imu_worker = create_imu_worker<5>();
  async_context_add_at_time_worker_in_ms(&context.core, &imu_worker, 0);

  while (1) {
    handle_msg();
    async_context_poll(&context.core);
  }

  return 0;
}
}