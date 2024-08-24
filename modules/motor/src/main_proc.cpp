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
        auto const r = cnt.value();
        push_msg(message::EncoderMsg{ message::MotorDevice::REAR_LEFT, r });
      }
      if (auto const cnt = motor_front_left.get_encoder_value(); cnt.has_value()) {
        auto const r = cnt.value();
        push_msg(message::EncoderMsg{ message::MotorDevice::FRONT_LEFT, r });
      }
      if (auto const cnt = motor_rear_right.get_encoder_value(); cnt.has_value()) {
        auto const r = cnt.value();
        push_msg(message::EncoderMsg{ message::MotorDevice::REAR_RIGHT, r });
      }
      if (auto const cnt = motor_front_right.get_encoder_value(); cnt.has_value()) {
        auto const r = cnt.value();
        push_msg(message::EncoderMsg{ message::MotorDevice::FRONT_RIGHT, r });
      }
    }
  );
}

FixedSizeQueue<message::MotorMsg, Config::QUEUE_SIZE> motor_msg_queue;
template<unsigned Interval>
async_at_time_worker_t create_motor_drive_worker()
{
  return task::create_scheduled_worker_in_ms<Interval>(
    [](async_context_t* context, async_at_time_worker_t* worker) {
      while (!motor_msg_queue.empty()) {
        auto const msg = motor_msg_queue.pop();
        if (!msg.has_value()) {
          continue;
        }

        auto const drive_power = msg.value().value / 32768.0;
        switch (msg.value().param) {
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
    }
  );
}

template<unsigned Interval>
async_at_time_worker_t create_imu_worker()
{
  return task::create_scheduled_worker_in_ms<Interval>(
    [](async_context_t* context, async_at_time_worker_t* worker) {
      auto const ret = imu_.read();
      push_msg(message::ImuMsg{ message::ImuData::ACCEL_X, ret.accel.x });
      push_msg(message::ImuMsg{ message::ImuData::ACCEL_Y, ret.accel.y });
      push_msg(message::ImuMsg{ message::ImuData::ACCEL_Z, ret.accel.z });
      push_msg(message::ImuMsg{ message::ImuData::GYRO_X, ret.gyro.x });
      push_msg(message::ImuMsg{ message::ImuData::GYRO_Y, ret.gyro.y });
      push_msg(message::ImuMsg{ message::ImuData::GYRO_Z, ret.gyro.z });
      push_msg(message::ImuMsg{ message::ImuData::TEMP, ret.temp });
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
  void operator()(message::MotorMsg const& msg) const { motor_msg_queue.push(msg); }
};

void dispathc_msg()
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

  async_at_time_worker_t motor_encoder_worker = create_motor_encoder_worker<100>();
  async_context_add_at_time_worker_in_ms(&context.core, &motor_encoder_worker, 0);

  async_at_time_worker_t imu_worker = create_imu_worker<100>();
  async_context_add_at_time_worker_in_ms(&context.core, &imu_worker, 0);

  async_at_time_worker_t motor_drive_worker = create_motor_drive_worker<100>();
  async_context_add_at_time_worker_in_ms(&context.core, &motor_drive_worker, 0);

  while (1) {
    dispathc_msg();
    async_context_poll(&context.core);
  }

  return 0;
}
}