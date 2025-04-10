#pragma once

#include <optional>

#include "encoder.hpp"
#include "pico/stdlib.h"

template<typename T>
concept Encoder = requires(T t) {
  { T::instance() } -> std::same_as<T&>;
  { t.get() } -> std::same_as<std::optional<uint16_t>>;
};

template<uint FORWARD_PIN, uint BACKWARD_PIN, Encoder ET>
class Motor
{
public:
  enum Direction
  {
    FORWARD,
    BACKWARD,
  };

  using GpioForward  = Gpio<FORWARD_PIN, GPIO_FUNC_PWM>;
  using GpioBackward = Gpio<BACKWARD_PIN, GPIO_FUNC_PWM>;
  using Encoder      = ET;

  inline static Motor& instance()
  {
    static Motor instance;
    return instance;
  }

  void drive(float value)
  {
    if (is_emergency)
    {
      return;
    }

    float const scaled_value = rescale(value);
    GpioForward::instance().write(scaled_value);
    GpioBackward::instance().write(1.0f - scaled_value);
    direction = value < 0.0f ? Direction::BACKWARD : Direction::FORWARD;
  }

  void emergency()
  {
    is_emergency = true;
    GpioForward::instance().write(0.0f);
    GpioBackward::instance().write(0.0f);
  }

  void release() { is_emergency = false; }

  std::optional<int16_t> get_encoder_value()
  {
    auto const encoder_value = Encoder::instance().get();
    if (!encoder_value.has_value())
    {
      return std::nullopt;
    }

    auto const value = static_cast<int16_t>(encoder_value.value());
    if (direction == Direction::BACKWARD)
    {
      return -value;
    }

    return value;
  }

private:
  Motor()
    : direction{ Direction::FORWARD }
    , is_emergency{ false }
  {
    GpioForward::instance().set_polariry(true);
    GpioBackward::instance().set_polariry(false);
  }

  static float rescale(float value) { return (1.0 + std::clamp(value, -1.0f, 1.0f)) / 2.0; }

  Direction direction;
  bool      is_emergency;
};
