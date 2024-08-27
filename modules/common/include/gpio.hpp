#pragma once

#include <algorithm>
#include <cmath>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "mp.hpp"

using _gpio_tag = struct
{};

template<unsigned PIN, unsigned FUNC = GPIO_FUNC_SIO, bool DIR=GPIO_OUT>
class Gpio
{
public:
  inline static Gpio& instance()
  {
    static Gpio instance;
    return instance;
  }

  bool read()
    requires mp::SameValue<FUNC, GPIO_FUNC_SIO>
  {
    return gpio_get(PIN);
  }

  void write(bool value)
    requires mp::SameValue<FUNC, GPIO_FUNC_SIO>
  {
    gpio_put(PIN, value);
  }

  void write(float value)
    requires mp::SameValue<FUNC, GPIO_FUNC_PWM>
  {
    auto const slice_num{ pwm_gpio_to_slice_num(PIN) };
    auto       clamped_value = std::clamp(value, 0.0f, 1.0f);
    uint16_t   level = static_cast<uint16_t>(std::round(pwm_hw->slice[slice_num].top * clamped_value));
    pwm_set_gpio_level(PIN, level);
  }

  void write(uint16_t value)
    requires mp::SameValue<FUNC, GPIO_FUNC_PWM>
  {
    pwm_set_gpio_level(PIN, value);
  }

  void set_polariry(bool inverted)
    requires mp::SameValue<FUNC, GPIO_FUNC_PWM>
  {
    auto const slice_num{ pwm_gpio_to_slice_num(PIN) };
    auto const channel{ pwm_gpio_to_channel(PIN) };
    auto const a_inverted{ (pwm_hw->slice[slice_num].csr & PWM_CH0_CSR_A_INV_BITS) != 0 };
    auto const b_inverted{ (pwm_hw->slice[slice_num].csr & PWM_CH0_CSR_B_INV_BITS) != 0 };
    auto const new_a_inverted = (channel == 0) ? inverted : a_inverted;
    auto const new_b_inverted = (channel == 1) ? inverted : b_inverted;
    pwm_set_output_polarity(slice_num, new_a_inverted, new_b_inverted);
  }

private:
  static constexpr auto _reg = mp::assoc_list::append<PIN, std::nullptr_t, _gpio_tag>();

  Gpio()
  {
    gpio_set_function(PIN, static_cast<gpio_function>(FUNC));

    if constexpr (FUNC == GPIO_FUNC_SIO) {
      gpio_init(PIN);
      gpio_set_dir(PIN, DIR);
    }
    if constexpr (FUNC == GPIO_FUNC_PWM) {
      auto conf{ pwm_get_default_config() };
      pwm_init(pwm_gpio_to_slice_num(PIN), &conf, true);
      pwm_set_wrap(pwm_gpio_to_slice_num(PIN), 0xffff);
    }
  }
};
