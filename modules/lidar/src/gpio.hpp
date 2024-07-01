#ifndef GPIO_HPP_
#define GPIO_HPP_

#include "hardware/gpio.h"
#include <algorithm>
#include <cmath>

#include "mp.hpp"

using _gpio_tag = struct {};

template <unsigned PIN, int FUNC = GPIO_FUNC_SIO> class Gpio {
public:
  static Gpio &instance() {
    static Gpio instance;
    return instance;
  }

  void write(bool value) { gpio_put(PIN, value); }

  void write(float value) {
    auto clamped_value = std::clamp(value, 0.0f, 1.0f);
    uint16_t level = static_cast<uint16_t>(
        std::round(pwm_hw->slice[PIN].top * clamped_value));
    pwm_set_gpio_level(PIN, level);
  }

private:
  static constexpr auto _reg =
      mp::assoc_list::append<PIN, std::nullptr_t, _gpio_tag>();

  Gpio() {
    gpio_set_function(PIN, static_cast<gpio_function>(FUNC));

    if constexpr (FUNC == GPIO_FUNC_SIO) {
      gpio_init(PIN);
      gpio_set_dir(PIN, GPIO_OUT);
    }
    if constexpr (FUNC == GPIO_FUNC_PWM) {
      auto conf{pwm_get_default_config()};
      pwm_init(pwm_gpio_to_slice_num(PIN), &conf, true);
    }
  }
};

#endif // GPIO_HPP_