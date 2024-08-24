#pragma once

#include <optional>

#include "encoder.pio.h"
#include "gpio.hpp"

template<unsigned PIO_BASE>
class ProgramLoader
{
public:
  static void load()
  {
    static bool loaded{ false };
    if (!loaded) {
      loaded   = true;
      auto pio = reinterpret_cast<PIO>(PIO_BASE);
      pio_add_program(pio, &encoder_program);
    }
  }
};

template<unsigned PIN, unsigned PIO_BASE, unsigned SM>
class RotaryEncoder
{
public:
  inline static RotaryEncoder& instance()
  {
    static RotaryEncoder instance;
    return instance;
  }

  std::optional<uint16_t> get()
  {
    std::optional<uint16_t> ret{ std::nullopt };
    auto                    pio = reinterpret_cast<PIO>(PIO_BASE);
    while (0 < pio_sm_get_rx_fifo_level(pio, SM)) {
      ret = 255 - static_cast<uint16_t>(pio_sm_get(pio, SM));
    }

    return ret;
  }

private:
  static constexpr auto _reg = mp::assoc_list::append<PIN, std::nullptr_t, _gpio_tag>();
  RotaryEncoder()
  {
    ProgramLoader<PIO_BASE>::load();
    encoder_program_init(reinterpret_cast<PIO>(PIO_BASE), SM, 4, PIN);
  }
};
