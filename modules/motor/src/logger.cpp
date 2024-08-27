#include <string_view>

#include <pico/mutex.h>

#include "logger.hpp"

namespace {
static mutex_t                        m;
std::function<void(std::string_view)> writer = [](std::string_view msg) {};
}

namespace logger {
void _log_impl(std::string_view msg)
{
  uint32_t owner;
  if (!mutex_try_enter(&::m, &owner)) {
    if (owner == get_core_num()) {
      return;
    }
    mutex_enter_blocking(&::m);
  }

  auto const abs_time = get_absolute_time();
  auto const time_ms  = to_ms_since_boot(abs_time);

  writer(std::format("[{:08}] ", time_ms));
  writer(msg);
  writer("\r\n");

  mutex_exit(&::m);
}

void init(std::function<void(std::string_view)> writer)
{
  mutex_init(&::m);
  ::writer = writer;
}
}