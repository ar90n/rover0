#pragma once

#include <format>
#include <functional>

#include <pico/mutex.h>

namespace logger {
void _log_impl(std::string_view msg);
void init(std::function<void(std::string_view)> writer);
template<typename... _Args>
void log(std::format_string<_Args...> __fmt, _Args&&... __args)
{
  _log_impl(std::format(__fmt, std::forward<_Args>(__args)...));
}
}
