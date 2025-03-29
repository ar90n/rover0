#pragma once

#include "pico/async_context_poll.h"
#include <concepts>
#include <type_traits>
#include <utility>

/**
 * @brief Concept for callable objects that can be used as tasks
 */
template<typename F>
concept TaskCallable = requires(F f) {
  { f() } -> std::same_as<void>;
};

// functions in follwoing namespace are not exported
namespace {
template<TaskCallable F>
void* as_user_data(F f)
{
  if constexpr (std::is_pointer_v<F>) {
    return reinterpret_cast<void*>(f);
  } else {
    return reinterpret_cast<void*>(&f);
  }
}

template<TaskCallable F>
F as_callable(void* f)
{
  if constexpr (std::is_pointer_v<F>) {
    return reinterpret_cast<F>(f);
  } else {
    return *reinterpret_cast<F*>(f);
  }
}

/**
 * @brief Creates a worker that runs at regular intervals
 *
 * @tparam Interval Time interval in milliseconds
 * @tparam F Type of callable function
 * @param func Function object to call
 * @return async_at_time_worker_t Configured worker
 *
 * @note The function object must have a lifetime that exceeds the worker
 */
template<unsigned Interval, TaskCallable F>
async_at_time_worker_t create_scheduled_worker_in_ms(F func)
{
  return {
    .do_work =
      [](async_context_t* context, async_at_time_worker_t* worker) {
        as_callable<F>(worker->user_data)();
        async_context_add_at_time_worker_in_ms(context, worker, Interval);
      },
    .user_data = as_user_data(func),
  };
}

bool invoke_scheduled_worker(async_context_poll_t& context, async_at_time_worker_t* worker)
{
  return async_context_add_at_time_worker_in_ms(&(context.core), worker, 0);
}

}

template<unsigned Interval, TaskCallable F>
async_at_time_worker_t invoke_as_task(async_context_poll_t& context, F func)
{
  auto worker = create_scheduled_worker_in_ms<Interval>(func);
  invoke_scheduled_worker(context, &worker);
  return worker;
}

void poll_once(async_context_poll_t& context)
{
  async_context_poll(&context.core);
}

void poll(async_context_poll_t& context)
{
  while (true) {
    poll_once(context);
    sleep_ms(10);
  }
}