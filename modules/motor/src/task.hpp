#pragma once

#include "pico/async_context_poll.h"

namespace task {

using Callback = void (*)(async_context_t* context, async_at_time_worker_t* user_data);

template<unsigned Interval, typename T = void>
async_at_time_worker_t create_scheduled_worker_in_ms(Callback callback, T* user_data = nullptr)
{
  struct _container
  {
    Callback callback;
    T*       user_data;
  };

  return { .do_work =
             [](async_context_t* context, async_at_time_worker_t* worker) {
               ((Callback)worker->user_data)(context, worker);
               async_context_add_at_time_worker_in_ms(context, worker, Interval);
             },
           .user_data = (void*)callback };
}
}