#pragma once

#include "pico/async_context_poll.h"
#include <concepts>
#include <type_traits>
#include <utility>
#include <functional>
#include <memory>
#include <vector>

/**
 * @brief Concept for callable objects that can be used as tasks
 */
template<typename F>
concept TaskCallable = requires(F f) {
  { f() } -> std::same_as<void>;
};

// Forward declaration of TaskContext for use in task_internal
class TaskContext;

// Forward declarations for internal implementation details
namespace task_internal {
  bool schedule_worker(TaskContext& context, async_at_time_worker_t* worker);
}

/**
 * @brief Wrapper class for the async context to encapsulate PICO SDK dependencies
 */
class TaskContext {
private:
  async_context_poll_t context;

public:
  TaskContext() {
    async_context_poll_init_with_defaults(&context);
  }

  ~TaskContext() {
    // Cleanup if needed
  }

  async_context_poll_t& get_native_context() {
    return context;
  }

  // Prevent copying
  TaskContext(const TaskContext&) = delete;
  TaskContext& operator=(const TaskContext&) = delete;

  // Allow task_internal to access private members
  friend bool task_internal::schedule_worker(TaskContext& context, async_at_time_worker_t* worker);
};

// Global storage for function objects
namespace task_storage {
  inline std::vector<std::function<void()>> functions;
  
  inline size_t store_function(std::function<void()> func) {
    functions.push_back(func);
    return functions.size() - 1;
  }
  
  inline void execute_function(size_t index) {
    if (index < functions.size()) {
      functions[index]();
    }
  }
}

/**
 * @brief Wrapper class for a scheduled task to encapsulate PICO SDK dependencies
 */
template<unsigned Interval>
class ScheduledTask {
private:
  async_at_time_worker_t worker;
  size_t function_index;

public:
  template<TaskCallable F>
  ScheduledTask(F func) {
    // Store function in global storage
    function_index = task_storage::store_function(func);
    
    // Create worker
    worker = {
      .do_work =
        [](async_context_t* context, async_at_time_worker_t* worker) {
          // Execute the function
          size_t index = reinterpret_cast<size_t>(worker->user_data);
          task_storage::execute_function(index);
          
          // Schedule next execution
          async_context_add_at_time_worker_in_ms(context, worker, Interval);
        },
      .user_data = reinterpret_cast<void*>(function_index),
    };
  }

  void schedule(TaskContext& context) {
    task_internal::schedule_worker(context, &worker);
  }

  async_at_time_worker_t& get_native_worker() {
    return worker;
  }
  
  // Prevent copying
  ScheduledTask(const ScheduledTask&) = delete;
  ScheduledTask& operator=(const ScheduledTask&) = delete;
};

/**
 * @brief Creates and schedules a task with the specified interval
 * 
 * @tparam Interval Time interval in milliseconds
 * @tparam F Type of callable function
 * @param context The task context
 * @param func Function object to call
 */
template<unsigned Interval, TaskCallable F>
void schedule_task(TaskContext& context, F func) {
  auto task = ScheduledTask<Interval>(func);
  task.schedule(context);
}

/**
 * @brief Processes all pending tasks once
 * 
 * @param context The task context
 */
void process_tasks_once(TaskContext& context) {
  async_context_poll(&context.get_native_context().core);
}

/**
 * @brief Continuously processes tasks in a loop
 * 
 * @param context The task context
 * @param poll_interval_ms Time to sleep between processing cycles in milliseconds
 */
void run_task_loop(TaskContext& context, uint32_t poll_interval_ms = 10) {
  while (true) {
    process_tasks_once(context);
    sleep_ms(poll_interval_ms);
  }
}

// Implementation details
namespace task_internal {
  bool schedule_worker(TaskContext& context, async_at_time_worker_t* worker) {
    return async_context_add_at_time_worker_in_ms(&(context.get_native_context().core), worker, 0);
  }
}
