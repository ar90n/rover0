#include "pico/async_context_poll.h"
#include "task.hpp"
#include "utest.h"
#include <atomic>
#include <functional>

// Define counters as global variables for test state tracking
std::atomic<int> g_task_execution_count{ 0 };
std::atomic<int> g_first_task_count{ 0 };
std::atomic<int> g_second_task_count{ 0 };
std::atomic<int> g_fast_task_count{ 0 };
std::atomic<int> g_slow_task_count{ 0 };
bool             g_exception_caught = false;
bool             g_exit_loop = false;

// Define task functions for testing
void increment_execution_counter()
{
  g_task_execution_count++;
}

void increment_first_counter()
{
  g_first_task_count++;
}

void increment_second_counter()
{
  g_second_task_count++;
}

void increment_counter_with_exit()
{
  g_task_execution_count++;
  if (g_task_execution_count >= 5) {
    g_exit_loop = true;
  }
}

void increment_fast_counter()
{
  g_fast_task_count++;
}

void increment_slow_counter()
{
  g_slow_task_count++;
}

void throw_and_catch_exception()
{
  try {
    throw std::runtime_error("Test exception");
  } catch (...) {
    g_exception_caught = true;
  }
}

/**
 * @brief Reset all global test state before each test
 */
void reset_globals()
{
  g_task_execution_count = 0;
  g_first_task_count = 0;
  g_second_task_count = 0;
  g_fast_task_count = 0;
  g_slow_task_count = 0;
  g_exception_caught = false;
  g_exit_loop = false;
  pico_mock::reset();
}

// Function pointer type alias for task functions
using TaskFunction = void (*)();

// Test for single task execution
UTEST(TaskTest, SingleTaskExecution)
{
  reset_globals();

  TaskRunner runner(create_scheduled_task<10>(increment_execution_counter));

  // Verify that one task is scheduled
  EXPECT_EQ(pico_mock::get_scheduled_task_count(), 1);

  // Process tasks once
  runner.poll();

  // One increment from initial execution
  EXPECT_EQ(g_task_execution_count.load(), 1);

  // Advance time and verify task execution
  pico_mock::advance_time_ms(50); // Advance 50ms
  runner.poll();

  // Initial execution + 50ms/10ms = 5 executions for a total of 6
  EXPECT_EQ(g_task_execution_count.load(), 6);
}

// Test for scheduling multiple tasks
UTEST(TaskTest, MultipleTaskScheduling)
{
  reset_globals();

  TaskRunner runner{ 
    create_scheduled_task<10>(increment_first_counter),
    create_scheduled_task<20>(increment_second_counter) 
  };

  // Verify that two tasks are scheduled
  EXPECT_EQ(pico_mock::get_scheduled_task_count(), 2);

  // Initial execution
  runner.poll();
  EXPECT_EQ(g_first_task_count.load(), 1);
  EXPECT_EQ(g_second_task_count.load(), 1);

  // Advance 30ms
  pico_mock::advance_time_ms(30);
  runner.poll();

  // first_counter: initial + 30ms/10ms = 1 + 3 = 4 times
  // second_counter: initial + 30ms/20ms = 1 + 1 = 2 times
  EXPECT_EQ(g_first_task_count.load(), 4);
  EXPECT_EQ(g_second_task_count.load(), 2);
}

// Test for task loop with early exit condition
UTEST(TaskTest, TaskLoopWithEarlyExit)
{
  reset_globals();

  TaskRunner runner{ create_scheduled_task<10>(increment_counter_with_exit) };

  // Use controllable loop instead of run_forever
  for (int i = 0; i < 10 && !g_exit_loop; i++) {
    runner.poll();
    pico_mock::advance_time_ms(10);
  }

  // Verify counter is at least 5 (exit condition)
  EXPECT_GE(g_task_execution_count.load(), 5);
}

// Edge case: task throws an exception
UTEST(TaskTest, TaskExceptionHandling)
{
  reset_globals();

  TaskRunner runner{ create_scheduled_task<10>(throw_and_catch_exception) };

  // Execute task
  runner.poll();

  // Verify exception was caught
  EXPECT_TRUE(g_exception_caught);
}

// Simulation of tasks with different intervals
UTEST(TaskTest, DifferentIntervalTasksSimulation)
{
  reset_globals();

  TaskRunner runner{
    create_scheduled_task<10>(increment_fast_counter), 
    create_scheduled_task<100>(increment_slow_counter)
  };

  // 1 second simulation
  for (int i = 0; i < 100; i++) {
    runner.poll();
    pico_mock::advance_time_ms(10);
  }

  // 1 second = 100 executions of 10ms task
  EXPECT_EQ(g_fast_task_count.load(), 101); // Initial execution + 100 times

  // 1 second = 10 executions of 100ms task
  EXPECT_EQ(g_slow_task_count.load(), 11); // Initial execution + 10 times
}

// Test for execute_all_scheduled_tasks mock function
UTEST(TaskTest, ExecuteAllScheduledTasks)
{
  reset_globals();

  TaskRunner runner{
    create_scheduled_task<10>(increment_execution_counter),
    create_scheduled_task<20>(increment_execution_counter),
    create_scheduled_task<30>(increment_execution_counter)
  };

  // Execute all tasks
  pico_mock::execute_all_scheduled_tasks();

  // Verify all three tasks were executed
  EXPECT_EQ(g_task_execution_count.load(), 3);
  EXPECT_EQ(pico_mock::get_scheduled_task_count(), 3); // Tasks are rescheduled
}
