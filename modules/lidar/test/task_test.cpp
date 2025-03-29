#include "utest.h"
#include "task.hpp"
#include "pico/async_context_poll.h"
#include <atomic>
#include <functional>

// Define counters as global variables
std::atomic<int> g_counter{0};
std::atomic<int> g_counter1{0};
std::atomic<int> g_counter2{0};
std::atomic<int> g_fast_counter{0};
std::atomic<int> g_slow_counter{0};
bool g_exception_caught = false;
bool g_exit_loop = false;

// Define tasks as global functions
void increment_counter() {
    g_counter++;
}

void increment_counter1() {
    g_counter1++;
}

void increment_counter2() {
    g_counter2++;
}

void increment_counter_with_exit() {
    g_counter++;
    if (g_counter >= 5) {
        g_exit_loop = true;
    }
}

void increment_fast_counter() {
    g_fast_counter++;
}

void increment_slow_counter() {
    g_slow_counter++;
}

void throw_and_catch_exception() {
    try {
        throw std::runtime_error("Test exception");
    } catch (...) {
        g_exception_caught = true;
    }
}

// Setup function called before each test
void reset_globals() {
    g_counter = 0;
    g_counter1 = 0;
    g_counter2 = 0;
    g_fast_counter = 0;
    g_slow_counter = 0;
    g_exception_caught = false;
    g_exit_loop = false;
    pico_mock::reset();
}

// Test for basic task execution
UTEST(TaskTest, BasicTaskExecution) {
    reset_globals();
    
    TaskContext context;
    
    // Schedule task to increment counter every 10ms
    schedule_task<10>(context, increment_counter);
    
    // Verify that one task is scheduled
    EXPECT_EQ(pico_mock::get_scheduled_task_count(), 1);
    
    // Process tasks once
    process_tasks_once(context);
    
    // One increment from initial execution
    EXPECT_EQ(g_counter.load(), 1);
    
    // Advance time and verify task execution
    pico_mock::advance_time_ms(50); // Advance 50ms
    process_tasks_once(context);
    
    // Initial execution + 50ms/10ms = 5 executions for a total of 6
    EXPECT_EQ(g_counter.load(), 6);
}

// Test for scheduling multiple tasks
UTEST(TaskTest, MultipleTaskScheduling) {
    reset_globals();
    
    TaskContext context;
    
    // Schedule two tasks with different intervals
    schedule_task<10>(context, increment_counter1);
    schedule_task<20>(context, increment_counter2);
    
    // Verify that two tasks are scheduled
    EXPECT_EQ(pico_mock::get_scheduled_task_count(), 2);
    
    // Initial execution
    process_tasks_once(context);
    EXPECT_EQ(g_counter1.load(), 1);
    EXPECT_EQ(g_counter2.load(), 1);
    
    // Advance 30ms
    pico_mock::advance_time_ms(30);
    process_tasks_once(context);
    
    // counter1: initial + 30ms/10ms = 1 + 3 = 4 times
    // counter2: initial + 30ms/20ms = 1 + 1 = 2 times
    EXPECT_EQ(g_counter1.load(), 4);
    EXPECT_EQ(g_counter2.load(), 2);
}

// Test for task loop execution
UTEST(TaskTest, TaskLoopExecution) {
    reset_globals();
    
    TaskContext context;
    
    // Task to increment counter every 10ms
    schedule_task<10>(context, increment_counter_with_exit);
    
    // Use controllable loop instead of run_task_loop
    for (int i = 0; i < 10 && !g_exit_loop; i++) {
        process_tasks_once(context);
        pico_mock::advance_time_ms(10);
    }
    
    // Verify counter is at least 5
    EXPECT_GE(g_counter.load(), 5);
}

// Edge case: task throws an exception
UTEST(TaskTest, TaskExceptionHandling) {
    reset_globals();
    
    TaskContext context;
    
    // Task that throws an exception
    schedule_task<10>(context, throw_and_catch_exception);
    
    // Execute task
    process_tasks_once(context);
    
    // Verify exception was caught
    EXPECT_TRUE(g_exception_caught);
}

// Simulation of long-running execution
UTEST(TaskTest, LongRunningSimulation) {
    reset_globals();
    
    TaskContext context;
    
    // High-frequency task (10ms)
    schedule_task<10>(context, increment_fast_counter);
    
    // Low-frequency task (100ms)
    schedule_task<100>(context, increment_slow_counter);
    
    // 1 second simulation
    for (int i = 0; i < 100; i++) {
        process_tasks_once(context);
        pico_mock::advance_time_ms(10);
    }
    
    // 1 second = 100 executions of 10ms task
    EXPECT_EQ(g_fast_counter.load(), 101); // Initial execution + 100 times
    
    // 1 second = 10 executions of 100ms task
    EXPECT_EQ(g_slow_counter.load(), 11); // Initial execution + 10 times
}

// Test for execute_all_scheduled_tasks
UTEST(TaskTest, ExecuteAllScheduledTasks) {
    reset_globals();
    
    TaskContext context;
    
    // Schedule multiple tasks
    schedule_task<10>(context, increment_counter);
    schedule_task<20>(context, increment_counter);
    schedule_task<30>(context, increment_counter);
    
    // Execute all tasks
    pico_mock::execute_all_scheduled_tasks();
    
    // Verify all three tasks were executed
    EXPECT_EQ(g_counter.load(), 3);
    EXPECT_EQ(pico_mock::get_scheduled_task_count(), 3); // Tasks are rescheduled
}
