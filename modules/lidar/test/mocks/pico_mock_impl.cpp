#include "pico/async_context_poll.h"
#include <iostream>
#include <algorithm>

namespace pico_mock {
    uint64_t current_time_us = 0;
    std::vector<async_at_time_worker_t*> scheduled_workers;
    
    // Storage for lambda function copies
    std::vector<std::function<void()>> lambda_copies;
    
    void advance_time_us(uint64_t us) {
        uint64_t target_time = current_time_us + us;
        
        // Execute tasks while advancing time in 10ms increments until target time
        while (current_time_us < target_time) {
            // Calculate next step time (maximum up to target time)
            uint64_t next_step = std::min(current_time_us + 10000, target_time);
            current_time_us = next_step;
            
            // Identify workers that should be executed
            std::vector<async_at_time_worker_t*> to_execute;
            auto it = scheduled_workers.begin();
            while (it != scheduled_workers.end()) {
                if ((*it)->target_time_us <= current_time_us) {
                    to_execute.push_back(*it);
                    it = scheduled_workers.erase(it);
                } else {
                    ++it;
                }
            }
            
            // Execute workers
            for (auto* worker : to_execute) {
                try {
                    worker->do_work(nullptr, worker);
                } catch (const std::exception& e) {
                    std::cerr << "Exception in worker: " << e.what() << std::endl;
                }
            }
        }
    }
    
    void advance_time_ms(uint32_t ms) {
        advance_time_us(ms * 1000);
    }
    
    uint64_t get_current_time_us() {
        return current_time_us;
    }
    
    void execute_all_scheduled_tasks() {
        // Execute all tasks
        std::vector<async_at_time_worker_t*> workers_copy = scheduled_workers;
        scheduled_workers.clear();  // Clear first, leaving only rescheduled tasks
        
        for (auto* worker : workers_copy) {
            try {
                worker->do_work(nullptr, worker);
            } catch (const std::exception& e) {
                std::cerr << "Exception in worker: " << e.what() << std::endl;
            }
        }
    }
    
    size_t get_scheduled_task_count() {
        return scheduled_workers.size();
    }
    
    void reset() {
        current_time_us = 0;
        scheduled_workers.clear();
    }
}

// Mock implementation of Pico-SDK functions
void async_context_poll_init_with_defaults(async_context_poll_t* context) {
    context->data = nullptr;
}

void async_context_poll(async_context_t* context) {
    // Process tasks at current time
    // Identify workers that should be executed
    std::vector<async_at_time_worker_t*> to_execute;
    auto it = pico_mock::scheduled_workers.begin();
    while (it != pico_mock::scheduled_workers.end()) {
        if ((*it)->target_time_us <= pico_mock::current_time_us) {
            to_execute.push_back(*it);
            it = pico_mock::scheduled_workers.erase(it);
        } else {
            ++it;
        }
    }
    
    // Execute workers
    for (auto* worker : to_execute) {
        try {
            worker->do_work(context, worker);
        } catch (const std::exception& e) {
            std::cerr << "Exception in worker: " << e.what() << std::endl;
        }
    }
}

bool async_context_add_at_time_worker_in_ms(async_context_t* context, 
                                           async_at_time_worker_t* worker, 
                                           uint32_t delay_ms) {
    worker->target_time_us = pico_mock::current_time_us + (delay_ms * 1000);
    
    // Create a copy of the worker
    async_at_time_worker_t* worker_copy = new async_at_time_worker_t;
    *worker_copy = *worker;
    
    pico_mock::scheduled_workers.push_back(worker_copy);
    return true;
}

void sleep_ms(uint32_t ms) {
    pico_mock::advance_time_ms(ms);
}

uint32_t time_us_32() {
    return static_cast<uint32_t>(pico_mock::current_time_us);
}

uint64_t time_us_64() {
    return pico_mock::current_time_us;
}
