#pragma once

#include <functional>
#include <vector>
#include <cstdint>

// Pico-SDKの型定義
typedef struct async_context_t {
    void* data;
} async_context_t;

typedef struct async_context_poll_t {
    async_context_t core;
    void* data;
} async_context_poll_t;

typedef struct async_at_time_worker_t {
    void (*do_work)(async_context_t*, struct async_at_time_worker_t*);
    void* user_data;
    uint64_t target_time_us;
} async_at_time_worker_t;

// モック関数の宣言
void async_context_poll_init_with_defaults(async_context_poll_t* context);
void async_context_poll(async_context_t* context);
bool async_context_add_at_time_worker_in_ms(async_context_t* context, 
                                           async_at_time_worker_t* worker, 
                                           uint32_t delay_ms);
void sleep_ms(uint32_t ms);
uint32_t time_us_32();
uint64_t time_us_64();

// テスト用ヘルパー関数
namespace pico_mock {
    // 現在の時間を進める
    void advance_time_ms(uint32_t ms);
    void advance_time_us(uint64_t us);
    
    // 現在の時間を取得
    uint64_t get_current_time_us();
    
    // スケジュールされたタスクをすべて実行
    void execute_all_scheduled_tasks();
    
    // スケジュールされたタスクの数を取得
    size_t get_scheduled_task_count();
    
    // モック状態をリセット
    void reset();
    
    // 内部状態へのアクセス（テスト用）
    extern uint64_t current_time_us;
    extern std::vector<async_at_time_worker_t*> scheduled_workers;
}
