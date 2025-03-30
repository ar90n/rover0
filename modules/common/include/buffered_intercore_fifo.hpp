#pragma once

#include <stdint.h>
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "queue.hpp"

template <int N> class BufferedIntercoreFIFO {
public:
  enum class State { CREATED, INITIALIZED, CLOSED };

  inline static BufferedIntercoreFIFO &instance() {
    static BufferedIntercoreFIFO<N> instance;
    return instance;
  }

  ~BufferedIntercoreFIFO() {
    if (state == State::INITIALIZED) {
      close();
    }
  }

  void init() {
    irq_set_exclusive_handler(irq(), isr);
    irq_set_enabled(irq(), true);

    state = State::INITIALIZED;
  }

  void close() {
    irq_set_enabled(irq(), false);
    state = State::CLOSED;
  }

  bool has_data() { return !queue.empty(); }

  size_t size() { return queue.size(); }

  uint32_t read() { return queue.pop().value_or(0); }

  size_t read(uint32_t *buf, size_t len, int timeout) {
    for (size_t i = 0; i < len; i++) {
      auto v = queue.pop();
      if (!v.has_value()) {
        return i;
      }

      buf[i] = v.value();
    }
    return len;
  }

private:
  BufferedIntercoreFIFO() = default;

  State state = State::CREATED;
  static FixedSizeQueue<uint32_t, N> queue;

  static void isr() {
    while (multicore_fifo_rvalid()) {
      queue.push(multicore_fifo_pop_blocking());
    }
  }

  static uint32_t irq() {
    if (get_core_num() == 0) {
        return SIO_IRQ_PROC0;
    } else {
        return SIO_IRQ_PROC1;
    }
  }
};

template <int N>
FixedSizeQueue<uint32_t, N> BufferedIntercoreFIFO<N>::queue{};
