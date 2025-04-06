#pragma once

#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "mp.hpp"
#include "queue.hpp"

using _uart_tag = struct
{
};

template <int M, int TX, int RX, int N, bool WRITE_IN_ISR=true>
class Uart
{
public:
  enum class State
  {
    CREATED,
    INITIALIZED,
    CLOSED
  };

  inline static Uart &instance()
  {
    static Uart<M, TX, RX, N, WRITE_IN_ISR> instance;
    return instance;
  }

  ~Uart()
  {
    if (state == State::INITIALIZED)
    {
      close();
    }
  }

  void init(uint baudrate)
  {
    uart_init(inst(), baudrate);
    gpio_set_function(TX, GPIO_FUNC_UART);
    gpio_set_function(RX, GPIO_FUNC_UART);

    irq_set_exclusive_handler(irq(), isr);
    irq_set_enabled(irq(), true);
    uart_set_irq_enables(inst(), true, false);

    state = State::INITIALIZED;
  }

  void close()
  {
    uart_set_irq_enables(inst(), false, false);
    irq_set_enabled(irq(), false);
    uart_deinit(inst());
    state = State::CLOSED;
  }

  bool has_rx_data() { return !rx_queue.empty(); }
  bool has_tx_data() { return !tx_queue.empty(); }

  uint8_t read() { return rx_queue.pop().value_or(0); }
  bool write(const uint8_t data) requires (!WRITE_IN_ISR)
  {
    uart_putc_raw(inst(), data);
    return true;
  }
  bool write(const uint8_t data) requires (WRITE_IN_ISR)
  {
    auto const ret{tx_queue.push(data)};
    uart_set_irq_enables(inst(), true, true);
    return ret;
  }

  size_t write(const uint8_t *buf, size_t len)
  {
    size_t i = 0;
    for (; i < len; i++)
    {
      if (!write(buf[i]))
      {
        break;
      }
    }

    return i;
  }

  size_t read(uint8_t *buf, size_t len, int timeout)
  {
    for (size_t i = 0; i < len; i++)
    {
      auto v = rx_queue.pop();
      if (!v.has_value())
      {
        return i;
      }

      buf[i] = v.value();
    }
    return len;
  }

private:
  static constexpr auto _reg =
      mp::assoc_list::append<M, std::nullptr_t, _uart_tag>();

  Uart() = default;

  State state = State::CREATED;
  static FixedSizeQueue<uint8_t, N> rx_queue;
  static FixedSizeQueue<uint8_t, N> tx_queue;

  static void isr()
  {
    while (uart_is_readable(inst()))
    {
      rx_queue.push(uart_getc(inst()));
    }

    while (uart_is_writable(inst()) && (0 < tx_queue.size()))
    {
      uart_putc_raw(inst(), tx_queue.pop().value_or(0));
    }

    if (tx_queue.empty())
    {
      uart_set_irq_enables(inst(), true, false);
    }
  }

  constexpr static uart_inst_t *inst()
  {
    static_assert(M == 0 || M == 1, "Invalid UART instance");
    uart_inst_t *vs[] = {uart0, uart1};
    return vs[M];
  }

  constexpr static uint irq()
  {
    static_assert(M == 0 || M == 1, "Invalid UART instance");
    uint vs[] = {UART0_IRQ, UART1_IRQ};
    return vs[M];
  }
};

template <int M, int TX, int RX, int N, bool WRITE_IN_ISR>
FixedSizeQueue<uint8_t, N> Uart<M, TX, RX, N, WRITE_IN_ISR>::rx_queue{};

template <int M, int TX, int RX, int N, bool WRITE_IN_ISR>
FixedSizeQueue<uint8_t, N> Uart<M, TX, RX, N, WRITE_IN_ISR>::tx_queue{};
