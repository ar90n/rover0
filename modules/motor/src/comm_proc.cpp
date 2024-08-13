#include "hardware/irq.h"
#include "pico/multicore.h"

#include "comm_proc.hpp"
#include "config.hpp"
#include "message.hpp"
#include "queue.hpp"
#include "transport.hpp"
#include "uart.hpp"

namespace {

using UartControl = Uart<
  Config::UART_CONTROL,
  Config::UART_CONTROL_TX_PIN,
  Config::UART_CONTROL_RX_PIN,
  Config::UART_BUFFER_SIZE>;

static FixedSizeQueue<uint32_t, Config::QUEUE_SIZE> to_controller_queue;

void uart_write(const uint8_t* buf, size_t len)
{
  UartControl::instance().write(buf, len);
}

void sio_irq()
{
  while (multicore_fifo_rvalid()) {
    to_controller_queue.push(multicore_fifo_pop_blocking());
  }

  multicore_fifo_clear_irq();
}
}

namespace comm_proc {
void run()
{
  multicore_fifo_clear_irq();

  UartControl::instance().init(115200);
  transport::init(uart_write);

  irq_set_exclusive_handler(SIO_IRQ_PROC1, sio_irq);
  irq_set_enabled(SIO_IRQ_PROC1, true);

  while (1) {
    while (!to_controller_queue.empty()) {
      uint32_t const bytes = to_controller_queue.pop().value();
      transport::send(bytes);
    }

    //while (UartControl::instance().has_data()) {
    //  auto const byte{ UartControl::instance().read() };
    //  if (auto const recv{ transport::consume(byte) }; recv.has_value()) {
    //    multicore_fifo_push_blocking(recv.value());
    //  }
    //}
  }
}
}